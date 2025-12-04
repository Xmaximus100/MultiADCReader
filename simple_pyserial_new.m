function simple_pyserial_new()        
    % --- Stałe konfiguracyjne ---
    PORT = "COM50";      % zmień wg potrzeb
    BAUD = 115200;
    FRAME_LEN = 64;      % długość oczekiwanej ramki binarnej (główna)
    FRAME_LEN_32 = 32;   % długość alternatywnej ramki binarnej
    READ_TIMEOUT = 0.2;  % sekundy: krótki timeout, żeby pętla była responsywna
    
    % Komendy inicjalizujące (jako cell array stringów)
    INIT_CMDS = {
        "AT+ADC:SETUP 10000,2000,2,1\r\n";
        "AT+ADC:SETUP 100,20000,7,0\r\n";
        "AT+ADC:FORMAT C\r\n";
        "AT+ADC:START\r\n";
        "AT+ADC:STOP\r\n";
        "AT+CLK:SOURCE E"
    };
    
    BYTE_DATA_CMD = {"AT+ADC:FORMAT R"};
    STRING_DATA_CMD = {"AT+ADC:FORMAT C"};
    STRING_RESPONSE_CMD = {"AT+CLK:SOURCE"; "AT+CLK:CONFIG"; "AT+CLK:SPEED"; "AT+HELP"};
    
    % --- Zmienne globalne ---
    % Inicjalizacja zmiennych globalnych
    output_data = struct();
    output_data.ch0 = [];
    output_data.ch1 = [];
    output_data.ch2 = [];
    output_data.ch3 = [];
    output_data.ch4 = [];
    output_data.ch5 = [];
    output_data.ch6 = [];
    output_data.ch7 = [];
    output_data.raw = {};  % cell array dla ramek bajtowych
    
    repetitive_line = "";
    data_collected = false;
    data_amount = 0;
    frame_count_64 = 0;
    frame_count_32 = 0;
    last_frame_time = 0.0;
    timeout_ms = 150;
    extended_timeout_ms = 250;
    EXPECTED_FRAMES_64 = 4061;
    EXPECTED_FRAMES_32 = 1;
    FRAME_TOLERANCE = 2;
    auto_send_enabled = true;
    collection_stopped = false;
    bytes_data_expected = true;
    config_message_expected = false;
    message_timeout = 0.0;
    
    session = struct();
    session.port = PORT;
    session.baud = BAUD;
    session.read_timeout = READ_TIMEOUT;
    session.ser = [];
    session.read_buffer = uint8([]);
    session.stop_evt = false;
    serial_port = [];
    reader_timer = [];
    
    % --- Funkcje pomocnicze ---
    
    function ports = list_available_ports()
        % Zwraca listę dostępnych portów szeregowych
        try
            serial_info = serialportlist("available");
            ports = string(serial_info);
        catch
            ports = string([]);
        end
    end
    
    function [should_end, reason] = should_end_collection()
        % Sprawdza czy powinna zakończyć się kolekcja danych
        should_end = false;
        reason = "";
        
        if isempty(repetitive_line) || repetitive_line == ""
            return;
        end
        
        if last_frame_time == 0
            return;
        end
        
        % Czas w milisekundach (używamy now * 86400 * 1000 dla milisekund)
        current_time = now * 86400 * 1000;  % czas w milisekundach
        time_since_last_frame = current_time - last_frame_time;
        
        % Sprawdź czy mamy dokładnie oczekiwaną liczbę ramek
        if frame_count_64 == EXPECTED_FRAMES_64 && frame_count_32 == EXPECTED_FRAMES_32
            should_end = true;
            reason = "dokładna liczba ramek";
            return;
        end
        
        % Sprawdź timeout - podstawowy
        if time_since_last_frame >= timeout_ms
            min_frames_64 = EXPECTED_FRAMES_64 - FRAME_TOLERANCE;
            max_frames_64 = EXPECTED_FRAMES_64 + FRAME_TOLERANCE;
            
            if frame_count_64 >= min_frames_64 && frame_count_64 <= max_frames_64
                should_end = true;
                reason = sprintf("timeout (%dms) + rozsądna liczba ramek", timeout_ms);
                return;
            end
            
            % Jeśli za mało ramek, sprawdź przedłużony timeout
            if time_since_last_frame >= extended_timeout_ms
                if frame_count_64 < min_frames_64
                    should_end = true;
                    reason = sprintf("przedłużony timeout (%dms) - za mało ramek (%d)", extended_timeout_ms, frame_count_64);
                else
                    should_end = true;
                    reason = sprintf("przedłużony timeout (%dms) - za dużo ramek (%d)", extended_timeout_ms, frame_count_64);
                end
            end
        end
    end
    
    function finish_collection()
        % Kończy kolekcję danych - wywołuje dekodowanie i reset
        
        if data_collected || isempty(output_data.raw) || length(output_data.raw) == 0
            return;
        end
        
        data_collected = true;
        total_frames = length(output_data.raw);
        expected_samples = (frame_count_64 * 4) + (frame_count_32 * 2);
        [should_end, reason] = should_end_collection();
        
        fprintf("[INFO] Zakończenie kolekcji (timeout): %s\n", reason);
        fprintf("[INFO] Odebrano %d próbek (oczekiwane: %d).\n", data_amount, expected_samples);
        fprintf("[INFO] Ramki: %d total (%dx64B + %dx32B)\n", total_frames, frame_count_64, frame_count_32);
        
        % Sprawdź faktyczne rozmiary ramek w buforze
        actual_64 = 0;
        actual_32 = 0;
        for i = 1:length(output_data.raw)
            if length(output_data.raw{i}) == FRAME_LEN
                actual_64 = actual_64 + 1;
            elseif length(output_data.raw{i}) == FRAME_LEN_32
                actual_32 = actual_32 + 1;
            end
        end
        
        if actual_64 ~= frame_count_64 || actual_32 ~= frame_count_32
            fprintf("[WARN] Niespójność! Faktyczne ramki: %dx64B + %dx32B\n", actual_64, actual_32);
        end
        
        % Ostrzeżenie jeśli liczba ramek jest poza tolerancją
        if frame_count_64 < EXPECTED_FRAMES_64 - FRAME_TOLERANCE
            fprintf("[WARN] Za mało ramek 64B! Oczekiwane: %d, otrzymane: %d\n", EXPECTED_FRAMES_64, frame_count_64);
        elseif frame_count_64 > EXPECTED_FRAMES_64 + FRAME_TOLERANCE
            fprintf("[WARN] Za dużo ramek 64B! Oczekiwane: %d, otrzymane: %d\n", EXPECTED_FRAMES_64, frame_count_64);
        end
        
        fprintf("[INFO] Rozpoczynam dekodowanie...\n");
        
        % Automatyczne dekodowanie wszystkich ramek zgodnie z ich rozmiarem
        for i = 1:length(output_data.raw)
            frame_data = output_data.raw{i};
            if length(frame_data) == FRAME_LEN
                decode_channels(frame_data);
            elseif length(frame_data) == FRAME_LEN_32
                decode_channels_32(frame_data);
            end
        end
        fprintf("[INFO] Dekodowanie zakończone.\n");
        
        % Automatyczne wysłanie komendy SETUP jeśli jest ustawiona i auto_send_enabled
        if auto_send_enabled && ~isempty(repetitive_line) && repetitive_line ~= "" && ~isempty(serial_port) && isvalid(serial_port)
            try
                write(serial_port, repetitive_line, "string");
                fprintf("[TX repetitive auto] '%s'\n", repetitive_line);
                last_frame_time = 0;
            catch ME
                fprintf("[ERR] Błąd automatycznego wysłania komendy: %s\n", ME.message);
            end
        elseif ~auto_send_enabled
            fprintf("[INFO] Automatyczne wysyłanie zatrzymane - komenda SETUP nie została wysłana.\n");
        end
        
        % Reset liczników (ale NIE czyść danych - pozostają do odczytu)
        data_collected = false;
        data_amount = 0;
        frame_count_64 = 0;
        frame_count_32 = 0;
        if last_frame_time == 0
            last_frame_time = 0;
        end
        fprintf("[INFO] Kolekcja zakończona. Dane pozostają w buforze (użyj /data aby je odczytać, /clear aby wyczyścić).\n");
    end
    
    function safe_write(ser, data, ~)
        % Bezpieczne zapisywanie do portu szeregowego (lock jest ignorowany w MATLAB)
        try
            if isa(data, 'string') || isa(data, 'char')
                write(ser, data, "string");
            else
                write(ser, data, "uint8");
            end
        catch ME
            rethrow(ME);
        end
    end
    
    function decode_channels(frame)
        % Decode the channels from the frame 64B
        for i = 1:2:15  % 0, 2, 4, ..., 14
            ch_idx = floor((i-1) / 2);  % 0-7
            ch_name = sprintf("ch%d", ch_idx);
            
            % 4 próbki na kanał w ramce 64B: z przesunięciem o 0, 16, 32, 48
            % MATLAB używa 1-indexingu, więc i+1 zamiast i
            val1 = double(frame(i)) + double(frame(i+1)) * 256;  % little-endian
            val2 = double(frame(i+16)) + double(frame(i+17)) * 256;  % little-endian
            val3 = double(frame(i+32)) + double(frame(i+33)) * 256;  % little-endian
            val4 = double(frame(i+48)) + double(frame(i+49)) * 256;  % little-endian
            
            output_data.(ch_name) = [output_data.(ch_name), double(val1), double(val2), double(val3), double(val4)];
        end
    end
    
    function decode_channels_32(frame)
        % Decode the channels from the frame 32B
        for i = 1:2:15  % 0, 2, 4, ..., 14
            ch_idx = floor((i-1) / 2);  % 0-7
            ch_name = sprintf("ch%d", ch_idx);
            
            % 2 próbki na kanał w ramce 32B: z przesunięciem o 0, 16
            val1 = double(frame(i)) + double(frame(i+1)) * 256;  % little-endian
            val2 = double(frame(i+16)) + double(frame(i+17)) * 256;  % little-endian
            
            output_data.(ch_name) = [output_data.(ch_name), double(val1), double(val2)];
        end
    end
    
    function collect_data(frame, frame_size)
        % Collect the data from the frame    
        if collection_stopped
            return;
        end
        
        output_data.raw{end+1} = frame;
        
        % Aktualizuj czas ostatniej ramki
        last_frame_time = now * 86400 * 1000;  % czas w milisekundach
        
        % Licz ramki według rozmiaru
        if frame_size == FRAME_LEN
            frame_count_64 = frame_count_64 + 1;
        elseif frame_size == FRAME_LEN_32
            frame_count_32 = frame_count_32 + 1;
        end
        
        % Ramka 64B = 4 próbki, ramka 32B = 2 próbki
        if frame_size == FRAME_LEN
            samples_per_frame = 4;
        else
            samples_per_frame = 2;
        end
        data_amount = data_amount + samples_per_frame;
        
        % Sprawdź czy powinna zakończyć się kolekcja
        [should_end, reason] = should_end_collection();
        
        if should_end
            data_collected = true;
            total_frames = length(output_data.raw);
            expected_samples = (frame_count_64 * 4) + (frame_count_32 * 2);
            fprintf("[INFO] Zakończenie kolekcji: %s\n", reason);
            fprintf("[INFO] Odebrano %d próbek (oczekiwane: %d).\n", data_amount, expected_samples);
            fprintf("[INFO] Ramki: %d total (%dx64B + %dx32B)\n", total_frames, frame_count_64, frame_count_32);
            
            % Sprawdź faktyczne rozmiary ramek w buforze
            actual_64 = 0;
            actual_32 = 0;
            for i = 1:length(output_data.raw)
                if length(output_data.raw{i}) == FRAME_LEN
                    actual_64 = actual_64 + 1;
                elseif length(output_data.raw{i}) == FRAME_LEN_32
                    actual_32 = actual_32 + 1;
                end
            end
            
            if actual_64 ~= frame_count_64 || actual_32 ~= frame_count_32
                fprintf("[WARN] Niespójność! Faktyczne ramki: %dx64B + %dx32B\n", actual_64, actual_32);
            end
            if frame_count_64 < EXPECTED_FRAMES_64 - FRAME_TOLERANCE
                fprintf("[WARN] Za mało ramek 64B! Oczekiwane: %d, otrzymane: %d\n", EXPECTED_FRAMES_64, frame_count_64);
            elseif frame_count_64 > EXPECTED_FRAMES_64 + FRAME_TOLERANCE
                fprintf("[WARN] Za dużo ramek 64B! Oczekiwane: %d, otrzymane: %d\n", EXPECTED_FRAMES_64, frame_count_64);
            end
            
            fprintf("[INFO] Rozpoczynam dekodowanie...\n");
            
            % Automatyczne dekodowanie wszystkich ramek zgodnie z ich rozmiarem
            for i = 1:length(output_data.raw)
                frame_data = output_data.raw{i};
                if length(frame_data) == FRAME_LEN
                    decode_channels(frame_data);
                elseif length(frame_data) == FRAME_LEN_32
                    decode_channels_32(frame_data);
                end
            end
            fprintf("[INFO] Dekodowanie zakończone.\n");
            
            % Automatyczne wysłanie komendy SETUP
            if auto_send_enabled && ~isempty(repetitive_line) && repetitive_line ~= "" && ~isempty(serial_port) && isvalid(serial_port)
                try
                    write(serial_port, repetitive_line, "string");
                    fprintf("[TX repetitive auto] '%s'\n", repetitive_line);
                    last_frame_time = 0;
                catch ME
                    fprintf("[ERR] Błąd automatycznego wysłania komendy: %s\n", ME.message);
                end
            elseif ~auto_send_enabled
                fprintf("[INFO] Automatyczne wysyłanie zatrzymane - komenda SETUP nie została wysłana.\n");
            end
            
            % Reset liczników
            data_collected = false;
            data_amount = 0;
            frame_count_64 = 0;
            frame_count_32 = 0;
            if last_frame_time == 0
                last_frame_time = 0;
            end
            fprintf("[INFO] Kolekcja zakończona. Dane pozostają w buforze (użyj /data aby je odczytać, /clear aby wyczyścić).\n");
        end
    end
    
    function process_frame(frame, frame_size)
        % Przetwarzanie ramki danych (64B lub 32B)
        
        frame_len_str = sprintf("%dB", frame_size);
        hex_str = "";
        for i = 1:length(frame)
            hex_str = hex_str + sprintf("%02X ", frame(i));
        end
        fprintf("[RX %s] %s count: %d\n", frame_len_str, hex_str, data_amount);
        collect_data(frame, frame_size);
    end
    
    function result = repetitive_line_generator(frequency, device_count)
        result = sprintf("AT+ADC:SETUP 0, %d, %d, 0\r\n", frequency, device_count);
    end
    
    function filename = export_to_csv(custom_filename)
        % Eksportuje dane z output_data do pliku CSV
        
        if nargin < 1 || isempty(custom_filename)
            timestamp = datestr(now, 'yyyymmdd_HHMMSS');
            filename = sprintf("export_%s.csv", timestamp);
        else
            filename = custom_filename;
        end
        
        % Sprawdź czy są jakieś dane do eksportu
        max_samples = 0;
        for i = 0:7
            ch_name = sprintf("ch%d", i);
            max_samples = max(max_samples, length(output_data.(ch_name)));
        end
        
        if max_samples == 0
            error("Brak danych do eksportu");
        end
        
        try
            fid = fopen(filename, 'w', 'n', 'UTF-8');
            if fid == -1
                error("Nie można otworzyć pliku do zapisu");
            end
            
            % Nagłówek
            fprintf(fid, "ch0,ch1,ch2,ch3,ch4,ch5,ch6,ch7\n");
            
            % Dane
            for i = 1:max_samples
                row = [];
                for ch = 0:7
                    ch_name = sprintf("ch%d", ch);
                    ch_data = output_data.(ch_name);
                    if i <= length(ch_data)
                        row = [row, ch_data(i)];
                    else
                        row = [row, NaN];
                    end
                end
                % Formatuj wiersz z obsługą NaN
                row_str = "";
                for j = 1:length(row)
                    if isnan(row(j))
                        row_str = row_str + ",";
                    else
                        if j == 1
                            row_str = sprintf("%d", row(j));
                        else
                            row_str = row_str + sprintf(",%d", row(j));
                        end
                    end
                end
                fprintf(fid, "%s\n", row_str);
            end
            
            fclose(fid);
        catch ME
            if fid ~= -1
                fclose(fid);
            end
            error("Błąd eksportu do CSV: %s", ME.message);
        end
    end
    
    function filename = export_to_dat(custom_filename)
        % Eksportuje dane z output_data do pliku .dat (format MATLAB)
        
        if nargin < 1 || isempty(custom_filename)
            timestamp = datestr(now, 'yyyymmdd_HHMMSS');
            filename = sprintf("export_%s.dat", timestamp);
        else
            filename = custom_filename;
        end
        
        % Sprawdź czy są jakieś dane do eksportu
        max_samples = 0;
        for k = 0:7
            ch_name = sprintf('ch%d', k);
            max_samples = max(max_samples, numel(output_data.(ch_name)));
        end
        
        A = NaN(max_samples, 8);
        for k = 0:7
            ch_name = sprintf('ch%d', k);
            x = output_data.(ch_name);
            A(1:numel(x), k+1) = x(:);
        end
        
        if max_samples == 0
            error("Brak danych do eksportu");
        end
        
        try   
            save(filename, 'A', '-ascii');   % tekstowo
            % albo
            fid = fopen(filename, 'w');
            fwrite(fid, A, 'double');
            fclose(fid);
        catch ME
            if fid ~= -1
                fclose(fid);
            end
            error("Błąd eksportu do .dat: %s", ME.message);
        end
    end
    
    % --- Manager połączenia ---
    
    function connect_serial(session)
        % Otwórz port i wystartuj timer readera
        
        while true
            try
                fprintf("[INFO] Próba połączenia z %s @ %d...\n", session.port, session.baud);
                serial_port = serialport(session.port, session.baud);
                configureTerminator(serial_port, "CR/LF");
                configureCallback(serial_port, "off");
                fprintf("[INFO] Połączono z %s @ %d\n", session.port, session.baud);
                
                % Uruchom timer do czytania danych
                reader_timer = timer('ExecutionMode', 'fixedRate', ...
                                     'Period', 0.01, ...
                                     'TimerFcn', @(~,~) reader_timer_callback(), ...
                                     'StartDelay', 0.1);
                start(reader_timer);
                return;
            catch ME
                fprintf("[ERR] Nie udało się otworzyć %s: %s\n", session.port, ME.message);
                fprintf("[INFO] Ponowna próba za 2s...\n");
                pause(2.0);
            end
        end
    end
    
    function close_serial()
        % Zatrzymaj timer i zamknij port
        
        if ~isempty(reader_timer) && isvalid(reader_timer)
            stop(reader_timer);
            delete(reader_timer);
            reader_timer = [];
        end
        
        if ~isempty(serial_port) && isvalid(serial_port)
            try
                clear serial_port;
            catch
            end
            serial_port = [];
        end
    end
    
    function reconnect_serial(session)
        % Ręczne odnowienie połączenia
        fprintf("[INFO] Ręczne odnowienie połączenia...\n");
        close_serial();
        session.read_buffer = uint8([]);
        connect_serial(session);
    end
    
    % --- Reader thread (jako timer callback) ---
    
    function reader_timer_callback()
        % Callback timera do czytania danych z portu szeregowego
        FRAME_LEN = 64;
        FRAME_LEN_32 = 32;
        
        OK_RESPONSE = uint8(['O', 'K', char(13), char(10)]);  % "OK\r\n"
        
        if isempty(serial_port) || ~isvalid(serial_port)
            return;
        end
        
        try
            % Sprawdź dostępne dane
            if serial_port.NumBytesAvailable > 0
                % Czytaj wszystkie dostępne dane
                data = read(serial_port, serial_port.NumBytesAvailable, "uint8");
                
                if ~isempty(data)
                    % Dodaj dane do bufora
                    session.read_buffer = [session.read_buffer, data];
                    
                    % Usuń odpowiedzi "OK\r\n" z bufora
                    while true
                        % Szukaj sekwencji [79 75 13 10] = "OK\r\n"
                        idx = [];
                        for j = 1:length(session.read_buffer) - 3
                            if isequal(session.read_buffer(j:j+3), OK_RESPONSE)
                                idx = j;
                                break;
                            end
                        end
                        if isempty(idx)
                            break;
                        end
                        fprintf("[RX] Odebrano: OK idx: %d\n", idx);
                        session.read_buffer = session.read_buffer(idx + length(OK_RESPONSE):end);
                    end
                    
                    % Przetwarzaj pełne ramki tylko jeśli zbieranie nie jest zatrzymane
                    if ~collection_stopped && bytes_data_expected
                        % Przetwarzaj pełne ramki - preferuj 64B
                        while length(session.read_buffer) >= FRAME_LEN
                            frame = session.read_buffer(1:FRAME_LEN);
                            session.read_buffer = session.read_buffer(FRAME_LEN+1:end);
                            process_frame(frame, FRAME_LEN);
                        end
                        
                        % Przetwarzaj ramki 32B tylko jeśli bufor ma dokładnie 32B
                        if length(session.read_buffer) == FRAME_LEN_32
                            frame = session.read_buffer(1:FRAME_LEN_32);
                            session.read_buffer = session.read_buffer(FRAME_LEN_32+1:end);
                            process_frame(frame, FRAME_LEN_32);
                        end
                    else
                        fprintf("[RX] Odebrano: %s\n", char(session.read_buffer));
                    end
                end
            else
                % Jeśli nie ma danych, sprawdź timeout
                if ~data_collected && last_frame_time > 0 && ~collection_stopped && bytes_data_expected
                    [should_end, ~] = should_end_collection();
                    if should_end
                        finish_collection();
                    end
                end
            end
        catch ME
            if contains(ME.identifier, 'serialport')
                fprintf("[ERR] Serial read: %s\n", ME.message);
                session.stop_evt = true;
            else
                fprintf("[ERR] Reader: %s\n", ME.message);
            end
        end
    end
    
    % --- REPL Loop ---
    
    function repl_loop(session)
        % Proste REPL na stdin (używa input() w pętli)  
        help_text = sprintf(['Komendy: /exit, /ports, /reconn, /help, /data, /decode, /clear, /process_data, /setup, /timeout, /stop, /resume, /export\n' ...
                             'Wyślij: wpisz tekst i Enter -> wyśle jako ASCII + CRLF\n' ...
                             'Raw:    !<bajty> -> wyśle dokładnie to co wpiszesz (bez CRLF)\n' ...
                             'Zatrzymaj: Enter lub /stop -> zatrzymuje zbieranie danych i automatyczne wysyłanie SETUP (dane pozostają)\n' ...
                             'Wznów: /resume -> wznawia zbieranie danych i automatyczne wysyłanie SETUP\n' ...
                             'Timeout: /timeout/<ms> -> ustawia timeout zakończenia kolekcji (domyślnie 150ms)\n' ...
                             'Export: /export/csv [filename] lub /export/dat [filename] -> eksportuje dane do pliku\n']);
        fprintf("%s", help_text);
        
        while true
            try
                line = input('', 's');
                
                if isempty(line)
                    % Pusta linia (Enter) - zatrzymaj zbieranie
                    collection_stopped = true;
                    auto_send_enabled = false;
                    if ~data_collected && last_frame_time > 0
                        fprintf("[INFO] Zatrzymywanie zbierania danych...\n");
                        finish_collection();
                    else
                        fprintf("[INFO] Zatrzymano zbieranie danych i automatyczne wysyłanie SETUP.\n");
                    end
                    fprintf("[INFO] Dane pozostają w buforze (użyj /data aby je odczytać).\n");
                    fprintf("[INFO] Użyj /resume aby wznowić zbieranie i automatyczne wysyłanie.\n");
                    continue;
                end
                
                if strcmp(line, "/exit")
                    session.stop_evt = true;
                    break;
                end
                
                if strcmp(line, "/ports")
                    ports = list_available_ports();
                    if isempty(ports)
                        fprintf("Dostępne porty: (brak)\n");
                    else
                        fprintf("Dostępne porty: %s\n", strjoin(ports, ", "));
                    end
                    continue;
                end
                
                if strcmp(line, "/reconn")
                    reconnect_serial(session);
                    continue;
                end
                
                if strcmp(line, "/help")
                    fprintf("%s", help_text);
                    continue;
                end
                
                if strcmp(line, "/stop")
                    collection_stopped = true;
                    auto_send_enabled = false;
                    if ~data_collected && last_frame_time > 0
                        fprintf("[INFO] Zatrzymywanie zbierania danych...\n");
                        finish_collection();
                    else
                        fprintf("[INFO] Zatrzymano zbieranie danych i automatyczne wysyłanie SETUP.\n");
                    end
                    fprintf("[INFO] Dane pozostają w buforze (użyj /data aby je odczytać).\n");
                    fprintf("[INFO] Użyj /resume aby wznowić zbieranie i automatyczne wysyłanie.\n");
                    continue;
                end
                
                if strcmp(line, "/resume")
                    collection_stopped = false;
                    auto_send_enabled = true;
                    fprintf("[INFO] Wznowiono zbieranie danych i automatyczne wysyłanie SETUP.\n");
                    continue;
                end
                
                if startsWith(line, "/data")
                    parts = strsplit(line, "/");
                    if length(parts) == 2
                        % samo "/data"
                        disp(output_data);
                    else
                        % kanały od trzeciego elementu
                        for i = 3:length(parts)
                            ch = parts{i};
                            if isfield(output_data, ch)
                                fprintf("%s: %s\n", ch, mat2str(output_data.(ch)));
                            end
                        end
                    end
                    continue;
                end
                
                if strcmp(line, "/decode")
                    for i = 1:length(output_data.raw)
                        frame = output_data.raw{i};
                        if length(frame) == FRAME_LEN
                            decode_channels(frame);
                        elseif length(frame) == FRAME_LEN_32
                            decode_channels_32(frame);
                        end
                    end
                    continue;
                end
                
                if strcmp(line, "/clear")
                    for i = 0:7
                        ch_name = sprintf("ch%d", i);
                        output_data.(ch_name) = [];
                    end
                    output_data.raw = {};
                    fprintf("Dane wyczyszczone\n");
                    continue;
                end
                
                if startsWith(line, "/process_data")
                    parts = strsplit(line, "/");
                    for i = 1:length(output_data.raw)
                        frame = output_data.raw{i};
                        if length(frame) == FRAME_LEN
                            decode_channels(frame);
                        elseif length(frame) == FRAME_LEN_32
                            decode_channels_32(frame);
                        end
                    end
                    if length(parts) == 2
                        disp(output_data);
                    else
                        for i = 3:length(parts)
                            ch = parts{i};
                            if isfield(output_data, ch)
                                fprintf("%s: %s\n", ch, mat2str(output_data.(ch)));
                            end
                        end
                    end
                    for i = 0:7
                        ch_name = sprintf("ch%d", i);
                        output_data.(ch_name) = [];
                    end
                    output_data.raw = {};
                    fprintf("Dane wyczyszczone\n");
                    continue;
                end
                
                if startsWith(line, "/timeout")
                    parts = strsplit(line, "/");
                    if length(parts) == 2
                        try
                            new_timeout = str2double(parts{2});
                            if new_timeout < 50 || new_timeout > 1000
                                fprintf("[ERR] Timeout musi być między 50 a 1000 ms\n");
                            else
                                timeout_ms = new_timeout;
                                extended_timeout_ms = new_timeout + 100;
                                fprintf("[INFO] Ustawiono timeout: %dms (przedłużony: %dms)\n", timeout_ms, extended_timeout_ms);
                            end
                        catch
                            fprintf("[ERR] Nieprawidłowa wartość timeoutu. Użyj /timeout/<ms>\n");
                        end
                    else
                        fprintf("[INFO] Aktualny timeout: %dms (przedłużony: %dms)\n", timeout_ms, extended_timeout_ms);
                    end
                    continue;
                end
                
                if startsWith(line, "/export")
                    parts = strsplit(line, "/");
                    if length(parts) < 3
                        fprintf("[ERR] Nieprawidłowa komenda. Użyj /export/csv [filename] lub /export/dat [filename]\n");
                        continue;
                    end
                    
                    extension = lower(parts{3});
                    if length(parts) > 3
                        custom_filename = parts{4};
                    else
                        custom_filename = [];
                    end
                    
                    try
                        if strcmp(extension, "csv")
                            exported_file = export_to_csv(custom_filename);
                            fprintf("[INFO] Dane wyeksportowane do pliku CSV: %s\n", exported_file);
                        elseif strcmp(extension, "dat")
                            exported_file = export_to_dat(custom_filename);
                            fprintf("[INFO] Dane wyeksportowane do pliku .dat: %s\n", exported_file);
                        else
                            fprintf("[ERR] Nieobsługiwane rozszerzenie: %s. Użyj 'csv' lub 'dat'\n", extension);
                        end
                    catch ME
                        fprintf("[ERR] %s\n", ME.message);
                    end
                    continue;
                end
                
                if startsWith(line, "/setup")
                    parts = strsplit(line, "/");
                    if length(parts) == 4
                        repetitive_line = repetitive_line_generator(str2double(parts{3}), str2double(parts{4}));
                        data_collected = false;
                        data_amount = 0;
                        frame_count_64 = 0;
                        frame_count_32 = 0;
                        last_frame_time = 0;
                        auto_send_enabled = true;
                        collection_stopped = false;
                        fprintf("[INFO] Ustawiono linię repetitive: '%s'\n", repetitive_line);
                        
                        % Wysyłaj komendę SETUP natychmiast po ustawieniu
                        try
                            if ~isempty(serial_port) && isvalid(serial_port)
                                write(serial_port, repetitive_line, "string");
                                fprintf("[TX setup] '%s'\n", repetitive_line);
                            else
                                fprintf("[WARN] Port nie jest otwarty. Komenda nie została wysłana.\n");
                            end
                        catch ME
                            fprintf("[ERR] Błąd wysyłania komendy SETUP: %s\n", ME.message);
                        end
                        continue;
                    else
                        fprintf("[ERR] Nieprawidłowa komenda. Użyj /setup/<frequency>/<device_count>\n");
                        continue;
                    end
                end
                
                % zwykłe wysyłanie danych do urządzenia
                try
                    if isempty(serial_port) || ~isvalid(serial_port)
                        fprintf("[WARN] Port nie jest otwarty. Użyj /reconn aby spróbować ponownie.\n");
                        continue;
                    end
                    
                    if startsWith(line, "!")
                        payload = line(2:end);
                        write(serial_port, payload, "string");
                        fprintf("[TX raw] '%s'\n", payload);
                    else
                        % Sprawdź czy line zawiera którykolwiek string z tablicy
                        found = false;
                        for i = 1:length(STRING_RESPONSE_CMD)
                            if contains(line, STRING_RESPONSE_CMD{i})
                                config_message_expected = true;
                                fprintf("[INFO] Config message expected\n");
                                found = true;
                                break;
                            end
                        end
                        
                        for i = 1:length(BYTE_DATA_CMD)
                            if contains(line, BYTE_DATA_CMD{i})
                                bytes_data_expected = true;
                                fprintf("[INFO] Bytes data expected\n");
                                break;
                            end
                        end
                        
                        for i = 1:length(STRING_DATA_CMD)
                            if contains(line, STRING_DATA_CMD{i})
                                bytes_data_expected = false;
                                fprintf("[INFO] String data expected\n");
                                break;
                            end
                        end
                        
                        payload = line + "\r\n";
                        write(serial_port, payload, "string");
                        fprintf("[TX] '%s' + CRLF\n", line);
                    end
                catch ME
                    if contains(ME.identifier, 'serialport')
                        fprintf("[ERR] Serial write: %s\n", ME.message);
                        fprintf("[INFO] Połączenie prawdopodobnie zerwane. Użyj /reconn, aby spróbować ponownie.\n");
                    else
                        fprintf("[ERR] Writer: %s\n", ME.message);
                    end
                end
            catch ME
                if strcmp(ME.identifier, 'MATLAB:UndefinedFunction') && contains(ME.message, 'input')
                    % Użytkownik przerwał (Ctrl+C lub zamknięcie)
                    fprintf("\n[INFO] Przerwano przez użytkownika.\n");
                    break;
                else
                    fprintf("[ERR] Błąd w REPL: %s\n", ME.message);
                end
            end
        end
    end
    
    % --- Main ---
    
    % Główna funkcja programu
    try
        connect_serial(session);
        repl_loop(session);
    catch ME
        if strcmp(ME.identifier, 'MATLAB:handle:InvalidHandle')
            fprintf("\n[INFO] Przerwano przez użytkownika.\n");
        else
            fprintf("[ERR] Błąd w main: %s\n", ME.message);
        end
    finally
        close_serial();
    end
end

