import sys
import threading
import serial
from serial.tools import list_ports
import time
from typing import Optional
import struct

PORT = "COM50"      # zmień wg potrzeb
BAUD = 115200
FRAME_LEN = 64      # długość oczekiwanej ramki binarnej (główna)
FRAME_LEN_32 = 32   # długość alternatywnej ramki binarnej
READ_TIMEOUT = 0.2  # sekundy: krótki timeout, żeby pętla była responsywna

INIT_CMDS = [
    b"AT+ADC:SETUP 10000,2000,2,1\r\n",
    # komenda zgłaszająca zebranie 10k próbek z czest 2000, dla 2 urządzeń (tylko obserwacja pinu BUSY), w trybie continuous (1)
    # - czyli próbki będą zbierane cały czas, aż do wysłania komendy AT+ADC:STOP
    b"AT+ADC:SETUP 100,20000,7,0\r\n",
    # komenda zgłaszająca zebranie 100 próbek z czest 20000, dla 7 urządzeń (tylko obserwacja pinu BUSY), w trybie single (0)
    # - czyli po każdym zebraniu próbek należy ponownie uruchomić/skonfigurować odczyt
    b"AT+ADC:FORMAT C\r\n",
    # komenda określająca porządany format danych:
    # (C)lear - w postaci wartości napięć na każdym z kanałów,
    # (R)aw - w postaci bitowej ch0_lB, ch0_HB, ch1_LB, ...
    b"AT+ADC:START\r\n",
    # komenda uruchamiająca odczyt (działa tylko gdy odczyt jest wstrzymany, inaczej zgłasza wiadomość BUSY)
    b"AT+ADC:STOP\r\n",
    # komenda zatrzymująca odczyt w trybie continuous
    b"AT+CLK:SOURCE E"
    # komenda ustalająca źródło sygnału taktującego próbkowanie: (E)xternal - 10MHz z generatora zewnętrznego podłączonego do pinu PD2 (4CN7 na NUCLEO),
    # (I)nternal - 64MHz z wewnętrznego układu RC, ewetualnie można po prostu wpisać częstotliwość
]

BYTE_DATA_CMD = [
    "AT+ADC:FORMAT R"
]

STRING_DATA_CMD = [
     "AT+ADC:FORMAT C"
]

STRING_RESPONSE_CMD = [
    "AT+CLK:SOURCE",
    "AT+CLK:CONFIG",
    "AT+CLK:SPEED",
    "AT+HELP"
]

output_data = {
    "ch0": [],
    "ch1": [],
    "ch2": [],
    "ch3": [],
    "ch4": [],
    "ch5": [],
    "ch6": [],
    "ch7": [],
    "raw": []
}

repetitive_line = ""
data_collected = False
data_amount = 0
frame_count_64 = 0  # licznik ramek 64B
frame_count_32 = 0  # licznik ramek 32B
last_frame_time = 0.0  # czas ostatniej odebranej ramki (timestamp)
timeout_ms = 150  # timeout w milisekundach (domyślnie 150ms)
extended_timeout_ms = 250  # przedłużony timeout gdy za mało ramek (150 + 100)
EXPECTED_FRAMES_64 = 4061  # oczekiwana liczba ramek 64B
EXPECTED_FRAMES_32 = 1     # oczekiwana liczba ramek 32B
FRAME_TOLERANCE = 2        # tolerancja liczby ramek (±2)
auto_send_enabled = True    # flaga kontrolująca automatyczne wysyłanie SETUP
collection_stopped = False   # flaga zatrzymująca zbieranie danych (po wciśnięciu Enter)
bytes_data_expected = True
config_message_expected = False
message_timeout = 0.0

# --- pomocnicze --------------------------------------------------------------


def list_available_ports():
    return [p.device for p in list_ports.comports()]


def should_end_collection() -> tuple[bool, str]:
    """
    Sprawdza czy powinna zakończyć się kolekcja danych.
    Zwraca (should_end, reason) gdzie reason to powód zakończenia.
    """
    global last_frame_time, timeout_ms, extended_timeout_ms, frame_count_64, frame_count_32
    global EXPECTED_FRAMES_64, EXPECTED_FRAMES_32, FRAME_TOLERANCE

    if repetitive_line == "":
        return (False, "")
    
    # Jeśli nie rozpoczęliśmy jeszcze zbierania (last_frame_time == 0), nie kończ
    if last_frame_time == 0:
        return (False, "")
    
    current_time = time.time() * 1000  # czas w milisekundach
    time_since_last_frame = current_time - last_frame_time
    
    # Sprawdź czy mamy dokładnie oczekiwaną liczbę ramek
    if frame_count_64 == EXPECTED_FRAMES_64 and frame_count_32 == EXPECTED_FRAMES_32:
        return (True, "dokładna liczba ramek")
    
    # Sprawdź timeout - podstawowy
    if time_since_last_frame >= timeout_ms:
        # Sprawdź czy mamy rozsądną liczbę ramek (w tolerancji)
        min_frames_64 = EXPECTED_FRAMES_64 - FRAME_TOLERANCE
        max_frames_64 = EXPECTED_FRAMES_64 + FRAME_TOLERANCE
        
        if frame_count_64 >= min_frames_64 and frame_count_64 <= max_frames_64:
            return (True, f"timeout ({timeout_ms}ms) + rozsądna liczba ramek")
        
        # Jeśli za mało ramek, sprawdź przedłużony timeout
        if time_since_last_frame >= extended_timeout_ms:
            if frame_count_64 < min_frames_64:
                return (True, f"przedłużony timeout ({extended_timeout_ms}ms) - za mało ramek ({frame_count_64})")
            else:
                return (True, f"przedłużony timeout ({extended_timeout_ms}ms) - za dużo ramek ({frame_count_64})")
    
    return (False, "")


def finish_collection(session: Optional["SerialSession"] = None):
    """
    Kończy kolekcję danych - wywołuje dekodowanie i reset.
    Używane gdy timeout minął i nie ma nowych ramek.
    """
    global data_collected, repetitive_line, frame_count_64, frame_count_32, data_amount
    global last_frame_time
    
    if data_collected or len(output_data["raw"]) == 0:
        return  # Już zakończone lub brak danych
    
    data_collected = True
    total_frames = len(output_data["raw"])
    expected_samples = (frame_count_64 * 4) + (frame_count_32 * 2)
    should_end, reason = should_end_collection()
    print(f"[INFO] Zakończenie kolekcji (timeout): {reason}")
    print(f"[INFO] Odebrano {data_amount} próbek (oczekiwane: {expected_samples}).")
    print(f"[INFO] Ramki: {total_frames} total ({frame_count_64}x64B + {frame_count_32}x32B)")
    
    # Sprawdź faktyczne rozmiary ramek w buforze
    actual_64 = sum(1 for f in output_data["raw"] if len(f) == FRAME_LEN)
    actual_32 = sum(1 for f in output_data["raw"] if len(f) == FRAME_LEN_32)
    if actual_64 != frame_count_64 or actual_32 != frame_count_32:
        print(f"[WARN] Niespójność! Faktyczne ramki: {actual_64}x64B + {actual_32}x32B")
    
    # Ostrzeżenie jeśli liczba ramek jest poza tolerancją
    if frame_count_64 < EXPECTED_FRAMES_64 - FRAME_TOLERANCE:
        print(f"[WARN] Za mało ramek 64B! Oczekiwane: {EXPECTED_FRAMES_64}, otrzymane: {frame_count_64}")
    elif frame_count_64 > EXPECTED_FRAMES_64 + FRAME_TOLERANCE:
        print(f"[WARN] Za dużo ramek 64B! Oczekiwane: {EXPECTED_FRAMES_64}, otrzymane: {frame_count_64}")
    
    print(f"[INFO] Rozpoczynam dekodowanie...")
    
    # Automatyczne dekodowanie wszystkich ramek zgodnie z ich rozmiarem
    for frame_data in output_data["raw"]:
        if len(frame_data) == FRAME_LEN:
            decode_channels(frame_data)
        elif len(frame_data) == FRAME_LEN_32:
            decode_channels_32(frame_data)
    print(f"[INFO] Dekodowanie zakończone.")
    
    # Automatyczne wysłanie komendy SETUP jeśli jest ustawiona i auto_send_enabled
    global auto_send_enabled
    if auto_send_enabled and repetitive_line != "" and session is not None and session.ser is not None and session.ser.is_open:
        try:
            safe_write(session.ser, repetitive_line.encode("ascii", errors="ignore"), session.write_lock)
            print(f"[TX repetitive auto] {repetitive_line!r}")
            # Reset czasu - nowa seria zaczyna się po wysłaniu komendy
            last_frame_time = 0
        except Exception as e:
            print(f"[ERR] Błąd automatycznego wysłania komendy: {e}")
    elif not auto_send_enabled:
        print(f"[INFO] Automatyczne wysyłanie zatrzymane - komenda SETUP nie została wysłana.")
    
    # Reset liczników (ale NIE czyść danych - pozostają do odczytu)
    data_collected = False
    data_amount = 0
    frame_count_64 = 0
    frame_count_32 = 0
    if last_frame_time == 0:  # Jeśli nie wysłaliśmy jeszcze nowej komendy, resetuj czas
        last_frame_time = 0
    # NIE czyść output_data - dane pozostają do odczytu przez użytkownika
    print(f"[INFO] Kolekcja zakończona. Dane pozostają w buforze (użyj /data aby je odczytać, /clear aby wyczyścić).")


def safe_write(ser: serial.Serial, data: bytes, lock: threading.Lock):
    with lock:
        ser.write(data)


def decode_channels(frame: bytes):
    """
    Decode the channels from the frame 64B.
    """
    for i in range(0, 16, 2):
        ch = f"ch{int(i / 2)}"
        # 4 próbki na kanał w ramce 64B: z przesunięciem o 0, 16, 32, 48
        output_data[ch].append(struct.unpack("<H", frame[i:i + 2])[0])
        output_data[ch].append(struct.unpack("<H", frame[i + 16:i + 18])[0])
        output_data[ch].append(struct.unpack("<H", frame[i + 32:i + 34])[0])
        output_data[ch].append(struct.unpack("<H", frame[i + 48:i + 50])[0])


def decode_channels_32(frame: bytes):
    """
    Decode the channels from the frame 32B.
    """
    for i in range(0, 16, 2):
        ch = f"ch{int(i / 2)}"
        # 2 próbki na kanał w ramce 32B: z przesunięciem o 0, 16
        output_data[ch].append(struct.unpack("<H", frame[i:i + 2])[0])
        output_data[ch].append(struct.unpack("<H", frame[i + 16:i + 18])[0])


def collect_data(frame: bytes, session: Optional["SerialSession"] = None, frame_size: int = FRAME_LEN):
    """
    Collect the data from the frame.
    Automatycznie kończy kolekcję gdy:
    - Osiągnięto dokładną liczbę ramek (4061x64B + 1x32B), LUB
    - Timeout minął i mamy rozsądną liczbę ramek (w tolerancji)
    frame_size: rozmiar ramki (64 lub 32) - określa ile próbek dodać do licznika
    """
    global data_amount, data_collected, repetitive_line, frame_count_64, frame_count_32, last_frame_time, collection_stopped
    
    # Jeśli zbieranie jest zatrzymane, nie kontynuuj
    if collection_stopped:
        return
    
    output_data["raw"].append(frame)
    
    # Aktualizuj czas ostatniej ramki
    last_frame_time = time.time() * 1000  # czas w milisekundach
    
    # Licz ramki według rozmiaru
    if frame_size == FRAME_LEN:
        frame_count_64 += 1
    elif frame_size == FRAME_LEN_32:
        frame_count_32 += 1
    
    # Ramka 64B = 4 próbki, ramka 32B = 2 próbki
    samples_per_frame = 4 if frame_size == FRAME_LEN else 2
    data_amount += samples_per_frame
    
    # Sprawdź czy powinna zakończyć się kolekcja (timeout lub dokładna liczba ramek)
    should_end, reason = should_end_collection()
    
    if should_end:
        data_collected = True
        total_frames = len(output_data["raw"])
        expected_samples = (frame_count_64 * 4) + (frame_count_32 * 2)
        print(f"[INFO] Zakończenie kolekcji: {reason}")
        print(f"[INFO] Odebrano {data_amount} próbek (oczekiwane: {expected_samples}).")
        print(f"[INFO] Ramki: {total_frames} total ({frame_count_64}x64B + {frame_count_32}x32B)")
        
        # Sprawdź faktyczne rozmiary ramek w buforze
        actual_64 = sum(1 for f in output_data["raw"] if len(f) == FRAME_LEN)
        actual_32 = sum(1 for f in output_data["raw"] if len(f) == FRAME_LEN_32)
        if actual_64 != frame_count_64 or actual_32 != frame_count_32:
            print(f"[WARN] Niespójność! Faktyczne ramki: {actual_64}x64B + {actual_32}x32B")
        
        # Ostrzeżenie jeśli liczba ramek jest poza tolerancją
        if frame_count_64 < EXPECTED_FRAMES_64 - FRAME_TOLERANCE:
            print(f"[WARN] Za mało ramek 64B! Oczekiwane: {EXPECTED_FRAMES_64}, otrzymane: {frame_count_64}")
        elif frame_count_64 > EXPECTED_FRAMES_64 + FRAME_TOLERANCE:
            print(f"[WARN] Za dużo ramek 64B! Oczekiwane: {EXPECTED_FRAMES_64}, otrzymane: {frame_count_64}")
        
        print(f"[INFO] Rozpoczynam dekodowanie...")
        
        # Automatyczne dekodowanie wszystkich ramek zgodnie z ich rozmiarem
        for frame_data in output_data["raw"]:
            if len(frame_data) == FRAME_LEN:
                decode_channels(frame_data)
            elif len(frame_data) == FRAME_LEN_32:
                decode_channels_32(frame_data)
        print(f"[INFO] Dekodowanie zakończone.")
        
        # Automatyczne wysłanie komendy SETUP jeśli jest ustawiona i auto_send_enabled
        global auto_send_enabled
        if auto_send_enabled and repetitive_line != "" and session is not None and session.ser is not None and session.ser.is_open:
            try:
                safe_write(session.ser, repetitive_line.encode("ascii", errors="ignore"), session.write_lock)
                print(f"[TX repetitive auto] {repetitive_line!r}")
                # Reset czasu - nowa seria zaczyna się po wysłaniu komendy
                last_frame_time = 0
            except Exception as e:
                print(f"[ERR] Błąd automatycznego wysłania komendy: {e}")
        elif not auto_send_enabled:
            print(f"[INFO] Automatyczne wysyłanie zatrzymane - komenda SETUP nie została wysłana.")
        
        # Reset liczników (ale NIE czyść danych - pozostają do odczytu)
        data_collected = False
        data_amount = 0
        frame_count_64 = 0
        frame_count_32 = 0
        if last_frame_time == 0:  # Jeśli nie wysłaliśmy jeszcze nowej komendy, resetuj czas
            last_frame_time = 0
        # NIE czyść output_data - dane pozostają do odczytu przez użytkownika
        print(f"[INFO] Kolekcja zakończona. Dane pozostają w buforze (użyj /data aby je odczytać, /clear aby wyczyścić).")


def process_frame(frame: bytes, session: Optional["SerialSession"] = None, frame_size: int = FRAME_LEN):
    """
    Przetwarzanie ramki danych (64B lub 32B).
    Aktualnie: wypisuje heks i surowe bajty.
    """
    frame_len_str = f"{frame_size}B"
    # ts = time.strftime("%H:%M:%S")
    # print(f"[{ts}] RX {frame_len_str}: {frame.hex(' ')}")
    print(f"[RX {frame_len_str}] {frame.hex(' ')}", "count: ", data_amount)
    # print("ASCII: ", frame.decode("ascii", errors="ignore"), "count: ", data_amount)
    collect_data(frame, session, frame_size)
    # Automatyczne dekodowanie pojedynczej ramki (opcjonalne, główne dekodowanie jest w collect_data)
    # if frame_size == FRAME_LEN:
    #     decode_channels(frame)
    # elif frame_size == FRAME_LEN_32:
    #     decode_channels_32(frame)

def repetitive_line_generator(frequency: int, device_count: int):
    return f"AT+ADC:SETUP 0, {frequency}, {device_count}, 0\r\n"


def export_to_csv(filename: Optional[str] = None) -> str:
    """
    Eksportuje dane z output_data do pliku CSV.
    Format: kolumny dla każdego kanału (ch0-ch7), każdy wiersz to jedna próbka.
    """
    import csv
    from datetime import datetime
    
    if filename is None:
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = f"export_{timestamp}.csv"
    
    # Sprawdź czy są jakieś dane do eksportu
    max_samples = max(len(output_data[f"ch{i}"]) for i in range(8))
    if max_samples == 0:
        raise ValueError("Brak danych do eksportu")
    
    try:
        with open(filename, 'w', newline='', encoding='utf-8') as csvfile:
            writer = csv.writer(csvfile)
            
            # Nagłówek
            header = [f"ch{i}" for i in range(8)]
            writer.writerow(header)
            
            # Dane - każdy wiersz to jedna próbka dla wszystkich kanałów
            for i in range(max_samples):
                row = []
                for ch in range(8):
                    ch_data = output_data[f"ch{ch}"]
                    if i < len(ch_data):
                        row.append(ch_data[i])
                    else:
                        row.append("")  # Pusta wartość jeśli kanał ma mniej próbek
                writer.writerow(row)
        
        return filename
    except Exception as e:
        raise Exception(f"Błąd eksportu do CSV: {e}")


def export_to_dat(filename: Optional[str] = None) -> str:
    """
    Eksportuje dane z output_data do pliku .dat (format MATLAB).
    Format: kolumny dla każdego kanału (ch0-ch7), każdy wiersz to jedna próbka.
    Separator: tabulator lub spacja.
    """
    from datetime import datetime
    
    if filename is None:
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = f"export_{timestamp}.dat"
    
    # Sprawdź czy są jakieś dane do eksportu
    max_samples = max(len(output_data[f"ch{i}"]) for i in range(8))
    if max_samples == 0:
        raise ValueError("Brak danych do eksportu")
    
    try:
        with open(filename, 'w', encoding='utf-8') as datfile:
            # Nagłówek (komentarz dla MATLAB)
            datfile.write(f"% Exported data from serial session\n")
            datfile.write(f"% Columns: ch0 ch1 ch2 ch3 ch4 ch5 ch6 ch7\n")
            datfile.write(f"% Total samples: {max_samples}\n")
            datfile.write(f"% Export time: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n")
            
            # Dane - każdy wiersz to jedna próbka dla wszystkich kanałów
            # Format: wartości oddzielone spacjami (MATLAB może czytać)
            for i in range(max_samples):
                row_values = []
                for ch in range(8):
                    ch_data = output_data[f"ch{ch}"]
                    if i < len(ch_data):
                        row_values.append(str(ch_data[i]))
                    else:
                        row_values.append("NaN")  # NaN dla MATLAB jeśli brak danych
                datfile.write(" ".join(row_values) + "\n")
        
        return filename
    except Exception as e:
        raise Exception(f"Błąd eksportu do .dat: {e}")

# --- manager połączenia ------------------------------------------------------


class SerialSession:
    def __init__(self, port: str, baud: int, read_timeout: float = READ_TIMEOUT):
        self.port = port
        self.baud = baud
        self.read_timeout = read_timeout

        self.ser: Optional[serial.Serial] = None
        self.write_lock = threading.Lock()
        self.stop_evt = threading.Event()
        self.reader_thread: Optional[threading.Thread] = None
        self.read_buffer = bytearray()  # bufor do gromadzenia danych przed przetworzeniem

    def _start_reader_thread(self):
        # uruchamia wątek czytający na aktualnym self.ser
        if self.reader_thread and self.reader_thread.is_alive():
            return
        self.stop_evt.clear()
        self.reader_thread = threading.Thread(
            target=reader_thread,
            args=(self,),
            daemon=True,
        )
        self.reader_thread.start()

    def connect(self):
        """Otwórz port i wystartuj wątek readera."""
        while True:
            try:
                print(f"[INFO] Próba połączenia z {self.port} @ {self.baud}...")
                self.ser = serial.Serial(self.port, self.baud, timeout=self.read_timeout)
                print(f"[INFO] Połączono z {self.port} @ {self.baud}")
                # ewentualnie można wysłać INIT_CMDS tutaj:
                # for cmd in INIT_CMDS:
                #     safe_write(self.ser, cmd, self.write_lock)
                #     time.sleep(0.05)
                self._start_reader_thread()
                return
            except serial.SerialException as e:
                print(f"[ERR] Nie udało się otworzyć {self.port}: {e}")
                print("[INFO] Ponowna próba za 2s...")
                time.sleep(2.0)

    def close(self):
        """Zatrzymaj wątek i zamknij port."""
        self.stop_evt.set()
        if self.reader_thread and self.reader_thread.is_alive():
            self.reader_thread.join(timeout=1.0)
        if self.ser is not None:
            try:
                if self.ser.is_open:
                    self.ser.close()
            except Exception as e:
                print(f"[WARN] Błąd zamykania portu: {e}")
        self.ser = None
        self.read_buffer.clear()  # wyczyść bufor przy zamknięciu

    def reconnect(self):
        """
        Ręczne odnowienie połączenia:
        - zatrzymuje readera,
        - zamyka port,
        - czyści bufor,
        - próbuje ponownie się połączyć.
        """
        print("[INFO] Ręczne odnowienie połączenia...")
        self.close()
        self.read_buffer.clear()  # wyczyść bufor przy reconnect
        self.connect()


# --- wątki I/O ---------------------------------------------------------------


def reader_thread(session: SerialSession):
    """
    Czyta dane z portu szeregowego, buforuje je i przetwarza pełne ramki 64B.
    Automatycznie ignoruje odpowiedzi "OK\r\n" od urządzenia.
    """

    global message_timeout, bytes_data_expected, config_message_expected, collection_stopped
    OK_RESPONSE = b"OK\r\n"
    
    while not session.stop_evt.is_set():
        ser = session.ser
        if ser is None or not ser.is_open:
            time.sleep(0.1)
            continue
        try:
            # Czytaj wszystkie dostępne dane (lub przynajmniej 1 bajt)
            available = ser.in_waiting
            if available > 0:
                data = ser.read(available)
                if data:
                    # Dodaj dane do bufora
                    session.read_buffer.extend(data)
                    
                    # Usuń odpowiedzi "OK\r\n" z bufora
                    while OK_RESPONSE in session.read_buffer:
                        idx = session.read_buffer.find(OK_RESPONSE)
                        print("[RX] Odebrano: ", session.read_buffer.decode("ascii", errors="ignore"), "idx: ", idx)
                        session.read_buffer = session.read_buffer[idx + len(OK_RESPONSE):]
                    
                    # Przetwarzaj pełne ramki tylko jeśli zbieranie nie jest zatrzymane
                    if not collection_stopped and bytes_data_expected:
                        # Przetwarzaj pełne ramki - preferuj 64B, ale obsługuj też 32B
                        # WAŻNE: Jeśli bufor ma między 32-63B, czekamy na więcej danych
                        # aby uniknąć błędnej interpretacji niepełnej ramki 64B jako ramki 32B
                        while len(session.read_buffer) >= FRAME_LEN:
                            # Mamy wystarczająco danych na ramkę 64B (preferowana)
                            frame = bytes(session.read_buffer[:FRAME_LEN])
                            session.read_buffer = session.read_buffer[FRAME_LEN:]
                            process_frame(frame, session, FRAME_LEN)
                        
                        # Przetwarzaj ramki 32B tylko jeśli bufor ma dokładnie 32B
                        # (nie więcej, bo wtedy czekamy na pełną ramkę 64B)
                        if len(session.read_buffer) == FRAME_LEN_32:
                            frame = bytes(session.read_buffer[:FRAME_LEN_32])
                            session.read_buffer = session.read_buffer[FRAME_LEN_32:]
                            process_frame(frame, session, FRAME_LEN_32)
                    else:
                        print("[RX] Odebrano: ", session.read_buffer.decode("ascii", errors="ignore"))
                        # session.read_buffer.clear()
            else:
                # Jeśli nie ma danych, sprawdź timeout (może czas zakończyć kolekcję)
                # Ale tylko jeśli zbieranie nie jest zatrzymane
                if not data_collected and last_frame_time > 0 and not collection_stopped and bytes_data_expected:
                    should_end, reason = should_end_collection()
                    if should_end:
                        finish_collection(session)
                # Krótka pauza
                time.sleep(0.01)
        except serial.SerialException as e:
            print(f"[ERR] Serial read: {e}")
            # kończymy wątek; użytkownik może wywołać /reconn
            session.stop_evt.set()
            break
        except Exception as e:
            print(f"[ERR] Reader: {e}")
            # nie przerywaj od razu, chyba że to krytyczne
            time.sleep(0.05)


def repl_loop(session: SerialSession):
    """
    Proste REPL na stdin:
      - linia zaczynająca się od '!' wysyłana jest surowo (bez CRLF)
      - pozostałe linie dostają automatycznie CRLF
      - komenda '/exit' kończy program
      - komenda '/ports' pokazuje dostępne porty
      - komenda '/reconn' odnawia połączenie (zamknięcie + ponowne otwarcie)
    """
    help_text = (
        "Komendy: /exit, /ports, /reconn, /help, /data, /decode, /clear, /process_data, /setup, /timeout, /stop, /resume, /export\n"
        "Wyślij: wpisz tekst i Enter -> wyśle jako ASCII + CRLF\n"
        "Raw:    !<bajty> -> wyśle dokładnie to co wpiszesz (bez CRLF)\n"
        "Acquisition: /setup/<frequency> -> ustawia parametry zbierania danych z zadaną częstotliwością\n"
        "Zatrzymaj: Enter lub /stop -> zatrzymuje zbieranie danych i automatyczne wysyłanie SETUP (dane pozostają)\n"
        "Wznów: /resume -> wznawia zbieranie danych i automatyczne wysyłanie SETUP\n"
        "Timeout: /timeout/<ms> -> ustawia timeout zakończenia kolekcji (domyślnie 150ms)\n"
        "Export: /export/csv [filename] lub /export/dat [filename] -> eksportuje dane do pliku\n"
    )
    print(help_text, flush=True)

    global repetitive_line, data_collected, data_amount, last_frame_time, frame_count_64, frame_count_32, message_timeout
    global timeout_ms, extended_timeout_ms, auto_send_enabled, collection_stopped, bytes_data_expected, config_message_expected  # deklaracja globalnych zmiennych

    for line in sys.stdin:
        if session.stop_evt.is_set():
            # jeśli reader został zatrzymany np. przez błąd portu,
            # nadal możemy obsługiwać komendy (np. /reconn)
            pass

        line = line.rstrip("\n")

        # Pusta linia (Enter) - zatrzymaj zbieranie danych i automatyczne wysyłanie SETUP
        if line == "":
            collection_stopped = True
            auto_send_enabled = False
            # Jeśli trwa zbieranie, zakończ je natychmiast
            if not data_collected and last_frame_time > 0:
                print("[INFO] Zatrzymywanie zbierania danych...")
                finish_collection(session)
            else:
                print("[INFO] Zatrzymano zbieranie danych i automatyczne wysyłanie SETUP.")
            print("[INFO] Dane pozostają w buforze (użyj /data aby je odczytać).")
            print("[INFO] Użyj /resume aby wznowić zbieranie i automatyczne wysyłanie.")
            continue

        if line == "/exit":
            session.stop_evt.set()
            break

        if line == "/ports":
            print("Dostępne porty:", ", ".join(list_available_ports()) or "(brak)")
            continue

        if line == "/reconn":
            # ręczne odnowienie połączenia
            session.reconnect()
            continue

        if line == "/help":
            print(help_text)
            continue

        if line == "/stop":
            collection_stopped = True
            auto_send_enabled = False
            # Jeśli trwa zbieranie, zakończ je natychmiast
            if not data_collected and last_frame_time > 0:
                print("[INFO] Zatrzymywanie zbierania danych...")
                finish_collection(session)
            else:
                print("[INFO] Zatrzymano zbieranie danych i automatyczne wysyłanie SETUP.")
            print("[INFO] Dane pozostają w buforze (użyj /data aby je odczytać).")
            print("[INFO] Użyj /resume aby wznowić zbieranie i automatyczne wysyłanie.")
            continue

        if line == "/resume":
            collection_stopped = False
            auto_send_enabled = True
            print("[INFO] Wznowiono zbieranie danych i automatyczne wysyłanie SETUP.")
            continue

        if line.startswith("/data"):
            # /data albo /data/ch0/ch1/...
            parts = line.split("/")
            if len(parts) == 2:
                # samo "/data"
                print(output_data)
                continue
            # kanały od trzeciego elementu
            for ch in parts[2:]:
                if ch in output_data:
                    print(f"{ch}: {output_data[ch]}")
            continue

        if line == "/decode":
            for frame in output_data["raw"]:
                if len(frame) == FRAME_LEN:
                    decode_channels(frame)
                elif len(frame) == FRAME_LEN_32:
                    decode_channels_32(frame)
            continue

        if line == "/clear":
            for key in output_data:
                output_data[key].clear()
            print("Dane wyczyszczone")
            continue

        if line.startswith("/process_data"):
            # /process_data lub /process_data/ch0/...
            parts = line.split("/")
            for frame in output_data["raw"]:
                if len(frame) == FRAME_LEN:
                    decode_channels(frame)
                elif len(frame) == FRAME_LEN_32:
                    decode_channels_32(frame)
            if len(parts) == 2:
                print(output_data)
            else:
                for ch in parts[2:]:
                    if ch in output_data:
                        print(f"{ch}: {output_data[ch]}")
            for key in output_data:
                output_data[key].clear()
            print("Dane wyczyszczone")
            continue

        if line.startswith("/timeout"):
            params = line.split("/")
            if len(params) == 2:
                try:
                    new_timeout = int(params[1])
                    if new_timeout < 50 or new_timeout > 1000:
                        print(f"[ERR] Timeout musi być między 50 a 1000 ms")
                    else:
                        timeout_ms = new_timeout
                        extended_timeout_ms = new_timeout + 100
                        print(f"[INFO] Ustawiono timeout: {timeout_ms}ms (przedłużony: {extended_timeout_ms}ms)")
                except ValueError:
                    print(f"[ERR] Nieprawidłowa wartość timeoutu. Użyj /timeout/<ms>")
            else:
                print(f"[INFO] Aktualny timeout: {timeout_ms}ms (przedłużony: {extended_timeout_ms}ms)")
            continue

        if line.startswith("/export"):
            params = line.split("/")
            if len(params) < 3:
                print("[ERR] Nieprawidłowa komenda. Użyj /export/csv [filename] lub /export/dat [filename]")
                continue
            
            extension = params[2].lower()
            filename = params[3] if len(params) > 3 else None
            
            try:
                if extension == "csv":
                    exported_file = export_to_csv(filename)
                    print(f"[INFO] Dane wyeksportowane do pliku CSV: {exported_file}")
                elif extension == "dat":
                    exported_file = export_to_dat(filename)
                    print(f"[INFO] Dane wyeksportowane do pliku .dat: {exported_file}")
                else:
                    print(f"[ERR] Nieobsługiwane rozszerzenie: {extension}. Użyj 'csv' lub 'dat'")
            except ValueError as e:
                print(f"[ERR] {e}")
            except Exception as e:
                print(f"[ERR] Błąd eksportu: {e}")
            continue

        if line.startswith("/setup"):
            params = line.split("/")
            if len(params) >= 3:
                if len(params) == 3:
                    repetitive_line = repetitive_line_generator(int(params[2]), 1)
                else:
                    repetitive_line = repetitive_line_generator(int(params[2]), int(params[3]))
                data_collected = False  # reset, bo automatyczne wysyłanie jest w collect_data()
                data_amount = 0
                frame_count_64 = 0
                frame_count_32 = 0
                last_frame_time = 0  # reset czasu - nowa seria zaczyna się
                auto_send_enabled = True  # włącz automatyczne wysyłanie przy ustawieniu nowej komendy
                collection_stopped = False  # wznowij zbieranie przy ustawieniu nowej komendy
                print(f"[INFO] Ustawiono linię repetitive: {repetitive_line!r}")
                
                # Wysyłaj komendę SETUP natychmiast po ustawieniu
                try:
                    ser = session.ser
                    if ser is not None and ser.is_open:
                        safe_write(ser, repetitive_line.encode("ascii", errors="ignore"), session.write_lock)
                        print(f"[TX setup] {repetitive_line!r}")
                    else:
                        print("[WARN] Port nie jest otwarty. Komenda nie została wysłana.")
                except Exception as e:
                    print(f"[ERR] Błąd wysyłania komendy SETUP: {e}")
                continue
            else:
                print(f"[ERR] Nieprawidłowa komenda. Użyj /setup/<frequency>/<device_count>")
                continue

        # zwykłe wysyłanie danych do urządzenia
        try:
            ser = session.ser
            if ser is None or not ser.is_open:
                print("[WARN] Port nie jest otwarty. Użyj /reconn aby spróbować ponownie.")
                continue

            if line.startswith("!"):
                payload = line[1:].encode("utf-8", errors="ignore")
                safe_write(ser, payload, session.write_lock)
                print(f"[TX raw] {payload!r}")
            else:
                # Sprawdź czy line zawiera którykolwiek string z tablicy
                if any(cmd in line for cmd in STRING_RESPONSE_CMD):
                    config_message_expected = True
                    print("[INFO] Config message expected")
                    # Obsługa specjalna dla komend z odpowiedziami
                    pass
                if any(cmd in line for cmd in BYTE_DATA_CMD):
                    bytes_data_expected = True
                    print("[INFO] Bytes data expected")
                    # Obsługa specjalna dla komend z odpowiedziami
                    pass
                if any(cmd in line for cmd in STRING_DATA_CMD):
                    bytes_data_expected = False 
                    print("[INFO] String data expected")
                    # Obsługa specjalna dla komend z odpowiedziami
                    pass
                payload = (line + "\r\n").encode("ascii", errors="ignore")
                safe_write(ser, payload, session.write_lock)
                print(f"[TX] {line!r} + CRLF")
        except serial.SerialException as e:
            print(f"[ERR] Serial write: {e}")
            print("[INFO] Połączenie prawdopodobnie zerwane. Użyj /reconn, aby spróbować ponownie.")
        except Exception as e:
            print(f"[ERR] Writer: {e}")


# --- main --------------------------------------------------------------------


def main(port: str = PORT, baud: int = BAUD):
    session = SerialSession(port, baud, read_timeout=READ_TIMEOUT)
    try:
        session.connect()
        repl_loop(session)
    except KeyboardInterrupt:
        print("\n[INFO] Przerwano przez użytkownika.")
    finally:
        session.close()


if __name__ == "__main__":
    main()
