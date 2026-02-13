/*
 * adc_mgr.c
 *
 *  Created on: Aug 28, 2025
 *      Author: Celelele
 */

#include "adc_mgr.h"

// Stałe pomocnicze
#define GPDMA_BNDT_MAX_BYTES   ((uint32_t)0xFFFFu)   // maska BNDT (16 bitów)
#define GPDMA_BNDT_ALIGN       (4u)                  // pakowanie 4×8 → 32, wyrównanie do 4 B
#define GPDMA_BNDT_MAX_ALIGNED (GPDMA_BNDT_MAX_BYTES & ~(GPDMA_BNDT_ALIGN-1u))

ADC_Handler g_adc;
ADC_Handler * const g_adc_mgr = &g_adc;
__IO uint32_t PSSI_HAL_PSSI_ReceiveComplete_count = 0;
uint32_t user_code_error = 0;
extern uint32_t *node;

static inline int NextSetBit(uint32_t *msk);
static inline uint32_t ADC_FetchReady(ADC_Handler *m);
static inline void ADC_ReportError(uint8_t error_code, void *user);
static inline uint16_t ADC_Calculate_Output(uint32_t sample);
static inline uint16_t bitrev16(uint16_t x);
static inline void deint_8x16_swar_msb_first(const uint8_t *b, uint16_t ch[8]);
static inline void RCCandGPIO_Config_Regs(void);
static inline void PSSI_Config_Regs(void);
//static inline void GPDMA1_CH7_Config_PSSI_P2M_LLI(ADC_Handler *m, bool update);
static inline void GPDMA1_CH7_Config_PSSI_P2M_LLI(ADC_Handler *m, bool update, uint32_t requested_samples);
static inline uint32_t GPDMA1_CH7_Build_PSSI_LL(ADC_Handler *m, uint32_t requested_samples);
static inline uint32_t compute_LA_offset(const DMA_Node *base, const DMA_Node *next);
static inline uint32_t make_CTR2_req_pssi(void);
static inline uint32_t make_CTR1_p2m_pack_4x8_to_32(void);


/*
 * ADC_PinToIndex - Convert GPIO pin number to pin index (0-15)
 * @param GPIO_Pin: GPIO pin mask (e.g., GPIO_PIN_0, GPIO_PIN_1, etc.)
 * @return Pin index (0-15) corresponding to the pin number
 * 
 * Uses compiler intrinsics to find the trailing zero count (bit position) of the pin mask.
 * This is used to map GPIO pins to device indices in the ADC handler.
 */
uint8_t ADC_PinToIndex(uint16_t GPIO_Pin){
	#if defined(__GNUC__)
		return (uint8_t)__builtin_ctz((unsigned)GPIO_Pin);
	#else
		return (uint8_t)(__CLZ(__RBIT(GPIO_Pin))); // CMSIS fallback
	#endif
}

/*
 * ADC_MarkReady - Mark ADC devices as ready based on their BUSY pin state
 * @param m: Pointer to ADC handler structure
 * @return none
 * 
 * Checks the BUSY pin state for each configured ADC device. If a device's BUSY pin is low,
 * it sets the corresponding bit in the ready_mask. This mask is used to track which devices
 * have completed their conversion and are ready for data readout.
 */
void ADC_MarkReady(ADC_Handler *m){
	//32-bit save is atomic in Cortex-M
//	__DMB(); //uncomment when static
	for (int id=0; id<m->ndevs; id++)
	{
		// we expect each busy pin to be low
		volatile uint32_t dev_ready = (m->ltc2368_devs[id].busy_pin->port->IDR & m->ltc2368_devs[id].busy_pin->pin) == 0;
		m->ready_mask |= (dev_ready << id);
	}
}
/*
 * NextSetBit - Find and clear the next set bit in a bitmask
 * @param msk: Pointer to bitmask (modified in place)
 * @return Index of the next set bit (0-31), or -1 if no bits are set
 * 
 * Finds the index of the least significant set bit, clears it, and returns the index.
 * Uses compiler intrinsics for efficient bit manipulation. Used for iterating through
 * device ready masks.
 */
static inline int NextSetBit(uint32_t *msk) {
    if (*msk == 0) return -1;
    uint32_t m = *msk;
    // indeks najmłodszego ustawionego bitu (0..31)
	#if defined(__GNUC__)
		int idx = __builtin_ctz(m);
	#else
		int idx = (int)__CLZ(__RBIT(m));     // CMSIS: reverse bits + count leading zeros
	#endif
		*msk &= (m - 1); // trik: kasuje najmłodszy ustawiony bit w O(1)
    return idx;
}

/*
 * ADC_FetchReady - Check if all devices are ready for display
 * @param m: Pointer to ADC handler structure
 * @return Non-zero if all devices are ready, zero otherwise
 * 
 * Compares the ready_mask (devices that have completed conversion) with the
 * ready_to_disp_mask (devices that should be ready). Returns true when all
 * expected devices have completed their conversions.
 */
static inline uint32_t ADC_FetchReady(ADC_Handler *m) {
    return m->ready_mask == m->ready_to_disp_mask;
}

/*
 * ADC_Init - Initialize the ADC handler structure with hardware resources
 * @param m: Pointer to ADC handler structure to initialize
 * @param chx_lli: Array of DMA linked list nodes for GPDMA channel
 * @param dma_handler: Pointer to DMA channel handler
 * @param buffer: Pointer to common buffer for ADC data storage
 * @param busy_pins: Array of GPIO assignments for BUSY pins
 * @param func: Write function for displaying data
 * @param user: User data pointer for write function
 * @return true on success, false on failure
 * 
 * Initializes the ADC handler with DMA configuration, buffer pointers, GPIO pins,
 * and display function. Configures RCC and GPIO registers for PSSI interface.
 * This function should be called before ADC_ManagerInit.
 */
bool ADC_Init(ADC_Handler *m, DMA_Node chx_lli[], DMA_Channel_TypeDef* dma_handler, uint32_t* buffer, const GPIO_Assignment busy_pins[], AT_WriteFunc func, void *user){
//	memset(m, 0, sizeof(*m));
	m->ready_mask = 0;
	m->ready_to_disp = 0;
	m->continuous = true;
	m->samples_requested = 0xFFF;
	m->display_func.write = func;
	m->display_func.user = user;
	m->chx_lli[0] = &chx_lli[0];
	m->chx_lli[1] = &chx_lli[1];
	m->chx_lli[2] = &chx_lli[2];
	m->chx_lli[3] = &chx_lli[3];
	m->dma_handler = dma_handler;
	m->dma_flags_reg = &dma_handler->CSR;
	m->dma_tc_flag_mask = DMA_CFCR_TCF;
	m->dma_us_flag_mask = DMA_CFCR_USEF;
	m->nodes_used = -1;
	m->common_buffer = buffer;
	for (uint8_t i=0;i<MAX_EXTI;i++) m->exti_to_dev[i] = 0xFF; //0xFF indicates empty line
	for (uint8_t i=0;i<MAX_DEVICES;i++) {
	    m->busy_pins[i] = busy_pins[i];
	}
	RCCandGPIO_Config_Regs();
//	GPDMA1_CH7_Config_PSSI_P2M_LLI(m, true);
//	PSSI_Config_Regs(); /* test whether reinitialization of pssi causes bit shift */
	return true;
}

/*
 * ADC_ManagerInit - Initialize ADC manager with device count and operation mode
 * @param m: Pointer to ADC handler structure
 * @param dev_amount: Number of connected ADC devices (must be < MAX_DEVICES)
 * @param mode: Operation mode (true = continuous, false = single shot)
 * @return true on success, false on failure
 * 
 * Configures the ADC manager for the specified number of devices and operation mode.
 * Sets up device structures, ready masks, and sample request pointers for each device.
 * This function should be called after ADC_Init and before starting sampling.
 */
bool ADC_ManagerInit(ADC_Handler *m, uint8_t dev_amount, bool mode){
	m->ndevs = dev_amount;
	m->ready_to_disp_mask = 0;
	m->common_ptr = 0;
	m->continuous = mode;
	for(uint8_t i=0;i<dev_amount;i++){
		m->ltc2368_devs[i].samples_requested = &m->samples_requested;
		m->ltc2368_devs[i].device_id = i;
		m->ltc2368_devs[i].busy_pin = &m->busy_pins[i];
		m->ready_to_disp_mask |= 1<<i;
		m->ready_to_disp &= 0<<i;
		m->ltc2368_devs[i].ready_to_disp = (uint8_t*)&m->ready_to_disp;
	}
	return true;
}

/*
 * ADC_ReportError - Report ADC error conditions via display function
 * @param error_code: Error code (1 = general error, 2 = busy timeout)
 * @param user: User data pointer (may contain error details for error_code 2)
 * @return none
 * 
 * Formats and sends error messages through the ADC handler's display function.
 * Used internally to report conversion errors or device timeout conditions.
 */
static inline void ADC_ReportError(uint8_t error_code, void *user){
	char buf[50];
	size_t used;
	switch(error_code){
		case 1: used = snprintf(buf, sizeof(buf), "ERROR OCCURED\r\n"); break;
		case 2: uint32_t busy_fail = (uint32_t)user; used = snprintf(buf, sizeof(buf), "ERROR BUSY %ld\r\n", busy_fail); break;
		default: used = snprintf(buf, sizeof(buf), "ERROR OCCURED\r\n"); break;
	}
	g_adc_mgr->display_func.write(buf, used);
}

/*
 * ADC_Acquire - Process completed ADC data acquisition and prepare for next cycle
 * @param m: Pointer to ADC handler structure
 * @return none
 * 
 * Called when DMA transfer is complete. Stops sampling, displays collected data
 * based on format setting (raw or clear), checks device ready status, and either
 * restarts sampling in continuous mode or stops in single-shot mode.
 * This function is typically called from the DMA completion interrupt handler.
 */
void ADC_Acquire(ADC_Handler *m){
	// We skim through ready_mask and operate on every id that has been reported
	if (PSSI_HAL_PSSI_ReceiveComplete_count!=m->nodes_used)
		return;
	LTC2368_StopSampling(&m->clock_handler);
	if (m->format == 0){
		if (ADC_DisplaySamples_Raw(m, RESET_BUF, 0) == false){
			ADC_ReportError(0, NULL);
		}
	}
	else {
		ADC_MarkReady(m);
		if (!ADC_FetchReady(m)){
			ADC_ReportError(2, (void*)m->ready_mask); //vulnerable
		}
		if (ADC_DisplaySamples_Clear(m, RESET_BUF, 0) == false){
			ADC_ReportError(0, NULL);
		}
	}
	PSSI_HAL_PSSI_ReceiveComplete_count = 0;
	if (m->continuous){
		GPDMA1_CH7_Config_PSSI_P2M_LLI(m, true, m->samples_requested);
		LTC2368_StartSampling(&m->clock_handler);
	}
	else {
		m->state = false;
	}
}

/*
 * ADC_DisplayConfig - Display current ADC clock configuration
 * @param m: Pointer to ADC handler structure
 * @return true on success, false on failure
 * 
 * Formats and displays the current ADC configuration including:
 * - Sampling frequency
 * - Reading frequency
 * - Reference frequency
 * - Number of DMA nodes used
 * Output is sent through the display function registered in the ADC handler.
 */
bool ADC_DisplayConfig(ADC_Handler *m){
	char buffer[256] = {0};
	size_t used = 0;
	used += snprintf(buffer+used, sizeof(buffer)-used, "ADC sampling freq: %ldHz\r\n",m->clock_handler.samp_freq);
	used += snprintf(buffer+used, sizeof(buffer)-used, "ADC reading freq: %ldHz\r\n",m->clock_handler.read_freq);
	used += snprintf(buffer+used, sizeof(buffer)-used, "ADC ref freq: %ldHz\r\n",m->clock_handler.ref_freq);
	used += snprintf(buffer+used, sizeof(buffer)-used, "ADC nodes used: %ld\r\n",m->nodes_used);
	if (m->display_func.write(buffer, used) != AT_OK) return false;
	return true;
}

/*
 * ADC_Calculate_Output - Convert raw ADC sample to voltage in millivolts
 * @param sample: Raw 16-bit ADC sample value
 * @return Voltage in millivolts (unsigned, range typically 0 to ADC_REFRENCE_V mV)
 * 
 * Converts a raw ADC sample to voltage using the reference voltage and resolution.
 * Assumes 0V to ADC_REFRENCE_V input range with internal 2.5V reference. Returns signed voltage
 * value in millivolts for display purposes.
 */
static inline uint16_t ADC_Calculate_Output(uint32_t sample){
	sample &= 0xFFFF; //we need 32bits only for extended operations
	uint32_t sample_voltage = (sample*REFRENCE_V)/RESOLUTION_BITS;
	return (uint16_t)sample_voltage;  //assuming range is 0 to ADC_REFRENCE_V mV and REF is internal 2,5V datasheet p.23
}


// transpozycja 8x8 bitów (Hacker's Delight)
/*
 * transpose8x8 - Transpose an 8x8 bit matrix (Hacker's Delight algorithm)
 * @param x: 64-bit value representing 8x8 bit matrix
 * @return Transposed 64-bit value
 * 
 * Efficiently transposes an 8x8 bit matrix using bit manipulation techniques.
 * Used for deinterleaving parallel ADC data from PSSI interface.
 */
static inline uint64_t transpose8x8(uint64_t x) {
    uint64_t t;
    t = (x ^ (x >> 7)) & 0x00AA00AA00AA00AAull; x ^= t ^ (t << 7);
    t = (x ^ (x >>14)) & 0x0000CCCC0000CCCCull; x ^= t ^ (t <<14);
    t = (x ^ (x >>28)) & 0x00000000F0F0F0F0ull; x ^= t ^ (t <<28);
    return x;
}

// 16-bit bit-reverse
/*
 * bitrev16 - Reverse the bits in a 16-bit value
 * @param x: 16-bit value to reverse
 * @return 16-bit value with bits reversed
 * 
 * Reverses all bits in a 16-bit value. Uses ARM RBIT instruction when available
 * for hardware acceleration, otherwise uses software bit manipulation.
 * Used for converting MSB-first to LSB-first data format.
 */
static inline uint16_t bitrev16(uint16_t x) {
#if defined(__ARMCC_VERSION) || defined(__GNUC__)
    // M33 ma instrukcję RBIT
    return (uint16_t)(__RBIT(x) >> 16);
#else
    // przenośne SW
    x = (uint16_t)(((x & 0x5555u) << 1) | ((x >> 1) & 0x5555u));
    x = (uint16_t)(((x & 0x3333u) << 2) | ((x >> 2) & 0x3333u));
    x = (uint16_t)(((x & 0x0F0Fu) << 4) | ((x >> 4) & 0x0F0Fu));
    return (uint16_t)((x << 8) | (x >> 8));
#endif
}

/*
 * deint_8x16_swar_msb_first - Deinterleave 8 parallel 16-bit samples from byte stream
 * @param b: Pointer to 16-byte input buffer (8 channels × 2 bytes)
 * @param ch: Output array of 8 uint16_t values (one per channel)
 * @return none
 * 
 * Deinterleaves parallel ADC data received via PSSI interface. The input is 16 bytes
 * representing 8 channels with bits interleaved. Uses SIMD-like operations (SWAR)
 * to efficiently transpose and extract individual channel samples in MSB-first format.
 */
static inline void deint_8x16_swar_msb_first(const uint8_t *b, uint16_t ch[8]) {
    uint64_t lo, hi;
    memcpy(&lo, b + 0, 8);
    memcpy(&hi, b + 8, 8);

    uint64_t T0 = transpose8x8(lo);   // bity 0..7
    uint64_t T1 = transpose8x8(hi);   // bity 8..15

    for (int k = 0; k < 8; ++k) {
        uint16_t lo8 = (uint16_t)((T0 >> (k * 8)) & 0xFFu);
        uint16_t hi8 = (uint16_t)((T1 >> (k * 8)) & 0xFFu);
        uint16_t v   = (uint16_t)(lo8 | (hi8 << 8));   // LSB-first
        ch[k] = bitrev16(v);                           // MSB-first
    }
}

/*
 * ADC_DisplaySamples_Clear - Display ADC samples in human-readable format
 * @param m: Pointer to ADC handler structure
 * @param reset_buf: If true, reset buffer pointer before display
 * @param custom_samples_requested: Number of samples to display (0 = use handler's samples_requested)
 * @return true on success, false on failure
 * 
 * Displays collected ADC samples in a human-readable format with voltage values.
 * Shows channel headers and converts raw samples to voltage (mV) with decimal formatting.
 * Output is sent through the display function in formatted text.
 */
bool ADC_DisplaySamples_Clear(ADC_Handler *m, bool reset_buf, uint32_t custom_samples_requested){
	char buffer[256] = {0};
	size_t used = 0;
	uint16_t v;
	uint32_t samples_requested;
	if (custom_samples_requested == 0) samples_requested = m->samples_requested;
	else samples_requested = custom_samples_requested;
	used += snprintf(buffer+used, sizeof(buffer)-used, "Collected samples:%ld\r\n",samples_requested);
	for (uint8_t i=0; i<m->ndevs; i++){
		used += snprintf(buffer+used, sizeof(buffer)-used, "CH%d\t", i);
	}
	if (reset_buf == 1)
		m->common_ptr = 0;
	used += snprintf(buffer+used, sizeof(buffer)-used, "\r\n");
	m->display_func.write(buffer, used);
	used = 0;
	const uint8_t *raw = (const uint8_t*)m->common_buffer;
	for(uint32_t off=0; off<samples_requested*16; off+=16){
		deint_8x16_swar_msb_first(raw + off, m->output_buf);
		for(uint8_t k=0; k<8; k++){
			v = ADC_Calculate_Output((uint32_t)m->output_buf[k]);
			used += snprintf(buffer+used, sizeof(buffer)-used, "%d.%dV\t", v/1000,abs(v%1000));
		}
		used += snprintf(buffer+used, sizeof(buffer)-used, "\r\n");
		if (m->display_func.write(buffer, used) != AT_OK) return false;
		used = 0;
	}
	return true;
}

/*
 * ADC_DisplaySamples_Raw - Display ADC samples in raw binary format
 * @param m: Pointer to ADC handler structure
 * @param reset_buf: If true, reset buffer pointer before display
 * @param custom_samples_requested: Number of samples to display (0 = use handler's samples_requested)
 * @return true on success, false on failure
 * 
 * Displays collected ADC samples in raw binary format for high-speed data transfer.
 * Data is sent in 128-byte frames (MAX_BUFFER_SIZE) for efficient USB transmission.
 * Each sample is 2 bytes (16-bit) in little-endian format. Used for streaming applications.
 */
bool ADC_DisplaySamples_Raw(ADC_Handler *m, bool reset_buf, uint32_t custom_samples_requested)
{
    if (reset_buf) m->common_ptr = 0;

    uint32_t samples_requested = custom_samples_requested ? custom_samples_requested : m->samples_requested;
    uint8_t *frame = m->usb_stream;                 // bufor o rozmiarze MAX_BUFFER_SIZE (128 B)
    uint32_t in_frame = 0;                          // offset w bieżącej ramce 0..127
    const uint8_t *raw = (const uint8_t*)m->common_buffer;

    for (uint32_t off = 0; off < samples_requested * 16; off += 16) {
        deint_8x16_swar_msb_first(raw + off, m->output_buf); // 8 próbek po 2 B = 16 B

        for (uint8_t k = 0; k < 8; k++) {
            memcpy(frame + in_frame, &m->output_buf[k], 2);
            in_frame += 2;

            if (in_frame == MAX_BUFFER_SIZE) {      // dokładnie 128 B
                if (m->display_func.write((char*)frame, MAX_BUFFER_SIZE) != AT_OK) return false;
                in_frame = 0;                       // rozpoczynamy nową ramkę od początku frame
            }
        }
    }

    if (in_frame) {                                 // ogon niepełnej ramki
        if (m->display_func.write((char*)frame, in_frame) != AT_OK) return false;
    }

    return true;
}

/*
 * ADC_BusyCheck - Check if ADC is currently sampling
 * @param none
 * @return true if ADC is sampling, false if idle
 * 
 * Returns the current sampling state of the ADC system. Used by command handlers
 * to prevent configuration changes during active sampling.
 */
bool ADC_BusyCheck(void){
	return g_adc.state;
}


/* ========================= PSSI ========================= */

static inline void PSSI_Config_Regs(void)
{
	/* Wyczyść ewentualny OVR i włącz przerwanie OVR (IER=0x2) */
	WRITE_REG(PSSI->ICR, PSSI_ICR_OVR_ISC);
	WRITE_REG(PSSI->IER, PSSI_IER_OVR_IE);

	/* Konfiguracja PSSI_CR:
	 - CKPOL = 1 (jak na zrzucie)
	 - ENABLE = 1
	 - DMAEN  = 1
	 Reszta pól = 0: 8-bit, 8 linii, DE/RDY wyłączone, polaryzacje DE/RDY nieużywane
	*/
	WRITE_REG(PSSI->CR, PSSI_CR_DMAEN);   // PSSI_CR_CKPOL -> 1 if rising edge
	PSSI->CR |= PSSI_CR_ENABLE;

	/* ---- Wartości po tej konfiguracji ----
	 PSSI->CR  = 0x40000020
	 PSSI->IER = 0x00000002
	 PSSI->ICR = 0x00000002  (zapis skasuje flagę)
	*/
}

/* ========================= GPDMA ========================= */

static inline void RCCandGPIO_Config_Regs(void)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
    __HAL_RCC_DCMI_PSSI_CLK_ENABLE();

    /* Takty dla PSSI (DCMI/PSSI) i dla GPDMA1, jeśli jeszcze nie włączone */
    SET_BIT(RCC->AHB2ENR, RCC_AHB2ENR_DCMI_PSSIEN);

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    /**PSSI GPIO Configuration
    PA4     ------> PSSI_DE
    PA6     ------> PSSI_PDCK
    PB15     ------> PSSI_D2
    PC7     ------> PSSI_D1
    PC9     ------> PSSI_D3
    PA9     ------> PSSI_D0
    PC11     ------> PSSI_D4
    PB4(NJTRST)     ------> PSSI_D7
    PB6     ------> PSSI_D5
    PB7     ------> PSSI_RDY
    PB8     ------> PSSI_D6
    */
    GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_6|GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF13_PSSI;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_15|GPIO_PIN_4|GPIO_PIN_6|GPIO_PIN_7
                          |GPIO_PIN_8;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF13_PSSI;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_7|GPIO_PIN_9|GPIO_PIN_11;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF13_PSSI;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
}

/*
 * ADC_ChangeRequestedSamples - Change the number of samples to collect
 * @param m: Pointer to ADC handler structure
 * @param new_request: New number of samples to request
 * @return none
 * 
 * Updates the number of samples to collect and reconfigures the DMA linked list
 * to match the new sample count. Disables PSSI, rebuilds DMA chain, and reconfigures
 * PSSI registers. This function should only be called when sampling is stopped.
 */
void ADC_ChangeRequestedSamples(ADC_Handler *m, uint16_t new_request)
{
	m->samples_requested = new_request;
	if (g_adc_mgr->nodes_used == -1){
		PSSI->CR &= ~PSSI_CR_ENABLE; /* test whether reinitialization of pssi causes bit shift */
		GPDMA1_CH7_Config_PSSI_P2M_LLI(m, true, m->samples_requested);
		PSSI_Config_Regs(); /* test whether reinitialization of pssi causes bit shift */
	}
	else
		GPDMA1_CH7_Config_PSSI_P2M_LLI(m, true, m->samples_requested);

}


static inline uint32_t make_CTR1_p2m_pack_4x8_to_32(void)
{
    return  (0u << DMA_CTR1_SDW_LOG2_Pos)   /* 8-bit source for collecting data*/
          | (0u << DMA_CTR1_SINC_Pos)       /* source pointer FIXED */
          | (2u << DMA_CTR1_DDW_LOG2_Pos)   /* 32-bit destination to save data  */
          |  DMA_CTR1_DINC                  /* increment destination with each read */
          |  DMA_CTR1_DAP                   /* port1 for destination */
          | (2u << DMA_CTR1_PAM_Pos);       /* 4×8→32 packing */
}

static inline uint32_t make_CTR2_req_pssi(void)
{
    return ((uint32_t)DCMI_PSSI_IRQn & DMA_CTR2_REQSEL_Msk);
}

/* Returns offset to the next node in reference to CLBAR value */
static inline uint32_t compute_LA_offset(const DMA_Node *base, const DMA_Node *next)
{
    uintptr_t b = (uintptr_t)base;
    uintptr_t n = (uintptr_t)next;
    uint32_t  off = (uint32_t)(n - b);  // w bajtach
    /* CLLR.LA is coded in 4B words (bits[15:2]); with 4KB align you cannot exceed 0xFFF */
    return ((off >> 2) & 0x3FFFu) << DMA_CLLR_LA_Pos;  // Firstly, we limit data as required, then moving it to proper position
}

/* Nodes list configuration for requested samples */
static inline uint32_t GPDMA1_CH7_Build_PSSI_LL(ADC_Handler *m, uint32_t requested_samples)
{
	DMA_Node **nodes = m->chx_lli;           // ch7_lli[4] from 4KB alligned section
    const uint32_t total_bytes = 16u * requested_samples;   // 8 channels x 2B = 16B per sample
    uint32_t remaining = total_bytes;
    int32_t nodes_used = 0;

    /* Target pointer having 4 bytes of data packed */
    uint32_t *dst = (uint32_t*)m->common_buffer;

    while (remaining && nodes_used < 4u)
    {
        DMA_Node *nd = nodes[nodes_used];

        /* DMA buffer size must not exceed GPDMA_BNDT_MAX_ALIGNED */
        uint32_t chunk = (remaining > GPDMA_BNDT_MAX_ALIGNED) ? GPDMA_BNDT_MAX_ALIGNED : remaining;

        nd->CTR1 = make_CTR1_p2m_pack_4x8_to_32();
        nd->CTR2 = make_CTR2_req_pssi();

        /* BNDT – amount of bytes to collect (required to be alligned to 3bits, bits 0 and 1 are reserved) */
        nd->CBR1 = (chunk & DMA_CBR1_BNDT_Msk);

        nd->CSAR = (uint32_t)&PSSI_NS->DR;
        nd->CDAR = (uint32_t)dst;

        nd->CTR3 = 0;
        nd->CBR2 = 0;

        nd->CLLR = 0;

        /* Move pointer for value of chunk/4 (division by 4 yields from casting uint8_t to uint32_t) */
        dst       += (chunk / 4u);
        remaining -= chunk;
        nodes_used++;
    }

    for (int32_t i = 0; i + 1 < nodes_used; ++i)
    {
        uint32_t la = compute_LA_offset(nodes[0], nodes[i+1]);
        /* Set LA offset (to the next node) and set Uxx bytes, determining which registers are influenced */
        nodes[i]->CLLR = (nodes[i]->CLLR & ~(DMA_CLLR_LA_Msk)) | la
        		 | DMA_CLLR_ULL | DMA_CLLR_USA | DMA_CLLR_UDA
				 | DMA_CLLR_UB1 | DMA_CLLR_UT1 | DMA_CLLR_UT2;
    }

    /* Last node's LA register needs to be 0 to indicate it is last one */
    return nodes_used;
}

static inline void GPDMA1_CH7_Config_PSSI_P2M_LLI(ADC_Handler *m, bool update, uint32_t requested_samples)
{
  DMA_Channel_TypeDef *ch = g_adc_mgr->dma_handler;

  /* Disable DMA channel and reset flags */
  CLEAR_BIT(ch->CCR, DMA_CCR_EN);
  SET_BIT(ch->CCR, DMA_CCR_RESET);
  WRITE_REG(ch->CFCR, DMA_CFCR_TCF|DMA_CFCR_HTF|DMA_CFCR_DTEF|DMA_CFCR_ULEF|DMA_CFCR_USEF|DMA_CFCR_TOF);

  /* Define used nodes for required buffer size */
  m->nodes_used = GPDMA1_CH7_Build_PSSI_LL(m, requested_samples);

  /* Pointer to the first node (needs to be 4KB alligned) */
  WRITE_REG(ch->CLBAR, (uint32_t)m->chx_lli[0]);

  /* Load CLLR register from first node */
  WRITE_REG(ch->CLLR, //m->chx_lli[0]->CLLR );
		  	  	  	 	 DMA_CLLR_ULL
                     |  DMA_CLLR_USA
                     |  DMA_CLLR_UDA
                     |  DMA_CLLR_UB1
                     |  DMA_CLLR_UT1
                     |  DMA_CLLR_UT2);

  /* Set channel IRQ and start DMA */
  SET_BIT(ch->CCR, DMA_CCR_TCIE | DMA_CCR_USEIE);
  CLEAR_BIT(ch->CCR, DMA_CCR_HTIE | DMA_CCR_DTEIE | DMA_CCR_TOIE | DMA_CCR_ULEIE);
  SET_BIT(ch->CCR, DMA_CCR_EN);
}

/*
 * ADC_TIM_IRQHandler - Timer interrupt handler for ADC slave timing
 * @param none
 * @return none
 * 
 * Handles timer interrupts from the ADC slave timer. Processes the interrupt flag,
 * enables the delay timer for communication timing, and increments the common pointer.
 * This function should be registered as the interrupt handler for the slave timer.
 */
void ADC_TIM_IRQHandler(void){
	LTC2368_SlaveIrqHandling(&g_adc_mgr->clock_handler.tim_delay, &g_adc_mgr->clock_handler.tim_slave, &g_adc_mgr->common_ptr);
}

/*
 * ADC_DMA_IRQHandler - DMA interrupt handler for ADC data transfer
 * @param none
 * @return none
 * 
 * Handles DMA interrupts for ADC data acquisition:
 * - Transfer Complete (TC): Increments completion counter and clears flag
 * - Underrun/Suspend Error (USE): Attempts to recover by reloading DMA configuration
 *   (up to 2 retries before calling Error_Handler)
 * This function should be registered as the interrupt handler for the DMA channel.
 */
void ADC_DMA_IRQHandler(void){
	if (*g_adc_mgr->dma_flags_reg & g_adc_mgr->dma_tc_flag_mask) {
		WRITE_REG(g_adc_mgr->dma_handler->CFCR, g_adc_mgr->dma_tc_flag_mask);                                           /* restart */               /* :contentReference[oaicite:18]{index=18} */
		PSSI_HAL_PSSI_ReceiveComplete_count++;
	}
	if (*g_adc_mgr->dma_flags_reg & g_adc_mgr->dma_us_flag_mask)
	{
		if (user_code_error < 2u)
		{
			WRITE_REG(g_adc_mgr->dma_handler->CFCR, DMA_CFCR_USEF);
			WRITE_REG(g_adc_mgr->dma_handler->CLBAR, (uint32_t)g_adc_mgr->chx_lli[0]);        /* LBA 4KB base = node address */
			WRITE_REG(g_adc_mgr->dma_handler->CLLR,  DMA_CLLR_ULL                 /* update link addr from node    */
						   |  DMA_CLLR_USA                /* LOAD SAR                 */
						   |  DMA_CLLR_UDA                /* LOAD DAR                 */
						   |  DMA_CLLR_UB1                /* LOAD BR1                 */
						   |  DMA_CLLR_UT1                /* LOAD TR1                 */
						   |  DMA_CLLR_UT2);              /* LOAD TR2                 */
			SET_BIT(g_adc_mgr->dma_handler->CCR, DMA_CCR_EN);
			user_code_error++;
		}
		else
		{
		  Error_Handler();
		}
	}
}
/*
 * Those functions I decided to leave as a placeholder for alternative changes in architecture
 */

/*
bool ADC_StartSampling(void){
	return true;
}

bool ADC_StopSampling(void){
	return true;
}
*/
