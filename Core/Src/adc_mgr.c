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
static int16_t ADC_Calculate_Output(uint32_t sample);
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


uint8_t ADC_PinToIndex(uint16_t GPIO_Pin){
	#if defined(__GNUC__)
		return (uint8_t)__builtin_ctz((unsigned)GPIO_Pin);
	#else
		return (uint8_t)(__CLZ(__RBIT(GPIO_Pin))); // CMSIS fallback
	#endif
}

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

static inline uint32_t ADC_FetchReady(ADC_Handler *m) {
    return m->ready_mask == m->ready_to_disp_mask;
}

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
//	PSSI_Config_Regs();
	return true;
}

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

static int16_t ADC_Calculate_Output(uint32_t sample){
	sample &= 0xFFFF;
	int16_t sample_voltage = (sample*REFRENCE_V)/RESOLUTION_BITS;
	return sample_voltage;  //assuming range is +/-10V and REF is internal 2,5V datasheet p.23
}


// transpozycja 8x8 bitów (Hacker's Delight)
static inline uint64_t transpose8x8(uint64_t x) {
    uint64_t t;
    t = (x ^ (x >> 7)) & 0x00AA00AA00AA00AAull; x ^= t ^ (t << 7);
    t = (x ^ (x >>14)) & 0x0000CCCC0000CCCCull; x ^= t ^ (t <<14);
    t = (x ^ (x >>28)) & 0x00000000F0F0F0F0ull; x ^= t ^ (t <<28);
    return x;
}

// 16-bit bit-reverse
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

bool ADC_DisplaySamples_Clear(ADC_Handler *m, bool reset_buf, uint32_t custom_samples_requested){
//	LTC2368_StopSampling(&m->clock_handler);
	char buffer[256] = {0};
	size_t used = 0;
	int16_t v;
	uint32_t samples_requested;
//	uint16_t tmp_buffer[8];
	if (custom_samples_requested == 0) samples_requested = m->samples_requested;
	else samples_requested = custom_samples_requested;
	used += snprintf(buffer+used, sizeof(buffer)-used, "Collected samples:%ld\r\n",samples_requested);
//	m->display_func.write(s, (uint16_t) len);
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

void ADC_ChangeRequestedSamples(ADC_Handler *m, uint16_t new_request)
{
	m->samples_requested = new_request;
	PSSI->CR &= ~PSSI_CR_ENABLE;
	GPDMA1_CH7_Config_PSSI_P2M_LLI(m, true, m->samples_requested);
	PSSI_Config_Regs();
}


static inline uint32_t make_CTR1_p2m_pack_4x8_to_32(void)
{
    return  (0u << DMA_CTR1_SDW_LOG2_Pos)   /* źródło 8-bit */
          | (0u << DMA_CTR1_SINC_Pos)       /* źródło FIXED */
          | (2u << DMA_CTR1_DDW_LOG2_Pos)   /* cel 32-bit   */
          |  DMA_CTR1_DINC                  /* inkrementuj cel */
          |  DMA_CTR1_DAP                   /* port1 (dest) */
          | (2u << DMA_CTR1_PAM_Pos);       /* pakowanie 4×8→32 */
}

static inline uint32_t make_CTR2_req_pssi(void)
{
    return ((uint32_t)DCMI_PSSI_IRQn & DMA_CTR2_REQSEL_Msk);
}

/* Zwraca offset LA (bits[15:2]) do następnego węzła względem CLBAR.
   Węzły muszą leżeć w tej samej 4KB stronie (masz aligned(4096)). */
static inline uint32_t compute_LA_offset(const DMA_Node *base, const DMA_Node *next)
{
    uintptr_t b = (uintptr_t)base;
    uintptr_t n = (uintptr_t)next;
    uint32_t  off = (uint32_t)(n - b);  // w bajtach
    /* CLLR.LA jest kodowane w słowach 4B (bits[15:2]); przy 4KB align nie przekroczysz 0xFFF */
    return ((off >> 2) & 0x3FFFu) << DMA_CLLR_LA_Pos;  // użyj właściwej pozycji bitów LA
}

/* Konfiguracja listy węzłów dla PSSI → pamięć (LLI) pod żądane próbki */
static inline uint32_t GPDMA1_CH7_Build_PSSI_LL(ADC_Handler *m, uint32_t requested_samples)
{
	DMA_Node **nodes = m->chx_lli;           // ch7_lli[4] w sekcji 4KB aligned
    const uint32_t total_bytes = 16u * requested_samples;   // 8 kanałów × 2 B = 16 B na „paczkę”
    uint32_t remaining = total_bytes;
    int32_t nodes_used = 0;

    /* wskaźnik docelowy (pakowanie 4×8→32 => na 4 bajty powstaje 1 słowo) */
    uint32_t *dst = (uint32_t*)m->common_buffer;

    while (remaining && nodes_used < 4u)
    {
        DMA_Node *nd = nodes[nodes_used];
        uint32_t chunk = (remaining > GPDMA_BNDT_MAX_ALIGNED) ? GPDMA_BNDT_MAX_ALIGNED : remaining;

        nd->CTR1 = make_CTR1_p2m_pack_4x8_to_32();
        nd->CTR2 = make_CTR2_req_pssi();

        /* BNDT – liczba bajtów, wyrównana do 4 (wymóg pakowania) */
        nd->CBR1 = (chunk & DMA_CBR1_BNDT_Msk);

        nd->CSAR = (uint32_t)&PSSI_NS->DR;
        nd->CDAR = (uint32_t)dst;

        nd->CTR3 = 0;
        nd->CBR2 = 0;

        /* Domyślnie: aktualizuj wszystko; LA ustawiony niżej jeśli jest kolejny węzeł */
        nd->CLLR = 0;
//        nd->CLLR = DMA_CLLR_ULL | DMA_CLLR_USA | DMA_CLLR_UDA
//                 | DMA_CLLR_UB1 | DMA_CLLR_UT1 | DMA_CLLR_UT2;

        /* przesuwamy pointer docelowy o chunk/4 słów (pakowanie) */
        dst       += (chunk / 4u);
        remaining -= chunk;
        nodes_used++;
    }

    /* Dowiąż kolejne węzły LLI przez ustawienie LA w CLLR poprzedniego */
    for (int32_t i = 0; i + 1 < nodes_used; ++i)
    {
        uint32_t la = compute_LA_offset(nodes[0], nodes[i+1]);
        /* Wpisz LA do CLLR, zachowując bity ULL/USA/UDA/UB1/UT1/UT2 */
        nodes[i]->CLLR = (nodes[i]->CLLR & ~(DMA_CLLR_LA_Msk)) | la
        		 | DMA_CLLR_ULL | DMA_CLLR_USA | DMA_CLLR_UDA
				 | DMA_CLLR_UB1 | DMA_CLLR_UT1 | DMA_CLLR_UT2;
    }

    /* Ostatni węzeł: LA = 0 (koniec listy) – zostaw jak jest; ULL/USA/... mogą pozostać ustawione. */
    return nodes_used;
}

static inline void GPDMA1_CH7_Config_PSSI_P2M_LLI(ADC_Handler *m, bool update, uint32_t requested_samples)
{
  DMA_Channel_TypeDef *ch = g_adc_mgr->dma_handler;

  /* Wyłącz kanał i wyczyść flagi */
  CLEAR_BIT(ch->CCR, DMA_CCR_EN);
//  if (ch->CSR & DMA_CSR_IDLEF)
  SET_BIT(ch->CCR, DMA_CCR_RESET);
  WRITE_REG(ch->CFCR, DMA_CFCR_TCF|DMA_CFCR_HTF|DMA_CFCR_DTEF|DMA_CFCR_ULEF|DMA_CFCR_USEF|DMA_CFCR_TOF);

  /* Zbuduj LLI pod aktualny rozmiar */
  m->nodes_used = GPDMA1_CH7_Build_PSSI_LL(m, requested_samples);

  /* Seed: baza 4KB wskazuje na pierwszy węzeł */
  WRITE_REG(ch->CLBAR, (uint32_t)m->chx_lli[0]);

  /* Przy loadzie z CLBAR/CLLR: załaduj wszystko z pierwszego węzła */
  WRITE_REG(ch->CLLR, //m->chx_lli[0]->CLLR );
		  	  	  	 	 DMA_CLLR_ULL
                     |  DMA_CLLR_USA
                     |  DMA_CLLR_UDA
                     |  DMA_CLLR_UB1
                     |  DMA_CLLR_UT1
                     |  DMA_CLLR_UT2);

  /* IRQ kanału i start */
  SET_BIT(ch->CCR, DMA_CCR_TCIE | DMA_CCR_USEIE);
  CLEAR_BIT(ch->CCR, DMA_CCR_HTIE | DMA_CCR_DTEIE | DMA_CCR_TOIE | DMA_CCR_ULEIE);
  SET_BIT(ch->CCR, DMA_CCR_EN);
}

void ADC_TIM_IRQHandler(void){
	LTC2368_SlaveIrqHandling(&g_adc_mgr->clock_handler.tim_delay, &g_adc_mgr->clock_handler.tim_slave, &g_adc_mgr->common_ptr);
	/* Temporal solution for overloading data collection */
//	if (g_adc_mgr->common_ptr>g_adc_mgr->samples_requested && PSSI_HAL_PSSI_ReceiveComplete_count<g_adc_mgr->nodes_used){
//		LTC2368_StopSampling(&g_adc_mgr->clock_handler);
//		PSSI_HAL_PSSI_ReceiveComplete_count=0;
//		g_adc_mgr->common_ptr=0;
//		if (g_adc_mgr->continuous){
//			GPDMA1_CH7_Config_PSSI_P2M_LLI(g_adc_mgr, true, g_adc_mgr->samples_requested);
//			LTC2368_StartSampling(&g_adc_mgr->clock_handler);
//		}
//		else {
//			g_adc_mgr->state = false;
//		}
//	}

}

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
			WRITE_REG(g_adc_mgr->dma_handler->CLBAR, (uint32_t)g_adc_mgr->chx_lli[0]);        /* LBA = baza 4KB = adres węzła */
			WRITE_REG(g_adc_mgr->dma_handler->CLLR,  DMA_CLLR_ULL                 /* update link addr z węzła    */
						   |  DMA_CLLR_USA                /* ZAŁADUJ SAR                 */
						   |  DMA_CLLR_UDA                /* ZAŁADUJ DAR                 */
						   |  DMA_CLLR_UB1                /* ZAŁADUJ BR1                 */
						   |  DMA_CLLR_UT1                /* ZAŁADUJ TR1                 */
						   |  DMA_CLLR_UT2);              /* ZAŁADUJ TR2                 */
			SET_BIT(g_adc_mgr->dma_handler->CCR, DMA_CCR_EN);
			user_code_error++;
		}
		else
		{
		  Error_Handler();
		}
	//		  /* clear USE */             /* :contentReference[oaicite:22]{index=22}turn20file2 */
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
