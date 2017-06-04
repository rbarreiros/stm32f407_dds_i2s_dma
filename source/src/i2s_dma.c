/**
 * I2S transmitter specs:
 * 	SPI			: SPI3
 * 	DMA			: DMA1
 * 	DMA channel	: channel 0
 * 	DMA stream	: 5 or 7
 */

#include "i2s.h"
#include "dds.h"

#define Tx_DMAStream  DMA1_Stream7
#define TRM_Speed	GPIO_Speed_100MHz

dmabuf_t DMATRMBUF_0[DMATRMBUF_SIZE];
dmabuf_t DMATRMBUF_1[DMATRMBUF_SIZE];

#ifdef I2S_ENABLE_RX
#define WCLK_TIMEOUT  5 // in ms
#define Rx_DMAStream  DMA1_Stream3
#define RCV_Speed   GPIO_Speed_100MHz
volatile dmabuf_t DMARCVBUF[DMARCVBUF_SIZE];

extern u32 TOTmr1ms;

static void RxDMADisable(void) {
	DMA_Cmd( Rx_DMAStream, DISABLE );
	while( DMA_GetCmdStatus( Rx_DMAStream ) == ENABLE );
	__ISB();
}
#endif

volatile tSample LDataSend, RDataSend;

static void TxDMADisable(void) {
	DMA_Cmd( Tx_DMAStream, DISABLE);
	while (DMA_GetCmdStatus( Tx_DMAStream) == ENABLE)
		;
	__ISB();
}

/**
 * DMA Transmitter
 */
static __INLINE void I2S3_DMA_Init(void)
{
	DMA_InitTypeDef DMA_InitStruct;
	NVIC_InitTypeDef NVIC_InitStructure;

	DMA_DeInit(Tx_DMAStream);
	TxDMADisable();
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);

	// To:
	DMA_InitStruct.DMA_PeripheralBaseAddr = (uint32_t) &(SPI3->DR); // I2S Data
	// From:
	DMA_InitStruct.DMA_Memory0BaseAddr = (uint32_t) &DMATRMBUF_0; 	// memory start
	DMA_InitStruct.DMA_BufferSize = DMATRMBUF_SIZE*2;

	// With parameters:
	DMA_InitStruct.DMA_Channel = DMA_Channel_0;						// DMA1, channel 0, stream 7
	DMA_InitStruct.DMA_DIR = DMA_DIR_MemoryToPeripheral;			// direction
	DMA_InitStruct.DMA_PeripheralInc = DMA_PeripheralInc_Disable;	// disabled
	DMA_InitStruct.DMA_MemoryInc = DMA_MemoryInc_Enable;			// enable buffer incr
	DMA_InitStruct.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord; // 16 bit per
	DMA_InitStruct.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord; // 16 bit memory
	DMA_InitStruct.DMA_Mode = DMA_Mode_Circular;
	DMA_InitStruct.DMA_Priority = DMA_Priority_High;
	DMA_InitStruct.DMA_FIFOMode = DMA_FIFOMode_Enable; 				// DMA_FIFOMode_Disable;
	DMA_InitStruct.DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull;
	DMA_InitStruct.DMA_MemoryBurst = DMA_MemoryBurst_INC4; 			//DMA_MemoryBurst_Single;
	DMA_InitStruct.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
	while (DMA_GetCmdStatus( Tx_DMAStream) == ENABLE);				// wait for the last DMA end
	DMA_Init( Tx_DMAStream, &DMA_InitStruct);						// Init New

	DMA_DoubleBufferModeConfig( Tx_DMAStream, (uint32_t) &DMATRMBUF_1, DMA_Memory_0);
	DMA_ITConfig( Tx_DMAStream, DMA_IT_TC, ENABLE);
	DMA_DoubleBufferModeCmd( Tx_DMAStream, ENABLE);

	/* Configure interrupts */
	NVIC_InitStructure.NVIC_IRQChannel = DMA1_Stream7_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}

void DMA1_Stream7_IRQHandler(void)
{
	if (DMA_GetFlagStatus(Tx_DMAStream, DMA_FLAG_TCIF7) != RESET)
	{
		/* Enable DMA1_Stream7 */
		DMA_ClearFlag(Tx_DMAStream, DMA_FLAG_TCIF7);
		if ((Tx_DMAStream->CR & DMA_SxCR_CT) == 0)//get number of current buffer
		{
			PIN_DEBUG_PORT->ODR |= PIN_DEBUG1;
			/* calculate new buffer 0, while buffer 1 is transmitting */
			DDS_calculate(DMATRMBUF_1, DMATRMBUF_SIZE,
					&glb.phaseAccumulator_L, glb.dds_freq_L,
					&glb.phaseAccumulator_R, glb.dds_freq_R);
		}
		else {
			PIN_DEBUG_PORT->ODR &= ~PIN_DEBUG1;
			/* calculate new buffer 0, while buffer 0 is transmitting */
			DDS_calculate(DMATRMBUF_0, DMATRMBUF_SIZE,
					&glb.phaseAccumulator_L, glb.dds_freq_L,
					&glb.phaseAccumulator_R, glb.dds_freq_R);
		}
	}
}

void I2S_PLL_Config(uint16_t N,uint16_t R) {
    // Set PLL Clock for I2S
    RCC_PLLI2SCmd(DISABLE);
    RCC_I2SCLKConfig(RCC_I2S2CLKSource_PLLI2S);
//    RCC->CFGR &= ~RCC_CFGR_I2SSRC;
    /* Configure PLLI2S */
    RCC_PLLI2SConfig(N,R);
    /* Enable PLLI2S */
    RCC_PLLI2SCmd(ENABLE);
    /* Wait till PLLI2S is ready */
    while (RCC_GetFlagStatus(RCC_FLAG_PLLI2SRDY)==RESET){};
}

/**
 * Init master I2S transmitter
 * For clock setup see stm32f4xxx reference manual 28.4.4 Clock generator
 * HSE = 8MHz
 * PLLI2SCLK = (HSE/8) * 192 / 5 = 39.6MHz
 * PLLI2S_N = 384
 * PLLI2S_R = 5
 *
 * The correct values are 192 and 5, but because the internal PLL really
 * sucks when use an 8MHz external crystal, I've found for this board that
 * to get the correct output frequency of 48KHz, then the appropriate values
 * are 200 and 5.
 *
 * --- For 32-bit audio
 * I2SDIV = ?
 * I2SODD = ?
 * I2S clock 32-bit = PLLI2SCLK / (32*2*((2*I2SDIV)+I2SODD)) = 48000
 * --- For 16-bit audio
 * I2SDIV = 24
 * I2SODD = 2
 * I2S clock 16-bit = PLLI2SCLK / (16*2*((2*I2SDIV)+I2SODD)) = 48000
 */
void InitI2S3(void)
{

	GPIO_InitTypeDef GPIO_InitStructure;
// NVIC_InitTypeDef   NVIC_InitStructure;
	I2S_InitTypeDef I2S3_InitStructure;

	TxDMADisable();

	I2S_PLL_Config(200, 5);	// 192, 5
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI3, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA | RCC_AHB1Periph_GPIOB | RCC_AHB1Periph_GPIOC, ENABLE);

	// WCLK/LRCK: word clock output
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource15, GPIO_AF_SPI3);
	GPIO_StructInit(&GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_Speed = TRM_Speed;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	// BCLK:
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource10, GPIO_AF_SPI3);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	// SD:
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource12, GPIO_AF_SPI3);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
//	// CK_IN:
//	GPIO_PinAFConfig(GPIOC, GPIO_PinSource9, GPIO_AF_SPI2);
//	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
//	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
//	GPIO_Init(GPIOC, &GPIO_InitStructure);
	//------------------------------------------------------------------------
	I2S_StructInit(&I2S3_InitStructure);

	I2S3_InitStructure.I2S_CPOL = I2S_CPOL_Low;
	I2S3_InitStructure.I2S_Mode = I2S_Mode_MasterTx;
	I2S3_InitStructure.I2S_MCLKOutput = I2S_MCLKOutput_Disable;

	I2S3_InitStructure.I2S_Standard = I2S_Standard_MSB;
	I2S3_InitStructure.I2S_DataFormat = I2S_DataFormat_16b; //I2S_DataFormat_32b;

	TRACE(("I2S Init\n"));
	I2S_Init(SPI3, &I2S3_InitStructure);

//	SPI3->I2SPR = FSDIV;
	SPI3->I2SPR = (uint16_t)((uint16_t)12 | (uint16_t)1);

	I2S3_DMA_Init();
	SPI_I2S_DMACmd( SPI3, SPI_I2S_DMAReq_Tx, ENABLE);

	I2S_Cmd(SPI3, ENABLE);
	DMA_Cmd(Tx_DMAStream, ENABLE);
}

#ifdef I2S_ENABLE_RX
static __INLINE void I2S2_DMA_Init( void ) {// ******* D M A RECEIVER ****************
	DMA_InitTypeDef DMA_InitStruct;
// NVIC_InitTypeDef   NVIC_InitStructure;

	DMA_Cmd( Rx_DMAStream, DISABLE );
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);

	// From:
	DMA_InitStruct.DMA_PeripheralBaseAddr = (uint32_t)&(SPI2->DR);// I2S Data
	// To:
	DMA_InitStruct.DMA_Memory0BaseAddr = (u32)&DMARCVBUF[0];// memory start
	DMA_InitStruct.DMA_BufferSize = DMARCVBUF_SIZE;// length

	// With parameters:
	DMA_InitStruct.DMA_Channel = DMA_Channel_0;
	DMA_InitStruct.DMA_DIR = DMA_DIR_PeripheralToMemory;// direction
	DMA_InitStruct.DMA_PeripheralInc = DMA_PeripheralInc_Disable;//
	DMA_InitStruct.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStruct.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;// 16 bit  per
	DMA_InitStruct.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;// 16 bit memory
	DMA_InitStruct.DMA_Mode = DMA_Mode_Circular;// circ
	DMA_InitStruct.DMA_Priority = DMA_Priority_High;// prority
	DMA_InitStruct.DMA_FIFOMode = DMA_FIFOMode_Enable;// DMA_FIFOMode_Disable;
	DMA_InitStruct.DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull;
	DMA_InitStruct.DMA_MemoryBurst = DMA_MemoryBurst_INC4;//DMA_MemoryBurst_Single;
	DMA_InitStruct.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;

	while( DMA_GetCmdStatus( Rx_DMAStream ) == ENABLE );// wait for the last DMA end
	DMA_Init( Rx_DMAStream, &DMA_InitStruct );// Init New

	DMA_DoubleBufferModeConfig( Rx_DMAStream, (uint32_t)&DMARCVBUF[DMARCVBUF_SIZE/2], DMA_Memory_0 );
	DMA_DoubleBufferModeCmd( Rx_DMAStream, ENABLE );

	SPI_I2S_DMACmd( SPI2, SPI_I2S_DMAReq_Rx, ENABLE);

}
//-----------------------------------------------------------------
// wait for the defined level, return 0 if timeout occures
static u8 WaitForWCLK(u8 level) {
	TOTmr1ms = WCLK_TIMEOUT;
	if(level>0) { // wait for WCLK=1
		while (0==(GPIOB->IDR & GPIO_Pin_12)) {
			if(TOTmr1ms==0)
			return 0;
		}
		return 1;
	} else {	 // wait for WCLK=0
		while (0!=(GPIOB->IDR & GPIO_Pin_12)) {
			if(TOTmr1ms==0)
			return 0;
		}
		return 1;

	}
}
//----------------------------------------------
u8 InitI2S2(void) { // slave receiver
	GPIO_InitTypeDef GPIO_InitStructure;
// NVIC_InitTypeDef   NVIC_InitStructure;
	I2S_InitTypeDef I2S2_InitStructure;
	volatile u32 d;

	RxDMADisable();
	RCC_I2SCLKConfig(RCC_I2S2CLKSource_Ext);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE);
	// GPIO
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource12, GPIO_AF_SPI2);// WCLK
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource13, GPIO_AF_SPI2);// BCLK
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource15, GPIO_AF_SPI2);// SDATA

	GPIO_StructInit(&GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_15;// WCLK-BCLK-SDATA
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_Speed = RCV_Speed;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	//------------------------------------------------
	I2S_StructInit(&I2S2_InitStructure);
	I2S2_InitStructure.I2S_AudioFreq = I2S_AudioFreq_Default;
	I2S2_InitStructure.I2S_CPOL = I2S_CPOL_Low;
	I2S2_InitStructure.I2S_Mode = I2S_Mode_SlaveRx;
	I2S2_InitStructure.I2S_MCLKOutput = I2S_MCLKOutput_Disable;

	I2S2_InitStructure.I2S_Standard = I2S_Standard_MSB;
	I2S2_InitStructure.I2S_DataFormat = I2S_DataFormat_32b;
	if(0==WaitForWCLK(1)) return 0;// return if timeout
	if(0==WaitForWCLK(0)) return 0;// ---//----

	I2S_Init(SPI2, &I2S2_InitStructure);

	I2S2_DMA_Init();
	DMA_Cmd( Rx_DMAStream, ENABLE );

	I2S_Cmd(SPI2, ENABLE);
	d=(uint32_t)SPI3->DR;// to clear RXNE bit
	__ISB();
	return 1;
}
#endif

//*********************************************************************************************************
