//  ***************************************************************************** 
#ifndef __I2S_H
#define __I2S_H

#include "stm32f4xx.h"
#include "platform_config.h"

#ifdef __cplusplus
 extern "C" {
#endif


//#define I2S_ENABLE_RX	1
#define I2S_ENABLE_TX	1

#pragma pack(1)
typedef union{
	 s16 W16;
	 u8  SH16[2];		// 0 - low half-word, 1 - high
} dmasmp_t;

#pragma pack(1)
typedef struct {
	 dmasmp_t  LEFT;
	 dmasmp_t  RIGHT;
} dmabuf_t;

typedef union{
	u8 SH[2];
	u16 W;
} tSample;

#define DMARCVBUF_SIZE (32)
#define DMATRMBUF_SIZE (32)
extern dmabuf_t DMATRMBUF_0[DMATRMBUF_SIZE];
extern dmabuf_t DMATRMBUF_1[DMATRMBUF_SIZE];

// Transmitter:
void InitI2S3(void); // master transmitter  init

#ifdef I2S_ENABLE_RX
u8 InitI2S2(void);
#endif

#ifdef __cplusplus
}
#endif

#endif
//----
