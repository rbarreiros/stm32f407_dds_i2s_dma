/*
 * synth.c
 *
 *  Created on: 22 Apr 2017
 *      Author: dimtass
 */

#include <math.h>
#include <stdint.h>
#include "platform_config.h"
#include "dds_defs.h"
#include "dds.h"

#define SAMPLE_RATE		DDS_SAMPLE_RATE

#define _2PI            6.283185307f
#define _PI             3.14159265f
#define TABLE_SIZE 		1024						// size of wavetable
#define TABLE_FREQ		(SAMPLE_RATE / TABLE_SIZE)	// frequency of one wavetable period
static float sine[TABLE_SIZE+1];	// +1 for interpolation

// Q16.16
#define FRAC		16
#define Q16(X)		(X * (float)(1<<FRAC))
#define QFLOAT(X)	(X / (float)(1<<FRAC))
#define Q16_TO_UINT(X)	((uint32_t)X / (1<<FRAC))


static uint32_t phaseAccumulator=0; // fixed-point (16.16) phase accumulator


void DDS_sine_init(void)
{
	int i = 0;
	// populate table table[k]=sin(2*pi*k/N)
	for(i = 0; i < TABLE_SIZE; i++) {
		// calculating sine wave
		sine[i] = sinf(_2PI * ((float)i/TABLE_SIZE));
	}
	/* set the last byte equal to first for interpolation */
	sine[TABLE_SIZE] = sine[0];
	phaseAccumulator = 0;
}

void DDS_Init(void)
{
	DDS_sine_init();
}

inline float DDS_calculate_channel_out(uint32_t * phaseAccumulator, uint32_t phaseIncrement, uint32_t index)
{
	/* Increment the phase accumulator */
	*phaseAccumulator += phaseIncrement;
	*phaseAccumulator &= TABLE_SIZE*(1<<16) - 1;
	index = *phaseAccumulator >> 16;

	/* interpolation */
	float v1 = sine[index];
	float v2 = sine[index+1];
	float fmul = (*phaseAccumulator & 65535)/65536.0f;
//		float fmul = fix32_frac(phaseAccumulator, 16);
	float out = v1 + (v2-v1)*fmul;

	out = out * 32768.0f;
	if (out > 32767) out = 32767;
	if (out < -32768) out = -32768;

	return(out);
}


void DDS_calculate(dmabuf_t * buffer, uint16_t buffer_size,
					uint32_t * phaseAccumulator_L, float frequency_L,
					uint32_t * phaseAccumulator_R, float frequency_R)
{
	uint32_t index_L = 0;
	uint32_t index_R = 0;

	uint32_t phaseIncrement_L = Q16((float)frequency_L*TABLE_SIZE / SAMPLE_RATE);
	uint32_t phaseIncrement_R = Q16((float)frequency_R*TABLE_SIZE / SAMPLE_RATE);
	int i = 0;

	for(i=0; i<buffer_size; i++)
    {
		/* Left channel */
		buffer[i].LEFT.W16 = (s16) DDS_calculate_channel_out(phaseAccumulator_L, phaseIncrement_L, index_L);
		/* Right channel */
		buffer[i].RIGHT.W16 = (s16) DDS_calculate_channel_out(phaseAccumulator_R, phaseIncrement_R, index_R);
    }
}
