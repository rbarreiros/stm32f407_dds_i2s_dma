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

void DDS_calculate(dmabuf_t * buffer, uint16_t buffer_size, float frequency)
{
	uint32_t phaseIncrement = Q16((float)frequency*TABLE_SIZE / SAMPLE_RATE);
	uint32_t index = 0;
	int i = 0;

	for(i=0; i<buffer_size; i++)
    {
//		phaseAccumulator += 1;
//		buffer[i].LEFT.W16 = -32768; //phaseAccumulator & 0x3FFF; //65535;
//		buffer[i].RIGHT.W16 = 32767; //phaseAccumulator & 1023; //65535;

		/* Increment the phase accumulator */
		phaseAccumulator += phaseIncrement;

		phaseAccumulator &= TABLE_SIZE*(1<<16) - 1;

		index = phaseAccumulator >> 16;

		/* interpolation */
		float v1 = sine[index];
		float v2 = sine[index+1];
		float fmul = (phaseAccumulator & 65535)/65536.0f;
//		float fmul = fix32_frac(phaseAccumulator, 16);
		float out = v1 + (v2-v1)*fmul;

		out = out * 32768.0f;
		if (out > 32767) out = 32767;
		if (out < -32768) out = -32768;

//		uint16_t out_i = out;

//		buffer[i].LEFT.W16 = (uint16_t)(65536 * out);
//		buffer[i].RIGHT.W16 = (uint16_t)(65536 * out);uint16_t out_i = out;

//		buffer[i].LEFT.W16 = ((out_i & 0xFF00) >> 8) | ((out_i & 0xFF) << 8);
//		buffer[i].LEFT.W16 = ((out_i & 0xFF00) >> 8) | ((out_i & 0xFF) << 8);

		buffer[i].LEFT.W16 = (s16) out;
		buffer[i].RIGHT.W16 = (s16) out;
    }
}
