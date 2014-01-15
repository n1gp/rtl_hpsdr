/*	This file downsample.c is part of rtl_hpsdr.
 *
 *	rtl_hpsdr - an RTL to HPSDR software translation server
 *	Copyright (C) 2014 Richard Koch
 *
 *	rtl_hpsdr is free software: you can redistribute it and/or modify
 *	it under the terms of the GNU General Public License as published by
 *	the Free Software Foundation, either version 3 of the License, or
 *	(at your option) any later version.
 *
 *	rtl_hpsdr is distributed in the hope that it will be useful,
 *	but WITHOUT ANY WARRANTY; without even the implied warranty of
 *	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *	GNU General Public License for more details.
 *
 *	You should have received a copy of the GNU General Public License
 *	along with rtl_hpsdr.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "rtl_hpsdr.h"

#ifdef INCLUDE_NEON
#define vector_type float32x4_t
#define vector_zero vmovq_n_f32(0)
#define vector_mac(x,y,z) vmlaq_f32((x), vld1q_f32((y)), (z))
#define vector_store(x,y,z) vst1q_f32((x), vaddq_f32((y), vcombine_f32(vget_low_f32((z)), vget_high_f32((y)))))
#elif defined INCLUDE_SSE2
#define vector_type __m128
#define vector_zero _mm_setzero_ps()
#define vector_mac(x,y,z) _mm_add_ps((x), _mm_mul_ps(_mm_load_ps((y)), (z)))
#define vector_store(x,y,z) _mm_store_ps((x), _mm_add_ps((y), _mm_shuffle_ps((y), (y), _MM_SHUFFLE(1,0,3,2))))
#else
#define vector_type float
#endif

void
down_loop(struct rcvr_cb* rcb, int pass) {
	int i, j, k = 0;
	int dsample, count, coeff_len;
	bool IQ_swap;
	float* buf, *out;
	float* pSrc, *orig_buf;
	const vector_type* pFil;
	vector_type sum1;//, sum2;
	struct main_cb* mcb = rcb->mcb;

	if(1 == pass) {
		// 1st pass filter is hb for 768000
		IQ_swap = true;
		dsample = 2;
		count = RTL_READ_COUNT;
		coeff_len = COEFF3072_H_16_LENGTH;
		buf = &(rcb->iq_buf[0]);
		out = &(rcb->iq_buf_final[COEFF1536_H_32_LENGTH * 2]);
	} else {
		IQ_swap = false;
		count = RTL_READ_COUNT / 2;
		coeff_len = mcb->length_fir;
		buf = &(rcb->iq_buf_final[0]);
		out = &(rcb->iqSamples[rcb->iqSamples_remaining * 2]);

		switch(mcb->output_rate) {
		case 48000:
			dsample = DOWNSAMPLE_192 * 2;
			break;

		case 96000:
			dsample = DOWNSAMPLE_192;
			break;

		case 192000:
			dsample = DOWNSAMPLE_192 / 2;
			break;

		case 384000:
			dsample = DOWNSAMPLE_192 / 4;
			break;
		}
	}

	orig_buf = buf;

#if defined(INCLUDE_NEON) || defined(INCLUDE_SSE2)

	// filter is evaluated for two I/Q samples with each iteration, thus use of 'j += 2'
	for(j = 0; j < count / 2; j += 2) {
		pSrc = buf;

		// filter coefficients. NOTE: Assumes coefficients are aligned to 16-byte boundary
		if(1 == pass)
			pFil = (const vector_type*) mcb->align3072_768_H;
		else {
			switch(mcb->output_rate) {
			case 48000:
				pFil = (const vector_type*) mcb->align1536_48_H;
				break;

			case 96000:
				pFil = (const vector_type*) mcb->align1536_96_H;
				break;

			case 192000:
				pFil = (const vector_type*) mcb->align1536_192_H;
				break;

			case 384000:
				pFil = (const vector_type*) mcb->align1536_384_H;
				break;
			}
		}

		sum1 = vector_zero;
//        sum1 = sum2 = vector_zero;

		for(i = 0; i < coeff_len / 8; i++) {
			// Unroll loop for efficiency & calculate filter for 2*2 I/Q samples
			// at each pass

			// sum1 is accu for 2*2 filtered I/Q data at the primary data offset
			// sum2 is accu for 2*2 filtered I/Q data for the next sample offset.
			sum1 = vector_mac(sum1, pSrc, pFil[0]);
//            sum2 = vector_mac(sum2, pSrc+2, pFil[0]);

			sum1 = vector_mac(sum1, pSrc + 4, pFil[1]);
//            sum2 = vector_mac(sum2, pSrc+6, pFil[1]);

			sum1 = vector_mac(sum1, pSrc + 8, pFil[2]);
//            sum2 = vector_mac(sum2, pSrc+10, pFil[2]);

			sum1 = vector_mac(sum1, pSrc + 12, pFil[3]);
//            sum2 = vector_mac(sum2, pSrc+14, pFil[3]);

			pSrc += 16;
			pFil += 4;
		}

		buf += 4;

		// Now sum1 and sum2 both have a filtered 2-channel sample each, but we still need
		// to sum the two hi- and lo-floats of these registers together. Only perform this
		// if we actually need it for the downsampled output data.
		// UPDATE: since we're throwing away sum2, don't bother calculating it.
		if(0 == ((j + 2) % dsample)) {
			// post-shuffle & add the filtered values and store to dest.
			vector_store(rcb->dest, sum1, sum1);
#if 0
			_mm_store_ps(rcb->dest, _mm_add_ps(_mm_shuffle_ps(sum1, sum2, _MM_SHUFFLE(1, 0, 3, 2)),       // s2_1 s2_0 s1_3 s1_2
			                                   _mm_shuffle_ps(sum1, sum2, _MM_SHUFFLE(3, 2, 1, 0))      // s2_3 s2_2 s1_1 s1_0
			                                  ));
#endif

			// Since we're decimating we only need one of the sums
			out[k++] = (IQ_swap) ? rcb->dest[0] : rcb->dest[1];
			out[k++] = (IQ_swap) ? rcb->dest[1] : rcb->dest[0];

#else // non SIMD

	for(j = 0; j < count; j += 2) {

		pSrc = buf;

		if(1 == pass)
			pFil = mcb->coeff_768;
		else {
			switch(mcb->output_rate) {
			case 48000:
				pFil = mcb->coeff_48;
				break;

			case 96000:
				pFil = mcb->coeff_96;
				break;

			case 192000:
				pFil = mcb->coeff_192;
				break;

			case 384000:
				pFil = mcb->coeff_384;
				break;
			}
		}

		sum1 = sum2 = 0.0f;

		for(i = 0; i < coeff_len; i++) {
			sum1 += pSrc[0] * pFil[0];
			sum2 += pSrc[1] * pFil[0];
			pSrc += 2;
			pFil++;
		}

		buf += 2;

		if(0 == (((j + 2) / 2) % dsample)) {
			out[k++] = (IQ_swap) ? sum2 : sum1;
			out[k++] = (IQ_swap) ? sum1 : sum2;
#endif
		}
	}

	// Move the last coeff_len*2 length of buffer to the front for the next call
	memmove(orig_buf, &orig_buf[count - (coeff_len * 2)],
	        coeff_len * 2 * sizeof(float));
}

void
downsample(struct rcvr_cb * rcb) {
	down_loop(rcb, 1);
	down_loop(rcb, 2);
}
