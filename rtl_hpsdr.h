#ifndef _RTL_HPSDR_H
#define _RTL_HPSDR_H

/*	This file rtl_hpsdr.h is part of rtl_hpsdr.
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

#include <stdlib.h>
#include <stdio.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <sys/timeb.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <netdb.h>
#include <net/if_arp.h>
#include <net/if.h>
#include <ifaddrs.h>
#include <pthread.h>
#include <stdbool.h>
#include <semaphore.h>
#include <time.h>
#include <signal.h>
#include <unistd.h>
#include <math.h>
#include <limits.h>
#include <string.h>
#include <libgen.h>
#include <errno.h>
#include <rtl-sdr.h>
#ifdef INCLUDE_NEON
#include <arm_neon.h>
#elif defined INCLUDE_SSE2
#include <xmmintrin.h>
#endif

#define PRG_VERSION "1.1" // see ChangeLog for history

#define HERMES_FW_VER 24    //2.4
#define PORT 1024
#define REVEAL_SEND_PORT PORT
#define DATA_PORT PORT
#define MAX_BUFFER_LEN 2048
#define HPSDR_FRAME_LEN 1032
#define IQ_FRAME_DATA_LEN 63
#define DOWNSAMPLE_192 8    // downsample value used to get 192khz
#define RTL_SAMPLE_RATE (192000 * DOWNSAMPLE_192)
#define RTL_READ_COUNT (2048 * DOWNSAMPLE_192)
#define MAX_RCVRS 7     // current limit imposed by cuSDR64
#define IQ_FRAME_DATA_LEN 63
#define MAXSTR 64

#define COEFF3072_H_16_LENGTH 16
#define COEFF1536_H_32_LENGTH 32
#define COEFF1536_H_64_LENGTH 64
float coeff3072_768_H_16[COEFF3072_H_16_LENGTH];
float coeff1536_384_H_32[COEFF1536_H_32_LENGTH];
float coeff1536_192_H_32[COEFF1536_H_32_LENGTH];
float coeff1536_96_H_32[COEFF1536_H_32_LENGTH];
float coeff1536_48_H_32[COEFF1536_H_32_LENGTH];
float coeff1536_384_H_64[COEFF1536_H_64_LENGTH];
float coeff1536_192_H_64[COEFF1536_H_64_LENGTH];
float coeff1536_96_H_64[COEFF1536_H_64_LENGTH];
float coeff1536_48_H_64[COEFF1536_H_64_LENGTH];

struct main_cb {
	int total_num_rcvrs;
	int active_num_rcvrs;
	u_int rcvrs_mask;
	int nsamps_packet;
	int frame_offset1;
	int frame_offset2;
	int output_rate;
	int length_fir;
	char sound_dev[32];
	char net_dev[32];

	// the last array member is used to remember last settings
	int agc_mode[MAX_RCVRS + 1];
	int direct_mode[MAX_RCVRS + 1];
	int gain[MAX_RCVRS + 1];
	int freq_offset[MAX_RCVRS + 1];
	int rcvr_order[MAX_RCVRS + 1];
	int signal_multiplier[MAX_RCVRS + 1];

	struct timeb freq_ltime[MAX_RCVRS];
	struct timeb freq_ttime[MAX_RCVRS];

	// filter coefficient table pointers
	float* align1536_48_H;
	float* align1536_96_H;
	float* align1536_192_H;
	float* align1536_384_H;
	float* align3072_768_H;
	float* coeff_48;
	float* coeff_96;
	float* coeff_192;
	float* coeff_384;
	float* coeff_768;

	struct rcvr_cb {
		float dest[4] __attribute__((aligned(16)));

		int rcvr_num;
		int freq;
		u_int rcvr_mask;
		rtlsdr_dev_t* rtldev;
		struct main_cb* mcb;
		pthread_t hpsdrsim_sendiq_thr;
		pthread_t rtl_read_thr;

		int iqSample_offset;
		int iqSamples_remaining;
		float iqSamples[(RTL_READ_COUNT + (IQ_FRAME_DATA_LEN * 2))];

		u_char rtl_buf[RTL_READ_COUNT];
		float* iq_buf;
		float* iq_buf_final;
	} rcb[MAX_RCVRS];
};

void downsample(struct rcvr_cb* rcb);
void format_payload(void);
int init_rtl(int rcvr_num, int dev_index);
void load_packet(struct rcvr_cb* rcb);
void rtl_sighandler(int signum);
int get_addr(int sock, char* ifname);
void hpsdrsim_reveal(void);

pthread_t hpsdrsim_thread_id;
void* hpsdrsim_thread(void* arg);
pthread_t watchdog_thread_id;
void* hpsdrsim_watchdog_thread(void* arg);
void* hpsdrsim_sendiq_thr_func(void* arg);
void* rtl_read_thr_func(void* arg);
void hpsdrsim_stop_threads();

void open_local_sound(char* adevice);
void close_local_sound();
void write_local_sound(unsigned char* samples);
void reopen_local_sound();

#endif // _RTL_HPSDR_H
