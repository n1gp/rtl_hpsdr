/*  XMMS - ALSA output plugin
 *    Copyright (C) 2001 Matthieu Sozeau
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.
 */

/*	This file local_sound.c is part of rtl_hpsdr.
 *
 *	rtl_hpsdr - an RTL to HPSDR software translation server
 *	Copyright (C) 2014 Richard Koch
 *
 *	This code module was derived in great part from the
 *	XMMS - ALSA output plugin as noted above.
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

#include <alsa/asoundlib.h>
#include <alsa/pcm_plugin.h>
#include <ctype.h>
#include <pthread.h>
#include <math.h>
#include <sys/types.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdbool.h>
#include <sys/timeb.h>

#define MIN( a, b ) ( a ) < ( b ) ? ( a ) : ( b )

struct alsa_control_block {
	char pcm_device[64];
	int buffer_time;
	int period_time;
	int thread_buffer_time;
	u_char buffer[8192];
} alsa_cb;

static snd_pcm_t* alsa_pcm;

/* Number of bytes that we have received from the input plugin */
static u_int alsa_total_written;

/* Number of bytes that we have sent to the sound card */
static u_int alsa_hw_written;
static u_int output_time_offset;

/* device buffer/period sizes in bytes */
static int hw_buffer_size, hw_period_size;  /* in output bytes */
static int hw_buffer_size_in, hw_period_size_in;    /* in input bytes */

static bool going;
static bool prebuffer, remove_prebuffer;

/* for audio thread */
static pthread_t audio_thread;  /* audio loop thread */
static int thread_buffer_size;  /* size of intermediate buffer in bytes */
static char* thread_buffer; /* audio intermediate buffer */
static int rd_index, wr_index;  /* current read/write position in int-buffer */
static int flush_request;   /* flush status (time) currently requested */
static int prebuffer_size;

struct snd_format {
	unsigned int rate;
	unsigned int channels;
	snd_pcm_format_t format;
	int sample_bits;
	int bps;
};

static struct snd_format* inputf = NULL;
static struct snd_format* outputf = NULL;

static int alsa_setup(struct snd_format* f);
static void alsa_write_audio(char* data, int length);
static int get_thread_buffer_filled(void);

void
alsa_init(char* adevice) {
	memset(&alsa_cb, 0, sizeof(alsa_cb));
	alsa_cb.buffer_time = 500;
	alsa_cb.period_time = 50;
	alsa_cb.thread_buffer_time = 1000;
	strcpy(alsa_cb.pcm_device, adevice);
}

static struct snd_format*
snd_format_alloc(int rate, int channels) {
	struct snd_format* f = malloc(sizeof(struct snd_format));

	f->format = SND_PCM_FORMAT_S16_BE;
	f->rate = rate;
	f->channels = channels;
	f->sample_bits = snd_pcm_format_physical_width(f->format);
	f->bps = (rate * f->sample_bits * channels) >> 3;

	return f;
}

int
alsa_playing(void) {
	if(!going || alsa_pcm == NULL)
		return false;

	return snd_pcm_state(alsa_pcm) == SND_PCM_STATE_RUNNING;
}

static int
xrun_recover(void) {
	return snd_pcm_prepare(alsa_pcm);
}

static int
suspend_recover(void) {
	int err;

	while((err = snd_pcm_resume(alsa_pcm)) == -EAGAIN)
		/* wait until suspend flag is released */
		sleep(1);

	if(err < 0) {
		printf("alsa_handle_error(): " "snd_pcm_resume() failed.\n");
		return snd_pcm_prepare(alsa_pcm);
	}

	return err;
}

/* handle generic errors */
static int
alsa_handle_error(int err) {
	switch(err) {
	case -EPIPE:
		//printf("local_sound: -EPIPE\n");
		return xrun_recover();

	case -ESTRPIPE:
		//printf("local_sound: -ESTRPIPE\n");
		return suspend_recover();
	}

	return err;
}

/* update and get the available space on h/w buffer (in frames) */
static snd_pcm_sframes_t
alsa_get_avail(void) {
	snd_pcm_sframes_t ret;

	if(alsa_pcm == NULL)
		return 0;

	while((ret = snd_pcm_avail_update(alsa_pcm)) < 0) {
		ret = alsa_handle_error(ret);

		if(ret < 0) {
			printf("alsa_get_avail(): snd_pcm_avail_update() failed: %s\n",
			       snd_strerror(-ret));
			return 0;
		}
	}

	return ret;
}

/* get the free space on buffer */
int
alsa_free(void) {
	if(remove_prebuffer && prebuffer) {
		prebuffer = false;
		remove_prebuffer = false;
	}

	if(prebuffer)
		remove_prebuffer = true;

	return thread_buffer_size - get_thread_buffer_filled() - 1;
}

/* close PCM and release associated resources */
static void
alsa_close_pcm(void) {
	if(alsa_pcm) {
		int err;
		snd_pcm_drop(alsa_pcm);

		if((err = snd_pcm_close(alsa_pcm)) < 0)
			printf("alsa_pcm_close() failed: %s\n", snd_strerror(-err));

		alsa_pcm = NULL;
	}
}

void
alsa_close(void) {
	if(!going)
		return;

	going = 0;

	pthread_join(audio_thread, NULL);

	free(inputf);
	inputf = NULL;
	free(outputf);
	outputf = NULL;
}

/* reopen ALSA PCM */
static int
alsa_reopen(struct snd_format* f) {
	/* remember the current position */
	output_time_offset += (alsa_hw_written * 1000) / outputf->bps;
	alsa_hw_written = 0;

	alsa_close_pcm();

	return alsa_setup(f);
}

/* do flush (drop) operation */
static void
alsa_do_flush(int time) {
	if(alsa_pcm) {
		snd_pcm_drop(alsa_pcm);
		snd_pcm_prepare(alsa_pcm);
	}

	/* correct the offset */
	output_time_offset = time;
	alsa_total_written = (uint) time * inputf->bps / 1000;
	rd_index = wr_index = alsa_hw_written = 0;
}

void
alsa_flush(int time) {
	flush_request = time;

	while(flush_request != -1)
		usleep(10000);
}

/*
 * audio stuff
 */

/* return the size of audio data filled in the audio thread buffer */
static int
get_thread_buffer_filled(void) {
	if(wr_index >= rd_index)
		return wr_index - rd_index;

	return thread_buffer_size - (rd_index - wr_index);
}

int
alsa_get_output_time(void) {
	snd_pcm_sframes_t delay;
	u_int bytes = alsa_hw_written;

	if(!going || alsa_pcm == NULL)
		return 0;

	if(!snd_pcm_delay(alsa_pcm, &delay)) {
		unsigned int d = snd_pcm_frames_to_bytes(alsa_pcm, delay);

		if(bytes < d)
			bytes = 0;
		else
			bytes -= d;
	}

	return output_time_offset + (bytes * 1000) / outputf->bps;
}

int
alsa_get_written_time(void) {
	if(!going)
		return 0;

	return (alsa_total_written * 1000) / inputf->bps;
}

/* transfer data to audio h/w; length is given in bytes
 *
 */

/* write callback */
void
alsa_write(void* data, int length) {
	int cnt;
	char* src = (char*) data;

	remove_prebuffer = false;

	alsa_total_written += length;

	while(length > 0) {
		int wr;
		cnt = MIN(length, thread_buffer_size - wr_index);
		memcpy(thread_buffer + wr_index, src, cnt);
		wr = (wr_index + cnt) % thread_buffer_size;
		wr_index = wr;
		length -= cnt;
		src += cnt;
	}
}

/* transfer data to audio h/w via normal write */
static void
alsa_write_audio(char* data, int length) {
	snd_pcm_sframes_t written_frames;

	while(length > 0) {
		int frames = snd_pcm_bytes_to_frames(alsa_pcm, length);
		written_frames = snd_pcm_writei(alsa_pcm, data, frames);

		if(written_frames > 0) {
			int written = snd_pcm_frames_to_bytes(alsa_pcm,
			                                      written_frames);
			length -= written;
			data += written;
			alsa_hw_written += written;
		} else {
			int err = alsa_handle_error((int) written_frames);

			if(err < 0) {
				printf("alsa_write_audio(): write error: %s\n",
				       snd_strerror(-err));
				break;
			}
		}
	}
}

/* transfer audio data from thread buffer to h/w */
static void
alsa_write_out_thread_data(void) {
	int length, cnt, avail;

	length = MIN(hw_period_size_in, get_thread_buffer_filled());
	avail = snd_pcm_frames_to_bytes(alsa_pcm, alsa_get_avail());
	length = MIN(length, avail);

	while(length > 0) {
		int rd;
		cnt = MIN(length, thread_buffer_size - rd_index);
		alsa_write_audio(thread_buffer + rd_index, cnt);
		rd = (rd_index + cnt) % thread_buffer_size;
		rd_index = rd;
		length -= cnt;
	}
}

/* audio thread loop */
/* FIXME: proper lock? */
static void*
alsa_loop(void* arg) {
	int npfds = snd_pcm_poll_descriptors_count(alsa_pcm);
	struct pollfd* pfds;
	unsigned short* revents;

	if(npfds <= 0)
		goto _error;

	//printf("Starting alsa_loop thread...\n");

	pfds = alloca(sizeof(*pfds) * npfds);
	revents = alloca(sizeof(*revents) * npfds);

	while(going && alsa_pcm) {
		if(get_thread_buffer_filled() > prebuffer_size)
			prebuffer = false;

		if(!prebuffer && get_thread_buffer_filled() > hw_period_size_in) {
			snd_pcm_poll_descriptors(alsa_pcm, pfds, npfds);

			if(poll(pfds, npfds, 10) > 0) {
				/*
				 * need to check revents.  poll() with
				 * dmix returns a postive value even
				 * if no data is available
				 */
				int i;
				snd_pcm_poll_descriptors_revents(alsa_pcm, pfds,
				                                 npfds, revents);

				for(i = 0; i < npfds; i++)
					if(revents[i] & POLLOUT) {
						alsa_write_out_thread_data();
						break;
					}
			}
		} else
			usleep(10000);

		if(flush_request != -1) {
			alsa_do_flush(flush_request);
			flush_request = -1;
			prebuffer = true;
		}
	}

_error:
	alsa_close_pcm();
	free(thread_buffer);
	thread_buffer = NULL;
	pthread_exit(NULL);
}

/* open callback */
int
alsa_open(int rate, int nch, int width) {
	inputf = snd_format_alloc(rate, nch);

	if(alsa_setup(inputf) < 0) {
		alsa_close();
		return 0;
	}

	output_time_offset = 0;
	alsa_total_written = alsa_hw_written = 0;
	going = true;
	prebuffer = true;
	remove_prebuffer = false;

	thread_buffer_size =
	  (u_int) alsa_cb.thread_buffer_time * inputf->bps / 1000;

	if(thread_buffer_size < hw_buffer_size)
		thread_buffer_size = hw_buffer_size * 2;

	if(thread_buffer_size < width)
		thread_buffer_size = width;

	prebuffer_size = thread_buffer_size / 2;

	if(prebuffer_size < width)
		prebuffer_size = width;

	thread_buffer_size += hw_buffer_size;
	thread_buffer_size -= thread_buffer_size % hw_period_size;
	thread_buffer = calloc(1, thread_buffer_size);
	wr_index = rd_index = 0;
	flush_request = -1;

	pthread_create(&audio_thread, NULL, alsa_loop, NULL);
	return 1;
}

static int
alsa_setup(struct snd_format* f) {
	int err;
	snd_pcm_hw_params_t* hwparams;
	snd_pcm_sw_params_t* swparams;
	unsigned int alsa_buffer_time, alsa_period_time;
	snd_pcm_uframes_t alsa_buffer_size, alsa_period_size;

	free(outputf);
	outputf = snd_format_alloc(f->rate, f->channels);

	printf("  Opening device %s ... ", alsa_cb.pcm_device);

	if((err = snd_pcm_open(&alsa_pcm, alsa_cb.pcm_device,
	                       SND_PCM_STREAM_PLAYBACK, SND_PCM_NONBLOCK)) < 0) {
		printf("alsa_setup(): Failed to open pcm device (%s): %s\n",
		       alsa_cb.pcm_device, snd_strerror(-err));
		alsa_pcm = NULL;
		free(outputf);
		outputf = NULL;
		return -1;
	}

	/* doesn't care about non-blocking */
	/* snd_pcm_nonblock(alsa_pcm, 0); */

	if(0) {           //debug
		snd_pcm_info_t* info;
		int alsa_card, alsa_device, alsa_subdevice;

		snd_pcm_info_alloca(&info);
		snd_pcm_info(alsa_pcm, info);
		alsa_card = snd_pcm_info_get_card(info);
		alsa_device = snd_pcm_info_get_device(info);
		alsa_subdevice = snd_pcm_info_get_subdevice(info);
		printf("Card %i, Device %i, Subdevice %i\n",
		       alsa_card, alsa_device, alsa_subdevice);
	}

	snd_pcm_hw_params_alloca(&hwparams);

	if((err = snd_pcm_hw_params_any(alsa_pcm, hwparams)) < 0) {
		printf("alsa_setup(): No configuration available for "
		       "playback: %s\n", snd_strerror(-err));
		return -1;
	}

	if((err = snd_pcm_hw_params_set_access(alsa_pcm, hwparams,
	                                       SND_PCM_ACCESS_RW_INTERLEAVED)) <
	    0) {
		printf("alsa_setup(): Cannot set direct write mode: %s\n",
		       snd_strerror(-err));
		return -1;
	}

	if((err =
	      snd_pcm_hw_params_set_format(alsa_pcm, hwparams,
	                                   outputf->format)) < 0) {
		printf("alsa_setup(): Sample format not "
		       "available for playback: %s\n", snd_strerror(-err));
		return -1;
	}

	if((err = snd_pcm_hw_params_set_channels_near(alsa_pcm, hwparams,
	                                              &outputf->channels)) < 0) {
		printf
		("alsa_setup(): snd_pcm_hw_params_set_channels_near failed: %s.\n",
		 snd_strerror(-err));
		return -1;
	}

	snd_pcm_hw_params_set_rate_near(alsa_pcm, hwparams, &outputf->rate, 0);

	if(outputf->rate == 0) {
		printf("alsa_setup(): No usable samplerate available.\n");
		return -1;
	}

	outputf->sample_bits = snd_pcm_format_physical_width(outputf->format);
	outputf->bps =
	  (outputf->rate * outputf->sample_bits * outputf->channels) >> 3;

	alsa_buffer_time = alsa_cb.buffer_time * 1000;

	if((err = snd_pcm_hw_params_set_buffer_time_near(alsa_pcm, hwparams,
	                                                 &alsa_buffer_time,
	                                                 0)) < 0) {
		printf("alsa_setup(): Set buffer time failed: %s.\n",
		       snd_strerror(-err));
		return -1;
	}

	alsa_period_time = alsa_cb.period_time * 1000;

	if((err = snd_pcm_hw_params_set_period_time_near(alsa_pcm, hwparams,
	                                                 &alsa_period_time,
	                                                 0)) < 0) {
		printf("alsa_setup(): Set period time failed: %s.\n",
		       snd_strerror(-err));
		return -1;
	}

	if(snd_pcm_hw_params(alsa_pcm, hwparams) < 0) {
		printf("alsa_setup(): Unable to install hw params\n");
		return -1;
	}

	if((err =
	      snd_pcm_hw_params_get_buffer_size(hwparams, &alsa_buffer_size)) < 0) {
		printf("alsa_setup(): snd_pcm_hw_params_get_buffer_size() "
		       "failed: %s\n", snd_strerror(-err));
		return -1;
	}

	if((err =
	      snd_pcm_hw_params_get_period_size(hwparams, &alsa_period_size,
	                                        0)) < 0) {
		printf("alsa_setup(): snd_pcm_hw_params_get_period_size() "
		       "failed: %s\n", snd_strerror(-err));
		return -1;
	}

	snd_pcm_sw_params_alloca(&swparams);
	snd_pcm_sw_params_current(alsa_pcm, swparams);

	if((err = snd_pcm_sw_params_set_start_threshold(alsa_pcm,
	                                                swparams,
	                                                alsa_buffer_size -
	                                                alsa_period_size) < 0))
		printf("alsa_setup(): setting start " "threshold failed: %s\n",
		       snd_strerror(-err));

	if(snd_pcm_sw_params(alsa_pcm, swparams) < 0) {
		printf("alsa_setup(): Unable to install sw params\n");
		return -1;
	}

	hw_buffer_size = snd_pcm_frames_to_bytes(alsa_pcm, alsa_buffer_size);
	hw_period_size = snd_pcm_frames_to_bytes(alsa_pcm, alsa_period_size);

	if(inputf->bps != outputf->bps) {
		int align = (inputf->sample_bits * inputf->channels) / 8;
		hw_buffer_size_in = ((u_int) hw_buffer_size * inputf->bps +
		                     outputf->bps / 2) / outputf->bps;
		hw_period_size_in = ((u_int) hw_period_size * inputf->bps +
		                     outputf->bps / 2) / outputf->bps;
		hw_buffer_size_in -= hw_buffer_size_in % align;
		hw_period_size_in -= hw_period_size_in % align;
	} else {
		hw_buffer_size_in = hw_buffer_size;
		hw_period_size_in = hw_period_size;
	}

#if 0
	printf("Device setup: buffer time: %i, size: %i.\n", alsa_buffer_time,
	       hw_buffer_size);
	printf("Device setup: period time: %i, size: %i.\n", alsa_period_time,
	       hw_period_size);
	printf("bits per sample: %i; frame size: %i; Bps: %i\n",
	       snd_pcm_format_physical_width(outputf->format),
	       snd_pcm_frames_to_bytes(alsa_pcm, 1), outputf->bps);
#endif
	printf("success.\n");
	return 0;
}


int
write_local_sound(unsigned char* samples) {
	int i;
	static bool do_once = true;
	static int in_count = 0;
	int insert = 0;

  if (do_once)
	{
		if (in_count++ > alsa_cb.thread_buffer_time)
			do_once = false;
			if (in_count > alsa_cb.thread_buffer_time / 2
				&& in_count < alsa_cb.thread_buffer_time)
			return 0;
	}

	for(i = 8; i < 512; i += 8) {
		//bytes are L,R,I,Q skip the I,Q
		alsa_cb.buffer[insert++] = samples[i];     //left
		alsa_cb.buffer[insert++] = samples[i + 1];
		alsa_cb.buffer[insert++] = samples[i + 2]; //right
		alsa_cb.buffer[insert++] = samples[i + 3];
	}

	for(i = 520; i < 1024; i += 8) {
		alsa_cb.buffer[insert++] = samples[i];
		alsa_cb.buffer[insert++] = samples[i + 1];
		alsa_cb.buffer[insert++] = samples[i + 2];
		alsa_cb.buffer[insert++] = samples[i + 3];
	}

	alsa_write(alsa_cb.buffer, 504);

#if 0 // test the audio sample rate
	{
  struct timeb start_time;
  struct timeb end_time;
	static int sample_count = 0;
	static float rate = 0.0;
	static float rate_count = 0.0;

		sample_count+=504;
		if(sample_count >= 48000) {

			ftime(&end_time);

			// skip the 1st one, it's bogus
			if (0.0 == rate_count) {
				rate_count = 1;
				sample_count = 0;
				ftime(&start_time);
				return 0;
			}

			rate += (float)((sample_count*1000/4)
				/(((end_time.time*1000)+end_time.millitm)-
				((start_time.time*1000)+start_time.millitm)));
			printf("sample rate avg: %.1f/sec\n", rate / rate_count);
			sample_count=0;
			rate_count += 1.0;
			ftime(&start_time);
		}
	}
#endif

	return 0;
}

void
close_local_sound() {
	alsa_close();
}

void
reopen_local_sound() {
	xrun_recover();
	suspend_recover();
	alsa_reopen(inputf);
}

void
open_local_sound(char* adevice) {
	alsa_init(adevice);
	alsa_open(48000, 2, 8192);
}
