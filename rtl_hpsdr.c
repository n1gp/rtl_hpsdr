/*	This file rtl_hpsdr.c is part of rtl_hpsdr.
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

struct main_cb mcb;

static pthread_mutex_t iqready_lock = PTHREAD_MUTEX_INITIALIZER;
static pthread_cond_t iqready_cond = PTHREAD_COND_INITIALIZER;
static u_int rcvr_flags = 0;
static pthread_mutex_t send_lock = PTHREAD_MUTEX_INITIALIZER;
static pthread_cond_t send_cond = PTHREAD_COND_INITIALIZER;
static u_int send_flags = 0;
static pthread_mutex_t done_send_lock = PTHREAD_MUTEX_INITIALIZER;
static pthread_cond_t done_send_cond = PTHREAD_COND_INITIALIZER;

static pthread_t rtl_read_thr[MAX_RCVRS];
static pthread_t hpsdrsim_sendiq_thr[MAX_RCVRS];

static int num_copy_rcvrs = 0, do_exit = 0;
static int copy_rcvr[MAX_RCVRS];
static u_char last_num_rcvrs = 0;
static u_char last_rate = 0;
static int last_freq[MAX_RCVRS] = { 0, };

static int common_freq = 0;

static int reveal_socket;
static struct sockaddr_in my_addr;
static socklen_t my_length;
static struct sockaddr_in their_addr;
static socklen_t their_length;

static int nsamps_packet[8] = { 126, 72, 50, 38, 30, 26, 22, 20 };
static int frame_offset1[8] = { 520, 520, 516, 510, 496, 510, 500, 516 };
static int frame_offset2[8] =
{ 1032, 1032, 1028, 1022, 1008, 1022, 1012, 1028 };

static int revealed = 0;
static int running = 0;

static unsigned char hw_address[6];
static long ip_address;
static char server_ip_address[16];

static u_char buffer[MAX_BUFFER_LEN];
static u_char payload[HPSDR_FRAME_LEN];

static u_int hpsdr_sequence = 0;

static u_int pc_sequence;

static float rtl_lut[256];

void
rtl_sighandler(int signum) {
	printf("Signal caught, exiting!\n");
	do_exit = 1;
	hpsdrsim_stop_threads();
}

#define inaddrr(x) (*(struct in_addr *) &ifr->x[sizeof sa.sin_port])

int
get_addr(int sock, char* ifname) {

	struct ifreq* ifr;
	struct ifreq ifrr;
	struct sockaddr_in sa;
	unsigned char* u;
	int i;

	ifr = &ifrr;
	ifrr.ifr_addr.sa_family = AF_INET;
	strncpy(ifrr.ifr_name, ifname, sizeof(ifrr.ifr_name));

	if(ioctl(sock, SIOCGIFADDR, ifr) < 0) {
		printf("No %s interface.\n", ifname);
		return -1;
	}

	ip_address = inaddrr(ifr_addr.sa_data).s_addr;

	if(ioctl(sock, SIOCGIFHWADDR, ifr) < 0) {
		printf("No %s interface.\n", ifname);
		return -1;
	}

	u = (unsigned char*) &ifr->ifr_addr.sa_data;

	for(i = 0; i < 6; i++)
		hw_address[i] = u[i];

	return 0;
}

void
hpsdrsim_reveal(void) {
	int rc, bytes_read;
	int i, on = 1;
	u_char init_buffer[12] =
	{ 0xEF, 0xFE, 2 + running, 0, 0, 0, 0, 0, 0, HERMES_FW_VER, 1, 1 };
	char s_ip[16];

	printf("Revealing myself as a Hermes rcvr.\n");

	reveal_socket = socket(PF_INET, SOCK_DGRAM, IPPROTO_UDP);

	if(reveal_socket < 0) {
		perror("create socket failed for reveal_socket\n");
		exit(1);
	}

	// get my MAC address and IP address
	if(get_addr(reveal_socket, mcb.net_dev) < 0) {
		exit(1);
	}

	sprintf(s_ip, "%ld.%ld.%ld.%ld",
	        ip_address & 0xFF,
	        (ip_address >> 8) & 0xFF,
	        (ip_address >> 16) & 0xFF, (ip_address >> 24) & 0xFF);

	printf("%s IP Address: %s\n", mcb.net_dev, s_ip);

	printf("%s MAC Address: %02x:%02x:%02x:%02x:%02x:%02x\n",
	       mcb.net_dev,
	       hw_address[0], hw_address[1], hw_address[2], hw_address[3],
	       hw_address[4], hw_address[5]);

	for(i = 0; i < 6; i++)
		init_buffer[3 + i] = hw_address[i];

	my_addr.sin_family = AF_INET;
	my_addr.sin_port = htons(REVEAL_SEND_PORT);
	my_addr.sin_addr.s_addr = htonl(INADDR_BROADCAST);
	memset(&(my_addr.sin_zero), '\0', 8);

	rc = setsockopt(reveal_socket, SOL_SOCKET, SO_REUSEADDR, &on, sizeof(on));

	if(rc != 0) {
		printf("cannot set SO_REUSEADDR: rc=%d\n", rc);
		exit(1);
	}

	if(bind(reveal_socket, (struct sockaddr*) &my_addr, sizeof(my_addr)) < 0) {
		perror("bind socket failed for reveal_socket");
		exit(1);
	}

#if 0
	// allow broadcast on the socket
	rc = setsockopt(reveal_socket, SOL_SOCKET, SO_BROADCAST, &on, sizeof(on));

	if(rc != 0) {
		printf("cannot set SO_BROADCAST: rc=%d\n", rc);
		exit(1);
	}

#endif

	their_length = sizeof(their_addr);

	// Get discovered
	while(!revealed) {
		if((bytes_read = recvfrom(reveal_socket, buffer, sizeof(buffer), 0,
		                          (struct sockaddr*) &their_addr, &their_length)) < 0) {
			if(!do_exit)
				printf("Bad recvfrom, NOT discovered!");

			exit(0);
		} else
			printf("Waiting to be discovered...\n");

		if(buffer[0] == 0xEF && buffer[1] == 0xFE && buffer[2] == 0x02) {
			revealed++;
			strcpy(server_ip_address, inet_ntoa(their_addr.sin_addr));
			printf("Was discovered by %s\n", server_ip_address);

			// Send acknowledgement of discovery
			for(i = 0; i < 64; i++) {
				buffer[i] = 1;
			}

			for(i = 0; i < sizeof(init_buffer); i++) {
				buffer[i] = init_buffer[i];
			}

			if(sendto(reveal_socket, buffer, 60, 0, (struct sockaddr*) &their_addr,
			          sizeof(their_addr)) < 0) {
				perror("sendto() failed sending acknowledgement!");
				exit(1);
			} else
				printf("Sent discovery acknowledgement.\n");
		} else {
			printf("Was NOT discovered by %s\n", inet_ntoa(their_addr.sin_addr));
		}
	}

	if((reveal_socket = socket(PF_INET, SOCK_DGRAM, IPPROTO_UDP)) == -1) {
		perror("socket() reveal_socket error!");
		exit(1);
	}

	my_length = sizeof(my_addr);
	memset(&my_addr, 0, my_length);
	my_addr.sin_family = AF_INET;
	my_addr.sin_port = htons(DATA_PORT);

	rc = setsockopt(reveal_socket, SOL_SOCKET, SO_REUSEADDR, &on, sizeof(on));

	if(rc != 0) {
		printf("cannot set SO_REUSEADDR: rc=%d\n", rc);
		exit(1);
	}

	if(bind(reveal_socket, (struct sockaddr*) &my_addr, my_length) < 0) {
		perror("bind socket failed for reveal_socket");
		exit(1);
	}

	rc = pthread_create(&hpsdrsim_thread_id, NULL, hpsdrsim_thread, NULL);

	if(rc != 0) {
		printf("pthread_create failed on hpsdr_thread: rc=%d\n", rc);
		exit(1);
	}
}

void
load_packet(struct rcvr_cb* rcb) {
	int b, i, j, k, copy_total = (mcb.active_num_rcvrs - 1) + num_copy_rcvrs;
	// if we need to copy a receiver we'll choose the last active 'real' one
	bool do_copy = ((num_copy_rcvrs > 0)
	                && (rcb->rcvr_num == mcb.active_num_rcvrs - 1));
	int offset1 = (num_copy_rcvrs > 0) ? frame_offset1[copy_total] : mcb.frame_offset1;
	int offset2 = (num_copy_rcvrs > 0) ? frame_offset2[copy_total] : mcb.frame_offset2;
	int offsetx;
	float* out_buf = &rcb->iqSamples[rcb->iqSample_offset * 2];
	short IQdata;

	i = 0;
	k = 0;
	b = 16;

	// insert data in lower and upper bank for each of the receivers
	while(k++ < 2) {
		offsetx = (i == 0) ? offset1 : offset2;
		while(b < offsetx) {
			for(j = 0; j < mcb.active_num_rcvrs + num_copy_rcvrs; j++) {
				if((j == rcb->rcvr_num) || (do_copy && copy_rcvr[j] == j)) {
					IQdata = (short) out_buf[i];
					payload[b++] = IQdata >> 8;
					payload[b++] = IQdata & 0xff;
					payload[b++] = 0;
					IQdata = (short) out_buf[i + 1];
					payload[b++] = IQdata >> 8;
					payload[b++] = IQdata & 0xff;
					payload[b++] = 0;

					if(do_copy) {
						if(j == copy_total)
							i += 2;
					} else {
						i += 2;
					}
				} else {
					b += 6;
				}
			}
			b += 2;   // skip mic data
		}
		b = 528;
	}

	pthread_mutex_lock(&send_lock);
	send_flags |= rcb->rcvr_mask;
	pthread_cond_broadcast(&send_cond);
	pthread_mutex_unlock(&send_lock);

	pthread_mutex_lock(&done_send_lock);

	while(send_flags & rcb->rcvr_mask) {
		pthread_cond_wait(&done_send_cond, &done_send_lock);
	}

	pthread_mutex_unlock(&done_send_lock);
}

void*
hpsdrsim_sendiq_thr_func(void* arg) {
	int samps_packet;
	struct rcvr_cb* rcb = (struct rcvr_cb*) arg;
	rcb->iqSample_offset = rcb->iqSamples_remaining = 0;

	//printf("ENTERING hpsdrsim_sendiq_thr_func() rcvr %d...\n", rcb->rcvr_num+1);
	while(!do_exit) {
		samps_packet = (num_copy_rcvrs > 0)
		               ? nsamps_packet[(mcb.active_num_rcvrs - 1) + num_copy_rcvrs]
		               : mcb.nsamps_packet;
		pthread_mutex_lock(&iqready_lock);

		while(!(rcvr_flags & rcb->rcvr_mask)) {
			pthread_cond_wait(&iqready_cond, &iqready_lock);
		}

		rcvr_flags &= ~rcb->rcvr_mask;
		pthread_mutex_unlock(&iqready_lock);

		// can happen when switching between rcvr numbers
		if(rcb->iqSamples_remaining < 0)
			rcb->iqSamples_remaining = 0;

		// downsample starting at any remaining offset
		downsample(rcb);

		switch(mcb.output_rate) {
		case 48000:
			rcb->iqSamples_remaining += RTL_READ_COUNT / (DOWNSAMPLE_192 * 8);
			break;

		case 96000:
			rcb->iqSamples_remaining += RTL_READ_COUNT / (DOWNSAMPLE_192 * 4);
			break;

		case 192000:
			rcb->iqSamples_remaining += RTL_READ_COUNT / (DOWNSAMPLE_192 * 2);
			break;

		case 384000:
			rcb->iqSamples_remaining += RTL_READ_COUNT / DOWNSAMPLE_192;
			break;
		}

		while(rcb->iqSamples_remaining > samps_packet) {
			load_packet(rcb);
			rcb->iqSamples_remaining -= samps_packet;
			rcb->iqSample_offset += samps_packet;
		}

		// move remaining samples to beginning of buffer
		if(rcb->iqSample_offset && (rcb->iqSamples_remaining > 0)) {
			memcpy(&(rcb->iqSamples[0]),
			       &(rcb->iqSamples[rcb->iqSample_offset * 2]),
			       rcb->iqSamples_remaining * 2 * sizeof(float));
			rcb->iqSample_offset = 0;
		}
	}

	pthread_exit(NULL);
	//printf("EXITING hpsdrsim_sendiq_thr_func() rcvr_mask%d...\n", rcb->rcvr_mask);
}

void*
hpsdrsim_thread(void* arg) {
	int bytes_read, ep;
	int i, j, rc, offset;
	int freq, num_rcvrs, xtra;
	u_char C0_1, C0_2;

	//printf("ENTERING hpsdrsim_thread active rcvrs: %d\n", mcb.active_num_rcvrs);
//    reopen_local_sound();

	// start a watchdog to make sure we are sending frames
	rc =
	  pthread_create(&watchdog_thread_id, NULL, hpsdrsim_watchdog_thread,
	                 NULL);

	if(rc != 0) {
		printf("pthread_create failed on hpsdrsim_watchdog_thread: rc=%d\n",
		       rc);
		exit(1);
	}

	while(!do_exit) {
		bytes_read =
		  recvfrom(reveal_socket, buffer, sizeof(buffer), 0,
		           (struct sockaddr*) &my_addr, &my_length);

		if(bytes_read < 0) {
			perror("recvfrom socket failed for hpsdrsim_thread");
			exit(1);
		}  //else printf("hpsdrsim_thread, RECV'D %d bytes\n", bytes_read); //1036 cuSDR64, 1032 ghpsdr3

		if(buffer[0] == 0xEF && buffer[1] == 0xFE) {
			switch(buffer[2]) {
			case 1:
				// get the end point
				ep = buffer[3] & 0xFF;


				// for some reason cuSDR 3.2.13 sends command
				// packets by an extra 4 bytes, cuSDR 3.2.14 is OK
				xtra = ((buffer[8] == 0x7f && buffer[9] == 0x7f && buffer[10] == 0x7f)) ? 0 : 4;

				switch(ep) {
				case 6:
					printf("EP6 data\n");
					break;

				case 4:
					printf("EP4 data\n");
					break;

				case 2:
					// REMEMBER, 2 USB packets of data here (audio, C0, ...)
					//printf("EP2 data\n");
					// get the pc_sequence number
					//pc_sequence=((buffer[4]&0xFF)<<24)+((buffer[5]&0xFF)<<16)+((buffer[6]&0xFF)<<8)+(buffer[7]&0xFF);
					//printf("Received data ep=%d pc_sequence=%d\n",ep,pc_sequence);
					C0_1 = buffer[11 + xtra] & 0xFE;
					C0_2 = buffer[523 + xtra] & 0xFE;
					freq = 0;

					if((C0_1 >= 4) && (C0_1 <= (4 + ((mcb.total_num_rcvrs - 1) * 2)))) {
						offset = xtra;
						freq = 1;
						j = (C0_1 - 4) / 2;
					} else if((C0_2 >= 4) && (C0_2 <= (4 + ((mcb.total_num_rcvrs - 1) * 2)))) {
						offset = 512 + xtra;
						freq = 1;
						j = (C0_2 - 4) / 2;
					}

					if(freq) {
						freq = (int) buffer[12 + offset] << 24 | (int) buffer[13 + offset]
						       << 16 | (int) buffer[14 + offset] << 8 | (int) buffer[15 + offset];

						if(last_freq[j] != freq) {
							// squelch minor (scrolling) freq changes to reduce print output
							ftime(&mcb.freq_ttime[j]);

							if(((((mcb.freq_ttime[j].time * 1000) + mcb.freq_ttime[j].millitm) -
							     (mcb.freq_ltime[j].time * 1000) + mcb.freq_ltime[j].millitm)) > 2000) {
								if(common_freq)
									printf("Received common freq(%d) %d hz, with offset %d hz, for all rcvrs\n",
									       j + 1, freq, freq + mcb.freq_offset[j]);
								else
									printf("Received freq %d hz, with offset %d hz, on rcvr %d\n",
									       freq, freq + mcb.freq_offset[j], j + 1);
							}

							if(common_freq) {
								for(i = 0; i < mcb.active_num_rcvrs; i++) {
									rc = rtlsdr_set_center_freq(mcb.rcb[i].rtldev,
									                            freq + mcb.freq_offset[i]);

									if(rc < 0)
										printf("WARNING: Failed to set Common dongle freq %d\n", freq);
								}
							} else if(j < mcb.total_num_rcvrs) {
								rc = rtlsdr_set_center_freq(mcb.rcb[j].rtldev, freq +
								                            mcb.freq_offset[j]);

								if(rc < 0)
									printf("WARNING: Failed to set dongle freq for rcvr %d\n", j);
							}

							last_freq[j] = freq;
							ftime(&mcb.freq_ltime[j]);
						}
					}

					if((C0_1 == 0x00) || (C0_2 == 0x00)) {
						offset = (C0_1 == 0x00) ? xtra : 512 + xtra;

						if(last_rate != (buffer[12 + offset] & 3)) {
							last_rate = (buffer[12 + offset] & 3);

							switch(last_rate) {
							case 3:
								mcb.output_rate = 384000;
								break;

							case 2:
								mcb.output_rate = 192000;
								break;

							case 1:
								mcb.output_rate = 96000;
								break;

							case 0:
								mcb.output_rate = 48000;
								break;

							default:
								printf("WARNING: UNSUPPORTED RATE: %x!!!\n", last_rate);
							}

							printf("Setting hpsdr output rate to %d hz\n",
							       mcb.output_rate);
						}

						if(last_num_rcvrs != (buffer[15 + offset] & 0x38)) {
							last_num_rcvrs = (buffer[15 + offset] & 0x38);
							num_rcvrs = (last_num_rcvrs >> 3) + 1;

							if(num_rcvrs > MAX_RCVRS) {
								printf("ERROR: Attempt to exceed max number of rcvrs: %d\n", MAX_RCVRS);
								hpsdrsim_stop_threads();
								exit(-1);
							} else if(num_rcvrs > 1) {
								if(num_rcvrs <= mcb.total_num_rcvrs)
									num_copy_rcvrs = 0;
								else
									num_copy_rcvrs =
									  num_rcvrs - mcb.total_num_rcvrs;

								mcb.active_num_rcvrs = (num_copy_rcvrs > 0)
					        ? mcb.total_num_rcvrs : mcb.active_num_rcvrs + (num_rcvrs - 1);

								mcb.nsamps_packet = nsamps_packet[mcb.active_num_rcvrs - 1];
								mcb.frame_offset1 = frame_offset1[mcb.active_num_rcvrs - 1];
								mcb.frame_offset2 = frame_offset2[mcb.active_num_rcvrs - 1];
								mcb.rcvrs_mask = 1;

								// disable all previous rcvrs except rcvr 1
								for(i = 1; i < mcb.total_num_rcvrs; i++) {
									mcb.rcb[i].rcvr_mask = 0;
								}

								// now enable any new ones
								for(i = 1; i < mcb.active_num_rcvrs; i++) {
									mcb.rcvrs_mask |= 1 << i;
									mcb.rcb[i].rcvr_mask = 1 << i;
								}

								printf("Requested %d Activated %d actual rcvr(s)\n",
								       num_rcvrs, mcb.active_num_rcvrs);

								if(num_copy_rcvrs > 0) {
									for(i = mcb.active_num_rcvrs; i < num_rcvrs; i++) {
										copy_rcvr[i] = i;
									}

									printf("Activated %d COPY(S) of rcvr %d\n",
										num_copy_rcvrs, mcb.active_num_rcvrs);
								}
							}
						}
						common_freq = (buffer[15 + xtra] & 0x80) ? 1 : 0;
					}

					// handle the audio data if we assigned an audio device
					if(mcb.sound_dev[0])
						write_local_sound(&(buffer[8 + xtra]));

					break;

				default:
					printf("unexpected EP %d length=%d\n", ep, bytes_read);
					break;
				}

				break;

			case 4:     // start / stop command
				if(buffer[3] & 1) {
					printf("Received Start command\n");
					running = 1;
				} else {
					printf("Received Stop command\n");
					running = 0;
					hpsdr_sequence = 0;
				}

				break;

			default:
				printf("unexpected packet type: 0x%02X\n", buffer[2]);
				break;
			}
		} else {
			printf("Received bad header bytes on data port %02X,%02X\n",
			       buffer[0], buffer[1]);
		}
	}

	//printf("EXITING hpsdrsim_thread()\n");
	pthread_exit(NULL);
}

void
hpsdrsim_stop_threads() {
	int i;

	revealed = running = 0;

	for(i = 0; i < mcb.total_num_rcvrs; i++) {
		rtlsdr_cancel_async(mcb.rcb[i].rtldev);
		pthread_cancel(mcb.rcb[i].rtl_read_thr);
		pthread_cancel(mcb.rcb[i].hpsdrsim_sendiq_thr);
	}

	// unblock held mutexes so we can exit
	pthread_mutex_lock(&send_lock);
	send_flags = mcb.rcvrs_mask;
	pthread_cond_broadcast(&send_cond);
	pthread_mutex_unlock(&send_lock);

	pthread_mutex_lock(&iqready_lock);
	rcvr_flags = mcb.rcvrs_mask;
	pthread_cond_broadcast(&iqready_cond);
	pthread_mutex_unlock(&iqready_lock);

	pthread_cancel(watchdog_thread_id);
	pthread_cancel(hpsdrsim_thread_id);
}

void*
hpsdrsim_watchdog_thread(void* arg) {
	u_int i, last_sequence = 0xffffffff;

	// wait until we get a start command
	while(!running)
		sleep(1);

	//printf("ENTERING hpsdrsim_watchdog_thread active rcvrs: %d\n", mcb.active_num_rcvrs);

	// sleep for 1 second, check if we're sending packets
	while(1) {
		sleep(1);

		if(last_sequence == hpsdr_sequence) {
			printf("No hpsdr packets sent for 1 second, restarting...\n");
			break;
		}

		last_sequence = hpsdr_sequence;
	}

	running = revealed = 0;

	pthread_cancel(hpsdrsim_thread_id);

	// unblock held mutexes so we can exit
	pthread_mutex_lock(&send_lock);
	send_flags = mcb.rcvrs_mask;
	pthread_cond_broadcast(&send_cond);
	pthread_mutex_unlock(&send_lock);

	pthread_mutex_lock(&iqready_lock);
	rcvr_flags = mcb.rcvrs_mask;
	pthread_cond_broadcast(&iqready_cond);
	pthread_mutex_unlock(&iqready_lock);

  // set everything back to a 1 rcvr state
	mcb.rcvrs_mask = 1;
	mcb.rcb[0].rcvr_mask = 1;

	for(i = 1; i < mcb.total_num_rcvrs; i++) {
		mcb.rcb[i].rcvr_mask = 0;
	}

	mcb.active_num_rcvrs = 1;
	num_copy_rcvrs = 0;

	for(i = 0; i < MAX_RCVRS; i++)
		copy_rcvr[i] = -1;

	mcb.nsamps_packet = nsamps_packet[0];
	mcb.frame_offset1 = frame_offset1[0];
	mcb.frame_offset2 = frame_offset2[0];

	rcvr_flags &= 1;
	send_flags &= 1;
	last_num_rcvrs = last_rate = 0;
	mcb.output_rate = 48000;
	printf("Setting hpsdr output rate to %d hz\n", mcb.output_rate);
	hpsdr_sequence = 0;

	//printf("EXITING hpsdrsim_watchdog_thread\n");
	pthread_exit(NULL);
}

void
rtlsdr_callback(unsigned char* buf, uint32_t len, void* ctx) {
	int i, j;
	struct rcvr_cb* rcb = (struct rcvr_cb*) ctx;

	if(do_exit || !running || !ctx) {
		return;
	} else if(rcb->rcvr_mask == 0)
		return;

	if(RTL_READ_COUNT != len) {
		perror("rtlsdr_callback(): RTL_READ_COUNT != len!");
		return;
	}

	// Convert to float and copy data to buffer, offset by coefficient length * 2.
	// The downsample routine will move the previous last coefficient length * 2
	// to the beginning of the buffer. This is because of the FIR filter length, the
	// filtering routine takes in 'filter_length' more samples than it outputs or
	// coefficient length * 2 for I&Q (stereo) input samples.
	for(i = 0, j = COEFF3072_H_16_LENGTH * 2; i < RTL_READ_COUNT; i++, j++)
		rcb->iq_buf[j] = rtl_lut[buf[i]] * mcb.signal_multiplier[rcb->rcvr_num];
 		// rcb->iq_buf[j] = (float)(buf[i]-127);

	pthread_mutex_lock(&iqready_lock);
	rcvr_flags |= rcb->rcvr_mask;
	pthread_cond_broadcast(&iqready_cond);
	pthread_mutex_unlock(&iqready_lock);
}

void
format_payload(void) {
	int i;
	u_char hpsdr_header[8] = { 0xEF, 0xFE, 1, 6, 0, 0, 0, 0 };
	u_char proto_header[8] = { 0x7f, 0x7f, 0x7f, 0, 0x1e, 0, 0, HERMES_FW_VER };

	for(i = 0; i < HPSDR_FRAME_LEN; i++)
		payload[i] = 0;

	for(i = 0; i < 8; i++)
		payload[i] = hpsdr_header[i];

	for(i = 8; i < 16; i++)
		payload[i] = proto_header[i - 8];

	for(i = 520; i < 528; i++)
		payload[i] = proto_header[i - 520];
}

void*
rtl_read_thr_func(void* arg) {
	struct rcvr_cb* rcb = (struct rcvr_cb*) arg;
	int r, i = rcb->rcvr_num;

	//printf("ENTERING rtl_read_thr_func() rcvr %d\n", i+1);
	r = rtlsdr_read_async(rcb->rtldev, rtlsdr_callback,
	                      (void*)(&mcb.rcb[i]), 1, RTL_READ_COUNT);

	//printf("EXITING rtl_read_thr_func() rcvr %d\n", i+1);
	pthread_exit(NULL);
}

int
init_rtl(int dev_index) {
	int r;
	char num[16];
	rtlsdr_dev_t* rtldev;

	r = rtlsdr_open(&(mcb.rcb[dev_index].rtldev), dev_index);

	if(r < 0) {
		printf("Failed to open rtlsdr device\n");
		return (-1);
	}

	rtldev = mcb.rcb[dev_index].rtldev;

	r = rtlsdr_set_sample_rate(rtldev, RTL_SAMPLE_RATE);

	if(r < 0) {
		printf("WARNING: Failed to set sample rate to %d!\n", RTL_SAMPLE_RATE);
		return (-1);
	}

	sprintf(num, "%d", mcb.gain[dev_index]);

	if(mcb.gain[dev_index]) {
		r = rtlsdr_set_tuner_gain_mode(rtldev, 1);
		r |= rtlsdr_set_tuner_gain(rtldev, mcb.gain[dev_index]);
	} else
		r = rtlsdr_set_tuner_gain_mode(rtldev, 0);

	if(r < 0) {
		printf("WARNING: Failed to set tuner gain!\n");
		return (-1);
	} else
		printf("  tuner gain\t\t%s\n", (mcb.gain[dev_index]) ? num : "auto");

	r = rtlsdr_set_direct_sampling(rtldev, mcb.direct_mode[dev_index]);

	if(r < 0) {
		printf("WARNING: Failed to set direct sampling!\n");
		return (-1);
	} else
		printf("  direct sampling\t%s\n",
		       (mcb.direct_mode[dev_index]) ? "on" : "off");

	r = rtlsdr_set_agc_mode(rtldev, mcb.agc_mode[dev_index]);

	if(r < 0) {
		printf("WARNING: Failed to set automatic gain!\n");
		return (-1);
	} else
		printf("  agc mode\t\t%s\n\n", (mcb.agc_mode[dev_index]) ? "on" : "off");

	r = rtlsdr_reset_buffer(rtldev);

	if(r < 0) {
		printf("WARNING: Failed to reset buffers!\n");
		return (-1);
	}

	return (0);
}

void
usage(char* progname) {
	printf(
	  "\n%s, an HPSDR Hermes simulator for RTL2832 based DVB-T receivers", progname);
	printf(
	  "\n\nUsage:\n" "\tPer rcvr options (comma separated i.e. 1,0,1,1):\n"
	  "\t[-a internal agc of the rtl2832 0|1 (defaults 0 or off)]\n"
	  "\t[-d direct sampling mode 0|1|2 (defaults 0 or off, 1=I 2=Q)]\n"
	  "\t[-f freq offset in hz (defaults 0)]\n"
	  "\t[-g gain in tenths of a db (defaults 0 for auto)]\n"
	  "\t[-m signal multiplier (default 1)]\n\n"
	  "\tGlobal options:\n"
	  "\t[-c path to config file (overrides these options)]\n"
	  "\t[-h help (prints this usage)]\n"
	  "\t[-l length of fir coeffients 32|64 (default 32)]\n"
	  "\t[-n network device (default eth0)]\n"
	  "\t[-r number of rcvrs to use (defaults to all detected)]\n"
	  "\t[-s sound device (alsa) for audio (i.e. plughw:0,0 defaults to none)]\n"
	  "\t[-v print out version info]\n\n");
	exit(1);
}

int
set_option(int* option, char* value) {
	char params[MAX_RCVRS][MAXSTR];
	int i, count = 0;
	char* token;
	const char s[2] = ",";

	// get the first token
	token = strtok(value, s);

	// walk through other tokens
	while((token != NULL) && (count < MAX_RCVRS)) {
		strcpy(&(params[count++][0]), token);
		token = strtok(NULL, s);
	}

	for(i = 0; i < MAX_RCVRS; i++) {
		if(i < count) {
			option[i] = atoi(params[i]);
			option[MAX_RCVRS] = option[i];  // save last
		} else
			option[i] = option[MAX_RCVRS];  // set to last
	}

	return (count);
}

int
parse_config(char* conf_file) {
	FILE* fp;
	int count, line = 0;
	char option[MAXSTR], value[MAXSTR];
	char confbuf[MAXSTR];

	if((fp = fopen(conf_file, "r")) != NULL) {
		printf("\nParsing config file %s\n\n", conf_file);

		while(fgets(confbuf, MAXSTR, fp) != NULL) {
			line++;

			if(strchr(confbuf, '#') != NULL)
				continue;

			option[0] = value[0] = '\0';
			sscanf(confbuf, "%s %s", option, value);

			if(!strcmp("direct_mode", option)) {
				count = set_option(mcb.direct_mode, value);
			} else if(!strcmp("tuner_gain", option)) {
				count = set_option(mcb.gain, value);
			} else if(!strcmp("freq_offset", option)) {
				count = set_option(mcb.freq_offset, value);
			} else if(!strcmp("agc_mode", option)) {
				count = set_option(mcb.agc_mode, value);
			} else if(!strcmp("signal_multiplier", option)) {
				count = set_option(mcb.signal_multiplier, value);
			} else if(!strcmp("sound_dev", option)) {
				strcpy(mcb.sound_dev, value);
			} else if(!strcmp("length_fir", option)) {
				mcb.length_fir = atoi(value);
			} else if(!strcmp("total_num_rcvrs", option)) {
				mcb.total_num_rcvrs = atoi(value);
			} else if(!strcmp("net_dev", option)) {
				strcpy(mcb.net_dev, value);
			}
		}

		return (0);
	} else {
		printf("Cannot find %s\n", conf_file);
		exit (-1);
	}
}

int
main(int argc, char* argv[]) {
	int i, r, opt;
	bool loop = true;
	struct sigaction sigact;
	char conf_file[MAXSTR];
	char* progname = basename(argv[0]);

	// set defaults
	mcb.length_fir = 32;
	mcb.sound_dev[0] = 0;
	conf_file[0] = 0;
	mcb.output_rate = 48000;
	strcpy(mcb.net_dev, "eth0");

	for(i = 0; i < MAX_RCVRS; i++) {
		mcb.agc_mode[i] = 0;
		mcb.direct_mode[i] = 0;
		mcb.gain[i] = 0;
		mcb.freq_offset[i] = 0;
		mcb.signal_multiplier[i] = 1;
		copy_rcvr[i] = -1;
		memset(&mcb.freq_ltime[i], 0, sizeof(mcb.freq_ltime[i]));
	}

	while(loop && ((opt = getopt(argc, argv, "c:a:d:f:g:hm:n:r:s:v")) != -1)) {
		switch(opt) {
		case 'a':
			r = set_option(mcb.agc_mode, optarg);
			break;

		case 'd':
			r = set_option(mcb.direct_mode, optarg);
			break;

		case 'c':
			strcpy(conf_file, optarg);
			parse_config(conf_file);
			loop = false;
			break;

		case 'f':
			r = set_option(mcb.freq_offset, optarg);
			break;

		case 'g':
			r = set_option(mcb.gain, optarg);
			break;

		case 'l':
			mcb.length_fir = atoi(optarg);

			if((mcb.length_fir != 32) && (mcb.length_fir != 64)) {
				printf("Invalid coeffient length %d\n", mcb.length_fir);
				usage(progname);
			}

			break;

		case 'm':
			r = set_option(mcb.signal_multiplier, optarg);
			break;

		case 'n':
			strcpy(mcb.net_dev, optarg);
			break;

		case 'r':
			mcb.total_num_rcvrs = atoi(optarg);
			break;

		case 's':
			strcpy(mcb.sound_dev, optarg);
			break;

		case 'v':
			printf("\nGNU %s version %s\n", progname, PRG_VERSION);
			printf("Copyright (C) 2014 Free Software Foundation, Inc.\n"
			       "License GPLv3+: GNU GPL version 3 or later <http://gnu.org/licenses/gpl.html>\n"
			       "This is free software: you are free to change and redistribute it.\n"
			       "There is NO WARRANTY, to the extent permitted by law.\n\n");
			exit(0);

		case 'h':
		default:
			usage(progname);
			break;
		}
	}

	r = rtlsdr_get_device_count();

	if(r) {
		printf("Found %d RTL device(s)", r);

		if((mcb.total_num_rcvrs > r) || (0 == mcb.total_num_rcvrs))
			mcb.total_num_rcvrs = r;

		if(mcb.total_num_rcvrs > MAX_RCVRS)
			mcb.total_num_rcvrs = MAX_RCVRS;

		printf(", using %d.\n", mcb.total_num_rcvrs);
	} else {
		printf("No RTL devices found, exiting.\n");
		exit(0);
	}

	printf("RTL base sample rate: %d hz\n\n", RTL_SAMPLE_RATE);
	printf("Global settings:\n");
	printf("  config file:\t\t%s\n", (0 == conf_file[0]) ? "none" : conf_file);
	printf("  network device:\t%s\n", mcb.net_dev);
	printf("  length of fir:\t%d\n", mcb.length_fir);
	printf("  number of rcvrs:\t%d\n", mcb.total_num_rcvrs);
	printf("  hpsdr output rate:\t%d hz\n", mcb.output_rate);
	printf("  sound device:\t\t%s\n",
	       (0 == mcb.sound_dev[0]) ? "none" : mcb.sound_dev);

#if 0

	if(argc <= optind) {
		usage(progname);
	}

#endif

	sigact.sa_handler = rtl_sighandler;
	sigemptyset(&sigact.sa_mask);
	sigact.sa_flags = 0;
	sigaction(SIGINT, &sigact, NULL);
	sigaction(SIGTERM, &sigact, NULL);
	sigaction(SIGQUIT, &sigact, NULL);
	sigaction(SIGPIPE, &sigact, NULL);

	format_payload();

	switch(mcb.length_fir) {
	case 32:
		mcb.coeff_48 = coeff1536_48_H_32;
		mcb.coeff_96 = coeff1536_96_H_32;
		mcb.coeff_192 = coeff1536_192_H_32;
		mcb.coeff_384 = coeff1536_384_H_32;
		break;

	case 64:
		mcb.coeff_48 = coeff1536_48_H_64;
		mcb.coeff_96 = coeff1536_96_H_64;
		mcb.coeff_192 = coeff1536_192_H_64;
		mcb.coeff_384 = coeff1536_384_H_64;
		break;
	}

	mcb.coeff_768 = coeff3072_768_H_16;

	r = posix_memalign((void**) & (mcb.align1536_48_H), 16,
	                   mcb.length_fir * 2 * sizeof(float));
	r |= posix_memalign((void**) & (mcb.align1536_96_H), 16,
	                    mcb.length_fir * 2 * sizeof(float));
	r |= posix_memalign((void**) & (mcb.align1536_192_H), 16,
	                    mcb.length_fir * 2 * sizeof(float));
	r |= posix_memalign((void**) & (mcb.align1536_384_H), 16,
	                    mcb.length_fir * 2 * sizeof(float));
	r |= posix_memalign((void**) & (mcb.align3072_768_H), 16,
	                    COEFF3072_H_16_LENGTH * 2 * sizeof(float));

	if(r != 0) {
		printf("failed to allocate coeff aligned memory: r=%d\n", r);
		return (r);
	}

	// rearrange coeff table for SIMD operations
	for(i = 0; i < mcb.length_fir; i++) {
		mcb.align1536_48_H[2 * i + 0] =
		  mcb.align1536_48_H[2 * i + 1] = mcb.coeff_48[i];
		mcb.align1536_96_H[2 * i + 0] =
		  mcb.align1536_96_H[2 * i + 1] = mcb.coeff_96[i];
		mcb.align1536_192_H[2 * i + 0] =
		  mcb.align1536_192_H[2 * i + 1] = mcb.coeff_192[i];
		mcb.align1536_384_H[2 * i + 0] =
		  mcb.align1536_384_H[2 * i + 1] = mcb.coeff_384[i];
	}

	for(i = 0; i < COEFF3072_H_16_LENGTH; i++) {
		mcb.align3072_768_H[2 * i + 0] =
		  mcb.align3072_768_H[2 * i + 1] = mcb.coeff_768[i];
	}

	// create a lookup table for float values
	for(i = 0; i < 256; i++) {
		rtl_lut[i] = (float)(i - 127);
	}

	pthread_mutex_init(&iqready_lock, NULL);
	pthread_cond_init(&iqready_cond, NULL);
	pthread_mutex_init(&send_lock, NULL);
	pthread_cond_init(&send_cond, NULL);
	pthread_mutex_init(&done_send_lock, NULL);
	pthread_cond_init(&done_send_cond, NULL);

	if(mcb.sound_dev[0])
		open_local_sound(mcb.sound_dev);

	// enable the 1st rcvr until we get the active count
	mcb.rcb[0].rcvr_mask = 1;
	mcb.active_num_rcvrs = 1;
	mcb.rcvrs_mask = 1;
	mcb.nsamps_packet = nsamps_packet[0];
	mcb.frame_offset1 = frame_offset1[0];
	mcb.frame_offset2 = frame_offset2[0];

	for(i = 0; i < mcb.total_num_rcvrs; i++) {
		mcb.rcb[i].mcb = &mcb;
		printf("\nRcvr %d settings...\n", i + 1);
		printf("  freq offset\t\t%d hz\n", mcb.freq_offset[i]);
		printf("  signal multiplier\t%d\n", mcb.signal_multiplier[i]);

		if(0 != init_rtl(i)) {
			printf("ERROR: Failed init_rtl rcvr%d hardware!\n", i + 1);
			return (-1);
		}

		mcb.rcb[i].rtl_read_thr = rtl_read_thr[i];
		mcb.rcb[i].hpsdrsim_sendiq_thr = hpsdrsim_sendiq_thr[i];
		mcb.rcb[i].rcvr_num = i;
		r = posix_memalign((void**) & (mcb.rcb[i].iq_buf), 16,
		                   (RTL_READ_COUNT + (COEFF3072_H_16_LENGTH * 2)) * sizeof(float));
		r |= posix_memalign((void**) & (mcb.rcb[i].iq_buf_final), 16,
		                    ((RTL_READ_COUNT / 2) + (mcb.length_fir * 2)) * sizeof(float));

		if(r != 0) {
			printf("failed to allocate iq_buf aligned memory: r=%d\n",
			       r);
			return (r);
		}

		if((r = pthread_create(&rtl_read_thr[i], NULL, rtl_read_thr_func,
		                       &mcb.rcb[i]))) {
			printf("error: pthread_create, r: %d\n", r);
			return (r);
		}

		if((r = pthread_create(&hpsdrsim_sendiq_thr[i], NULL,
		                       hpsdrsim_sendiq_thr_func, &mcb.rcb[i]))) {
			printf("pthread_create failed on hpsdrsim_sendiq_thr: r=%d\n", r);
			return (r);
		}
	}

	while(!do_exit) {

		// reveal myself
		hpsdrsim_reveal();

		while(revealed) {
			// wait for all rcvrs to have formatted data ready before sending
			pthread_mutex_lock(&send_lock);

			while(send_flags != mcb.rcvrs_mask) {
				pthread_cond_wait(&send_cond, &send_lock);
			}

			pthread_mutex_unlock(&send_lock);

#if 0 // dump the frame for analysis
			printf("rcvrs_mask:%x send_flags:%d\n", mcb.rcvrs_mask,
			       send_flags);

			for(i = 0; i < HPSDR_FRAME_LEN; i++) {
				printf("%4d:%2x ", i, payload[i]);

				if(!((i + 1) % 8))
					printf("\n");
			}

			exit(0);
#endif

			pthread_mutex_lock(&done_send_lock);
			send_flags = 0;
			pthread_cond_broadcast(&done_send_cond);
			pthread_mutex_unlock(&done_send_lock);

			payload[4] = (hpsdr_sequence >> 24) & 0xff;
			payload[5] = (hpsdr_sequence >> 16) & 0xff;
			payload[6] = (hpsdr_sequence >> 8) & 0xff;
			payload[7] = hpsdr_sequence & 0xff;

			if(sendto(reveal_socket, payload, HPSDR_FRAME_LEN, 0,
			          (struct sockaddr*) &their_addr,
			          sizeof(their_addr)) < 0) {
				if(running) {
					perror("sendto() reveal_socket, error!");
					return 0;
				}
			}

			hpsdr_sequence += 1;
		}
	}

	// clean up
	for(i = 0; i < mcb.total_num_rcvrs; i++) {
		rtlsdr_close(mcb.rcb[i].rtldev);
		free(mcb.rcb[i].iq_buf);
		free(mcb.rcb[i].iq_buf_final);
	}

	free(mcb.align1536_48_H);
	free(mcb.align1536_96_H);
	free(mcb.align1536_192_H);
	free(mcb.align1536_384_H);
	free(mcb.align3072_768_H);

	if(close(reveal_socket) != 0)
		printf("sockfd closing failed!\n");

	if(mcb.sound_dev[0])
		close_local_sound();

	printf("Exiting Program.\n");

	return 0;
}
