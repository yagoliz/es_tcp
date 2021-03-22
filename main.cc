/*
 * rtl-sdr, turns your Realtek RTL2832 based DVB dongle into a SDR receiver
 * Copyright (C) 2012 by Steve Markgraf <steve@steve-m.de>
 * Copyright (C) 2012-2013 by Hoernchen <la@tfc-server.de>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

/*
 * es_tcp: This version was modified to support Electrosense's downconverter
 * and reach frequencies up to 6 GHz
 *
 * Author: Yago Lizarribar
 * Email: yago.lizarribar@gmail.com
 */

#include <iostream>
#include <string>
#include <errno.h>
#include <signal.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

#include <unistd.h>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <netdb.h>
#include <netinet/in.h>
#include <fcntl.h>

#include <pthread.h>

#include "rtl-sdr.h"

extern "C"
{
#include "convenience/convenience.h"
#include "converter/converter.h"
}

#define closesocket close
#define SOCKADDR struct sockaddr
#define SOCKET int
#define SOCKET_ERROR -1

static SOCKET s;

static pthread_t tcp_worker_thread;
static pthread_t command_thread;
static pthread_cond_t exit_cond;
static pthread_mutex_t exit_cond_lock;

static pthread_mutex_t ll_mutex;
static pthread_cond_t cond;

struct llist {
    char *data;
    size_t len;
    struct llist *next;
};

typedef struct { /* structure size must be multiple of 2 bytes */
    char magic[4];
    uint32_t tuner_type;
    uint32_t tuner_gain_count;
} dongle_info_t;

static rtlsdr_dev_t *dev = NULL;
static converter es_converter;
static bool mConverterEnabled = false;
static uint64_t proxy_freq = 0, previous_proxy_freq = 0;
static bool mustInvert = false;

static int enable_biastee = 0;
static int global_numq = 0;
static struct llist *ll_buffers = 0;
static int llbuf_num = 500;

static volatile int do_exit = 0;

void usage(void)
{
  printf("es_tcp, an I/Q spectrum server for RTL2832 based DVB-T receivers\n\n"
         "Usage: \t[-h Show this menu]\n"
         "\t[-a Listen address]\n"
         "\t[-c Converter path (default: /dev/ttyACM0)"
         "\t[-p Listen port (default: 1234)]\n"
         "\t[-f Frequency to tune to [Hz]]\n"
         "\t[-g Gain (default: 0 for auto)]\n"
         "\t[-s Sample rate in Hz (default: 2048000 Hz)]\n"
         "\t[-b Number of buffers (default: 15, set by library)]\n"
         "\t[-n Max number of linked list buffers to keep (default: 500)]\n"
         "\t[-d Device index (default: 0)]\n"
         "\t[-P PPM error (default: 0)]\n"
         "\t[-T Enable bias-T on GPIO PIN 0 (works for rtl-sdr.com v3 dongles)]\n");
}

static void sighandler(int signum)
{
  fprintf(stderr, "Signal caught, exiting!\n");
  rtlsdr_cancel_async(dev);
  do_exit = 1;
}

void rtlsdr_callback(unsigned char *buf, uint32_t len, void *ctx)
{
  if(!do_exit) {
    struct llist *rpt = (struct llist*)malloc(sizeof(struct llist));
    rpt->data = (char*)malloc(len);
    if (mustInvert) {
      for (int i = 0; i < len; i += 2) {
        buf[i] = 255 - buf[i];
      }
    }
    memcpy(rpt->data, buf, len);
    rpt->len = len;
    rpt->next = NULL;

    pthread_mutex_lock(&ll_mutex);

    if (ll_buffers == NULL) {
      ll_buffers = rpt;
    } else {
      struct llist *cur = ll_buffers;
      int num_queued = 0;

      while (cur->next != NULL) {
        cur = cur->next;
        num_queued++;
      }

      if(llbuf_num && llbuf_num == num_queued-2){
        struct llist *curelem;

        free(ll_buffers->data);
        curelem = ll_buffers->next;
        free(ll_buffers);
        ll_buffers = curelem;
      }

      cur->next = rpt;

      if (num_queued > global_numq)
        printf("ll+, now %d\n", num_queued);
      else if (num_queued < global_numq)
        printf("ll-, now %d\n", num_queued);

      global_numq = num_queued;
    }
    pthread_cond_signal(&cond);
    pthread_mutex_unlock(&ll_mutex);
  }
}

static void *tcp_worker(void *arg)
{
  struct llist *curelem,*prev;
  int bytesleft,bytessent, index;
  struct timeval tv= {1,0};
  struct timespec ts;
  struct timeval tp;
  fd_set writefds;
  int r = 0;

  while(1) {
    if(do_exit)
      pthread_exit(0);

    pthread_mutex_lock(&ll_mutex);
    gettimeofday(&tp, NULL);
    ts.tv_sec  = tp.tv_sec+5;
    ts.tv_nsec = tp.tv_usec * 1000;
    r = pthread_cond_timedwait(&cond, &ll_mutex, &ts);
    if(r == ETIMEDOUT) {
      pthread_mutex_unlock(&ll_mutex);
      printf("Worker cond timeout\n");
      sighandler(0);
      pthread_exit(NULL);
    }

    curelem = ll_buffers;
    ll_buffers = 0;
    pthread_mutex_unlock(&ll_mutex);

    while(curelem != 0) {
      bytesleft = curelem->len;
      index = 0;
      bytessent = 0;
      while(bytesleft > 0) {
        FD_ZERO(&writefds);
        FD_SET(s, &writefds);
        tv.tv_sec = 1;
        tv.tv_usec = 0;
        r = select(s+1, NULL, &writefds, NULL, &tv);
        if(r) {
          bytessent = send(s,  &curelem->data[index], bytesleft, 0);
          bytesleft -= bytessent;
          index += bytessent;
        }
        if(bytessent == SOCKET_ERROR || do_exit) {
          printf("Worker socket bye\n");
          sighandler(0);
          pthread_exit(NULL);
        }
      }
      prev = curelem;
      curelem = curelem->next;
      free(prev->data);
      free(prev);
    }
  }
}

static int set_gain_by_index(rtlsdr_dev_t *_dev, unsigned int index)
{
  int res = 0;
  int* gains;
  int count = rtlsdr_get_tuner_gains(_dev, NULL);

  if (count > 0 && (unsigned int)count > index) {
    gains = (int *)malloc(sizeof(int) * count);
    count = rtlsdr_get_tuner_gains(_dev, gains);

    res = rtlsdr_set_tuner_gain(_dev, gains[index]);

    free(gains);
  }

  return res;
}

static void set_frequency(rtlsdr_dev_t *dev, unsigned int frequency) {
  if (mConverterEnabled) {
    if (!converterTune(&es_converter, frequency / 1e3, &proxy_freq, &mustInvert)) {
      throw std::logic_error("Failed to converterTune");
    }

    int r = rtlsdr_set_center_freq(dev, proxy_freq * 1e3);
    if (r != 0) {
      std::cerr << "Error: unable to set center frequency: "<< proxy_freq * 1e3 << std::endl;
    }
  } else {
    rtlsdr_set_center_freq(dev, frequency);
  }
}

struct command{
    unsigned char cmd;
    unsigned int param;
}__attribute__((packed));

static void *command_worker(void *arg)
{
  int left, received = 0;
  fd_set readfds;
  struct command cmd={0, 0};
  struct timeval tv= {1, 0};
  int r = 0;
  uint32_t tmp;

  while(1) {
    left=sizeof(cmd);
    while(left >0) {
      FD_ZERO(&readfds);
      FD_SET(s, &readfds);
      tv.tv_sec = 1;
      tv.tv_usec = 0;
      r = select(s+1, &readfds, NULL, NULL, &tv);
      if(r) {
        received = recv(s, (char*)&cmd+(sizeof(cmd)-left), left, 0);
        left -= received;
      }
      if(received == SOCKET_ERROR || do_exit) {
        printf("Comm recv bye\n");
        sighandler(0);
        pthread_exit(NULL);
      }
    }
    switch(cmd.cmd) {
      case 0x01:
        printf("Set freq %d\n", ntohl(cmd.param));
        set_frequency(dev, ntohl(cmd.param));
//        rtlsdr_set_center_freq(dev,ntohl(cmd.param));
        break;
      case 0x02:
        printf("Set sample rate %d\n", ntohl(cmd.param));
        rtlsdr_set_sample_rate(dev, ntohl(cmd.param));
        break;
      case 0x03:
        printf("Set gain mode %d\n", ntohl(cmd.param));
        rtlsdr_set_tuner_gain_mode(dev, ntohl(cmd.param));
        break;
      case 0x04:
        printf("Set gain %d\n", ntohl(cmd.param));
        rtlsdr_set_tuner_gain(dev, ntohl(cmd.param));
        break;
      case 0x05:
        printf("Set freq correction %d\n", ntohl(cmd.param));
        rtlsdr_set_freq_correction(dev, ntohl(cmd.param));
        break;
      case 0x06:
        tmp = ntohl(cmd.param);
        printf("Set if stage %d gain %d\n", tmp >> 16, (short)(tmp & 0xffff));
        rtlsdr_set_tuner_if_gain(dev, tmp >> 16, (short)(tmp & 0xffff));
        break;
      case 0x07:
        printf("Set test mode %d\n", ntohl(cmd.param));
        rtlsdr_set_testmode(dev, ntohl(cmd.param));
        break;
      case 0x08:
        printf("Set agc mode %d\n", ntohl(cmd.param));
        rtlsdr_set_agc_mode(dev, ntohl(cmd.param));
        break;
      case 0x09:
        printf("Set direct sampling %d\n", ntohl(cmd.param));
        rtlsdr_set_direct_sampling(dev, ntohl(cmd.param));
        break;
      case 0x0a:
        printf("set offset tuning %d\n", ntohl(cmd.param));
        rtlsdr_set_offset_tuning(dev, ntohl(cmd.param));
        break;
      case 0x0b:
        printf("Set rtl xtal %d\n", ntohl(cmd.param));
        rtlsdr_set_xtal_freq(dev, ntohl(cmd.param), 0);
        break;
      case 0x0c:
        printf("Set tuner xtal %d\n", ntohl(cmd.param));
        rtlsdr_set_xtal_freq(dev, 0, ntohl(cmd.param));
        break;
      case 0x0d:
        printf("Set tuner gain by index %d\n", ntohl(cmd.param));
        set_gain_by_index(dev, ntohl(cmd.param));
        break;
      case 0x0e:
        printf("Set bias tee %d\n", ntohl(cmd.param));
        rtlsdr_set_bias_tee(dev, (int)ntohl(cmd.param));
        break;
      default:
        break;
    }
    cmd.cmd = 0xff;
  }
}

int finish(int r, int listensocket) {
  rtlsdr_close(dev);
  closesocket(listensocket);
  closesocket(s);
  delete [] es_converter.portPath;
  printf("Bye!\n");
  return r >= 0 ? r : -r;
}

int main(int argc, char **argv)
{
  int r, opt, i;
  char *addr = "127.0.0.1";
  char *port = "1234";
  std::string converter_path = "/dev/ttyACM0";
  uint32_t frequency = 100000000, samp_rate = 2048000;
  struct sockaddr_storage local, remote;
  struct addrinfo *ai;
  struct addrinfo *aiHead;
  struct addrinfo  hints;
  char hostinfo[NI_MAXHOST];
  char portinfo[NI_MAXSERV];
  char remhostinfo[NI_MAXHOST];
  char remportinfo[NI_MAXSERV];
  int aiErr;
  uint32_t buf_num = 0;
  int dev_index = 0;
  int dev_given = 0;
  int gain = 0;
  int ppm_error = 0;
  struct llist *curelem,*prev;
  pthread_attr_t attr;
  void *status;
  struct timeval tv = {1,0};
  struct linger ling = {1,0};
  SOCKET listensocket = 0;
  socklen_t rlen;
  fd_set readfds;
  u_long blockmode = 1;
  dongle_info_t dongle_info;

  struct sigaction sigact, sigign;

  while ((opt = getopt(argc, argv, "a:c:p:f:g:s:b:n:d:P:Th")) != -1) {
    switch (opt) {
      case 'd':
        dev_index = verbose_device_search(optarg);
        dev_given = 1;
        break;
      case 'f':
        frequency = (uint32_t)atofs(optarg);
        break;
      case 'g':
        gain = (int)(atof(optarg) * 10); /* tenths of a dB */
        break;
      case 's':
        samp_rate = (uint32_t)atofs(optarg);
        break;
      case 'a':
        addr = strdup(optarg);
        break;
      case 'c':
        converter_path = strdup(optarg);
        break;
      case 'p':
        port = strdup(optarg);
        break;
      case 'b':
        buf_num = atoi(optarg);
        break;
      case 'n':
        llbuf_num = atoi(optarg);
        break;
      case 'h':
        usage();
        exit(0);
        break;
      case 'P':
        ppm_error = atoi(optarg);
        break;
      case 'T':
        enable_biastee = 1;
        break;
      default:
        usage();
        break;
    }
  }

  if (argc < optind) {
    usage();
    exit(1);
  }

  if (!dev_given) {
    dev_index = verbose_device_search("0");
  }

  if (dev_index < 0) {
    exit(1);
  }

  rtlsdr_open(&dev, (uint32_t)dev_index);
  if (NULL == dev) {
    fprintf(stderr, "Failed to open rtlsdr device #%d.\n", dev_index);
    exit(1);
  }

  sigact.sa_handler = sighandler;
  sigemptyset(&sigact.sa_mask);
  sigact.sa_flags = 0;
  sigign.sa_handler = SIG_IGN;
  sigaction(SIGINT, &sigact, NULL);
  sigaction(SIGTERM, &sigact, NULL);
  sigaction(SIGQUIT, &sigact, NULL);
  sigaction(SIGPIPE, &sigign, NULL);

  /* Set the tuner error */
  verbose_ppm_set(dev, ppm_error);

  /* Set the sample rate */
  r = rtlsdr_set_sample_rate(dev, samp_rate);
  if (r < 0)
    fprintf(stderr, "WARNING: Failed to set sample rate.\n");

  if (0 == gain) {
    /* Enable automatic gain */
    r = rtlsdr_set_tuner_gain_mode(dev, 0);
    if (r < 0)
      fprintf(stderr, "WARNING: Failed to enable automatic gain.\n");
  } else {
    /* Enable manual gain */
    r = rtlsdr_set_tuner_gain_mode(dev, 1);
    if (r < 0)
      fprintf(stderr, "WARNING: Failed to enable manual gain.\n");

    /* Set the tuner gain */
    r = rtlsdr_set_tuner_gain(dev, gain);
    if (r < 0)
      fprintf(stderr, "WARNING: Failed to set tuner gain.\n");
    else
      fprintf(stderr, "Tuner gain set to %f dB.\n", gain/10.0);
  }

  rtlsdr_set_bias_tee(dev, enable_biastee);
  if (enable_biastee)
    fprintf(stderr, "Activated bias-T on GPIO PIN 0\n");

  /* Reset endpoint before we start reading from it (mandatory) */
  r = rtlsdr_reset_buffer(dev);
  if (r < 0)
    fprintf(stderr, "WARNING: Failed to reset buffers.\n");

  /* Check whether the converter is present */
  es_converter.portPath = new char[converter_path.size() + 1];
  std::copy(converter_path.begin(), converter_path.end(),
            es_converter.portPath);
  es_converter.portPath[converter_path.size()] = '\0';

  if (!converterInit(&es_converter)) {
    std::cerr << "Warning: Failed to open the converter" << std::endl;
    // throw std::logic_error("Failed to open the converter");
  } else {
    std::cerr << "Converter has been detected properly" << std::endl;
    mConverterEnabled = true;
  }

  /* Set center frequencies */
  if (mConverterEnabled) { /* Converter */
    if (!converterTune(&es_converter, frequency / 1e3, &proxy_freq, &mustInvert)) {
      throw std::logic_error("Failed to converterTune");
    }

    int r = rtlsdr_set_center_freq(dev, proxy_freq * 1e3);
    if (r != 0) {
      std::cerr << "Error: unable to set center frequency: "<< proxy_freq * 1e3 << std::endl;
    }
  }
  else { /* Native RTL-SDR */
    /* Set the frequency */
    r = rtlsdr_set_center_freq(dev, frequency);
    if (r < 0)
      fprintf(stderr, "WARNING: Failed to set center freq.\n");
    else
      fprintf(stderr, "Tuned to %i Hz.\n", frequency);
  }

  pthread_mutex_init(&exit_cond_lock, NULL);
  pthread_mutex_init(&ll_mutex, NULL);
  pthread_mutex_init(&exit_cond_lock, NULL);
  pthread_cond_init(&cond, NULL);
  pthread_cond_init(&exit_cond, NULL);

  hints.ai_flags  = AI_PASSIVE; /* Server mode. */
  hints.ai_family = PF_UNSPEC;  /* IPv4 or IPv6. */
  hints.ai_socktype = SOCK_STREAM;
  hints.ai_protocol = IPPROTO_TCP;

  if ((aiErr = getaddrinfo(addr,
                           port,
                           &hints,
                           &aiHead )) != 0)
  {
    fprintf(stderr, "local address %s ERROR - %s.\n",
            addr, gai_strerror(aiErr));
    return(-1);
  }
  memcpy(&local, aiHead->ai_addr, aiHead->ai_addrlen);

  for (ai = aiHead; ai != NULL; ai = ai->ai_next) {
    aiErr = getnameinfo((struct sockaddr *)ai->ai_addr, ai->ai_addrlen,
                        hostinfo, NI_MAXHOST,
                        portinfo, NI_MAXSERV, NI_NUMERICSERV | NI_NUMERICHOST);
    if (aiErr)
      fprintf( stderr, "getnameinfo ERROR - %s.\n",hostinfo);

    listensocket = socket(ai->ai_family, ai->ai_socktype, ai->ai_protocol);
    if (listensocket < 0)
      continue;

    r = 1;
    setsockopt(listensocket, SOL_SOCKET, SO_REUSEADDR, (char *)&r, sizeof(int));
    setsockopt(listensocket, SOL_SOCKET, SO_LINGER, (char *)&ling, sizeof(ling));

    if (bind(listensocket, (struct sockaddr *)&local, sizeof(local)))
      fprintf(stderr, "es_tcp bind error: %s", strerror(errno));
    else
      break;
  }

  r = fcntl(listensocket, F_GETFL, 0);
  r = fcntl(listensocket, F_SETFL, r | O_NONBLOCK);

  while(1) {
    printf("Listening...\n");
    printf("Use the device argument 'rtl_tcp=%s:%s' in OsmoSDR "
           "(gr-osmosdr) source\n"
           "to receive samples in GRC and control "
           "rtl_tcp parameters (frequency, gain, ...).\n",
           hostinfo, portinfo);
    listen(listensocket,1);

    while(1) {
      FD_ZERO(&readfds);
      FD_SET(listensocket, &readfds);
      tv.tv_sec = 1;
      tv.tv_usec = 0;
      r = select(listensocket+1, &readfds, NULL, NULL, &tv);
      if(do_exit) {
        return finish(r, listensocket);
      } else if(r) {
        rlen = sizeof(remote);
        s = accept(listensocket,(struct sockaddr *)&remote, &rlen);
        break;
      }
    }

    setsockopt(s, SOL_SOCKET, SO_LINGER, (char *)&ling, sizeof(ling));

    getnameinfo((struct sockaddr *)&remote, rlen,
                remhostinfo, NI_MAXHOST,
                remportinfo, NI_MAXSERV, NI_NUMERICSERV);
    printf("Client accepted! %s %s\n", remhostinfo, remportinfo);

    memset(&dongle_info, 0, sizeof(dongle_info));
    memcpy(&dongle_info.magic, "RTL0", 4);

    r = rtlsdr_get_tuner_type(dev);
    if (r >= 0)
      dongle_info.tuner_type = htonl(r);

    r = rtlsdr_get_tuner_gains(dev, NULL);
    if (r >= 0)
      dongle_info.tuner_gain_count = htonl(r);

    r = send(s, (const char *)&dongle_info, sizeof(dongle_info), 0);
    if (sizeof(dongle_info) != r)
      printf("Failed to send dongle information\n");

    pthread_attr_init(&attr);
    pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_JOINABLE);
    r = pthread_create(&tcp_worker_thread, &attr, tcp_worker, NULL);
    r = pthread_create(&command_thread, &attr, command_worker, NULL);
    pthread_attr_destroy(&attr);

    r = rtlsdr_read_async(dev, rtlsdr_callback, NULL, buf_num, 0);

    pthread_join(tcp_worker_thread, &status);
    pthread_join(command_thread, &status);

    closesocket(s);

    printf("All threads dead..\n");
    curelem = ll_buffers;
    ll_buffers = 0;

    while(curelem != 0) {
      prev = curelem;
      curelem = curelem->next;
      free(prev->data);
      free(prev);
    }

    do_exit = 0;
    global_numq = 0;
  }
}

