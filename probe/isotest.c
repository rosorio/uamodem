/*
 * isotest - drive the Apple USB Modem audio streams directly with libusb
 * (uaudio detached), together with hook control and notifications.
 *
 * usage: isotest [-o capture.raw] [-p play.raw] [-r rate] op...
 *   ops (executed sequentially on a timeline thread):
 *     hN        SET_HOOK_STATE N (interface 0)
 *     sMS       sleep
 *     c:RT:REQ:VAL:IDX:LEN|HEX   raw control request
 *     play      start feeding play file (otherwise silence is sent)
 *     dtmf:DIGITS  queue DTMF digits into playback
 *     answer:SECS  wait until the call is answered (ringback stops or
 *                  voice is heard); hang up and stop if SECS pass first
 * capture: 16-bit LE mono at the selected rate.
 */
#include <err.h>
#include <libusb.h>
#include <math.h>
#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

#define VID 0x05ac
#define PID 0x1401
#define EP_INT 0x81
#define EP_CAP 0x82
#define EP_PLAY 0x03
#define IF_CAP 3
#define IF_PLAY 4
#define NPKT 8
#define NXFER 4

static libusb_device_handle *h;
static volatile int running = 1;
static struct timespec t0;
static int rate = 8000;
static FILE *capf;
static long cap_samples, cap_pkts, cap_zero;
static double cap_energy;

/*
 * Answer detection on the captured line audio, in 50 ms blocks.
 *   ringback = loud, energy mostly at 440 Hz (France: 1.5 s on, 3.5 s off)
 *   sound    = loud, not ringback (voice, "allo", music...)
 * The call counts as answered when:
 *   1. 200 ms of sound is heard, or
 *   2. a ringback burst stops early (< 1.2 s instead of 1.5 s): the
 *      pickup cut the tone, or
 *   3. ringback was heard and the next burst is overdue (3.8 s of quiet
 *      after a burst; the normal gap is 3.5 s): picked up between rings.
 */
static double now(void);
#define ANS_BLOCK	400		/* 50 ms at 8 kHz */
#define ANS_MS(ms)	((ms) / 50)
static double ans_dc, ans_s1, ans_s2, ans_e;
static int ans_n;
static volatile int ans_answered, ans_armed;
static double ans_level = 25;	/* RMS threshold: idle line ~7, faint voice ~15-50 */
static long ans_file_samples = -1;	/* >= 0: offline mode (-a), clock */

/* detector state, reset by ans_reset() */
static int ans_ring_on, ans_ring_len, ans_ring_full, ans_bursts;
static int ans_off, ans_gap, ans_voice, ans_first;

static double
ans_now(void)
{
	return (ans_file_samples >= 0 ? ans_file_samples / 8000.0 : now());
}

static void
ans_reset(void)
{
	ans_answered = 0;
	ans_ring_on = ans_ring_len = ans_ring_full = ans_bursts = 0;
	ans_off = ans_gap = ans_voice = 0;
	ans_first = 1;
	ans_dc = ans_s1 = ans_s2 = ans_e = 0;
	ans_n = 0;
}

static void
ans_set(const char *why, double rms)
{
	if (ans_answered)
		return;
	printf("[%7.3f] answered: %s (rms %.0f)\n", ans_now(), why, rms);
	fflush(stdout);
	ans_answered = 1;
}

static void
ans_block(void)
{
	double c = 2 * cos(2 * M_PI * 440 / rate);
	double p440 = (ans_s1 * ans_s1 + ans_s2 * ans_s2 - c * ans_s1 * ans_s2);
	double rms = sqrt(ans_e / ANS_BLOCK);
	/* fraction of the block's energy that is at 440 Hz */
	double frac = ans_e > 0 ? 2 * p440 / ((double)ANS_BLOCK * ans_e) : 0;
	int loud = rms > ans_level;
	int ring = loud && frac > 0.5;

	if (ring) {
		ans_off = ans_voice = 0;
		if (!ans_ring_on) {
			ans_ring_on = 1;
			ans_ring_len = 0;
			/* a burst already on when we start is of unknown length */
			ans_ring_full = !ans_first;
			printf("[%7.3f] ringback\n", ans_now());
			fflush(stdout);
		}
		ans_ring_len++;
	} else {
		ans_off++;
		/* end of a burst: 2 blocks without tone (ignores a flicker) */
		if (ans_ring_on && ans_off >= 2) {
			int ms = (ans_ring_len) * 50;

			ans_ring_on = 0;
			ans_bursts++;
			ans_gap = ans_off;
			if (ans_ring_full && ms < 1200) {
				printf("[%7.3f] ringback cut after %d ms\n",
				    ans_now(), ms);
				ans_set("ringback interrupted", rms);
			}
		} else if (!ans_ring_on && ans_bursts > 0) {
			ans_gap++;
			if (ans_gap >= ANS_MS(3800))
				ans_set("next ringback overdue", rms);
		}
		if (loud) {
			if (++ans_voice >= ANS_MS(200))
				ans_set("sound", rms);
		} else
			ans_voice = 0;
	}
	ans_first = 0;
}

static void
ans_sample(int16_t v)
{
	double x, s;

	if (!ans_armed)
		return;
	ans_dc += (v - ans_dc) / 64;
	x = v - ans_dc;
	s = x + 2 * cos(2 * M_PI * 440 / rate) * ans_s1 - ans_s2;
	ans_s2 = ans_s1;
	ans_s1 = s;
	ans_e += x * x;
	if (++ans_n == ANS_BLOCK) {
		ans_block();
		ans_n = 0;
		ans_s1 = ans_s2 = ans_e = 0;
	}
}

/* -a FILE:START  run the answer detector over a raw 8 kHz recording */
static int
ans_offline(const char *arg)
{
	char *name = strdup(arg), *c = strrchr(name, ':');
	double start = 0;
	int16_t v;
	FILE *f;

	if (c != NULL) {
		*c = 0;
		start = atof(c + 1);
	}
	if ((f = fopen(name, "r")) == NULL)
		err(1, "%s", name);
	ans_reset();
	ans_armed = 1;
	for (ans_file_samples = 0; fread(&v, 2, 1, f) == 1 && !ans_answered;
	    ans_file_samples++)
		if (ans_file_samples >= start * 8000)
			ans_sample(v);
	if (!ans_answered)
		printf("no answer detected\n");
	return (0);
}

/* Returns 0 when answered, -1 on timeout. */
static int
wait_answer(double secs)
{
	double end = now() + secs;

	ans_reset();
	ans_armed = 1;
	printf("[%7.3f] waiting up to %.0f s for an answer\n", now(), secs);
	while (now() < end && !ans_answered)
		usleep(20000);
	ans_armed = 0;
	return (ans_answered ? 0 : -1);
}

/* playback ring */
static int16_t *pbuf;
static size_t plen, ppos;
static pthread_mutex_t pmtx = PTHREAD_MUTEX_INITIALIZER;
static double frac;

static double
now(void)
{
	struct timespec t;
	clock_gettime(CLOCK_MONOTONIC, &t);
	return (t.tv_sec - t0.tv_sec) + (t.tv_nsec - t0.tv_nsec) / 1e9;
}

static int
ctrl(uint8_t rt, uint8_t req, uint16_t val, uint16_t idx, unsigned char *d,
    uint16_t len)
{
	int r = libusb_control_transfer(h, rt, req, val, idx, d, len, 1000);
	printf("[%7.3f] ctrl %02x %02x %04x %04x %u -> ", now(), rt, req, val,
	    idx, len);
	if (r < 0)
		printf("%s\n", libusb_error_name(r));
	else {
		printf("%d", r);
		if (rt & 0x80)
			for (int i = 0; i < r; i++)
				printf(" %02x", d[i]);
		printf("\n");
	}
	return r;
}

static void
queue_pcm(const int16_t *s, size_t n)
{
	pthread_mutex_lock(&pmtx);
	pbuf = realloc(pbuf, (plen + n) * 2);
	memcpy(pbuf + plen, s, n * 2);
	plen += n;
	pthread_mutex_unlock(&pmtx);
}

static void
queue_dtmf(const char *digits)
{
	static const char *keys = "123A456B789C*0#D";
	static const double lo[] = { 697, 770, 852, 941 };
	static const double hi[] = { 1209, 1336, 1477, 1633 };
	int n = rate * 120 / 1000, gap = rate * 100 / 1000;
	int16_t *b = calloc(n + gap, 2);
	for (const char *d = digits; *d; d++) {
		const char *p = strchr(keys, *d);
		if (p == NULL)
			continue;
		int k = p - keys;
		for (int i = 0; i < n; i++)
			b[i] = 7000 * (sin(2 * M_PI * lo[k / 4] * i / rate) +
			    sin(2 * M_PI * hi[k % 4] * i / rate));
		memset(b + n, 0, gap * 2);
		queue_pcm(b, n + gap);
	}
	free(b);
}

static void LIBUSB_CALL
cap_cb(struct libusb_transfer *x)
{
	for (int i = 0; i < x->num_iso_packets; i++) {
		struct libusb_iso_packet_descriptor *d = &x->iso_packet_desc[i];
		unsigned char *p = libusb_get_iso_packet_buffer_simple(x, i);
		cap_pkts++;
		if (d->status != LIBUSB_TRANSFER_COMPLETED || d->actual_length == 0) {
			cap_zero++;
			continue;
		}
		int16_t *s = (int16_t *)p;
		for (unsigned j = 0; j < d->actual_length / 2; j++)
			cap_energy += (double)s[j] * s[j];
		cap_samples += d->actual_length / 2;
		for (unsigned j = 0; j < d->actual_length / 2; j++)
			ans_sample(s[j]);
		if (capf)
			fwrite(p, 1, d->actual_length, capf);
	}
	if (running)
		libusb_submit_transfer(x);
}

static void
fill_play(struct libusb_transfer *x)
{
	unsigned char *b = x->buffer;
	size_t off = 0;
	for (int i = 0; i < x->num_iso_packets; i++) {
		/* rate/1000 samples per 1 ms frame, carrying the fraction */
		frac += rate / 1000.0;
		int ns = (int)frac;
		frac -= ns;
		x->iso_packet_desc[i].length = ns * 2;
		pthread_mutex_lock(&pmtx);
		for (int j = 0; j < ns; j++) {
			int16_t v = ppos < plen ? pbuf[ppos++] : 0;
			memcpy(b + off + j * 2, &v, 2);
		}
		pthread_mutex_unlock(&pmtx);
		off += ns * 2;
	}
	x->length = off;
}

static void LIBUSB_CALL
play_cb(struct libusb_transfer *x)
{
	if (!running)
		return;
	fill_play(x);
	libusb_submit_transfer(x);
}

static void LIBUSB_CALL
int_cb(struct libusb_transfer *x)
{
	if (x->status == LIBUSB_TRANSFER_COMPLETED) {
		printf("[%7.3f] NOTIFY:", now());
		for (int i = 0; i < x->actual_length; i++)
			printf(" %02x", x->buffer[i]);
		printf("\n");
	}
	if (running)
		libusb_submit_transfer(x);
}

static void *
evloop(void *a)
{
	(void)a;
	struct timeval tv = { 0, 100000 };
	double last = 0;
	while (running) {
		libusb_handle_events_timeout(NULL, &tv);
		if (now() - last >= 0.5) {
			printf("[%7.3f] cap: pkts=%ld empty=%ld samples=%ld rms=%.1f\n",
			    now(), cap_pkts, cap_zero, cap_samples,
			    cap_samples ? sqrt(cap_energy / cap_samples) : 0);
			cap_energy = 0;
			cap_samples = 0;
			last = now();
		}
	}
	return NULL;
}

static void
set_rate(int ep)
{
	unsigned char d[3] = { rate & 0xff, (rate >> 8) & 0xff, rate >> 16 };
	ctrl(0x22, 0x01, 0x0100, ep, d, 3);	/* SET_CUR SAMPLING_FREQ */
	ctrl(0xa2, 0x81, 0x0100, ep, d, 3);	/* GET_CUR */
}

int
main(int argc, char **argv)
{
	const char *capname = NULL, *playname = NULL;
	int ch, r;
	pthread_t ev;

	setvbuf(stdout, NULL, _IOLBF, 0);
	clock_gettime(CLOCK_MONOTONIC, &t0);
	while ((ch = getopt(argc, argv, "a:o:p:r:")) != -1) {
		switch (ch) {
		case 'a': return (ans_offline(optarg));
		case 'o': capname = optarg; break;
		case 'p': playname = optarg; break;
		case 'r': rate = atoi(optarg); break;
		default: errx(1, "usage");
		}
	}
	argc -= optind;
	argv += optind;

	if (capname && (capf = fopen(capname, "w")) == NULL)
		err(1, "%s", capname);
	libusb_init(NULL);
	h = libusb_open_device_with_vid_pid(NULL, VID, PID);
	if (h == NULL)
		errx(1, "modem not found");
	for (int i = 0; i <= 4; i++) {
		if (libusb_kernel_driver_active(h, i) == 1) {
			r = libusb_detach_kernel_driver(h, i);
			printf("detach if%d: %s\n", i, r ? libusb_error_name(r) : "ok");
		}
		if ((r = libusb_claim_interface(h, i)) < 0)
			printf("claim if%d: %s\n", i, libusb_error_name(r));
	}
	if ((r = libusb_set_interface_alt_setting(h, IF_CAP, 1)) < 0)
		errx(1, "alt cap: %s", libusb_error_name(r));
	if ((r = libusb_set_interface_alt_setting(h, IF_PLAY, 1)) < 0)
		errx(1, "alt play: %s", libusb_error_name(r));
	set_rate(EP_CAP);
	set_rate(EP_PLAY);

	/* interrupt */
	struct libusb_transfer *ix = libusb_alloc_transfer(0);
	libusb_fill_interrupt_transfer(ix, h, EP_INT, malloc(64), 64, int_cb,
	    NULL, 0);
	libusb_submit_transfer(ix);
	/* capture + playback iso */
	struct libusb_transfer *cx[NXFER], *px[NXFER];
	for (int i = 0; i < NXFER; i++) {
		cx[i] = libusb_alloc_transfer(NPKT);
		libusb_fill_iso_transfer(cx[i], h, EP_CAP, malloc(NPKT * 24),
		    NPKT * 24, NPKT, cap_cb, NULL, 0);
		libusb_set_iso_packet_lengths(cx[i], 24);
		if ((r = libusb_submit_transfer(cx[i])) < 0)
			errx(1, "submit cap: %s", libusb_error_name(r));
		px[i] = libusb_alloc_transfer(NPKT);
		libusb_fill_iso_transfer(px[i], h, EP_PLAY, malloc(NPKT * 24),
		    NPKT * 24, NPKT, play_cb, NULL, 0);
		fill_play(px[i]);
		if ((r = libusb_submit_transfer(px[i])) < 0)
			errx(1, "submit play: %s", libusb_error_name(r));
	}
	pthread_create(&ev, NULL, evloop, NULL);

	for (int a = 0; a < argc; a++) {
		char *op = argv[a];
		if (op[0] == 'h')
			ctrl(0x21, 0x11, atoi(op + 1), 0, NULL, 0);
		else if (op[0] == 's')
			usleep(atoi(op + 1) * 1000);
		else if (!strncmp(op, "dtmf:", 5)) {
			printf("[%7.3f] dtmf %s\n", now(), op + 5);
			queue_dtmf(op + 5);
		} else if (!strcmp(op, "play") && playname) {
			FILE *f = fopen(playname, "r");
			if (f == NULL)
				err(1, "%s", playname);
			int16_t b[4096];
			size_t n;
			while ((n = fread(b, 2, 4096, f)) > 0)
				queue_pcm(b, n);
			fclose(f);
			printf("[%7.3f] play queued %zu samples\n", now(), plen - ppos);
		} else if (!strncmp(op, "answer:", 7)) {
			if (wait_answer(atof(op + 7)) != 0) {
				printf("[%7.3f] no answer: hanging up\n", now());
				ctrl(0x21, 0x11, 0, 0, NULL, 0);
				break;
			}
		} else if (!strncmp(op, "c:", 2)) {
			unsigned v[4], len = 0;
			char data[200];
			unsigned char buf[256];
			if (sscanf(op + 2, "%x:%x:%x:%x:%199s", &v[0], &v[1], &v[2],
			    &v[3], data) != 5)
				errx(1, "bad op %s", op);
			if (v[0] & 0x80)
				len = strtoul(data, NULL, 0);
			else
				for (const char *p = data; p[0] && p[1]; p += 2)
					sscanf(p, "%2hhx", &buf[len++]);
			ctrl(v[0], v[1], v[2], v[3], buf, len);
		} else
			errx(1, "bad op %s", op);
	}
	usleep(300000);
	running = 0;
	pthread_join(ev, NULL);
	struct timeval tv = { 0, 200000 };
	for (int i = 0; i < NXFER; i++) {
		libusb_cancel_transfer(cx[i]);
		libusb_cancel_transfer(px[i]);
	}
	libusb_cancel_transfer(ix);
	libusb_handle_events_timeout(NULL, &tv);
	libusb_set_interface_alt_setting(h, IF_CAP, 0);
	libusb_set_interface_alt_setting(h, IF_PLAY, 0);
	for (int i = 0; i <= 4; i++) {
		libusb_release_interface(h, i);
		libusb_attach_kernel_driver(h, i);
	}
	if (capf)
		fclose(capf);
	libusb_close(h);
	libusb_exit(NULL);
	return 0;
}
