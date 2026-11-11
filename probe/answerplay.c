/*
 * answerplay - answering machine for the Apple USB Modem (05ac:1401).
 *
 * Waits for an incoming call, answers it, plays one of the given raw
 * audio files (picked at random), then hangs up when the file ends or
 * the caller hangs up, and waits for the next call.  Ctrl-C hangs up
 * and exits.
 *
 * usage: answerplay [-n rings] [-r rate] [-v] file.raw...
 *   files: 16-bit LE signed mono PCM at the stream rate (8000 Hz default)
 *     e.g. sox msg.wav -t raw -r 8000 -e signed -b 16 -c 1 msg.raw
 *   -n rings  answer after this many rings (default 2)
 *   -r rate   stream sample rate (default 8000)
 *   -v        log every USB notification and loop current reading
 *
 * Built on probe/isotest.c (same USB setup: uaudio/uamodem detached,
 * iso capture + playback, CDC hook control).  See NOTES.md.
 *
 * Ring detection: the firmware sends RING_DETECT for every cycle of the
 * ringing voltage (and continuously for mains hum on a bad line), so one
 * ring = a notification after >= 1 s without any.  Continuous hum then
 * counts as a single ring and never reaches -n 2.
 *
 * Caller hang-up detection (any of):
 *   - busy tone heard on the line: 440 Hz bursts of ~0.5 s on / 0.5 s off
 *     (France; 3 bursts), or a 440 Hz tone lasting > 3 s (dial tone);
 *   - loop current drop: was >= 10 mA, then < 3 mA for 300 ms (lines
 *     that signal disconnect by interrupting the loop).
 */
#include <err.h>
#include <libusb.h>
#include <math.h>
#include <pthread.h>
#include <signal.h>
#include <stdint.h>
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
#define IF_CMD 1
#define IF_CAP 3
#define IF_PLAY 4
#define NPKT 8
#define NXFER 4

#define DAA_DCTL1	5	/* bit 0 = OH (off-hook) */
#define DAA_LCS2	28	/* loop current, 1.1 mA/bit */
#define MIN_OFFHOOK	0.5	/* s; firmware ignores on-hook sooner */

static libusb_device_handle *h;
static volatile sig_atomic_t quit;
static volatile int running = 1;
static struct timespec t0;
static int rate = 8000;
static int verbose;

static double
now(void)
{
	struct timespec t;

	clock_gettime(CLOCK_MONOTONIC, &t);
	return (t.tv_sec - t0.tv_sec) + (t.tv_nsec - t0.tv_nsec) / 1e9;
}

#define LOG(...) do {						\
	printf("[%8.3f] ", now());				\
	printf(__VA_ARGS__);					\
	printf("\n");						\
} while (0)

static void
onsig(int s)
{
	(void)s;
	quit = 1;
}

/* ------------------------------------------------------------------ */
/* Audio files                                                         */

struct clip {
	const char *name;
	int16_t *pcm;
	size_t n;
};
static struct clip *clips;
static int nclips;

static void
load_clip(struct clip *c, const char *name)
{
	FILE *f;
	long sz;

	if ((f = fopen(name, "r")) == NULL)
		err(1, "%s", name);
	fseek(f, 0, SEEK_END);
	sz = ftell(f);
	rewind(f);
	if (sz < 2)
		errx(1, "%s: empty", name);
	c->name = name;
	c->n = sz / 2;
	if ((c->pcm = malloc(c->n * 2)) == NULL)
		err(1, "malloc");
	if (fread(c->pcm, 2, c->n, f) != c->n)
		err(1, "%s: read", name);
	fclose(f);
}

/* ------------------------------------------------------------------ */
/* Playback: the iso OUT callbacks pull from the current clip.         */

static pthread_mutex_t pmtx = PTHREAD_MUTEX_INITIALIZER;
static const int16_t *pbuf;
static size_t plen, ppos;
static double pfrac;

static void
play_start(const struct clip *c)
{
	pthread_mutex_lock(&pmtx);
	pbuf = c->pcm;
	plen = c->n;
	ppos = 0;
	pthread_mutex_unlock(&pmtx);
}

static void
play_stop(void)
{
	pthread_mutex_lock(&pmtx);
	pbuf = NULL;
	plen = ppos = 0;
	pthread_mutex_unlock(&pmtx);
}

static int
play_done(void)
{
	int d;

	pthread_mutex_lock(&pmtx);
	d = ppos >= plen;
	pthread_mutex_unlock(&pmtx);
	return (d);
}

static void
fill_play(struct libusb_transfer *x)
{
	unsigned char *b = x->buffer;
	size_t off = 0;

	pthread_mutex_lock(&pmtx);
	for (int i = 0; i < x->num_iso_packets; i++) {
		/* rate/1000 samples per 1 ms frame, carrying the fraction */
		pfrac += rate / 1000.0;
		int ns = (int)pfrac;
		pfrac -= ns;
		x->iso_packet_desc[i].length = ns * 2;
		for (int j = 0; j < ns; j++) {
			int16_t v = ppos < plen ? pbuf[ppos++] : 0;
			memcpy(b + off + j * 2, &v, 2);
		}
		off += ns * 2;
	}
	pthread_mutex_unlock(&pmtx);
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

/* ------------------------------------------------------------------ */
/* Hang-up tone detector on the captured line audio (50 ms blocks).    */

#define BLK		400		/* 50 ms at 8 kHz */
#define BLK_MS(ms)	((ms) / 50)
#define TONE_LEVEL	25		/* RMS: idle line ~7 */

static volatile int det_armed, det_hangup;
static double det_dc, det_s1, det_s2, det_e, det_coef;
static int det_n, det_on, det_len, det_off, det_bursts;

static void
det_reset(void)
{
	det_hangup = 0;
	det_dc = det_s1 = det_s2 = det_e = 0;
	det_n = det_on = det_len = det_off = det_bursts = 0;
	det_coef = 2 * cos(2 * M_PI * 440 / rate);
}

static void
det_block(void)
{
	double p = det_s1 * det_s1 + det_s2 * det_s2 - det_coef * det_s1 * det_s2;
	double rms = sqrt(det_e / BLK);
	double frac = det_e > 0 ? 2 * p / ((double)BLK * det_e) : 0;
	int tone = rms > TONE_LEVEL && frac > 0.4;

	if (tone) {
		if (!det_on) {
			det_on = 1;
			det_len = 0;
			/* a gap that is not a busy gap breaks the cadence */
			if (det_off > BLK_MS(800))
				det_bursts = 0;
		}
		det_off = 0;
		if (++det_len >= BLK_MS(3000) && !det_hangup) {
			LOG("continuous 440 Hz tone: caller gone");
			det_hangup = 1;
		}
	} else {
		det_off++;
		/* burst ends after 2 blocks without tone (ignores flicker) */
		if (det_on && det_off >= 2) {
			int ms = det_len * 50;

			det_on = 0;
			if (ms >= 300 && ms <= 750) {
				det_bursts++;
				if (verbose)
					LOG("busy burst %d (%d ms)", det_bursts,
					    ms);
				if (det_bursts >= 3 && !det_hangup) {
					LOG("busy tone: caller hung up");
					det_hangup = 1;
				}
			} else
				det_bursts = 0;
		}
		if (det_off > BLK_MS(1500))
			det_bursts = 0;
	}
}

static void
det_sample(int16_t v)
{
	double x, s;

	det_dc += (v - det_dc) / 64;
	x = v - det_dc;
	s = x + det_coef * det_s1 - det_s2;
	det_s2 = det_s1;
	det_s1 = s;
	det_e += x * x;
	if (++det_n == BLK) {
		det_block();
		det_n = 0;
		det_s1 = det_s2 = det_e = 0;
	}
}

static void LIBUSB_CALL
cap_cb(struct libusb_transfer *x)
{
	for (int i = 0; i < x->num_iso_packets; i++) {
		struct libusb_iso_packet_descriptor *d = &x->iso_packet_desc[i];
		int16_t *s = (int16_t *)libusb_get_iso_packet_buffer_simple(x, i);

		if (d->status != LIBUSB_TRANSFER_COMPLETED || !det_armed)
			continue;
		for (unsigned j = 0; j < d->actual_length / 2; j++)
			det_sample(s[j]);
	}
	if (running)
		libusb_submit_transfer(x);
}

/* ------------------------------------------------------------------ */
/* Notifications: rings, line state, command responses.                */

static volatile int ring_events;
static double last_ring_notify = -10;

static void LIBUSB_CALL
int_cb(struct libusb_transfer *x)
{
	unsigned char *b = x->buffer;

	if (x->status == LIBUSB_TRANSFER_COMPLETED && x->actual_length >= 2) {
		if (verbose) {
			printf("[%8.3f] notify:", now());
			for (int i = 0; i < x->actual_length; i++)
				printf(" %02x", b[i]);
			printf("\n");
		}
		if (b[1] == 0x09) {		/* RING_DETECT */
			double t = now();

			if (t - last_ring_notify > 1.0)
				ring_events++;
			last_ring_notify = t;
		} else if (b[1] == 0x29 && x->actual_length > 2 && verbose)
			LOG("line state %d", b[2]);
	}
	if (running)
		libusb_submit_transfer(x);
}

static void *
evloop(void *a)
{
	struct timeval tv = { 0, 100000 };

	(void)a;
	while (running)
		libusb_handle_events_timeout(NULL, &tv);
	return (NULL);
}

/* ------------------------------------------------------------------ */
/* Control: hook and DAA registers.                                    */

static int
ctrl(uint8_t rt, uint8_t req, uint16_t val, uint16_t idx, unsigned char *d,
    uint16_t len)
{
	return (libusb_control_transfer(h, rt, req, val, idx, d, len, 1000));
}

static int
daa_cmd(uint8_t op, uint8_t a, uint8_t b)
{
	unsigned char c[3] = { op, a, b };

	return (ctrl(0x21, 0x00, 0, IF_CMD, c, 3) < 0 ? -1 : 0);
}

/* Read a DAA register: encapsulated command, then poll for the reply. */
static int
daa_read(uint8_t reg, uint8_t *v)
{
	unsigned char r[8];

	if (daa_cmd(0x80, reg, 0) != 0)
		return (-1);
	for (int i = 0; i < 10; i++) {
		usleep(10000);
		int n = ctrl(0xa1, 0x01, 0, IF_CMD, r, sizeof(r));
		if (n < 0)
			return (-1);
		if (n > 0) {
			*v = r[0];
			return (0);
		}
	}
	return (-1);
}

static double offhook_at;

static void
hook(int off)
{
	/* a request for the current state STALLs: ignore errors */
	(void)ctrl(0x21, 0x11, off, 0, NULL, 0);
	if (off)
		offhook_at = now();
}

/*
 * Go on-hook, insisting: the firmware ignores an on-hook request in the
 * first ~400 ms after going off-hook, and sometimes ACKs one without
 * acting on it.  DAA register 5 bit 0 is the real hook switch.
 */
static void
hangup(void)
{
	double wait = offhook_at + MIN_OFFHOOK - now();
	uint8_t v;

	if (wait > 0)
		usleep(wait * 1e6);
	for (int i = 0; i < 10; i++) {
		hook(0);
		usleep(100000);
		if (daa_read(DAA_DCTL1, &v) != 0 || !(v & 1))
			return;
	}
	LOG("firmware ignores on-hook, clearing DAA OH bit");
	(void)daa_cmd(0x02, DAA_DCTL1, 0xfe);
}

/* ------------------------------------------------------------------ */

static void
usage(void)
{
	fprintf(stderr,
	    "usage: answerplay [-n rings] [-r rate] [-v] file.raw...\n");
	exit(1);
}

int
main(int argc, char **argv)
{
	struct libusb_transfer *ix, *cx[NXFER], *px[NXFER];
	struct sigaction sa;
	pthread_t ev;
	int ch, r, nrings = 2, calls = 0;

	setvbuf(stdout, NULL, _IOLBF, 0);
	clock_gettime(CLOCK_MONOTONIC, &t0);
	while ((ch = getopt(argc, argv, "n:r:v")) != -1) {
		switch (ch) {
		case 'n': nrings = atoi(optarg); break;
		case 'r': rate = atoi(optarg); break;
		case 'v': verbose = 1; break;
		default: usage();
		}
	}
	argc -= optind;
	argv += optind;
	if (argc < 1 || nrings < 1 || rate < 1000 || rate > 48000)
		usage();
	nclips = argc;
	clips = calloc(nclips, sizeof(*clips));
	for (int i = 0; i < nclips; i++) {
		load_clip(&clips[i], argv[i]);
		LOG("loaded %s (%.1f s)", argv[i], (double)clips[i].n / rate);
	}

	memset(&sa, 0, sizeof(sa));
	sa.sa_handler = onsig;		/* no SA_RESTART: interrupt sleeps */
	sigaction(SIGINT, &sa, NULL);
	sigaction(SIGTERM, &sa, NULL);
	sigaction(SIGHUP, &sa, NULL);

	if ((r = libusb_init(NULL)) < 0)
		errx(1, "libusb_init: %s", libusb_error_name(r));
	h = libusb_open_device_with_vid_pid(NULL, VID, PID);
	if (h == NULL)
		errx(1, "modem %04x:%04x not found (root?)", VID, PID);
	for (int i = 0; i <= 4; i++) {
		if (libusb_kernel_driver_active(h, i) == 1 &&
		    (r = libusb_detach_kernel_driver(h, i)) < 0)
			LOG("detach if%d: %s", i, libusb_error_name(r));
		if ((r = libusb_claim_interface(h, i)) < 0)
			errx(1, "claim if%d: %s", i, libusb_error_name(r));
	}
	if ((r = libusb_set_interface_alt_setting(h, IF_CAP, 1)) < 0)
		errx(1, "alt cap: %s", libusb_error_name(r));
	if ((r = libusb_set_interface_alt_setting(h, IF_PLAY, 1)) < 0)
		errx(1, "alt play: %s", libusb_error_name(r));
	for (int ep = 0; ep < 2; ep++) {
		unsigned char d[3] = { rate & 0xff, (rate >> 8) & 0xff,
		    rate >> 16 };
		/* SET_CUR SAMPLING_FREQ */
		ctrl(0x22, 0x01, 0x0100, ep ? EP_PLAY : EP_CAP, d, 3);
	}

	ix = libusb_alloc_transfer(0);
	libusb_fill_interrupt_transfer(ix, h, EP_INT, malloc(64), 64, int_cb,
	    NULL, 0);
	libusb_submit_transfer(ix);
	int cmax = libusb_get_max_iso_packet_size(libusb_get_device(h), EP_CAP);
	if (cmax <= 0)
		cmax = 24;
	for (int i = 0; i < NXFER; i++) {
		cx[i] = libusb_alloc_transfer(NPKT);
		libusb_fill_iso_transfer(cx[i], h, EP_CAP, malloc(NPKT * cmax),
		    NPKT * cmax, NPKT, cap_cb, NULL, 0);
		libusb_set_iso_packet_lengths(cx[i], cmax);
		if ((r = libusb_submit_transfer(cx[i])) < 0)
			errx(1, "submit cap: %s", libusb_error_name(r));
		px[i] = libusb_alloc_transfer(NPKT);
		libusb_fill_iso_transfer(px[i], h, EP_PLAY, malloc(NPKT * 96),
		    NPKT * 96, NPKT, play_cb, NULL, 0);
		fill_play(px[i]);
		if ((r = libusb_submit_transfer(px[i])) < 0)
			errx(1, "submit play: %s", libusb_error_name(r));
	}
	pthread_create(&ev, NULL, evloop, NULL);

	/* start from a known state */
	hook(0);
	offhook_at = -10;

	while (!quit) {
		/* ---- wait for the call ---- */
		int rings = 0;
		double last_ring = 0, ignore_until = now() + 2;

		ring_events = 0;
		LOG("waiting for a call (answer after %d ring%s)", nrings,
		    nrings > 1 ? "s" : "");
		while (!quit && rings < nrings) {
			if (ring_events > 0) {
				ring_events = 0;
				if (now() >= ignore_until) {
					rings++;
					last_ring = now();
					LOG("RING %d", rings);
				}
			}
			if (rings > 0 && now() - last_ring > 8) {
				LOG("ringing stopped");
				rings = 0;
			}
			usleep(50000);
		}
		if (quit)
			break;

		/* ---- answer and play ---- */
		const struct clip *c = &clips[arc4random_uniform(nclips)];
		int lc_seen = 0, lc_low = 0;
		double next_poll = 0, end_at = 0;
		const char *why = NULL;

		calls++;
		LOG("call %d: answering, playing %s", calls, c->name);
		hook(1);
		usleep(500000);		/* firmware line check, ~430 ms */
		det_reset();
		det_armed = 1;
		play_start(c);
		while (why == NULL) {
			if (quit)
				why = "interrupted";
			else if (det_hangup)
				why = "caller hung up (tone)";
			else if (play_done()) {
				/* let the queued iso transfers drain */
				if (end_at == 0)
					end_at = now() + 0.3;
				else if (now() >= end_at)
					why = "end of file";
			}
			if (why == NULL && now() >= next_poll) {
				uint8_t lc;

				next_poll = now() + 0.1;
				if (daa_read(DAA_LCS2, &lc) == 0) {
					if (verbose)
						LOG("loop current %.1f mA",
						    lc * 1.1);
					if (lc * 1.1 >= 10)
						lc_seen = 1;
					if (lc_seen && lc * 1.1 < 3) {
						if (++lc_low >= 3)
							why = "caller hung up "
							    "(loop current lost)";
					} else
						lc_low = 0;
				}
			}
			usleep(20000);
		}
		det_armed = 0;
		play_stop();
		LOG("hanging up: %s", why);
		hangup();
	}

	LOG("exiting after %d call%s", calls, calls == 1 ? "" : "s");
	hangup();
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
	libusb_close(h);
	libusb_exit(NULL);
	return (0);
}
