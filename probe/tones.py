#!/usr/bin/env python3.12
"""Analyse an 8 kHz s16le mono recording: per-window RMS and dominant
call-progress tones (Goertzel on a set of known frequencies)."""
import math, struct, sys

RATE = 8000
FREQS = [350, 400, 425, 440, 450, 480, 620, 697, 770, 852, 941,
         1209, 1336, 1477, 1633, 2100]

def goertzel(x, f):
    w = 2 * math.pi * f / RATE
    c = 2 * math.cos(w)
    s1 = s2 = 0.0
    for v in x:
        s1, s2 = v + c * s1 - s2, s1
    return math.sqrt(max(s1 * s1 + s2 * s2 - c * s1 * s2, 0)) / len(x)

data = open(sys.argv[1], 'rb').read()
x = struct.unpack('<%dh' % (len(data) // 2), data[:len(data) // 2 * 2])
win = int(RATE * float(sys.argv[2] if len(sys.argv) > 2 else 0.25))
for i in range(0, len(x) - win + 1, win):
    seg = x[i:i + win]
    mean = sum(seg) / win
    seg = [v - mean for v in seg]
    rms = math.sqrt(sum(v * v for v in seg) / win)
    mags = sorted(((goertzel(seg, f), f) for f in FREQS), reverse=True)[:3]
    print('%6.2fs rms=%8.1f dc=%7.1f  %s' % (
        i / RATE, rms, mean,
        '  '.join('%dHz:%.0f' % (f, m) for m, f in mags)))
