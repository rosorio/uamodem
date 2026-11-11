# Apple USB Modem (05ac:1401) on FreeBSD — reverse-engineering notes

## Hardware
- Motorola SM56 USB softmodem firmware (bcdDevice 2.02, fw version `02 02`).
- DAA: **Silicon Labs Si3050 (system side) + Si3018 (line side)**; reg 11 = 0x14
  (LSID=1 → Si3018, rev 4). Register map = Si3050 datasheet; DAHDI
  `wctdm24xxp/base.c` + `fxo_modes.h` (in `re/docs/dahdi/`) document usage.
- Source driver: OS X Lion `SM56KUSBAudioFamily.kext` (in `re/`), symbols intact.

## USB interfaces
| if | class | use |
|----|-------|-----|
| 0 | CDC TCM (comm) | `SET_HOOK_STATE` (0x21/0x11, wIndex 0, wValue 0=on,1=off); notifications on EP 0x81 |
| 1 | vendor | DAA/LED command channel via CDC encapsulated cmds; `GET/SET_COMM_FEATURE(2)` = country (ISO numeric as BCD-ish hex, e.g. 0x0250 France, 0x0840 USA) |
| 2 | audio control | — |
| 3 alt1 | audio stream IN, EP 0x82 iso, 16-bit mono | line → host |
| 4 alt1 | audio stream OUT, EP 0x03 iso | host → line |
| 5 | DFU | **do not touch** |

Sample rate: `SET_CUR` (0x22/0x01, wValue 0x0100, wIndex=EP) 3-byte LE; 8000 Hz → 8 samples per 1 ms frame.

## Command channel (interface 1)
`SEND_ENCAPSULATED_COMMAND` (bmRT 0x21, bReq 0x00, wValue 0, wIndex 1, 3 bytes `[op, a, b]`).
Replies: notification `a1 02 ...` then `GET_ENCAPSULATED_RESPONSE` (0xa1/0x01, wIndex 1).

| op | meaning |
|----|---------|
| 00 reg val | write DAA register |
| 01 / 02 / 03 reg val | OR / AND / XOR DAA register |
| 80 reg 00 | read DAA register (1-byte reply) |
| 10 00 val | set LEDs (11/12/13 presumably OR/AND/XOR) |
| 20 a b | Wait |
| 30 00 00 | flush command buffer |
| 40 00 00 | reset DAA |
| d0 00 00 | firmware version (2-byte reply) |

## Notifications (EP 0x81, 8 bytes)
- `a1 29 xx 00 01 ..` LINE_STATE_CHANGE (seen 2,3,4,5,7)
- `a1 09 ..` RING_DETECT, `a1 08 0x ..` AUX_JACK_HOOK_STATE, `a1 02 ..` response available

## Useful DAA registers (Si3050)
5 OH (bit0) / ONHM (bit3); 6 power-down; 10 digital loopback (bit0); 12 FDT(6) + LCS loop current;
16/26/30/31 country (France per DAHDI: 16=00, 26=c2, 30=02, 31=ab); 28 loop current; 29 line voltage (signed V).

## Status (2026-11-10)
- Hook control, notifications, DAA register access: working.
- Isochronous audio both directions: working — verified with Si3050 digital loopback (DTMF 0626 decoded).
- Line side: on-hook line voltage oscillates ±20 V at 50 Hz (no DC battery); off-hook loop current 0 →
  **line appears dead / open (mains hum pickup)**. Needs physical verification.

## Line-side diagnosis (2026-11-10, after cable fix)
- LINE_STATE values decoded from Apple kext `intReadComplete`: **4 = RJ11 "Unplugged", 5 = "Plugged"**.
  Every off-hook (CDC hook or reg 5 write, US or France profile, after DAA reset, either line
  polarity) → `2,7,4`: firmware itself reports the line vanishing when the loop is closed.
- On-hook: LVS ±20 V 50 Hz AC only, never the line DC; vanishes when cable unplugged.
- Off-hook: LCS=0, LVS=0, ADC pinned at -31703.
- Outgoing call (DTMF dial 0626852858 + TTS message): phone did not ring.
- Incoming call answered twice: caller kept hearing ringback, nothing heard → **modem cannot seize the line**.
- Same cable/adapter + plain phone: dial tone OK. Conclusion: fault in the modem's line-side DC path
  (between RJ11 jack and Si3018: fuse/PTC, surge protector, bridge, hook-switch transistor).

## Hook-state firmware behaviour (measured 2026-11-10)
- SET_HOOK_STATE for the current state may STALL (seen as LIBUSB_ERROR_NO_DEVICE in libusb).
- After off-hook the firmware runs a line check (~430 ms, LINE_STATE 7 then 4/5). An on-hook
  sent within ~400 ms is ACKed but IGNORED (line stays seized). Truth = DAA reg 5 bit 0 (OH).
- Reg 5 OH bit can be toggled directly with ops 01/02 (OR/AND); the firmware doesn't fight it.
  Clearing it reads back 0x19, setting it reads back 0x01.
- RING_DETECT fires every ~40 ms from mains hum on this line.
- Firmware country resets to USA (0x0840) on USB reset.
- modemprobe's listener thread can hang in libusb when uamodem.ko owns EP 0x81;
  `modemprobe raw` (no listener) works.

## Kernel driver
`driver/uamodem.ko`: see driver/README.md. Tested on hardware; works as far as the dead line allows.

## Tools
- `probe/answerplay` — answering machine, waits for incoming calls, play a random file to the caller. 
- `probe/modemprobe` — control requests, `seq` scripts (`hN`, `x:HEX` encapsulated, `r` read reply, `sMS`).
- `probe/isotest` — detaches uaudio, runs iso capture/playback + DTMF + control ops.
- `probe/tones.py` — Goertzel tone analysis of 8 kHz raw captures.
