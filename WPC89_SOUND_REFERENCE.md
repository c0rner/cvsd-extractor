# WPC-89 Sound Board Technical Reference
## BL U18 L1 — Motorola 6809E Firmware Decompilation

> Source: `BL_U18.L1.c` — near-complete decompile of the WPC-89 sound ROM

---

## Table of Contents

1. [Hardware Overview](#1-hardware-overview)
2. [Memory Map](#2-memory-map)
3. [Interrupt Architecture](#3-interrupt-architecture)
4. [Direct Page Variable Map](#4-direct-page-variable-map)
5. [Voice Block Layout](#5-voice-block-layout)
6. [Voice Pool and Linked List Management](#6-voice-pool-and-linked-list-management)
7. [WPC Bus Communication](#7-wpc-bus-communication)
8. [YM2151 FM Synthesizer Interface](#8-ym2151-fm-synthesizer-interface)
9. [HC55536 CVSD Codec](#9-hc55536-cvsd-codec)
10. [8-bit DAC (PCM Output)](#10-8-bit-dac-pcm-output)
11. [DS1267 Digital Volume Potentiometer](#11-ds1267-digital-volume-potentiometer)
12. [Voice Sequencer Architecture](#12-voice-sequencer-architecture)
13. [Sequence Opcode Reference](#13-sequence-opcode-reference)
14. [ROM Tables (Banked ROM)](#14-rom-tables-banked-rom)
15. [Key Functions Reference](#15-key-functions-reference)
16. [Startup / Reset Sequence](#16-startup--reset-sequence)
17. [Error and Diagnostic Handling](#17-error-and-diagnostic-handling)

---

## 1. Hardware Overview

| Component | Part | Description |
|-----------|------|-------------|
| CPU | Motorola 6809E | 8/16-bit CISC, direct page addressing |
| FM Synth | Yamaha YM2151 (OPM) | 8 FM channels, 4 operators each |
| Voice / Speech | Harris HC55536 CVSD | Compressed speech/voice playback |
| PCM | 8-bit DAC | Raw sample output at `0x2800` |
| WPC bus | Latch interface | Communication with main-board CPU |
| Volume | DS1267 | Serial digital potentiometer, bit-bang via `0x3800` |

---

## 2. Memory Map

```
 _______________
|               |
| 0000 - 1FFF   | RAM (8 KB)
|_______________|
|               |
| 2000 - 2001   | BANK SWITCHER — write bank number to select 32 KB ROM window
|_______________|
|               |
| 2400 - 2401   | YM2151 (OPM FM Synthesizer)
|               |   0x2400 = address/register select (write)
|               |   0x2401 = data write / status read (bit 7 = busy flag)
|_______________|
|               |
| 2800 - 2801   | DAC — 8-bit PCM output
|_______________|
|               |
| 2C00 - 2C01   | HC55536 SET  (CVSD clock high / bit = 1)
|_______________|
|               |
| 3000 - 3001   | WPC LATCH READ — command byte from main CPU, triggers IRQ
|_______________|
|               |
| 3400 - 3401   | HC55536 CLEAR (CVSD clock low  / bit = 0)
|_______________|
|               |
| 3800 - 3801   | WPC VOLUME — serial bit-bang to DS1267 volume pot
|_______________|
|               |
| 3C00 - 3C01   | WPC LATCH WRITE — status/ack byte back to main CPU
|_______________|
|               |
| 4000 - BFFF   | BANKED ROM (32 KB window, bank selected via 0x2000)
|_______________|
|               |
| C000 - FFFF   | SYSTEM ROM (16 KB, always mapped)
|_______________|
```

**RAM regions of note:**

| Address | Size | Purpose |
|---------|------|---------|
| `0x0000–0x01FF` | 512 B | Scratch, stack, zero-page |
| `0x0200–0x02FF` | 256 B | Direct Page (DP=2) system variables |
| `0x0300–0x04C9` | ~450 B | RAM tables (pitch, KC, channel timing) |
| `0x04CA` | 2 B | Voice free-pool sentinel |
| `0x04CC` | 2 B | Sentinel next-ptr |
| `0x04F0–0x0B78` | 1,672 B | 44 × 38-byte voice blocks |
| `0x1FFB–0x1FFF` | 5 B | Diagnostic bread-crumb registers |

---

## 3. Interrupt Architecture

### IRQ — WPC Command Receive
- **Trigger:** Main CPU writes a sound-command byte to WPC latch at `0x3000`
- **Handler:** `VEC_IRQ_ISR`
- **Action:** Reads the byte and appends it to the 16-entry circular ring buffer at `0x0206–0x0215` (write pointer at `DP+0x16`)
- **Ring wrap:** Write pointer wraps from `0x0215` back to `0x0206`

### FIRQ — Audio Sample Rate Interrupt
- **Trigger:** YM2151 Timer A overflow (~8 kHz)
- **Handler:** `VEC_FIRQ_ISR`
- **Actions per interrupt (in order):**
  1. **CVSD bit output:** If CVSD counter (`DP+0x3F`) high byte ≥ 0, write next bit to HC55536 via `SET` (`0x2C00`) or `CLEAR` (`0x3400`). Decrement bit counter.
  2. **DAC sample output:** If DAC active (`DP+0x2C ≠ 0`), output `sample_byte × volume_multiplier >> 8` to `0x2800`. Advance sample pointer.
  3. **Tick counter:** Decrement `DP+0x3A`; when it hits 0, reload from `DP+0x38` and increment the 16-bit tick counter at `DP+0x36/0x37`.
  4. **YM2151 timer reset:** Write reg `0x14 = 0x15` to reload Timer A, scheduling the next FIRQ.
- CVSD bits are output **three times per FIRQ** (interleaved with DAC output and timer reset), meaning ~24 kHz effective CVSD bit rate at 8 kHz FIRQ rate.

---

## 4. Direct Page Variable Map

Base address = `0x0200` (DP register = 2). All offsets are from `0x0200`.

| Offset | Size | Name | Description |
|--------|------|------|-------------|
| `+0x00` | 2 | active voice list head | Ptr to first active voice block |
| `+0x02` | 2 | active voice list tail | Ptr to tail sentinel |
| `+0x04` | 2 | free voice list head | Ptr to first free block (pool head) |
| `+0x06` | 16 | WPC command ring buffer | `0x0206–0x0215` |
| `+0x16` | 2 | ring-buffer write ptr | Updated by IRQ handler |
| `+0x18` | 2 | ring-buffer read ptr | Updated by main loop |
| `+0x1A` | 1 | active FM channel bitmask | 1 bit per YM2151 channel (ch0=bit0) |
| `+0x1B` | 1 | active CVSD channel mask | Bitmask of channels with CVSD voice |
| `+0x1C` | 2 | CVSD voice pointer (scratch) | Set during CVSD sequence opcodes |
| `+0x1E` | 2 | CVSD/DAC voice slot ptr | Points to allocated CVSD voice block |
| `+0x20` | 1 | program-change sub-index | Non-zero enables indirect program lookup |
| `+0x21` | 1 | program-change index A | Used by `seq_op36` indirect dispatch |
| `+0x22` | 1 | program-change index B | Used by `seq_op36` indirect dispatch |
| `+0x23` | 1 | sub-sound index | Latched by `seq_op10` |
| `+0x24` | 1 | FM global TL delta (lo-prio) | Applied to all FM operator TL registers |
| `+0x25` | 1 | FM global TL delta (hi-prio) | Applied in high-priority mode |
| `+0x26` | 1 | last applied TL delta | Tracks previous delta for diff updates |
| `+0x28` | 1 | pitch / TL working register | Scratch used during pitch/volume compute |
| `+0x29` | 1 | computed TL result | Output of TL compute, written to YM2151 |
| `+0x2A` | 1 | operator enable mask | Bits 2–0 select which of 4 ops get TL writes |
| `+0x2B` | 1 | CVSD mode flags | Loaded from sample descriptor |
| `+0x2C` | 1 | DAC playback active flag | Non-zero = DAC outputting samples |
| `+0x2D` | 1 | master volume attenuation | Set by `FUN_f9c5` (volume set command) |
| `+0x2E` | 1 | volume attenuation copy | Previous `+0x2D` value for delta calc |
| `+0x2F` | 1 | applied CVSD volume | Running sum, updated by `seq_op22` |
| `+0x30` | 1 | DAC volume level | Low-priority DAC volume |
| `+0x31` | 1 | DAC volume multiplier | `0x00–0xFF`; applied per sample via `× >> 8` |
| `+0x34` | 2 | DAC sample end address | FIRQ stops playback when ptr reaches here |
| `+0x36` | 2 | tick counter | 16-bit, incremented every FIRQ (low byte `+0x37`, high `+0x36`) |
| `+0x38` | 1 | timer-rate reload value | Reloaded into `+0x3A` on expiry |
| `+0x39` | 1 | timer-rate override | If non-zero, overrides `+0x38` |
| `+0x3A` | 1 | timer-rate countdown | Decremented by FIRQ; drives tick rate |
| `+0x3D` | 2 | FIRQ saved accumulator D | Saved/restored by FIRQ handler |
| `+0x3F` | 2 | CVSD output state | High byte = remaining bit count; low byte = current sample byte |
| `+0x41` | 2 | CVSD read pointer | Points to next byte in CVSD sample data |
| `+0x43` | 2 | CVSD data end pointer | FIRQ marks CVSD done when read ptr reaches this |
| `+0x45` | 2 | FIRQ saved index Y | Saved/restored by FIRQ |
| `+0x48` | 1 | WPC latch-write enable | Non-zero = writes to `0x3C00` are enabled |
| `+0xAD` | 2 | global pitch offset (low) | Added to all channels when bit7 of voice[+4] is clear |
| `+0xAF` | 2 | global pitch offset (high) | Added to all channels when bit7 of voice[+4] is set |
| `+0xB1` | 1 | stereo/operator channel mask | `DP+0xB1 & DAT_e1dd[chan]` enables stereo mode |
| `+0xB2` | 1 | loop / iteration counter | Scratch counter used across several helpers |
| `+0xB3` | 1 | channel scratch | Temp channel index during key-off walks |
| `+0xB4` | 1 | priority mask scratch | `0xFF` (high-prio) or `0x7F` (low-prio) |
| `+0xB5–0xBC` | 8 | various scratch | Working registers for sequence opcodes |
| `+0xBC` | 1 | YM2151 pending write value | Written to `DAT_2401` in `ym2151_write_register` |
| `+0xC6` | 2 | computed KC/KF word | Final 16-bit pitch for YM2151 write |
| `+0xC9` | 1 | CVSD current data byte | Loaded by `load_cvsd_parameters` |
| `+0xCA` | 1 | CVSD playback mode | `0`=off, `1`=pitch-shifted, `2`=raw |
| `+0xCB` | 1 | CVSD pitch offset | Applied to KC computation in pitch-shift mode |
| `+0xCD` | 1 | YM2151 operator bit mask | Inverted mask of operator bits |
| `+0xCE` | 1 | CVSD FM-mode flag | Set when CVSD note is in FM-assisted mode |
| `+0xD0` | 2 | clone data pointer | Source pointer for `clone_voice_block` |
| `+0xD2–0xD8` | 8 | clone scratch | Workspace for `clone_voice_block` |

---

## 5. Voice Block Layout

Each voice block is **38 bytes (0x26)**.

| Offset | Size | Field | Description |
|--------|------|-------|-------------|
| `+0x00` | 2 | next ptr | Forward link in circular doubly-linked list |
| `+0x02` | 2 | prev ptr | Backward link |
| `+0x04` | 1 | opcode / flags | bits[5:0] = opcode index (0–63); bit6 = loop trigger; bit7 = high-priority |
| `+0x05` | 1 | chan/op config | bits[2:0] = YM2151 channel (0–7); bit3 = CVSD slot flag; bits[6:3] = operator enable mask |
| `+0x06` | 2 | ROM data pointer | Points into current sequence data in ROM |
| `+0x08` | 1 | loop / repeat count | Decremented by loop opcodes |
| `+0x09` | 8 | internal scratch | Opcode-specific working state (glide params, vibrato state) |
| `+0x11` | 2 | timing accumulator | Counts down; voice fires opcode when `≤ 0` |
| `+0x13` | 2 | timestamp | Tick-counter snapshot at last fire |
| `+0x15` | 1 | subroutine nest depth | 0–8; call depth for sequence `CALL`/`RETURN` opcodes |
| `+0x16` | 16 | return address stack | 8 × 2-byte return addresses (indexed by nest depth) |

**Purgeable opcode values** (voice may be evicted when pool is full):
- `0x1A` — low-priority purgeable (resting voice)
- `0x9A` — high-priority purgeable (`-0x66` signed)

---

## 6. Voice Pool and Linked List Management

### Pool constants

| Symbol | Value | Meaning |
|--------|-------|---------|
| Free-pool sentinel | `0x04CA` | Marks empty free list |
| Sentinel next-ptr | `0x04CC` | Always points back to `0x04CA` |
| First free block | `0x04F0` | Start of the 44-block pool |
| Last free block | `0x0B78` | End of the pool |
| Block size | `0x26` (38) | Bytes per voice block |
| Pool capacity | 44 | Maximum simultaneous voices |

### Active list structure
The active voice list is a **doubly-linked circular list** with a sentinel node at `0x04CA`. `DP+0x00` = head, `DP+0x02` = tail (sentinel). The list is empty when head == tail == sentinel.

### Key operations

| Function | Description |
|----------|-------------|
| `alloc_voice_block` | Pop from free list; if empty call `purge_lowest_priority_voice`; zero block+2..+0x25; insert before tail sentinel |
| `free_voice_block` | Unlink from active list; push onto free-list head (`DP+0x04`) |
| `clone_voice_block` | Allocate new block; copy parent's bytes `+0x11..+0x25`; insert adjacent to parent; used by CVSD vibrato |
| `purge_lowest_priority_voice` | Walk active list; first pass: free a `0x1A` opcode voice; second pass: free a `0x9A` opcode voice; else hang (fatal) |

---

## 7. WPC Bus Communication

| Port | Direction | Purpose |
|------|-----------|---------|
| `0x3000` | Read | Command byte from main CPU (triggers IRQ) |
| `0x3C00` | Write | Status / acknowledge byte to main CPU |
| `0x3800` | Write | Serial bit-bang to DS1267 volume pot |

### Command ring buffer
- Located at `0x0206–0x0215` (16 entries)
- Write pointer: `DP+0x16` (updated by IRQ)
- Read pointer: `DP+0x18` (updated by `process_wpc_command_buffer`)
- Both pointers wrap from `0x0215 + 1` back to `0x0206`

### Special command codes

| Command | Handler |
|---------|---------|
| `0x00` | `audio_system_soft_reset` |
| `0x79` | `volume_pot_step_down` |
| `≤ MAX_CMD_INDEX` | `sound_command_handler` |
| `> MAX_CMD_INDEX` | Silently ignored (advance read ptr only) |

### Status bytes written to `0x3C00`

| Value | Meaning |
|-------|---------|
| `0x01` | ACK / generic status |
| `0x80` | End of sequence (voice done) |
| `0x81` | Sound started |
| Error code | Diagnostic error code from checksum / YM2151 timeout |

---

## 8. YM2151 FM Synthesizer Interface

### Register write protocol
1. Spin-wait while `DAT_2401` bit 7 is set (chip busy)
2. Write register address to `DAT_2400`
3. Spin-wait again (post-address busy poll)
4. Write register value to `DAT_2401`

**Helper:** `ym2151_write_register(value, register)` encapsulates this sequence. Note argument order is `(value, register)` — reversed from conventional.

### Key YM2151 registers used

| Register | Symbol | Purpose |
|----------|--------|---------|
| `0x01` | Test/LFO | LFO reset (write `2` then `0` to reset) |
| `0x08` | Key-on | `(op_mask << 3) | chan | 0x78` = key-on; `chan` only = key-off |
| `0x14` | Timer A control | Write `0x15` to reload and reset Timer A → triggers next FIRQ |
| `0x18` | LFO frequency | Stored in `DAT_0118` |
| `0x19` | PMD/AMD depth | Stored in `DAT_0119` |
| `0x1A` | YM2151 reg 0x1A | Stored in `DAT_011A` |
| `0x1B` | CT2/CT1/Waveform | Stored in `DAT_011B` |
| `0x28+chan` | KC (key code) | Coarse pitch; 7-bit value |
| `0x30+chan` | KF (key fraction) | Fine pitch; 6-bit value |
| `0x40+op` | DT1/MUL | Operator detune 1 / multiplier |
| `0x60+op` | TL | Operator total level (volume/attenuation) |
| `0x80+op` | KS/AR | Key scale / attack rate |
| `0xA0+op` | AMS/D1R | AM sensitivity / decay rate 1 |
| `0xC0+op` | DT2/D2R | Detune 2 / decay rate 2 |
| `0xE0+op` | D1L/RR | Sustain level / release rate |

### Pitch computation (`ym2151_compute_write_kc_kf`)
1. Load base note from `DAT_0128[chan]` (lo-prio) or `DAT_0160[chan]` (hi-prio)
2. Look up KC byte from `DAT_f22f[note]` (note → KC table)
3. Add pitch fine-tune from `PITCH_FINE_TABLE_A[chan]` or `PITCH_FINE_TABLE_B[chan]`
4. Add global pitch offset from `DP+0xAD` (lo-prio) or `DP+0xAF` (hi-prio)
5. Clamp to range `[0x0000, 0x5EFF]`
6. Write KC to reg `0x28+chan`, KF (low byte) to reg `0x30+chan`

### FM patch format (`DAT_4001`)
Each patch is **26 bytes (0x1A)**:
```
Byte 0:    algorithm / feedback (reg 0x20+chan)
Byte 1:    LFO connect (reg 0x38+chan)
Bytes 2–25: 4 operators × 6 bytes each:
  DT1/MUL (reg 0x40+op), TL (reg 0x60+op), KS/AR (reg 0x80+op),
  AMS/D1R (reg 0xA0+op), DT2/D2R (reg 0xC0+op), D1L/RR (reg 0xE0+op)
```

### TL update helpers

| Function | Description |
|----------|-------------|
| `ym2151_tl_update_from_table` | Reads TL from RAM patch shadow, adds `DP+0x28` delta, writes to operator TL registers |
| `ym2151_tl_update_absolute` | Writes `DP+0x28` directly as TL to all enabled operators |
| `ym2151_tl_update_relative` | Adds `DP+0x28` signed delta to current TL for all enabled operators |

Number of operators updated is controlled by `DP+0x2A` (operator enable mask):
- bit2 set → update 2nd operator (TL at `base - 8`)
- bit2 + bit1 set → update 3rd operator (TL at `base - 16`)
- bits 2+1+0 all set (`0x07`) → update all 4 operators

### YM2151 LFO

| Function | Description |
|----------|-------------|
| `ym2151_lfo_reset` | Write `2` then `0` to reg `0x01` to clear LFO phase |
| `ym2151_lfo_update` | Write `DAT_0118/011B/0119/011A` to regs `0x18/0x1B/0x19/0x1A` |

---

## 9. HC55536 CVSD Codec

The HC55536 uses a 1-bit sigma-delta (CVSD) algorithm. Each bit is clocked by:
- Writing any value to `0x2C00` (SET) → bit = 1, clock high
- Writing any value to `0x3400` (CLEAR) → bit = 0, clock low

### CVSD state (in direct page)

| DP offset | Description |
|-----------|-------------|
| `+0x3F` high | Remaining bit count in current byte (starts at 8) |
| `+0x3F` low | Current sample byte being shifted out |
| `+0x41` | CVSD read pointer (advances byte-by-byte through sample data) |
| `+0x43` | CVSD end pointer (playback ends when read ptr ≥ end) |
| `+0x2B` | Mode flags (loaded from sample descriptor byte 0) |
| `+0xCA` | CVSD playback mode: `0`=off, `1`=pitch-shifted, `2`=raw |
| `+0xCB` | CVSD pitch offset (used in mode 1) |
| `+0xC9` | Current CVSD data byte (working register) |

### CVSD bit output (per FIRQ)
The FIRQ handler outputs **one bit** per call:
1. Check `DP+0x3F` sign — if high byte < 0 (bit count exhausted), do nothing.
2. Write `(byte)sVar2` to `HC55536_CLEAR` (`0x3400`)
3. Shift right by 1: `bVar7 = (byte)sVar2 >> 1`
4. Update `DP+0x3F`: high byte `= high - 1`, low byte `= bVar7`
5. Write `bVar7` to `HC55536_SET` (`0x2C00`)

When bit count reaches zero, load next byte from `DP+0x41`, reload count to 8. If `DP+0x41 >= DP+0x43`, mark CVSD done.

### Sample table (`DAT_4015`)
Each entry is a 4-byte descriptor:
```
Byte 0:    mode flags
Byte 1:    first data byte (also used to seed bit counter = 8)
Bytes 2-3: end address pointer (exclusive)
```

---

## 10. 8-bit DAC (PCM Output)

- **Port:** `0x2800` (write-only)
- **Activation:** `DP+0x2C` flag set to non-zero by CVSD stereo opcodes
- **Volume scaling:** `output = (sample_byte * DP+0x31) >> 8`
- **Volume multiplier:** `DP+0x31` set from lookup table `DAT_e165[volume_index]`
- **End detection:** When sample pointer `>= DP+0x34` (end address), `DP+0x2C` cleared

The DAC runs at the FIRQ rate (~8 kHz), outputting two samples per FIRQ interrupt (before and after the YM2151 timer reset).

---

## 11. DS1267 Digital Volume Potentiometer

Connected via serial bit-bang on port `0x3800`:
- Bit 0 = data
- Bit 1 = clock

| Function | Description |
|----------|-------------|
| `volume_pot_clock_pulse` | Generates a full clock burst: 0x6E (110) iterations of `WPC_VOLUME = 3` then `= 2` |
| `volume_pot_step_down` | Calls `volume_pot_clock_pulse`, then sends `count` decrement pulses: `WPC_VOLUME = 1` then `= 0` |

Volume-down is triggered by WPC command `0x79` from the main CPU.

---

## 12. Voice Sequencer Architecture

### Main loop (`main_voice_sequencer_loop`)
Never returns. Each iteration:
1. Write `0x15` to YM2151 reg `0x14` (reset Timer A, keep FIRQ ticking)
2. Call `process_wpc_command_buffer` to drain ring buffer
3. If active list head `== 0` (empty), loop back to step 1
4. For each voice in the active list:
   - Compute elapsed ticks: `remaining = voice[+0x11] - (tick_now - voice[+0x13])`
   - Update `voice[+0x13] = tick_now`, `voice[+0x11] = remaining`
   - If `remaining < 1` (time to fire):
     - Dispatch to `OPCODE_TABLE[(voice[+0x04] & 0x3F) * 2]`
     - The handler may re-fire the same voice (return `0`) or yield to next (return with advance flag)
5. Advance to `voice[+0x00]` (next in list)

### Opcode dispatch
- Jump table base: `OPCODE_TABLE` (near `0xE1ED` in system ROM)
- Index: `voice[+0x04] & 0x3F` (6-bit opcode, 0–63)
- Each entry is a 2-byte code pointer

### Return values from opcode handlers
| Return | Meaning |
|--------|---------|
| `0` | Re-check this voice (re-evaluate timing next tick; voice stays) |
| `1` | Advance to next voice |
| `0x80` | End of sequence — voice should be freed (caller advances list and discards) |

### Per-voice channel configuration (voice+5)

| Bits | Field |
|------|-------|
| `[2:0]` | YM2151 channel number (0–7) |
| `3` | CVSD/DAC slot flag (if set, this is the CVSD voice) |
| `[6:3]` | Operator enable mask for FM writes |

### Priority model
- `voice[+0x04]` bit 7: **high-priority** when set
- High-priority voices use KC tables at `DAT_0160` and pitch tables at `DAT_025D`
- Low-priority voices use `DAT_0128` and `DAT_0249`
- When a new sound starts, low-priority purgeable voices (opcode `0x1A` / `0x9A`) are evicted first

---

## 13. Sequence Opcode Reference

Opcodes are 6-bit indices (bits[5:0] of `voice[+0x04]`). Bit 7 is the priority flag; bit 6 is used as a loop-trigger by some opcodes. Arguments follow immediately in the ROM data stream.

| Opcode | Function | Args | Description |
|--------|----------|------|-------------|
| `0x00` | `seq_op00_end_of_sequence` | — | End of sequence / next-sample chain. If CVSD active and more samples remain, load next CVSD sample; else free voice, send status `0x80`. Sets `voice[+0x11] = 10`. |
| `0x01` | `seq_op01_nop` | — | No operation |
| `0x08` | `seq_op08_cvsd_sample_start_stereo` | `timing[2], opcode, channel_mask` | Start CVSD sample with stereo-aware channel masking. Loads CVSD data ptr, bit-count, end-ptr from `DAT_4003[idx]`. Adds timing. |
| `0x09` | `seq_op09_cvsd_sample_start` | `timing[2], opcode` | Simplified CVSD start (no stereo masking). Adds timing, loads next opcode. |
| `0x0A` | `seq_op0a_timing_advance` | `kc_byte, timing[1 or 2], opcode` | FM key-on with timing advance. Sends key-on to YM2151, reads variable-width timing (1 or 2 bytes based on `DP+0xBA` high bit). |
| `0x0B` | `seq_op0b_note_on_timing` | `timing[1 or 2], opcode` | Note-on with timing (no new KC). Writes current KC to YM2151. Variable-width timing. |
| `0x0C` | `seq_op0c_fm_patch_load` | `patch_index, opcode` | Load FM patch from `DAT_4001[patch_index]`. Writes all 26 bytes to YM2151 operator registers. Applies current TL delta if set. |
| `0x0D` | `seq_op0d_push_pitch_table` | `opcode, flags` | Push current data pointer onto pitch stack (`DAT_0249`/`DAT_025D` for lo/hi-priority). Used as loop-back target for repeat opcodes. |
| `0x0E` | `seq_op0e_repeat_loop` | — (reads from pitch stack) | Decrement loop counter in pitch table. If non-zero: jump back to stored loop target. If zero: advance past loop. |
| `0x0F` | `seq_op0f_set_absolute_pitch` | `note, opcode` | Set absolute pitch. Mode A (not stereo): store note in `DAT_0120` table, write TL via patch table. Mode B/C (stereo): apply note delta to up to 4 TL registers. |
| `0x10` | `seq_op10_trigger_sub_sound` | `sound_index` | Trigger sub-sound. Reads index, optionally remaps via `DAT_4009`, calls `start_sound_program`. |
| `0x12` | `seq_op12_inject_cmd_rerun` | `command_byte` | Inject a command byte into the ring buffer AND immediately call `main_voice_sequencer_loop` to process it in the same frame. |
| `0x13` | `seq_op13_subroutine_call` | `target_ptr[2]` | Subroutine call. Push return address (current ptr + 1) onto `voice[+0x16]` call stack. Advance to `target_ptr`. Max 8 levels; hang on overflow. |
| `0x14` | `seq_op14_subroutine_return` | — | Return from subroutine. If nest depth is 0: simple advance. Else: decrement depth, pop return address, advance there. |
| `0x15` | `seq_op15_fm_key_off` | — | FM key-off and channel teardown. Send key-off to YM2151. Write silence to all 4 operator TL registers. Remove channel from active bitmask. Free all co-voices on same channel. Free this voice. |
| `0x17` | `seq_op17_cvsd_fm_note_on_abs` | `note, flags` | Combined CVSD + FM absolute note-on. Loads CVSD parameters, applies pitch-shift if mode 1, writes KC to YM2151 if channel is active FM. |
| `0x18` | `seq_op18_cvsd_voice_update` | `note, flags` | CVSD voice update (additive). Similar to `0x17` but uses additive KC merging into existing CVSD register. |
| `0x19` | `seq_op19_cvsd_fm_note_on_delta` | `note_delta, flags` | CVSD + FM delta note-on. Applies a signed delta to current CVSD register value. |
| `0x1A` | `seq_op1a_cvsd_vibrato` | `mode, pitch_start, mask, pitch_delta[2], count, end_ptr[2]` | CVSD vibrato / pitch bend. On first call: clone voice and set up pitch sweep state. Each call: apply delta to CVSD playback pitch. When count expires: free clone. |
| `0x1B` | `seq_op1b_detune` | `timing_delta[2], opcode` | Set relative pitch / detune. Add signed timing delta to `voice[+0x11]`, store new flags in `voice[+4]`. |
| `0x1D` | (see `0x0E`) | — | Alias for `seq_op0e_repeat_loop` (high-priority variant) |
| `0x1E` | `seq_op1e_note_timing_combined` | `kc_byte, timing[1 or 2], opcode` | Note set (KC write only, no key-on) combined with variable-width timing advance. |
| `0x1F` | `seq_op1f_pitch_glide` | `start[2], delta[2], end[2], step_time[2], final_opcode, pad[2]` (11 bytes, loaded once) | Pitch glide / vibrato. First call loads parameters and starts. Subsequent calls step pitch by delta, update YM2151 KC/KF. Ends when step count exhausted. |
| `0x21` | `seq_op21_stop_dac_cvsd` | — | Stop DAC/CVSD voice. Clear `DP+0x2C`, update active masks, update DAC volume multiplier from `DAT_e165`, free voice, clear `DP+0x1E`. |
| `0x22` | `seq_op22_cvsd_volume_fade` | `delta(high byte), opcode(low byte)` | CVSD/DAC volume fade. High-priority: add delta to `DP+0x2F` (CVSD master vol). Low-priority: add delta to `DP+0x30` (DAC vol). Update `DP+0x31` multiplier. |
| `0x23` | `seq_op23_send_status_to_cpu` | `status_byte(high), next_opcode(low)` | Send status byte to main CPU via `0x3C00` (if latch-write enabled). Load next opcode from low byte. |
| `0x24` | `seq_op24_set_channel_timing` | `timing[2], opcode` | Store 16-bit timing in `CHANNEL_TIMING_TABLE[chan]`. Optionally mask KC byte. |
| `0x25` | `seq_op25_fm_key_on_with_timing` | `kc_byte, timing_offset` | FM key-on. Look up KC, send key-on via `ym2151_key_on_off`. Read timing offset, add `CHANNEL_TIMING_TABLE[chan]` to `voice[+0x11]`. |
| `0x26` | `seq_op26_add_timing_delta` | `delta[2], (advance)` | Add signed 16-bit delta to `CHANNEL_TIMING_TABLE[chan]`. Advance sequence pointer. |
| `0x27` | `seq_op27_fm_pitch_delta_tableB` | `delta[2], opcode` | Add 16-bit pitch delta to `PITCH_FINE_TABLE_B[chan]`. Update YM2151 KC/KF. |
| `0x28` | `seq_op28_fm_pitch_delta_tableA` | `delta[2], opcode` | Add 16-bit pitch delta to `PITCH_FINE_TABLE_A[chan]`. Low-prio: mask to `0x7F`. Update YM2151. |
| `0x29` | `seq_op29_fm_pitch_abs_tableB` | `value[2], opcode` | Set `PITCH_FINE_TABLE_B[chan]` to absolute 16-bit pitch value. Update YM2151. |
| `0x2A` | `seq_op2a_fm_pitch_abs_tableA` | `value[2], opcode` | Set `PITCH_FINE_TABLE_A[chan]` to absolute 16-bit pitch value. Low-prio: mask. Update YM2151. |
| `0x2B` | `seq_op2b_global_pitch_slide_hi` | `delta[2], (advance)` | Add delta to global pitch offset `DP+0xAF`. Force bit7 of `voice[+4]` set. Re-compute KC/KF for all 8 YM2151 channels. |
| `0x2C` | `seq_op2c_global_pitch_slide_lo` | `delta[2], (advance)` | Add delta to `DP+0xAD`. Clear bit7. Re-compute all 8 channels. |
| `0x2D` | `seq_op2d_set_global_pitch_abs_hi` | `value[2], (advance)` | Write absolute value to `DP+0xAF`, force bit7 set, update all channels. |
| `0x2E` | `seq_op2e_set_global_pitch_abs_lo` | `value[2], (advance)` | Write absolute value to `DP+0xAD`, clear bit7, update all channels. |
| `0x2F` | `seq_op2f_note_trigger_repeat` | `count, timing[2], opcode, tl_note, tl_end` | Repeat note trigger. If bit6 of `voice[+4]` set: clone voice, set repeat pointer. Each call: update TL, add timing. When count reaches 0: write final TL, free voice. |
| `0x30` | `seq_op30_set_stereo_mask` | `opcode` | OR this channel's bit into `DP+0xB1` stereo mask. Mask next opcode to `0x7F`. |
| `0x31` | `seq_op31_clear_channel_mask` | `opcode` | Clear this channel's bit from `DP+0xB1`. Read next opcode. |
| `0x32` | `seq_op32_indirect_opcode_load` | `ptr` | Dereference pointer from sequence data; load true opcode from that address. Indirect dispatch. |
| `0x33` | `seq_op33_inject_cmd_ring_buf` | `command_byte` | Inject command byte into WPC ring buffer (deferred to next ring-drain, unlike `0x12`). Advance sequence. |
| `0x35` | `seq_op35_timing_advance_alt` | `timing[1 or 2], (advance)` | Write YM2151 KC for this channel, then variable-width timing advance. |
| `0x36` | `seq_op36_program_change` | `bank(high), index(low)` | Program change. Indirect dispatch via `DAT_4005` bank/index table → call `sound_command_handler`. |
| `0x37` | `seq_op37_push_pitch_table_4b` | `opcode, flags, pitch_hi, pitch_lo` | 4-byte variant of push-pitch-table. Stores flags and 16-bit pitch value onto pitch stack. |
| `0x38` | (FM volume absolute) | `tl_value, opcode` | Set FM operator TL to absolute value via `ym2151_tl_update_absolute`. |
| `0x39` | (FM volume fade) | `tl_delta, opcode` | Relative FM volume fade via `ym2151_tl_update_relative`. |
| `0x3A` | `seq_op3a_fm_key_on_complex` | `kc_byte, timing[1 or 2]` (two-phase) | Two-phase FM key-on. Phase 0: store KC, call `ym2151_key_on_off`, load timing, subtract per-channel latency. Phase 1: send final KC to YM2151, advance. Splitting enables precise strobe timing. |
| `0x3B` | `seq_op3b_update_note_register` | `note_byte` | Store note byte in `DAT_02da[chan]` (lo) or `DAT_02e2[chan]` (hi). Advance sequence. |
| `0x3C` | `seq_op3c_note_register_delta` | `delta (signed 1B)` | Add signed delta to `DAT_02da[chan]` or `DAT_02e2[chan]`. Advance. |
| `0x3D` | `seq_op3d_free_voice_end` | — | Free this voice block and return `0x80` (end-of-sequence signal). |
| `0x23` | `seq_op_nop` / `seq_op23` | — | Some opcode slots map to NOP (`seq_op_nop` / `seq_op01_nop`) |

> **Opcode table dispatch formula:** `handler = OPCODE_TABLE[(opcode & 0x3F) * 2]`

---

## 14. ROM Tables (Banked ROM)

All tables reside in the banked ROM window `0x4000–0xBFFF` unless noted.

| Symbol | Address | Description |
|--------|---------|-------------|
| `MAX_CMD_INDEX` | `DAT_400e` | Maximum valid sound command index |
| `CMD_DISPATCH_TABLE` | `DAT_400f` | Command → `(handler_id, param)` table (2 bytes per entry) |
| `FM_PATCH_TABLE` | `DAT_4001` | YM2151 patch/instrument table (26 bytes per patch) |
| `DAT_4003` | `DAT_4003` | CVSD stereo sample → `(data_ptr, length)` table |
| `DAT_4005` | `DAT_4005` | FM program change → indirect voice pointer table |
| `CMD_VOICE_TYPE_TABLE` | `DAT_4007` | Sound command → voice-type bitmask (FM channel bits + CVSD bit) |
| `DAT_4009` | `DAT_4009` | Sub-sound indirect table for `seq_op10` |
| `SOUND_PROGRAM_TABLE` | `DAT_4011` | Sound command → sequence program pointer table |
| `CVSD_SAMPLE_TABLE` | `DAT_4015` | Note / sample index → `(flags, first_byte, end_ptr)` descriptor |
| `DAT_401b` | `DAT_401b` | Additional voice-type data for polyphonic start |
| `DAT_f22f` | `0xF22F` | Note number → YM2151 KC byte (system ROM) |
| `DAT_e1dd` | `0xE1DD` | YM2151 channel → stereo bit mask table (8 entries, system ROM) |
| `DAT_e1bd` | `0xE1BD` | Pitch detune bounds table (system ROM) |
| `OPCODE_TABLE` | ~`0xE1ED` | Sequence opcode jump table (system ROM) |
| `PITCH_FINE_TABLE_A` | `DAT_0285` | Per-channel fine-tune table A (RAM, 16 bytes) |
| `PITCH_FINE_TABLE_B` | `DAT_0299` | Per-channel fine-tune table B (RAM, 16 bytes) |
| `CHANNEL_TIMING_TABLE` | `DAT_0271` | Per-channel timing offset table (RAM, 16 bytes) |
| `KC_TABLE_LO_PRIORITY` | `DAT_02da` | Per-channel base note (low-priority, 8 bytes RAM) |
| `KC_TABLE_HI_PRIORITY` | `DAT_02e2` | Per-channel base note (high-priority, 8 bytes RAM) |
| `PITCH_KF_LOOKUP` | `0xE105?` | 96-byte lookup: pitch value → YM2151 KF byte |
| `DAT_e165` | `0xE165` | Volume index → DAC multiplier byte (DAT_e165 lookup table) |
| `DAT_dd00` | `0xDD00` | ROM checksum descriptor table (1 byte expected checksum per 32 KB bank) |

---

## 15. Key Functions Reference

### Interrupt handlers

| Function | Address | Description |
|----------|---------|-------------|
| `VEC_FIRQ_ISR` | system ROM | FIRQ: CVSD bit output, DAC sample, tick counter, YM2151 timer reset |
| `VEC_IRQ_ISR` | system ROM | IRQ: read WPC latch, append to ring buffer |
| `VEC_RESET_ISR` | `0xDDD4` | System reset / power-on initialisation |

### Main loop

| Function | Address | Description |
|----------|---------|-------------|
| `main_voice_sequencer_loop` | `0xDF41` | Main sequencer loop (never returns) |
| `process_wpc_command_buffer` | ~`0xF720` | Drain ring buffer, dispatch commands |
| `sound_command_handler` | ~`0xF7CF` | Allocate/configure voices for new command |
| `start_sound_program` | ~`0xF5XX` | Full voice allocation for a sound program |

### YM2151 helpers

| Function | Address | Description |
|----------|---------|-------------|
| `ym2151_write_register` | `0xFCF2` | Write `(value, reg)` with busy-wait |
| `ym2151_wait_not_busy` | `0xFD04` | Spin-poll bit 7 of `0x2401` |
| `ym2151_reset_all_regs` | `0xFC68` | Zero all 32 registers, key-off all 8 channels |
| `ym2151_key_on_off` | ~`0xE2XX` | Write reg `0x08` key-on or key-off |
| `ym2151_note_set` | ~`0xE2XX` | Write KC to `0x28+chan` without key-on |
| `ym2151_compute_write_kc_kf` | `0xF065` | Compute final KC/KF, write to YM2151 |
| `ym2151_lfo_reset` | `0xFEF0` | Assert then release LFO reset |
| `ym2151_lfo_update` | `0xFEF5` | Write all 4 LFO registers |
| `ym2151_tl_update_from_table` | ~`0xF6XX` | Write operator TL from patch table |
| `ym2151_tl_update_absolute` | ~`0xF6XX` | Write absolute TL to operators |
| `ym2151_tl_update_relative` | ~`0xF6XX` | Add delta TL to operators |

### Voice management

| Function | Address | Description |
|----------|---------|-------------|
| `alloc_voice_block` | `0xFD57` | Allocate from free pool; insert into active list |
| `free_voice_block` | `0xFDC6` | Unlink from active; return to free pool |
| `clone_voice_block` | `0xFE0A` | Fork voice; insert adjacent in active list |
| `purge_lowest_priority_voice` | `0xFE57` | Evict lowest-priority purgeable voice |
| `init_voice_linked_lists` | `0xFCC4` | Initialise free pool and empty active list |

### Sequence control

| Function | Address | Description |
|----------|---------|-------------|
| `advance_sequence_pointer` | `0xFF20` | Read next opcode from ROM, store in `voice[+4]`, advance `voice[+6]` |
| `set_sequence_pointer` | ~`0xF846` | Write ROM data pointer into `voice[+6]`, bank-switch if needed |
| `clear_pitch_offset` | ~`0xF863` | Zero `PITCH_FINE_TABLE_A/B[chan]` |
| `load_cvsd_parameters` | ~`0xDFAC` | Load CVSD mode, pitch offset, operator mask from ROM |

### System

| Function | Address | Description |
|----------|---------|-------------|
| `audio_system_init` | `0xF76E` | Full audio init (called from reset) |
| `audio_system_soft_reset` | `0xFC76` | Soft-reset: clear voices, re-init pitch, LFO |
| `init_pitch_table` | `0xFD29` | Zero `CHANNEL_TIMING_TABLE`, `PITCH_FINE_TABLE_A/B`, KC tables |
| `memset_zero` | `0xFD3D` | Zero N bytes at ptr |
| `memcpy_n` | `0xFD45` | Copy N bytes src → dst |
| `write_status_to_main_cpu` | `0xF559` | Write byte to `0x3C00` if latch-write enabled |
| `rom_checksum_verify` | `0xDE4B` | Verify ROM bank checksums via `DAT_DD00` table |
| `diagnostic_error_tone` | `0xFE87` | Output error code to CPU + DAC triangle-wave beep |
| `volume_pot_clock_pulse` | ~`0xF4F6` | Generate DS1267 clock burst |
| `volume_pot_step_down` | `0xF4AE` | Step volume pot down one notch |

---

## 16. Startup / Reset Sequence

`VEC_RESET_ISR` at `0xDDD4` performs the following in order:

1. Read `0x3000` (WPC latch) to acknowledge/clear any pending command
2. Call `ym2151_write_register` (initial YM2151 register clear)
3. Call `volume_pot_clock_pulse` (pulse DS1267 clock to establish state)
4. Call `volume_pot_step_down` (bring volume to minimum)
5. **Clear all RAM:** zero `0x0000–0x1FFF`
6. Zero `DAT_0000`
7. Set ring-buffer read and write pointers to `0x0206`
8. Call `ym2151_reset_all_regs` — full YM2151 register reset
9. Call `audio_system_init` — hardware soft-reset of audio subsystem
10. Call `audio_system_soft_reset` — voice list and pitch table initialisation
11. **Wait loop:** call `process_wpc_command_buffer` up to ~65536 times waiting for a valid startup command (bit 3 of received command byte set)
12. On valid command received: jump to `main_voice_sequencer_loop`
13. If loop expires without valid command:
    - **RAM test:** write `0xAA` (`-0x56`) to each location `0x0000–0x1FFF`, verify, then write `0x00`, verify
    - On failure: call `diagnostic_error_tone`, then proceed anyway
    - On success: call `rom_checksum_verify`, then `audio_system_soft_reset`, then `main_voice_sequencer_loop`

`DAT_1FFD` (2 bytes) is used as a "bread-crumb" debug register: it is written with the ROM address of each major call before the call is made, so a crash can be located post-mortem.

---

## 17. Error and Diagnostic Handling

### ROM checksum verification (`rom_checksum_verify`)

Iterates the descriptor table at `DAT_DD00`. Each entry is a 1-byte expected checksum for a 32 KB bank. The actual checksum is computed by summing (two's-complement subtraction) all bytes from `0x4080` through `0xC080`.

Sentinel value `0xFF` in the table signals success.

On failure, the upper 2 bits of the failing entry encode the error code:

| Entry bits[7:6] | Code | Meaning |
|-----------------|------|---------|
| `0x60` (`01`) | 3 | Bank 1 checksum fail |
| `0xA0` (`10`) | 4 | Bank 2 checksum fail |
| `0xC0` (`11`) | 5 | Bank 3 checksum fail |
| other | 6 | Unknown ROM error |

### YM2151 timeout error

If `ym2151_wait_not_busy` does not clear within 65,536 iterations:
- First occurrence: set `DAT_0B78 = 7`, call `diagnostic_error_tone(7)`
- Subsequent: write existing error code directly to `0x3C00`
- Call `VEC_RESET_ISR` to attempt recovery

### Diagnostic error tone (`diagnostic_error_tone`)

Writes error code to `0x3C00`, then generates a triangle-wave beep via the DAC, repeated `error_code` times. After all beeps, pauses 65,536 cycles, then dispatches via jump table (usually → checksum verifier or boot handler). Audible error identification pattern for operators.

### Stack overflow (subroutine call)

If `seq_op13_subroutine_call` exceeds 8 levels of nesting (`voice[+0x15] == 8`), it calls `diagnostic_error_tone(8, 0x10, ...)` then enters an infinite loop — this is a fatal error.

### Voice pool exhaustion

If `purge_lowest_priority_voice` cannot find any purgeable voice (`0x1A` or `0x9A`) in the active list, it enters an **infinite loop** — considered a fatal condition.
