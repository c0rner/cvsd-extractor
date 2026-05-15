# WPC Sound (CVSD) Extractor

Extract CVSD compressed audio and decode sound programs from Williams/Bally
WPC-89 pinball machine sound ROMs.

Research from decompile of CFTBL sound rom and the PinMame CVSD decoder (hc55516.c).

## Background

WPC-89 pinball machines (early 1990s) use a Motorola 6809-based sound board
with a Harris HC-55516 CVSD codec for digitised speech and sound effects,
combined with a Yamaha YM2151 FM synthesiser.  The audio data and sequencer
programs are stored across up to three ROM chips (U14, U15, U18), with U18
also containing the sound CPU firmware.

This tool reads the ROM images directly, parses the firmware table structures,
and can:

- **Extract** every CVSD audio sample as a standard WAV file.
- **Decode** the sound program bytecode that orchestrates FM synthesis and CVSD
  playback, producing a human-readable disassembly.

## Building

```sh
cargo build --release
```

Requires Rust 2024 edition (1.85+).

## Usage

### Extract CVSD audio

```sh
cvsd-extractor extract \
    --u14 rom/BL_U14.L1 \
    --u15 rom/BL_U15.L1 \
    --u18 rom/BL_U18.L1 \
    --output out
```

Each sample is written as a 16-bit mono WAV file named
`NNN_<chip>_<offset>_<size>.wav`.  The native sample rate is 22 372 Hz; use
`--output-rate` to resample (e.g. `--output-rate 44100`).

### Decode sound programs

```sh
cvsd-extractor programs --u18 rom/BL_U18.L1
```

Prints a report of every sound command (0x00–0xFF) showing:

- Command dispatch handler type
- Voice descriptor (FM channel mask, CVSD type)
- Disassembled sequencer bytecode for each channel

Example output:

```
--- Command 0x06: handler=0x04 (voice_type_table) ---
  Voice descriptor at 0xC800:
    FM mask: 0x01  CVSD: none
    Channel 0: seq @ 0xD120
      0000: FmPatchLoad       [0A 09]
      0002: SetAbsolutePitch  [46 0A]
      0004: TimingAdvance     [00 80 00]
      0007: EndOfSequence     [00]
```

## ROM format

The U18 ROM contains a header at offset 0x4000 (system bank) with pointers to:

| Offset | Field                  | Size |
|--------|------------------------|------|
| 0x01   | CVSD table pointer     | 2    |
| 0x07   | Voice type table ptr   | 2    |
| 0x0B   | Sound program table ptr| 2    |
| 0x0E   | Max command index      | 1    |
| 0x0F   | Dispatch table pointer | 2    |

The dispatch table maps each command number to a handler ID, which determines
how the command's parameters are interpreted:

- **Handler 0x04** — Voice type table: FM mask + per-channel sequence pointers +
  optional CVSD playback.  The most common handler (~60% of commands).
- **Handler 0x01** — Sound program table: 10-channel sequence pointer records
  for complex multi-voice sounds.
- **Handler 0x07** — Extended commands (implementation varies by game).
- **Handler 0x03** — Volume control commands.

## Sequencer opcodes

The voice sequencer executes a compact bytecode with ~63 opcodes (6-bit opcode
number, masked from an 8-bit byte).  Most opcodes embed the next opcode as
their final operand byte, forming a chain.  Key opcode categories:

| Range     | Category           | Examples                          |
|-----------|--------------------|-----------------------------------|
| 0x00      | End of sequence    | Frees the voice channel           |
| 0x01–0x07 | No-op / padding    | Used for alignment                |
| 0x08–0x09 | CVSD sample start  | Stereo/mono sample triggers       |
| 0x0A–0x0B | Timing / delays    | Variable-length timing advances   |
| 0x0C      | FM patch load      | Load YM2151 register patch        |
| 0x0D      | Pitch table        | Push pitch from lookup table      |
| 0x0E      | Repeat loop        | Loop the current sequence         |
| 0x12–0x13 | Subroutine         | Call/return (max 8 depth)         |
| 0x15–0x1B | FM note control    | Key on/off, vibrato, pitch glide  |
| 0x27–0x2E | Pitch slides       | Global pitch manipulation         |
| 0x34      | CVSD sample play   | Trigger CVSD by sample index      |
| 0x38–0x39 | Volume control     | Absolute set, relative fade       |
| 0x3E      | Bank switch        | Change ROM bank for sequence data |

## Supported games

Tested with the following ROM sets:

| Game           | Samples | Programs | Incomplete |
|----------------|---------|----------|------------|
| CFTBL          | 153     | 108      | 8          |
| Doctor Who     | 230     | 146      | 6          |
| Twilight Zone  | 144     | 159      | 3          |

## License

This project retains the PinMame BSD-3-Clause license from the CVSD decoder it is based on.