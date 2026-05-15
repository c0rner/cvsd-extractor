// SPDX-License-Identifier: BSD-3-Clause

//! WPC-89 sound-program extraction and sequence bytecode decoder.
//!
//! The WPC-89 sound firmware uses a voice-sequencer architecture to play
//! sounds.  Each sound command from the main CPU triggers one or more
//! "voices", each running a small bytecode program (sequence data).
//!
//! The bytecode is dispatched through an opcode table with 6-bit opcodes
//! (0x00–0x3D).  This module decodes the command dispatch tables, voice
//! descriptors, and sequence bytecode to produce a human-readable listing
//! of each sound program.
//!
//! # Architecture
//!
//! ```text
//! Command (0x00-0xFF)
//!   → Dispatch Table: (handler_id, voice_type_index)
//!     → Voice Type Table[voice_type_index]: pointer to descriptor
//!       → Voice Descriptor: FM_mask, seq_ptrs[], CVSD_type, cvsd_seq_ptr
//!         → Sequence bytecode per channel
//! ```
//!
//! # Reference
//!
//! All structures and opcodes are derived from the decompiled firmware
//! `reference/BL_U18.L1.c`.

use std::fmt;

use anyhow::{Context, Result, bail};

use crate::wpc89::{self, RomHeader};

// ---------------------------------------------------------------------------
// Sequencer opcodes
// ---------------------------------------------------------------------------

/// A 6-bit opcode for the WPC-89 voice sequencer.
///
/// The sequencer reads `voice[4] & 0x3F` and dispatches through
/// `OPCODE_TABLE[(opcode << 1)]`.  The variant names and numbers come
/// directly from the decompiled firmware function names (`seq_opNN_*`).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum SeqOpcode {
    /// 0x00 — End of sequence; free the voice block.
    EndOfSequence = 0x00,
    /// 0x01 — No-op (advance sequence pointer by 1 byte, continue).
    Nop = 0x01,
    // 0x02..0x07 — alias to 0x01 Nop in the jump table.
    /// 0x08 — Start CVSD sample with stereo panning (reads sample#, pan byte).
    CvsdSampleStartStereo = 0x08,
    /// 0x09 — Start CVSD sample (reads 1-byte sample index).
    CvsdSampleStart = 0x09,
    /// 0x0A — Timing advance (set voice delay counter).
    TimingAdvance = 0x0A,
    /// 0x0B — FM note-on with timing.
    NoteOnTiming = 0x0B,
    /// 0x0C — Load an FM patch (instrument) into the YM2151.
    FmPatchLoad = 0x0C,
    /// 0x0D — Push pitch from table (set note register from table).
    PushPitchTable = 0x0D,
    /// 0x0E — Repeat loop (decrement counter, branch back).
    RepeatLoop = 0x0E,
    /// 0x0F — Set absolute pitch on FM channel.
    SetAbsolutePitch = 0x0F,
    /// 0x10 — Trigger a sub-sound (inject command into ring buffer).
    TriggerSubSound = 0x10,
    /// 0x11 — Gap NOP (unused opcode, just returns).
    GapNop11 = 0x11,
    /// 0x12 — Inject command for immediate re-run (bypass ring buffer).
    InjectCmdRerun = 0x12,
    /// 0x13 — Subroutine call (push return address, jump to target).
    SubroutineCall = 0x13,
    /// 0x14 — Subroutine return (pop return address).
    SubroutineReturn = 0x14,
    /// 0x15 — FM key-off on the voice's channel.
    FmKeyOff = 0x15,
    /// 0x16 — Gap NOP (unused opcode, just returns).
    GapNop16 = 0x16,
    /// 0x17 — CVSD with FM note-on, absolute pitch.
    CvsdFmNoteOnAbs = 0x17,
    /// 0x18 — CVSD voice update (modify running CVSD parameters).
    CvsdVoiceUpdate = 0x18,
    /// 0x19 — CVSD with FM note-on, delta pitch.
    CvsdFmNoteOnDelta = 0x19,
    /// 0x1A — CVSD vibrato effect.
    CvsdVibrato = 0x1A,
    /// 0x1B — Detune (fine pitch offset).
    Detune = 0x1B,
    /// 0x1C — Push pitch from table (alias of 0x0D).
    PushPitchTableAlt = 0x1C,
    /// 0x1D — Repeat loop (alias of 0x0E).
    RepeatLoopAlt = 0x1D,
    /// 0x1E — Combined note + timing setup.
    NoteTimingCombined = 0x1E,
    /// 0x1F — Pitch glide (smooth pitch transition).
    PitchGlide = 0x1F,
    /// 0x20 — Gap NOP (unused opcode, just returns).
    GapNop20 = 0x20,
    /// 0x21 — Stop DAC/CVSD playback on this voice.
    StopDacCvsd = 0x21,
    /// 0x22 — CVSD volume fade (gradual volume change).
    CvsdVolumeFade = 0x22,
    /// 0x23 — Send status byte to the main CPU.
    SendStatusToCpu = 0x23,
    /// 0x24 — Set channel timing parameters.
    SetChannelTiming = 0x24,
    /// 0x25 — FM key-on with timing parameters.
    FmKeyOnWithTiming = 0x25,
    /// 0x26 — Add delta to timing value.
    AddTimingDelta = 0x26,
    /// 0x27 — FM pitch delta from table B.
    FmPitchDeltaTableB = 0x27,
    /// 0x28 — FM pitch delta from table A.
    FmPitchDeltaTableA = 0x28,
    /// 0x29 — FM pitch absolute from table B.
    FmPitchAbsTableB = 0x29,
    /// 0x2A — FM pitch absolute from table A.
    FmPitchAbsTableA = 0x2A,
    /// 0x2B — Global pitch slide (high byte).
    GlobalPitchSlideHi = 0x2B,
    /// 0x2C — Global pitch slide (low byte).
    GlobalPitchSlideLo = 0x2C,
    /// 0x2D — Set global absolute pitch (high byte).
    SetGlobalPitchAbsHi = 0x2D,
    /// 0x2E — Set global absolute pitch (low byte).
    SetGlobalPitchAbsLo = 0x2E,
    /// 0x2F — Note trigger with repeat.
    NoteTriggerRepeat = 0x2F,
    /// 0x30 — Set stereo output mask.
    SetStereoMask = 0x30,
    /// 0x31 — Clear channel mask bits.
    ClearChannelMask = 0x31,
    /// 0x32 — Indirect opcode load (read next opcode from data stream).
    IndirectOpcodeLoad = 0x32,
    /// 0x33 — Inject command into ring buffer.
    InjectCmdRingBuf = 0x33,
    /// 0x34 — Start CVSD sample playback (reads sample index + next opcode).
    CvsdSamplePlayback = 0x34,
    /// 0x35 — Timing advance (alternate form).
    TimingAdvanceAlt = 0x35,
    /// 0x36 — FM program change (switch instrument).
    ProgramChange = 0x36,
    /// 0x37 — Push pitch from table (variant with 4-byte data).
    PushPitchTable4b = 0x37,
    /// 0x38 — Set absolute volume (TL register).
    SetVolumeAbs = 0x38,
    /// 0x39 — Volume fade (relative TL change over time).
    VolumeFadeRel = 0x39,
    /// 0x3A — FM key-on with complex setup.
    FmKeyOnComplex = 0x3A,
    /// 0x3B — Update note register directly.
    UpdateNoteRegister = 0x3B,
    /// 0x3C — Note register delta (add to current note value).
    NoteRegisterDelta = 0x3C,
    /// 0x3D — Free voice and end sequence.
    FreeVoiceEnd = 0x3D,
    /// 0x3E — Set ROM bank for sequence reads (TZ+ firmware).
    SetBankSwitch = 0x3E,
}

impl SeqOpcode {
    /// Try to decode a 6-bit opcode value.
    pub fn from_u8(value: u8) -> Option<Self> {
        // The opcode is masked to 6 bits by the sequencer: `voice[4] & 0x3F`.
        match value & 0x3F {
            0x00 => Some(Self::EndOfSequence),
            0x01..=0x07 => Some(Self::Nop),
            0x08 => Some(Self::CvsdSampleStartStereo),
            0x09 => Some(Self::CvsdSampleStart),
            0x0A => Some(Self::TimingAdvance),
            0x0B => Some(Self::NoteOnTiming),
            0x0C => Some(Self::FmPatchLoad),
            0x0D => Some(Self::PushPitchTable),
            0x0E => Some(Self::RepeatLoop),
            0x0F => Some(Self::SetAbsolutePitch),
            0x10 => Some(Self::TriggerSubSound),
            0x11 => Some(Self::GapNop11),
            0x12 => Some(Self::InjectCmdRerun),
            0x13 => Some(Self::SubroutineCall),
            0x14 => Some(Self::SubroutineReturn),
            0x15 => Some(Self::FmKeyOff),
            0x16 => Some(Self::GapNop16),
            0x17 => Some(Self::CvsdFmNoteOnAbs),
            0x18 => Some(Self::CvsdVoiceUpdate),
            0x19 => Some(Self::CvsdFmNoteOnDelta),
            0x1A => Some(Self::CvsdVibrato),
            0x1B => Some(Self::Detune),
            0x1C => Some(Self::PushPitchTableAlt),
            0x1D => Some(Self::RepeatLoopAlt),
            0x1E => Some(Self::NoteTimingCombined),
            0x1F => Some(Self::PitchGlide),
            0x20 => Some(Self::GapNop20),
            0x21 => Some(Self::StopDacCvsd),
            0x22 => Some(Self::CvsdVolumeFade),
            0x23 => Some(Self::SendStatusToCpu),
            0x24 => Some(Self::SetChannelTiming),
            0x25 => Some(Self::FmKeyOnWithTiming),
            0x26 => Some(Self::AddTimingDelta),
            0x27 => Some(Self::FmPitchDeltaTableB),
            0x28 => Some(Self::FmPitchDeltaTableA),
            0x29 => Some(Self::FmPitchAbsTableB),
            0x2A => Some(Self::FmPitchAbsTableA),
            0x2B => Some(Self::GlobalPitchSlideHi),
            0x2C => Some(Self::GlobalPitchSlideLo),
            0x2D => Some(Self::SetGlobalPitchAbsHi),
            0x2E => Some(Self::SetGlobalPitchAbsLo),
            0x2F => Some(Self::NoteTriggerRepeat),
            0x30 => Some(Self::SetStereoMask),
            0x31 => Some(Self::ClearChannelMask),
            0x32 => Some(Self::IndirectOpcodeLoad),
            0x33 => Some(Self::InjectCmdRingBuf),
            0x34 => Some(Self::CvsdSamplePlayback),
            0x35 => Some(Self::TimingAdvanceAlt),
            0x36 => Some(Self::ProgramChange),
            0x37 => Some(Self::PushPitchTable4b),
            0x38 => Some(Self::SetVolumeAbs),
            0x39 => Some(Self::VolumeFadeRel),
            0x3A => Some(Self::FmKeyOnComplex),
            0x3B => Some(Self::UpdateNoteRegister),
            0x3C => Some(Self::NoteRegisterDelta),
            0x3D => Some(Self::FreeVoiceEnd),
            0x3E => Some(Self::SetBankSwitch),
            _ => None,
        }
    }
}

// ---------------------------------------------------------------------------
// Operand info — how many bytes each opcode consumes from the data stream
// ---------------------------------------------------------------------------

/// How the sequencer determines the next opcode after executing an instruction.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum NextOp {
    /// The last byte consumed from the stream is the next opcode.
    Embedded,
    /// The sequence terminates (voice freed or looping).
    Terminal,
    /// Control-flow transfer (subroutine call/return, indirect load).
    Branch,
}

impl SeqOpcode {
    /// Total bytes consumed from the data stream and next-opcode behavior.
    ///
    /// For [`NextOp::Embedded`], the last consumed byte is the next opcode.
    /// For variable-length opcodes, returns `(primary, Some(alternate))`.
    fn operand_info(self) -> (usize, Option<usize>, NextOp) {
        use SeqOpcode::*;
        match self {
            EndOfSequence => (1, None, NextOp::Terminal),
            Nop => (0, None, NextOp::Terminal),
            CvsdSampleStartStereo => (4, None, NextOp::Embedded),
            CvsdSampleStart => (3, None, NextOp::Embedded),
            TimingAdvance => (3, Some(4), NextOp::Embedded),
            NoteOnTiming => (2, Some(3), NextOp::Embedded),
            FmPatchLoad => (2, None, NextOp::Embedded),
            PushPitchTable | PushPitchTableAlt => (2, None, NextOp::Embedded),
            RepeatLoop | RepeatLoopAlt => (1, None, NextOp::Embedded),
            SetAbsolutePitch => (2, None, NextOp::Embedded),
            TriggerSubSound => (2, None, NextOp::Embedded),
            GapNop11 | GapNop16 | GapNop20 => (0, None, NextOp::Terminal),
            InjectCmdRerun => (1, None, NextOp::Terminal),
            SubroutineCall => (2, None, NextOp::Branch),
            SubroutineReturn => (0, None, NextOp::Branch),
            FmKeyOff => (0, None, NextOp::Terminal),
            CvsdFmNoteOnAbs => (3, None, NextOp::Embedded),
            CvsdVoiceUpdate => (3, None, NextOp::Embedded),
            CvsdFmNoteOnDelta => (3, None, NextOp::Embedded),
            CvsdVibrato => (5, None, NextOp::Embedded),
            Detune => (3, None, NextOp::Embedded),
            NoteTimingCombined => (2, Some(3), NextOp::Embedded),
            PitchGlide => (11, None, NextOp::Embedded),
            StopDacCvsd => (0, None, NextOp::Terminal),
            CvsdVolumeFade => (2, None, NextOp::Embedded),
            SendStatusToCpu => (2, None, NextOp::Embedded),
            SetChannelTiming => (3, None, NextOp::Embedded),
            FmKeyOnWithTiming => (1, None, NextOp::Embedded),
            AddTimingDelta => (3, None, NextOp::Embedded),
            FmPitchDeltaTableB => (3, None, NextOp::Embedded),
            FmPitchDeltaTableA => (3, None, NextOp::Embedded),
            FmPitchAbsTableB => (3, None, NextOp::Embedded),
            FmPitchAbsTableA => (3, None, NextOp::Embedded),
            GlobalPitchSlideHi => (3, None, NextOp::Embedded),
            GlobalPitchSlideLo => (3, None, NextOp::Embedded),
            SetGlobalPitchAbsHi => (3, None, NextOp::Embedded),
            SetGlobalPitchAbsLo => (3, None, NextOp::Embedded),
            NoteTriggerRepeat => (5, None, NextOp::Embedded),
            SetStereoMask => (1, None, NextOp::Embedded),
            ClearChannelMask => (1, None, NextOp::Embedded),
            IndirectOpcodeLoad => (2, None, NextOp::Branch),
            InjectCmdRingBuf => (2, None, NextOp::Embedded),
            CvsdSamplePlayback => (2, None, NextOp::Embedded),
            TimingAdvanceAlt => (2, Some(3), NextOp::Embedded),
            ProgramChange => (3, None, NextOp::Embedded),
            PushPitchTable4b => (4, None, NextOp::Embedded),
            SetVolumeAbs => (2, None, NextOp::Embedded),
            VolumeFadeRel => (2, None, NextOp::Embedded),
            FmKeyOnComplex => (2, Some(3), NextOp::Embedded),
            UpdateNoteRegister => (2, None, NextOp::Embedded),
            NoteRegisterDelta => (2, None, NextOp::Embedded),
            FreeVoiceEnd => (0, None, NextOp::Terminal),
            SetBankSwitch => (2, None, NextOp::Embedded),
        }
    }
}

// ---------------------------------------------------------------------------
// Decoded instruction
// ---------------------------------------------------------------------------

/// A single decoded sequencer instruction.
#[derive(Debug, Clone)]
pub struct SeqInstruction {
    /// Byte offset of this instruction's opcode within the sequence stream.
    pub pos: usize,
    /// Raw opcode byte (may include priority flag in bit 7).
    pub raw_opcode: u8,
    /// Decoded opcode.
    pub opcode: SeqOpcode,
    /// Data operand bytes (embedded next-opcode stripped for non-terminal ops).
    pub operands: Vec<u8>,
}

impl fmt::Display for SeqInstruction {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(
            f,
            "{:04X}: [{:02X}] {:<24}",
            self.pos,
            self.raw_opcode,
            format!("{:?}", self.opcode)
        )?;
        if !self.operands.is_empty() {
            let hex: Vec<String> = self.operands.iter().map(|b| format!("{:02X}", b)).collect();
            write!(f, " {}", hex.join(" "))?;
        }
        Ok(())
    }
}

// ---------------------------------------------------------------------------
// Decoded sequence
// ---------------------------------------------------------------------------

/// A fully decoded bytecode sequence for one channel.
#[derive(Debug, Clone)]
pub struct DecodedSequence {
    /// Channel label (e.g. "FM2", "CVSD").
    pub channel: String,
    /// 6809 start address of the sequence.
    pub start_addr: u16,
    /// Decoded instructions.
    pub instructions: Vec<SeqInstruction>,
    /// Whether decoding completed normally (terminal opcode reached).
    pub complete: bool,
    /// Reason for incomplete decoding.
    pub truncation: Option<String>,
}

const MAX_INSTRUCTIONS: usize = 500;
const MAX_CALL_DEPTH: usize = 8;

/// Decode a sequence starting at the given 6809 address.
fn decode_sequence(
    u18: &[u8],
    start_addr: u16,
    channel: &str,
    header: &RomHeader,
) -> DecodedSequence {
    let mut instructions = Vec::new();
    let mut call_stack: Vec<usize> = Vec::new();

    let start_file = header.to_file_offset(start_addr);
    if start_file >= u18.len() {
        return DecodedSequence {
            channel: channel.to_string(),
            start_addr,
            instructions,
            complete: false,
            truncation: Some("start address out of bounds".into()),
        };
    }

    // The first byte of the sequence is the initial opcode.
    let mut current_op_raw = u18[start_file];
    let mut data_pos = start_file + 1; // data pointer (past initial opcode)
    let mut stream_offset: usize = 0; // byte offset within the stream for display

    for _ in 0..MAX_INSTRUCTIONS {
        let opcode = match SeqOpcode::from_u8(current_op_raw) {
            Some(op) => op,
            None => {
                return DecodedSequence {
                    channel: channel.to_string(),
                    start_addr,
                    instructions,
                    complete: false,
                    truncation: Some(format!("unknown opcode 0x{:02X}", current_op_raw)),
                };
            }
        };

        let (primary, alternate, next_behavior) = opcode.operand_info();

        match next_behavior {
            NextOp::Terminal => {
                let byte_count = primary;
                if data_pos + byte_count > u18.len() {
                    return DecodedSequence {
                        channel: channel.to_string(),
                        start_addr,
                        instructions,
                        complete: false,
                        truncation: Some("truncated at terminal".into()),
                    };
                }
                let operands = u18[data_pos..data_pos + byte_count].to_vec();
                instructions.push(SeqInstruction {
                    pos: stream_offset,
                    raw_opcode: current_op_raw,
                    opcode,
                    operands,
                });
                return DecodedSequence {
                    channel: channel.to_string(),
                    start_addr,
                    instructions,
                    complete: true,
                    truncation: None,
                };
            }

            NextOp::Branch => {
                match opcode {
                    SeqOpcode::SubroutineCall => {
                        if data_pos + 2 > u18.len() {
                            return DecodedSequence {
                                channel: channel.to_string(),
                                start_addr,
                                instructions,
                                complete: false,
                                truncation: Some("truncated at call".into()),
                            };
                        }
                        let target_addr = wpc89::read_be_u16(u18, data_pos);
                        instructions.push(SeqInstruction {
                            pos: stream_offset,
                            raw_opcode: current_op_raw,
                            opcode,
                            operands: vec![u18[data_pos], u18[data_pos + 1]],
                        });

                        // Push return address (byte after the 2-byte target operand).
                        let return_pos = data_pos + 2;
                        if call_stack.len() >= MAX_CALL_DEPTH {
                            return DecodedSequence {
                                channel: channel.to_string(),
                                start_addr,
                                instructions,
                                complete: false,
                                truncation: Some("call stack overflow".into()),
                            };
                        }
                        call_stack.push(return_pos);
                        stream_offset += 3; // opcode position + 2 operand bytes consumed

                        // Jump to target.
                        let target_file = header.to_file_offset(target_addr);
                        if target_file >= u18.len() {
                            return DecodedSequence {
                                channel: channel.to_string(),
                                start_addr,
                                instructions,
                                complete: false,
                                truncation: Some(format!(
                                    "call target 0x{:04X} out of bounds",
                                    target_addr
                                )),
                            };
                        }
                        current_op_raw = u18[target_file];
                        data_pos = target_file + 1;
                    }
                    SeqOpcode::SubroutineReturn => {
                        instructions.push(SeqInstruction {
                            pos: stream_offset,
                            raw_opcode: current_op_raw,
                            opcode,
                            operands: vec![],
                        });

                        if let Some(ret_pos) = call_stack.pop() {
                            if ret_pos >= u18.len() {
                                return DecodedSequence {
                                    channel: channel.to_string(),
                                    start_addr,
                                    instructions,
                                    complete: false,
                                    truncation: Some("return address out of bounds".into()),
                                };
                            }
                            // The byte at ret_pos is the next opcode.
                            current_op_raw = u18[ret_pos];
                            data_pos = ret_pos + 1;
                            stream_offset += 1;
                        } else {
                            // Empty call stack — treat as terminal.
                            return DecodedSequence {
                                channel: channel.to_string(),
                                start_addr,
                                instructions,
                                complete: true,
                                truncation: None,
                            };
                        }
                    }
                    _ => {
                        // IndirectOpcodeLoad or other branch — stop decoding.
                        let byte_count = primary;
                        let end = (data_pos + byte_count).min(u18.len());
                        let operands = u18[data_pos..end].to_vec();
                        instructions.push(SeqInstruction {
                            pos: stream_offset,
                            raw_opcode: current_op_raw,
                            opcode,
                            operands,
                        });
                        return DecodedSequence {
                            channel: channel.to_string(),
                            start_addr,
                            instructions,
                            complete: true,
                            truncation: None,
                        };
                    }
                }
            }

            NextOp::Embedded => {
                // Determine byte count — try primary, then alternate if available.
                let byte_count = resolve_variable_size(u18, data_pos, primary, alternate);

                if data_pos + byte_count > u18.len() || byte_count == 0 {
                    return DecodedSequence {
                        channel: channel.to_string(),
                        start_addr,
                        instructions,
                        complete: false,
                        truncation: Some("truncated".into()),
                    };
                }

                let all_bytes = &u18[data_pos..data_pos + byte_count];
                // Data operands are all bytes except the last (which is the next opcode).
                let operands = all_bytes[..byte_count - 1].to_vec();
                let next_op_raw = all_bytes[byte_count - 1];

                instructions.push(SeqInstruction {
                    pos: stream_offset,
                    raw_opcode: current_op_raw,
                    opcode,
                    operands,
                });

                data_pos += byte_count;
                stream_offset += byte_count; // advance stream offset past operands + next_op
                current_op_raw = next_op_raw;
            }
        }
    }

    DecodedSequence {
        channel: channel.to_string(),
        start_addr,
        instructions,
        complete: false,
        truncation: Some("max instructions reached".into()),
    }
}

/// For variable-length opcodes, try both sizes and pick the one that yields
/// a valid subsequent opcode. Falls back to `primary` if ambiguous.
fn resolve_variable_size(
    data: &[u8],
    pos: usize,
    primary: usize,
    alternate: Option<usize>,
) -> usize {
    let alt = match alternate {
        Some(a) => a,
        None => return primary,
    };

    let sizes = [primary, alt];
    for &size in &sizes {
        if size == 0 || pos + size > data.len() {
            continue;
        }
        let candidate_next = data[pos + size - 1];
        if let Some(next_op) = SeqOpcode::from_u8(candidate_next) {
            // Lookahead: check that the instruction AFTER this one also makes sense.
            let (next_primary, _, next_behavior) = next_op.operand_info();
            match next_behavior {
                NextOp::Terminal | NextOp::Branch => return size,
                NextOp::Embedded => {
                    let next_end = pos + size + next_primary;
                    if next_end <= data.len() && next_primary > 0 {
                        let next_next = data[next_end - 1];
                        if SeqOpcode::from_u8(next_next).is_some() {
                            return size;
                        }
                    }
                }
            }
        }
    }
    // If nothing validated, return primary.
    primary
}

// ---------------------------------------------------------------------------
// Voice descriptor
// ---------------------------------------------------------------------------

/// Parsed voice-type descriptor from the ROM.
///
/// Each descriptor defines which FM channels and/or CVSD voice a sound
/// command uses, along with pointers to their sequence bytecode.
///
/// Format in ROM:
/// ```text
/// [FM_mask: 1]
/// [seq_ptr: 2 × popcount(FM_mask)]   one per set bit, low→high channel
/// [CVSD_type: 1]                      0 = no CVSD
/// [cvsd_seq_ptr: 2]                   only present if CVSD_type ≠ 0
/// ```
#[derive(Debug, Clone)]
pub struct VoiceDescriptor {
    /// 6809 address of this descriptor.
    pub addr: u16,
    /// FM channel bitmask (bit N = channel N active).
    pub fm_mask: u8,
    /// Sequence addresses per FM channel: `(channel_number, 6809_address)`.
    pub fm_channels: Vec<(u8, u16)>,
    /// CVSD type byte (0 = no CVSD).
    pub cvsd_type: u8,
    /// CVSD sequence address (present only when `cvsd_type != 0`).
    pub cvsd_seq_addr: Option<u16>,
}

/// Parse a voice-type descriptor at the given 6809 address.
fn parse_voice_descriptor(u18: &[u8], addr: u16, header: &RomHeader) -> Result<VoiceDescriptor> {
    let base = header.to_file_offset(addr);
    if base >= u18.len() {
        bail!("voice descriptor address 0x{:04X} out of bounds", addr);
    }

    let fm_mask = u18[base];
    let mut offset = base + 1;

    let mut fm_channels = Vec::new();
    for ch in 0..8u8 {
        if fm_mask & (1 << ch) != 0 {
            if offset + 2 > u18.len() {
                bail!("voice descriptor truncated reading FM ch{} pointer", ch);
            }
            let seq_addr = wpc89::read_be_u16(u18, offset);
            fm_channels.push((ch, seq_addr));
            offset += 2;
        }
    }

    if offset >= u18.len() {
        bail!("voice descriptor truncated reading CVSD type");
    }
    let cvsd_type = u18[offset];
    offset += 1;

    let cvsd_seq_addr = if cvsd_type != 0 {
        if offset + 2 > u18.len() {
            bail!("voice descriptor truncated reading CVSD sequence pointer");
        }
        Some(wpc89::read_be_u16(u18, offset))
    } else {
        None
    };

    Ok(VoiceDescriptor {
        addr,
        fm_mask,
        fm_channels,
        cvsd_type,
        cvsd_seq_addr,
    })
}

// ---------------------------------------------------------------------------
// Command dispatch entry
// ---------------------------------------------------------------------------

/// Decoded entry from the `CMD_DISPATCH_TABLE`.
///
/// The firmware's `process_wpc_command_buffer` looks up each incoming
/// command number in this table to determine how to handle it.
///
/// The table is an array of 2-byte entries: `(handler_id, param)`.
#[derive(Debug, Clone)]
pub struct CommandDispatchEntry {
    /// Handler type identifier (selects between different processing paths).
    pub handler_id: u8,
    /// Parameter passed to the handler (meaning depends on handler_id).
    pub param: u8,
}

/// Parse the command dispatch table from a U18 ROM.
fn parse_cmd_dispatch_table(u18: &[u8], header: &RomHeader) -> Result<Vec<CommandDispatchEntry>> {
    let table_file = header.to_file_offset(header.cmd_dispatch_table);
    let count = (header.max_cmd_index as usize) + 1;

    let mut entries = Vec::with_capacity(count);
    for i in 0..count {
        let offset = table_file + i * 2;
        if offset + 2 > u18.len() {
            break;
        }
        entries.push(CommandDispatchEntry {
            handler_id: u18[offset],
            param: u18[offset + 1],
        });
    }
    Ok(entries)
}

// ---------------------------------------------------------------------------
// Sound program
// ---------------------------------------------------------------------------

/// A fully extracted sound program for one command.
#[derive(Debug, Clone)]
pub struct SoundProgram {
    /// Sound command number (0x00–0xFF).
    pub command: u8,
    /// Handler type from the dispatch table.
    pub handler_id: u8,
    /// Voice type index (dispatch table param).
    pub voice_type_index: u8,
    /// Parsed voice descriptor.
    pub descriptor: VoiceDescriptor,
    /// Decoded sequences for each channel.
    pub sequences: Vec<DecodedSequence>,
}

/// Extract all sound programs from a U18 ROM.
///
/// Reads the dispatch table, voice-type table, voice descriptors, and
/// decodes the sequence bytecode for each channel of each command.
///
/// Supports two handler types:
/// - **0x04** (`sound_command_handler`): Uses the voice-type table → voice
///   descriptors with FM_mask + per-channel sequence pointers.
/// - **0x01** (`start_sound_program`): Uses the sound-program table → 10-entry
///   channel pointer arrays read from offset +0x12 backward.
pub fn extract_programs(u18_data: &[u8]) -> Result<Vec<SoundProgram>> {
    let header = RomHeader::from_u18(u18_data)?;
    let dispatch = parse_cmd_dispatch_table(u18_data, &header)?;

    // Parse the voice-type table: a table of 2-byte pointers to descriptors.
    let vt_table_file = header.to_file_offset(header.voice_type_table);

    // Find the maximum voice type index used by handler 0x04.
    let max_vt_idx = dispatch
        .iter()
        .filter(|e| e.handler_id == 0x04)
        .map(|e| e.param as usize)
        .max()
        .unwrap_or(0);

    // Read voice-type table pointers.
    let mut vt_pointers: Vec<Option<u16>> = Vec::with_capacity(max_vt_idx + 1);
    for i in 0..=max_vt_idx {
        let ptr_offset = vt_table_file + i * 2;
        if ptr_offset + 2 <= u18_data.len() {
            vt_pointers.push(Some(wpc89::read_be_u16(u18_data, ptr_offset)));
        } else {
            vt_pointers.push(None);
        }
    }

    // Sound program table for handler 0x01.
    let spt_table_file = header.to_file_offset(header.sound_program_table);

    let mut programs = Vec::new();

    for (cmd_idx, entry) in dispatch.iter().enumerate() {
        match entry.handler_id {
            0x04 => {
                // Voice-type table dispatch.
                let vt_idx = entry.param as usize;
                let desc_addr = match vt_pointers.get(vt_idx).copied().flatten() {
                    Some(addr) => addr,
                    None => continue,
                };
                let descriptor = match parse_voice_descriptor(u18_data, desc_addr, &header) {
                    Ok(d) => d,
                    Err(_) => continue,
                };

                let mut sequences = Vec::new();
                for &(ch, seq_addr) in &descriptor.fm_channels {
                    let label = format!("FM{}", ch);
                    sequences.push(decode_sequence(u18_data, seq_addr, &label, &header));
                }
                if let Some(cvsd_addr) = descriptor.cvsd_seq_addr {
                    sequences.push(decode_sequence(u18_data, cvsd_addr, "CVSD", &header));
                }

                programs.push(SoundProgram {
                    command: cmd_idx as u8,
                    handler_id: entry.handler_id,
                    voice_type_index: entry.param,
                    descriptor,
                    sequences,
                });
            }

            0x01 => {
                // Sound-program table: param indexes a table of 2-byte pointers
                // to program records. Each record has 10 two-byte sequence
                // pointers (channels 0–9). The firmware reads from offset +0x12
                // backward, assigning channels 9 down to 0.
                let param = entry.param as usize;
                let ptr_offset = spt_table_file + param * 2;
                if ptr_offset + 2 > u18_data.len() {
                    continue;
                }
                let prog_addr = wpc89::read_be_u16(u18_data, ptr_offset);
                if prog_addr < 0x4000 {
                    continue;
                }
                let prog_file = header.to_file_offset(prog_addr);
                if prog_file + 20 > u18_data.len() {
                    continue;
                }

                // Build a synthetic VoiceDescriptor from the program record.
                let mut fm_channels = Vec::new();
                let mut fm_mask: u8 = 0;
                for ch in 0u8..10 {
                    let seq_addr = wpc89::read_be_u16(u18_data, prog_file + (ch as usize) * 2);
                    if seq_addr >= 0x4000 {
                        if ch < 8 {
                            fm_mask |= 1 << ch;
                        }
                        fm_channels.push((ch, seq_addr));
                    }
                }

                let descriptor = VoiceDescriptor {
                    addr: prog_addr,
                    fm_mask,
                    fm_channels: fm_channels.clone(),
                    cvsd_type: 0,
                    cvsd_seq_addr: None,
                };

                let mut sequences = Vec::new();
                for &(ch, seq_addr) in &fm_channels {
                    let label = if ch < 8 {
                        format!("FM{}", ch)
                    } else {
                        format!("CH{}", ch)
                    };
                    sequences.push(decode_sequence(u18_data, seq_addr, &label, &header));
                }

                programs.push(SoundProgram {
                    command: cmd_idx as u8,
                    handler_id: entry.handler_id,
                    voice_type_index: entry.param,
                    descriptor,
                    sequences,
                });
            }

            _ => {}
        }
    }

    Ok(programs)
}

// ---------------------------------------------------------------------------
// Display / formatting
// ---------------------------------------------------------------------------

/// Format all programs into a human-readable report string.
pub fn format_programs(programs: &[SoundProgram], header: &RomHeader) -> String {
    let mut out = String::new();

    out.push_str("=== WPC-89 Sound Program Report ===\n");
    out.push_str(&format!(
        "Max command index: 0x{:02X}, Programs decoded: {}\n\n",
        header.max_cmd_index,
        programs.len(),
    ));

    for prog in programs {
        out.push_str(&format!(
            "--- Command 0x{:02X} (handler=0x{:02X}, voice_type=0x{:02X}) ---\n",
            prog.command, prog.handler_id, prog.voice_type_index,
        ));

        let desc = &prog.descriptor;
        out.push_str(&format!("  Descriptor @ 0x{:04X}:\n", desc.addr));

        let ch_list: Vec<String> = (0..8)
            .filter(|b| desc.fm_mask & (1 << b) != 0)
            .map(|b| b.to_string())
            .collect();
        out.push_str(&format!(
            "    FM mask: 0x{:02X} (ch {})\n",
            desc.fm_mask,
            if ch_list.is_empty() {
                "none".to_string()
            } else {
                ch_list.join(", ")
            },
        ));

        for &(ch, addr) in &desc.fm_channels {
            out.push_str(&format!("    FM ch{} seq @ 0x{:04X}\n", ch, addr));
        }

        match desc.cvsd_type {
            0 => out.push_str("    CVSD: none\n"),
            t => {
                out.push_str(&format!("    CVSD type: 0x{:02X}", t));
                if let Some(addr) = desc.cvsd_seq_addr {
                    out.push_str(&format!(", seq @ 0x{:04X}", addr));
                }
                out.push('\n');
            }
        }

        for seq in &prog.sequences {
            out.push('\n');
            let status = if seq.complete {
                "complete"
            } else {
                "INCOMPLETE"
            };
            out.push_str(&format!(
                "  [{}] @ 0x{:04X} ({}, {} instructions)\n",
                seq.channel,
                seq.start_addr,
                status,
                seq.instructions.len(),
            ));

            for inst in &seq.instructions {
                out.push_str(&format!("    {}\n", inst));
            }

            if let Some(reason) = &seq.truncation {
                out.push_str(&format!("    ; truncated: {}\n", reason));
            }
        }

        out.push('\n');
    }

    out
}

/// Produce a brief summary of sound programs found in a ROM.
pub fn summarise_programs(u18_path: &std::path::Path) -> Result<String> {
    let u18_data = std::fs::read(u18_path)
        .with_context(|| format!("failed to read U18 ROM: {}", u18_path.display()))?;
    let header = RomHeader::from_u18(&u18_data)?;
    let programs = extract_programs(&u18_data)?;

    let mut out = String::new();
    out.push_str(&format!(
        "ROM: {}\n",
        u18_path.file_name().unwrap_or_default().to_string_lossy(),
    ));
    out.push_str(&format_programs(&programs, &header));
    Ok(out)
}
