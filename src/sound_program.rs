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

use anyhow::Context;
use thiserror::Error;

use crate::wpc89::{self, RomError, RomHeader};

/// Fatal errors while parsing declared sound-program tables and records.
#[derive(Debug, Error)]
pub enum ProgramError {
    #[error(transparent)]
    Rom(#[from] RomError),
    #[error("{table} pointer {addr:#06x} is invalid: {source}")]
    InvalidTablePointer {
        table: &'static str,
        addr: u16,
        #[source]
        source: RomError,
    },
    #[error(
        "dispatch table is truncated: declared {expected} entries, only {available} bytes remain"
    )]
    TruncatedDispatch { expected: usize, available: usize },
    #[error("voice-type table entry {index} is truncated")]
    TruncatedVoiceTypeEntry { index: usize },
    #[error("command {command:#04x} references invalid voice descriptor {addr:#06x}: {source}")]
    InvalidVoiceDescriptor {
        command: u8,
        addr: u16,
        #[source]
        source: Box<ProgramError>,
    },
    #[error("voice descriptor at {addr:#06x} is truncated while reading {field}")]
    TruncatedVoiceDescriptor { addr: u16, field: String },
    #[error("sound-program table entry {index} is truncated")]
    TruncatedProgramEntry { index: usize },
    #[error("command {command:#04x} references invalid sound program address {addr:#06x}")]
    InvalidProgramAddress { command: u8, addr: u16 },
    #[error("sound program at {addr:#06x} is truncated")]
    TruncatedProgram { addr: u16 },
}

/// Nonfatal facts encountered while extracting otherwise valid programs.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum ProgramDiagnostic {
    UnsupportedHandler {
        command: u8,
        handler_id: u8,
        param: u8,
    },
}

/// Programs plus explicit diagnostics for commands that cannot be decoded.
#[derive(Debug, Clone)]
pub struct ProgramExtraction {
    pub programs: Vec<SoundProgram>,
    pub diagnostics: Vec<ProgramDiagnostic>,
}

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

    let start_file = match header.to_file_offset(start_addr) {
        Ok(offset) => offset,
        Err(error) => {
            return DecodedSequence {
                channel: channel.to_string(),
                start_addr,
                instructions,
                complete: false,
                truncation: Some(error.to_string()),
            };
        }
    };

    // The first byte of the sequence is the initial opcode.
    let Some(mut current_op_raw) = u18.get(start_file).copied() else {
        return DecodedSequence {
            channel: channel.to_string(),
            start_addr,
            instructions,
            complete: false,
            truncation: Some("start address out of bounds".into()),
        };
    };
    let mut data_pos = start_file.saturating_add(1); // data pointer (past initial opcode)
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
                let Some(end) = data_pos.checked_add(byte_count) else {
                    return incomplete_sequence(
                        channel,
                        start_addr,
                        instructions,
                        "terminal range overflow",
                    );
                };
                let Some(bytes) = u18.get(data_pos..end) else {
                    return DecodedSequence {
                        channel: channel.to_string(),
                        start_addr,
                        instructions,
                        complete: false,
                        truncation: Some("truncated at terminal".into()),
                    };
                };
                let operands = bytes.to_vec();
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
                        let Some(operand_bytes) = data_pos
                            .checked_add(2)
                            .and_then(|end| u18.get(data_pos..end))
                        else {
                            return DecodedSequence {
                                channel: channel.to_string(),
                                start_addr,
                                instructions,
                                complete: false,
                                truncation: Some("truncated at call".into()),
                            };
                        };
                        let target_addr = match wpc89::read_be_u16(u18, data_pos) {
                            Ok(addr) => addr,
                            Err(error) => {
                                return incomplete_sequence(
                                    channel,
                                    start_addr,
                                    instructions,
                                    &error.to_string(),
                                );
                            }
                        };
                        instructions.push(SeqInstruction {
                            pos: stream_offset,
                            raw_opcode: current_op_raw,
                            opcode,
                            operands: operand_bytes.to_vec(),
                        });

                        // Push return address (byte after the 2-byte target operand).
                        let return_pos = data_pos.saturating_add(2);
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
                        let target_file = match header.to_file_offset(target_addr) {
                            Ok(offset) => offset,
                            Err(error) => {
                                return incomplete_sequence(
                                    channel,
                                    start_addr,
                                    instructions,
                                    &format!("call target: {error}"),
                                );
                            }
                        };
                        let Some(target_opcode) = u18.get(target_file).copied() else {
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
                        };
                        current_op_raw = target_opcode;
                        data_pos = target_file.saturating_add(1);
                    }
                    SeqOpcode::SubroutineReturn => {
                        instructions.push(SeqInstruction {
                            pos: stream_offset,
                            raw_opcode: current_op_raw,
                            opcode,
                            operands: vec![],
                        });

                        if let Some(ret_pos) = call_stack.pop() {
                            let Some(return_opcode) = u18.get(ret_pos).copied() else {
                                return DecodedSequence {
                                    channel: channel.to_string(),
                                    start_addr,
                                    instructions,
                                    complete: false,
                                    truncation: Some("return address out of bounds".into()),
                                };
                            };
                            // The byte at ret_pos is the next opcode.
                            current_op_raw = return_opcode;
                            data_pos = ret_pos.saturating_add(1);
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
                        let end = data_pos.saturating_add(byte_count).min(u18.len());
                        let operands = u18.get(data_pos..end).unwrap_or_default().to_vec();
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

                let Some(end) = data_pos.checked_add(byte_count) else {
                    return incomplete_sequence(
                        channel,
                        start_addr,
                        instructions,
                        "operand range overflow",
                    );
                };
                let Some(all_bytes) = u18.get(data_pos..end) else {
                    return DecodedSequence {
                        channel: channel.to_string(),
                        start_addr,
                        instructions,
                        complete: false,
                        truncation: Some("truncated".into()),
                    };
                };
                if byte_count == 0 {
                    return incomplete_sequence(
                        channel,
                        start_addr,
                        instructions,
                        "zero-length operand rule",
                    );
                }
                // Data operands are all bytes except the last (which is the next opcode).
                let operands = all_bytes[..byte_count - 1].to_vec();
                let next_op_raw = all_bytes[byte_count - 1];

                instructions.push(SeqInstruction {
                    pos: stream_offset,
                    raw_opcode: current_op_raw,
                    opcode,
                    operands,
                });

                data_pos = end;
                stream_offset = stream_offset.saturating_add(byte_count); // advance stream offset past operands + next_op
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

fn incomplete_sequence(
    channel: &str,
    start_addr: u16,
    instructions: Vec<SeqInstruction>,
    reason: &str,
) -> DecodedSequence {
    DecodedSequence {
        channel: channel.to_string(),
        start_addr,
        instructions,
        complete: false,
        truncation: Some(reason.to_string()),
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
        let Some(end) = pos.checked_add(size) else {
            continue;
        };
        if size == 0 || end > data.len() {
            continue;
        }
        let Some(candidate_next) = data.get(end - 1).copied() else {
            continue;
        };
        if let Some(next_op) = SeqOpcode::from_u8(candidate_next) {
            // Lookahead: check that the instruction AFTER this one also makes sense.
            let (next_primary, _, next_behavior) = next_op.operand_info();
            match next_behavior {
                NextOp::Terminal | NextOp::Branch => return size,
                NextOp::Embedded => {
                    let Some(next_end) = end.checked_add(next_primary) else {
                        continue;
                    };
                    if next_end <= data.len() && next_primary > 0 {
                        let Some(next_next) = data.get(next_end - 1).copied() else {
                            continue;
                        };
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
fn parse_voice_descriptor(
    u18: &[u8],
    addr: u16,
    header: &RomHeader,
) -> std::result::Result<VoiceDescriptor, ProgramError> {
    let base = header.to_file_offset(addr)?;
    let fm_mask = u18
        .get(base)
        .copied()
        .ok_or_else(|| ProgramError::TruncatedVoiceDescriptor {
            addr,
            field: "FM mask".into(),
        })?;
    let mut offset = base.checked_add(1).ok_or(RomError::ArithmeticOverflow {
        context: "voice descriptor offset",
    })?;

    let mut fm_channels = Vec::new();
    for ch in 0..8u8 {
        if fm_mask & (1 << ch) != 0 {
            let seq_addr = wpc89::read_be_u16(u18, offset).map_err(|_| {
                ProgramError::TruncatedVoiceDescriptor {
                    addr,
                    field: format!("FM channel {ch} pointer"),
                }
            })?;
            fm_channels.push((ch, seq_addr));
            offset = offset.checked_add(2).ok_or(RomError::ArithmeticOverflow {
                context: "voice descriptor channel offset",
            })?;
        }
    }

    let cvsd_type =
        u18.get(offset)
            .copied()
            .ok_or_else(|| ProgramError::TruncatedVoiceDescriptor {
                addr,
                field: "CVSD type".into(),
            })?;
    offset = offset.saturating_add(1);

    let cvsd_seq_addr = if cvsd_type != 0 {
        Some(wpc89::read_be_u16(u18, offset).map_err(|_| {
            ProgramError::TruncatedVoiceDescriptor {
                addr,
                field: "CVSD sequence pointer".into(),
            }
        })?)
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
fn parse_cmd_dispatch_table(
    u18: &[u8],
    header: &RomHeader,
) -> std::result::Result<Vec<CommandDispatchEntry>, ProgramError> {
    let table_file = header
        .to_file_offset(header.cmd_dispatch_table)
        .map_err(|source| ProgramError::InvalidTablePointer {
            table: "command dispatch table",
            addr: header.cmd_dispatch_table,
            source,
        })?;
    let count = (header.max_cmd_index as usize) + 1;
    let byte_count = count.checked_mul(2).ok_or(RomError::ArithmeticOverflow {
        context: "dispatch table size",
    })?;
    let available = u18.len().saturating_sub(table_file);
    let end = table_file
        .checked_add(byte_count)
        .ok_or(RomError::ArithmeticOverflow {
            context: "dispatch table range",
        })?;
    let bytes = u18
        .get(table_file..end)
        .ok_or(ProgramError::TruncatedDispatch {
            expected: count,
            available,
        })?;

    let mut entries = Vec::with_capacity(count);
    for pair in bytes.chunks_exact(2) {
        entries.push(CommandDispatchEntry {
            handler_id: pair[0],
            param: pair[1],
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
pub fn extract_programs(u18_data: &[u8]) -> std::result::Result<ProgramExtraction, ProgramError> {
    let header = RomHeader::from_u18(u18_data)?;
    let dispatch = parse_cmd_dispatch_table(u18_data, &header)?;

    // Parse the voice-type table: a table of 2-byte pointers to descriptors.
    let vt_table_file = header
        .to_file_offset(header.voice_type_table)
        .map_err(|source| ProgramError::InvalidTablePointer {
            table: "voice-type table",
            addr: header.voice_type_table,
            source,
        })?;

    // Find the maximum voice type index used by handler 0x04.
    let max_vt_idx = dispatch
        .iter()
        .filter(|e| e.handler_id == 0x04)
        .map(|e| e.param as usize)
        .max()
        .unwrap_or(0);

    // Read voice-type table pointers.
    let mut vt_pointers: Vec<u16> = Vec::with_capacity(max_vt_idx + 1);
    for i in 0..=max_vt_idx {
        let ptr_offset = i
            .checked_mul(2)
            .and_then(|n| vt_table_file.checked_add(n))
            .ok_or(RomError::ArithmeticOverflow {
                context: "voice-type table offset",
            })?;
        vt_pointers.push(
            wpc89::read_be_u16(u18_data, ptr_offset)
                .map_err(|_| ProgramError::TruncatedVoiceTypeEntry { index: i })?,
        );
    }

    // Sound program table for handler 0x01.
    let spt_table_file = header
        .to_file_offset(header.sound_program_table)
        .map_err(|source| ProgramError::InvalidTablePointer {
            table: "sound-program table",
            addr: header.sound_program_table,
            source,
        })?;

    let mut programs = Vec::new();
    let mut diagnostics = Vec::new();

    for (cmd_idx, entry) in dispatch.iter().enumerate() {
        match entry.handler_id {
            0x04 => {
                // Voice-type table dispatch.
                let vt_idx = entry.param as usize;
                let desc_addr = vt_pointers[vt_idx];
                let descriptor =
                    parse_voice_descriptor(u18_data, desc_addr, &header).map_err(|source| {
                        ProgramError::InvalidVoiceDescriptor {
                            command: cmd_idx as u8,
                            addr: desc_addr,
                            source: Box::new(source),
                        }
                    })?;

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
                let ptr_offset = param
                    .checked_mul(2)
                    .and_then(|n| spt_table_file.checked_add(n))
                    .ok_or(RomError::ArithmeticOverflow {
                        context: "sound-program table offset",
                    })?;
                let prog_addr = wpc89::read_be_u16(u18_data, ptr_offset)
                    .map_err(|_| ProgramError::TruncatedProgramEntry { index: param })?;
                if prog_addr < 0x4000 {
                    return Err(ProgramError::InvalidProgramAddress {
                        command: cmd_idx as u8,
                        addr: prog_addr,
                    });
                }
                let prog_file = header.to_file_offset(prog_addr).map_err(|_| {
                    ProgramError::InvalidProgramAddress {
                        command: cmd_idx as u8,
                        addr: prog_addr,
                    }
                })?;
                let prog_end = prog_file
                    .checked_add(20)
                    .ok_or(RomError::ArithmeticOverflow {
                        context: "sound-program record range",
                    })?;
                let record = u18_data
                    .get(prog_file..prog_end)
                    .ok_or(ProgramError::TruncatedProgram { addr: prog_addr })?;

                // Build a synthetic VoiceDescriptor from the program record.
                let mut fm_channels = Vec::new();
                let mut fm_mask: u8 = 0;
                for ch in 0u8..10 {
                    let seq_pos = usize::from(ch) * 2;
                    let seq_addr = u16::from_be_bytes([record[seq_pos], record[seq_pos + 1]]);
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

            _ => diagnostics.push(ProgramDiagnostic::UnsupportedHandler {
                command: cmd_idx as u8,
                handler_id: entry.handler_id,
                param: entry.param,
            }),
        }
    }

    Ok(ProgramExtraction {
        programs,
        diagnostics,
    })
}

// ---------------------------------------------------------------------------
// Display / formatting
// ---------------------------------------------------------------------------

/// Format all programs into a human-readable report string.
pub fn format_programs(extraction: &ProgramExtraction, header: &RomHeader) -> String {
    let mut out = String::new();

    out.push_str("=== WPC-89 Sound Program Report ===\n");
    out.push_str(&format!(
        "Max command index: 0x{:02X}, Programs decoded: {}\n\n",
        header.max_cmd_index,
        extraction.programs.len(),
    ));

    for prog in &extraction.programs {
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

    if !extraction.diagnostics.is_empty() {
        out.push_str("=== Diagnostics ===\n");
        for diagnostic in &extraction.diagnostics {
            match diagnostic {
                ProgramDiagnostic::UnsupportedHandler {
                    command,
                    handler_id,
                    param,
                } => out.push_str(&format!(
                    "Unsupported handler: command=0x{command:02X}, handler=0x{handler_id:02X}, param=0x{param:02X}\n"
                )),
            }
        }
    }

    out
}

/// Produce a brief summary of sound programs found in a ROM.
pub fn summarise_programs(u18_path: &std::path::Path) -> anyhow::Result<String> {
    let u18_data = std::fs::read(u18_path)
        .with_context(|| format!("failed to read U18 ROM: {}", u18_path.display()))?;
    let header = RomHeader::from_u18(&u18_data)?;
    let extraction = extract_programs(&u18_data)?;

    let mut out = String::new();
    out.push_str(&format!(
        "ROM: {}\n",
        u18_path.file_name().unwrap_or_default().to_string_lossy(),
    ));
    out.push_str(&format_programs(&extraction, &header));
    Ok(out)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::wpc89::{
        ROM_HDR_CMD_DISPATCH_TABLE, ROM_HDR_CVSD_SAMPLE_TABLE, ROM_HDR_DAC_SAMPLE_TABLE,
        ROM_HDR_FM_PATCH_TABLE, ROM_HDR_FM_PROGRAM_TABLE, ROM_HDR_MAX_CMD_INDEX,
        ROM_HDR_SOUND_PROGRAM_TABLE, ROM_HDR_VOICE_TYPE_TABLE,
    };

    fn set_word(data: &mut [u8], pos: usize, value: u16) {
        data[pos..pos + 2].copy_from_slice(&value.to_be_bytes());
    }

    fn fixture(dispatch_addr: u16) -> Vec<u8> {
        let mut rom = vec![0; 0x20000];
        for (offset, address) in [
            (ROM_HDR_FM_PATCH_TABLE, 0x4400),
            (ROM_HDR_DAC_SAMPLE_TABLE, 0x4400),
            (ROM_HDR_FM_PROGRAM_TABLE, 0x4400),
            (ROM_HDR_VOICE_TYPE_TABLE, 0x4100),
            (ROM_HDR_CMD_DISPATCH_TABLE, dispatch_addr),
            (ROM_HDR_SOUND_PROGRAM_TABLE, 0x4300),
            (ROM_HDR_CVSD_SAMPLE_TABLE, 0x4400),
        ] {
            set_word(&mut rom, offset, address);
        }
        rom
    }

    #[test]
    fn zero_filled_128k_rom_returns_error_without_panicking() {
        let rom = vec![0; 0x20000];
        assert!(extract_programs(&rom).is_err());
    }

    #[test]
    fn truncated_declared_dispatch_table_is_fatal() {
        let mut rom = fixture(0xFFFF);
        rom[ROM_HDR_MAX_CMD_INDEX] = 1;
        let error = extract_programs(&rom).unwrap_err();
        assert!(matches!(error, ProgramError::TruncatedDispatch { .. }));
    }

    #[test]
    fn unsupported_handler_is_a_structured_diagnostic() {
        let mut rom = fixture(0x4200);
        rom[0x200..0x202].copy_from_slice(&[0x77, 0x55]);

        let extraction = extract_programs(&rom).unwrap();
        assert!(extraction.programs.is_empty());
        assert_eq!(
            extraction.diagnostics,
            vec![ProgramDiagnostic::UnsupportedHandler {
                command: 0,
                handler_id: 0x77,
                param: 0x55,
            }]
        );
    }

    #[test]
    fn referenced_voice_descriptor_pointer_is_never_a_sentinel() {
        let mut rom = fixture(0x4200);
        rom[0x200..0x202].copy_from_slice(&[0x04, 0x00]);
        set_word(&mut rom, 0x100, 0x0001);

        let error = extract_programs(&rom).unwrap_err();
        assert!(matches!(error, ProgramError::InvalidVoiceDescriptor { .. }));
    }

    #[test]
    fn truncated_referenced_voice_descriptor_is_fatal() {
        let mut rom = fixture(0x4200);
        rom[0x200..0x202].copy_from_slice(&[0x04, 0x00]);
        set_word(&mut rom, 0x100, 0xFFFF);
        rom[0x1FFFF] = 0x01;

        let error = extract_programs(&rom).unwrap_err();
        assert!(matches!(
            error,
            ProgramError::InvalidVoiceDescriptor { source, .. }
                if matches!(*source, ProgramError::TruncatedVoiceDescriptor { .. })
        ));
    }

    #[test]
    fn referenced_sound_program_pointer_is_never_a_sentinel() {
        let mut rom = fixture(0x4200);
        rom[0x200..0x202].copy_from_slice(&[0x01, 0x00]);
        set_word(&mut rom, 0x300, 0x0000);

        let error = extract_programs(&rom).unwrap_err();
        assert!(matches!(error, ProgramError::InvalidProgramAddress { .. }));
    }

    #[test]
    fn subwindow_channel_pointers_are_unused_slots() {
        let mut rom = fixture(0x4200);
        rom[0x200..0x202].copy_from_slice(&[0x01, 0x00]);
        set_word(&mut rom, 0x300, 0x4500);
        // The zero-filled 20-byte record contains ten legitimate unused slots.

        let extraction = extract_programs(&rom).unwrap();
        assert_eq!(extraction.programs.len(), 1);
        assert!(extraction.programs[0].sequences.is_empty());
    }
}
