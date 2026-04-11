//! WPC-89 sound-program extraction (stubbed for future work).
//!
//! The WPC-89 sound firmware uses a voice-sequencer architecture to play
//! sounds.  Each sound command from the main CPU triggers one or more
//! "voices", each running a small bytecode program (sequence data).
//!
//! The bytecode is dispatched through an opcode table with 6-bit opcodes
//! (0x00–0x3D).  This module defines the opcode enum and data structures
//! needed to parse the command tables.  The actual sequence-data decoder
//! is left as a TODO for a future session.
//!
//! # Reference
//!
//! All structures and opcodes are derived from the decompiled firmware
//! `reference/BL_U18.L1.c`.

use anyhow::{Context, Result};

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
    // 0x02..0x07 — not observed in the decompiled firmware; likely unused or aliases.
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
    // 0x11 — not observed.
    /// 0x12 — Inject command for immediate re-run (bypass ring buffer).
    InjectCmdRerun = 0x12,
    /// 0x13 — Subroutine call (push return address, jump to target).
    SubroutineCall = 0x13,
    /// 0x14 — Subroutine return (pop return address).
    SubroutineReturn = 0x14,
    /// 0x15 — FM key-off on the voice's channel.
    FmKeyOff = 0x15,
    // 0x16 — not observed.
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
    // 0x1C..0x1D — not observed.
    /// 0x1E — Combined note + timing setup.
    NoteTimingCombined = 0x1E,
    /// 0x1F — Pitch glide (smooth pitch transition).
    PitchGlide = 0x1F,
    // 0x20 — not observed.
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
    // 0x34 — not observed.
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
}

impl SeqOpcode {
    /// Try to decode a 6-bit opcode value.
    pub fn from_u8(value: u8) -> Option<Self> {
        // The opcode is masked to 6 bits by the sequencer: `voice[4] & 0x3F`.
        match value & 0x3F {
            0x00 => Some(Self::EndOfSequence),
            0x01 => Some(Self::Nop),
            0x08 => Some(Self::CvsdSampleStartStereo),
            0x09 => Some(Self::CvsdSampleStart),
            0x0A => Some(Self::TimingAdvance),
            0x0B => Some(Self::NoteOnTiming),
            0x0C => Some(Self::FmPatchLoad),
            0x0D => Some(Self::PushPitchTable),
            0x0E => Some(Self::RepeatLoop),
            0x0F => Some(Self::SetAbsolutePitch),
            0x10 => Some(Self::TriggerSubSound),
            0x12 => Some(Self::InjectCmdRerun),
            0x13 => Some(Self::SubroutineCall),
            0x14 => Some(Self::SubroutineReturn),
            0x15 => Some(Self::FmKeyOff),
            0x17 => Some(Self::CvsdFmNoteOnAbs),
            0x18 => Some(Self::CvsdVoiceUpdate),
            0x19 => Some(Self::CvsdFmNoteOnDelta),
            0x1A => Some(Self::CvsdVibrato),
            0x1B => Some(Self::Detune),
            0x1E => Some(Self::NoteTimingCombined),
            0x1F => Some(Self::PitchGlide),
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
            0x35 => Some(Self::TimingAdvanceAlt),
            0x36 => Some(Self::ProgramChange),
            0x37 => Some(Self::PushPitchTable4b),
            0x38 => Some(Self::SetVolumeAbs),
            0x39 => Some(Self::VolumeFadeRel),
            0x3A => Some(Self::FmKeyOnComplex),
            0x3B => Some(Self::UpdateNoteRegister),
            0x3C => Some(Self::NoteRegisterDelta),
            0x3D => Some(Self::FreeVoiceEnd),
            _ => None,
        }
    }
}

// ---------------------------------------------------------------------------
// Voice-type info
// ---------------------------------------------------------------------------

/// Decoded voice-type entry from the `CMD_VOICE_TYPE_TABLE`.
///
/// Each sound command maps to a voice-type byte describing which audio
/// channels it uses:
///
/// - Bits [0:7] form a bitmask of FM channels (0–7) allocated to the voice.
/// - If the CVSD flag is set, the command also triggers CVSD playback.
///
/// Source: `sound_command_handler` in the decompiled firmware.
#[derive(Debug, Clone)]
pub struct VoiceTypeInfo {
    /// Bitmask of FM channels used by this command.
    pub channel_mask: u8,
    /// Whether this command uses CVSD (compressed audio).
    pub uses_cvsd: bool,
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

// ---------------------------------------------------------------------------
// Sound program reference
// ---------------------------------------------------------------------------

/// A reference to one sound program's sequence data.
///
/// Each command that triggers a sound program maps to a pointer in the
/// `SOUND_PROGRAM_TABLE`.  That pointer leads to a chain of voice-type +
/// sequence-data entries that the `sound_command_handler` iterates over.
#[derive(Debug, Clone)]
pub struct SoundProgramRef {
    /// The sound command number.
    pub command: u8,
    /// Voice type info for this command.
    pub voice_type: VoiceTypeInfo,
    /// 6809 address of the program's sequence data.
    pub sequence_addr: u16,
    /// File offset of the sequence data in the U18 ROM.
    pub sequence_file_offset: usize,
}

// ---------------------------------------------------------------------------
// Table parsing
// ---------------------------------------------------------------------------

/// Parse the voice-type table from a U18 ROM.
///
/// Returns one [`VoiceTypeInfo`] per valid command index (0 through
/// `header.max_cmd_index`).
pub fn parse_voice_type_table(u18_data: &[u8], header: &RomHeader) -> Result<Vec<VoiceTypeInfo>> {
    let table_file = header.to_file_offset(header.voice_type_table);
    let count = (header.max_cmd_index as usize) + 1;

    let mut entries = Vec::with_capacity(count);
    for i in 0..count {
        let offset = table_file + i;
        if offset >= u18_data.len() {
            break;
        }
        let byte = u18_data[offset];
        entries.push(VoiceTypeInfo {
            channel_mask: byte & 0x0F,
            uses_cvsd: byte & 0x80 != 0,
        });
    }
    Ok(entries)
}

/// Parse the command dispatch table from a U18 ROM.
///
/// Returns one [`CommandDispatchEntry`] per valid command index.
pub fn parse_cmd_dispatch_table(
    u18_data: &[u8],
    header: &RomHeader,
) -> Result<Vec<CommandDispatchEntry>> {
    let table_file = header.to_file_offset(header.cmd_dispatch_table);
    let count = (header.max_cmd_index as usize) + 1;

    let mut entries = Vec::with_capacity(count);
    for i in 0..count {
        let offset = table_file + i * 2;
        if offset + 2 > u18_data.len() {
            break;
        }
        entries.push(CommandDispatchEntry {
            handler_id: u18_data[offset],
            param: u18_data[offset + 1],
        });
    }
    Ok(entries)
}

/// Parse sound-program references for all valid commands.
///
/// For each command that has a non-zero voice-type entry, reads the
/// corresponding pointer from the sound-program table.
///
/// **Note:** This only parses the table structure — it does NOT decode
/// the actual sequence bytecode.  Sequence decoding is deferred to a
/// future session.
pub fn parse_sound_programs(u18_data: &[u8], header: &RomHeader) -> Result<Vec<SoundProgramRef>> {
    let prog_table_file = header.to_file_offset(header.sound_program_table);
    let voice_types = parse_voice_type_table(u18_data, header)?;

    let mut programs = Vec::new();
    for (cmd_idx, vt) in voice_types.iter().enumerate() {
        // Skip commands with no channel allocation (silent / system commands).
        if vt.channel_mask == 0 && !vt.uses_cvsd {
            continue;
        }

        let ptr_offset = prog_table_file + cmd_idx * 2;
        if ptr_offset + 2 > u18_data.len() {
            break;
        }
        let seq_addr = wpc89::read_rom_header_ptr(u18_data, 0, ptr_offset);
        if seq_addr < 0x4000 {
            continue;
        }

        let seq_file = header.to_file_offset(seq_addr);

        programs.push(SoundProgramRef {
            command: cmd_idx as u8,
            voice_type: vt.clone(),
            sequence_addr: seq_addr,
            sequence_file_offset: seq_file,
        });
    }
    Ok(programs)
}

/// Summarise the sound programs found in a ROM set.
///
/// This is a convenience function for quick inspection. It reads the U18
/// ROM, parses the header and program table, and returns a formatted
/// summary string.
pub fn summarise_programs(u18_path: &std::path::Path) -> Result<String> {
    let u18_data = std::fs::read(u18_path)
        .with_context(|| format!("failed to read U18 ROM: {}", u18_path.display()))?;
    let header = RomHeader::from_u18(&u18_data)?;
    let programs = parse_sound_programs(&u18_data, &header)?;

    let mut out = String::new();
    out.push_str(&format!(
        "ROM header: max_cmd={:#04x}, {} sound programs found\n",
        header.max_cmd_index,
        programs.len()
    ));
    for p in &programs {
        out.push_str(&format!(
            "  cmd {:#04x}: voice_type(ch_mask={:#04x}, cvsd={}), seq_addr={:#06x}\n",
            p.command, p.voice_type.channel_mask, p.voice_type.uses_cvsd, p.sequence_addr,
        ));
    }
    // TODO: Future work — decode sequence bytecode from each program's
    //       sequence_file_offset, turning the raw bytes into a stream of
    //       SeqOpcode variants with their operands.  This will enable:
    //       - Disassembly / pretty-printing of sound programs
    //       - Identifying which CVSD samples each command plays
    //       - Potential re-synthesis or modification of sound sequences
    Ok(out)
}
