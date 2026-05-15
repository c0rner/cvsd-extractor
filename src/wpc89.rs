// SPDX-License-Identifier: BSD-3-Clause

use std::path::{Path, PathBuf};

use anyhow::{Context, Result, bail};

use crate::cvsd_chip::CvsdChip;

// ---------------------------------------------------------------------------
// ROM header table-pointer offsets
// ---------------------------------------------------------------------------
//
// The WPC-89 sound firmware stores a block of 16-bit big-endian pointers at
// the base of the default banked ROM page (6809 address 0x4000+).  Each
// pointer references a data table elsewhere in the same ROM page.
//
// These offsets are relative to the default bank base (i.e. add to the file
// position of address 0x4000 in the system bank).
//
// Source: decompiled firmware BL_U18.L1, ROM header comments and usage in
//         start_cvsd_sample(), sound_command_handler(), process_wpc_command_buffer().

/// Pointer to FM patch / instrument table (26 bytes per patch).
pub const ROM_HDR_FM_PATCH_TABLE: usize = 0x01;
/// Pointer to DAC (raw PCM) sample table.
pub const ROM_HDR_DAC_SAMPLE_TABLE: usize = 0x03;
/// Pointer to FM program-change table.
pub const ROM_HDR_FM_PROGRAM_TABLE: usize = 0x05;
/// Pointer to voice-type table (command → FM channel bitmask + CVSD flag).
pub const ROM_HDR_VOICE_TYPE_TABLE: usize = 0x07;
/// Maximum valid sound-command index (1 byte, NOT a pointer).
pub const ROM_HDR_MAX_CMD_INDEX: usize = 0x0E;
/// Pointer to command dispatch table (command → handler_id + param, 2 bytes each).
pub const ROM_HDR_CMD_DISPATCH_TABLE: usize = 0x0F;
/// Pointer to sound-program table (command → sequence-data pointers).
pub const ROM_HDR_SOUND_PROGRAM_TABLE: usize = 0x11;
/// Pointer to CVSD compressed-sample table.
pub const ROM_HDR_CVSD_SAMPLE_TABLE: usize = 0x15;

// ---------------------------------------------------------------------------
// Bank selector constants
// ---------------------------------------------------------------------------
//
// The bank register at I/O address 0x2000 uses **active-low chip-enable**
// signals in bits [7:5] to select one of three ROM sockets, plus a 5-bit
// page number in bits [4:0]:
//
//   bit 7 low  →  U18 selected   (bits [7:5] = 011  →  masked value 0x60)
//   bit 6 low  →  U15 selected   (bits [7:5] = 101  →  masked value 0xA0)
//   bit 5 low  →  U14 selected   (bits [7:5] = 110  →  masked value 0xC0)
//
// Source: decompiled firmware ROM-checksum routine and FIRQ ISR bank restore.

/// Chip-enable mask: the top three bits of the bank selector.
const CHIP_ENABLE_MASK: u8 = 0xE0;
/// Bank-number mask: the lower five bits of the bank selector.
const BANK_NUMBER_MASK: u8 = 0x1F;

/// Default / system bank selector for U18.  Written to 0x2000 at the end of
/// every FIRQ to restore access to the ROM header and sequencer tables.
/// Encodes U18 chip-enable (0x60) + page 0x1C.
pub const SYSTEM_BANK: u8 = 0x7C;

/// The three ROM chips on the WPC-89 sound board.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RomChip {
    U14,
    U15,
    U18,
}

/// A single decoded CVSD audio entry from the ROM table.
///
/// Each entry in the CVSD sample table is a 5-byte record:
///
/// | Offset | Size | Field                                              |
/// |--------|------|----------------------------------------------------|
/// | +0     | 1    | Bank selector (written to 0x2000 to page in data)  |
/// | +1     | 2    | Start address of CVSD data (big-endian, 0x4000+)   |
/// | +3     | 2    | End address of CVSD data (big-endian, exclusive)    |
///
/// The sample table itself is a list of 16-bit pointers to these records,
/// indexed by a 7-bit sample number from the voice sequencer.
pub struct CvsdEntry {
    /// Which ROM chip the audio data lives in.
    pub chip: RomChip,
    /// ROM bank number (bits \[4:0\] of the bank selector byte).
    pub bank: u8,
    /// Byte offset of the CVSD data in the ROM file.
    pub offset: usize,
    /// Number of bytes of CVSD data.
    pub size: usize,
    /// Sequential index of this entry in the table.
    pub index: usize,
}

/// Paths to the three sound ROM files.
pub struct RomSet {
    pub u14: PathBuf,
    pub u15: PathBuf,
    pub u18: PathBuf,
}

impl RomSet {
    pub fn new(u14: impl AsRef<Path>, u15: impl AsRef<Path>, u18: impl AsRef<Path>) -> Self {
        RomSet {
            u14: u14.as_ref().to_path_buf(),
            u15: u15.as_ref().to_path_buf(),
            u18: u18.as_ref().to_path_buf(),
        }
    }
}

/// Decode the bank selector byte into a chip identifier and bank number.
///
/// Bits [7:5] carry active-low chip-enable signals; bits [4:0] hold the
/// 32 KB page number within the selected chip.  Returns `None` if the
/// chip-enable pattern does not match any known ROM socket.
fn decode_bank_selector(bank_selector: u8) -> Option<(RomChip, u8)> {
    let chip = match bank_selector & CHIP_ENABLE_MASK {
        0xC0 => RomChip::U14,
        0xA0 => RomChip::U15,
        0x60 => RomChip::U18,
        _ => return None,
    };
    let bank = bank_selector & BANK_NUMBER_MASK;
    Some((chip, bank))
}

/// Read a big-endian u16 from `data` at byte position `pos`.
pub fn read_be_u16(data: &[u8], pos: usize) -> u16 {
    u16::from_be_bytes([data[pos], data[pos + 1]])
}

// ---------------------------------------------------------------------------
// RomHeader — parsed firmware table pointers
// ---------------------------------------------------------------------------

/// Parsed header pointers from the WPC-89 sound ROM.
///
/// All pointer fields are raw 6809 addresses (in the 0x4000–0xBFFF banked
/// window).  They can be converted to file offsets by subtracting 0x4000 and
/// adding [`system_bank_offset`].
///
/// This struct is cheap to construct and provides a typed view over the
/// ROM header so that downstream code (CVSD extraction, sound program
/// parsing, etc.) can share a single parsed representation.
#[derive(Debug, Clone)]
pub struct RomHeader {
    /// File offset of the system bank (U18 page 0x1C).
    pub system_bank_file: usize,
    /// 6809 address of the FM patch table.
    pub fm_patch_table: u16,
    /// 6809 address of the DAC sample table.
    pub dac_sample_table: u16,
    /// 6809 address of the FM program-change table.
    pub fm_program_table: u16,
    /// 6809 address of the voice-type table.
    pub voice_type_table: u16,
    /// Maximum valid sound-command index.
    pub max_cmd_index: u8,
    /// 6809 address of the command dispatch table.
    pub cmd_dispatch_table: u16,
    /// 6809 address of the sound-program table.
    pub sound_program_table: u16,
    /// 6809 address of the CVSD sample table.
    pub cvsd_sample_table: u16,
}

impl RomHeader {
    /// Parse a [`RomHeader`] from raw U18 ROM data.
    pub fn from_u18(u18_data: &[u8]) -> Result<Self> {
        let sbf = system_bank_offset(u18_data.len())
            .context("U18 ROM is too small (need at least 0x20000 bytes)")?;

        Ok(Self {
            system_bank_file: sbf,
            fm_patch_table: read_be_u16(u18_data, sbf + ROM_HDR_FM_PATCH_TABLE),
            dac_sample_table: read_be_u16(u18_data, sbf + ROM_HDR_DAC_SAMPLE_TABLE),
            fm_program_table: read_be_u16(u18_data, sbf + ROM_HDR_FM_PROGRAM_TABLE),
            voice_type_table: read_be_u16(u18_data, sbf + ROM_HDR_VOICE_TYPE_TABLE),
            max_cmd_index: u18_data[sbf + ROM_HDR_MAX_CMD_INDEX],
            cmd_dispatch_table: read_be_u16(u18_data, sbf + ROM_HDR_CMD_DISPATCH_TABLE),
            sound_program_table: read_be_u16(u18_data, sbf + ROM_HDR_SOUND_PROGRAM_TABLE),
            cvsd_sample_table: read_be_u16(u18_data, sbf + ROM_HDR_CVSD_SAMPLE_TABLE),
        })
    }

    /// Convert a 6809 ROM address to a U18 file offset.
    ///
    /// Handles both the banked window (0x4000–0xBFFF, mapped to the system
    /// bank page) and the fixed bank (0xC000–0xFFFF, always the last 16 KB
    /// of the ROM image).
    pub fn to_file_offset(&self, addr: u16) -> usize {
        if addr >= 0xC000 {
            // Fixed bank: last 16 KB of ROM, always accessible.
            let rom_len = self.system_bank_file + 0x20000;
            rom_len - 0x4000 + (addr as usize - 0xC000)
        } else {
            // Banked window: 0x4000–0xBFFF mapped to system bank.
            (addr as usize) - 0x4000 + self.system_bank_file
        }
    }

    /// Total ROM size in bytes.
    pub fn rom_len(&self) -> usize {
        self.system_bank_file + 0x20000
    }
}

/// Parse the CVSD entry table from the U18 ROM and return all entries.
///
/// The CVSD sample table pointer lives at ROM address 0x4015 (offset
/// [`ROM_HDR_CVSD_SAMPLE_TABLE`] into the default bank page).  It points to
/// a list of 16-bit pointers, each referencing a 5-byte sample descriptor.
///
/// The firmware indexes this table with a 7-bit sample number from the voice
/// sequencer (function `start_cvsd_sample` in the decompiled firmware).
/// For offline extraction we simply iterate until we hit an invalid entry.
pub fn parse_cvsd_table(roms: &RomSet) -> Result<Vec<CvsdEntry>> {
    let u18_data = std::fs::read(&roms.u18)
        .with_context(|| format!("failed to read u18 ROM: {}", roms.u18.display()))?;

    let hdr = RomHeader::from_u18(&u18_data)?;
    let cvsd_table_file = hdr.to_file_offset(hdr.cvsd_sample_table);

    let mut entries = Vec::new();
    let mut counter = 0usize;

    loop {
        // Read the pointer to the current entry from the table.
        let entry_ptr_pos = cvsd_table_file + counter * 2;
        if entry_ptr_pos + 2 > u18_data.len() {
            break;
        }
        let entry_ptr = read_be_u16(&u18_data, entry_ptr_pos);
        if entry_ptr < 0x4000 {
            // Values below the banked window are end-of-table sentinels.
            break;
        }
        let entry_file = hdr.to_file_offset(entry_ptr);

        if entry_file + 5 > u18_data.len() {
            break;
        }

        let bank_selector = u18_data[entry_file];
        let cvsd_data_start = read_be_u16(&u18_data, entry_file + 1) as usize;
        let cvsd_data_end = read_be_u16(&u18_data, entry_file + 3) as usize;

        let (chip, bank) = match decode_bank_selector(bank_selector) {
            Some(v) => v,
            None => break, // end of table sentinel
        };

        let rom_size = match chip {
            RomChip::U14 => std::fs::metadata(&roms.u14)
                .with_context(|| format!("cannot stat u14: {}", roms.u14.display()))?
                .len() as usize,
            RomChip::U15 => std::fs::metadata(&roms.u15)
                .with_context(|| format!("cannot stat u15: {}", roms.u15.display()))?
                .len() as usize,
            RomChip::U18 => hdr.rom_len(),
        };

        // Convert bank number + 6809 address to a raw file offset.
        //
        // The WPC-89 maps 32 KB ROM pages into the 6809 address range
        // 0x4000–0xBFFF.  Each ROM chip supports up to 32 pages (512 KB).
        // The mapping uses active-low chip enables, so the highest-numbered
        // pages sit at the *end* of the ROM image file.
        //
        //   file_offset = bank * 0x8000          — page start within a 1 MB address space
        //               - (0x100000 - rom_size)  — adjust for ROMs smaller than 1 MB
        //               + data_start - 0x4000    — offset within the 32 KB page
        //
        // The intermediate result can be negative for small banks with small ROMs,
        // so we use i64 arithmetic to avoid usize underflow.
        let offset_i64 =
            (bank as i64) * 0x8000 + rom_size as i64 - 0x100000 + cvsd_data_start as i64 - 0x4000;

        if offset_i64 < 0 {
            counter += 1;
            continue;
        }
        let offset = offset_i64 as usize;
        let size = cvsd_data_end.saturating_sub(cvsd_data_start);

        if size == 0 {
            counter += 1;
            continue;
        }

        entries.push(CvsdEntry {
            chip,
            bank,
            offset,
            size,
            index: counter,
        });

        counter += 1;
    }

    if entries.is_empty() {
        bail!("no CVSD entries found; check that the ROM files are correct WPC89 sound ROMs");
    }

    Ok(entries)
}

/// Decode a CVSD entry to a vector of signed 8-bit PCM samples.
///
/// The FIRQ ISR in the firmware outputs CVSD bits **LSB-first**: the current
/// data byte is written directly (bit 0 → HC55536 data-in on rising clock),
/// then shifted right for subsequent bits (bit 1, 2, …, 7).  After 8 bits
/// a new byte is loaded.  We replicate this order here.
///
/// See `VEC_FIRQ_ISR` in the decompiled firmware (BL_U18.L1.c, lines 263-362).
pub fn decode_entry(entry: &CvsdEntry, roms: &RomSet) -> Result<Vec<i8>> {
    let rom_path = match entry.chip {
        RomChip::U14 => &roms.u14,
        RomChip::U15 => &roms.u15,
        RomChip::U18 => &roms.u18,
    };

    let rom_data = std::fs::read(rom_path)
        .with_context(|| format!("failed to read ROM: {}", rom_path.display()))?;

    let end = entry.offset + entry.size;
    if end > rom_data.len() {
        bail!(
            "CVSD entry {} at offset 0x{:x} size {} exceeds ROM size {}",
            entry.index,
            entry.offset,
            entry.size,
            rom_data.len()
        );
    }

    let cvsd_bytes = &rom_data[entry.offset..end];

    let mut chip = CvsdChip::new();
    let mut samples = Vec::with_capacity(cvsd_bytes.len() * 8);

    for &byte in cvsd_bytes {
        // LSB-first bit order within each byte
        for bit_pos in 0..8u8 {
            let bit = (byte >> bit_pos) & 1 != 0;
            chip.process_bit(bit);
            samples.push(chip.to_pcm_i8());
        }
    }

    Ok(samples)
}

/// Produce a human-readable chip name for a [`RomChip`].
pub fn chip_name(chip: RomChip) -> &'static str {
    match chip {
        RomChip::U14 => "u14",
        RomChip::U15 => "u15",
        RomChip::U18 => "u18",
    }
}

/// Read a 16-bit big-endian pointer from a ROM header table.
///
/// `header_offset` is one of the `ROM_HDR_*` constants.
/// Returns the raw 6809 address stored at that location.
pub fn read_rom_header_ptr(u18_data: &[u8], system_bank_file: usize, header_offset: usize) -> u16 {
    read_be_u16(u18_data, system_bank_file + header_offset)
}

/// Compute the file offset of the system bank (U18 page 0x1C) within a U18 ROM.
///
/// The system bank always occupies the last 0x20000 bytes of the U18 image.
pub fn system_bank_offset(u18_size: usize) -> Option<usize> {
    if u18_size >= 0x20000 {
        Some(u18_size - 0x20000)
    } else {
        None
    }
}
