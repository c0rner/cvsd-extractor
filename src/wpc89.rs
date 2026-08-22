// SPDX-License-Identifier: BSD-3-Clause

use std::path::{Path, PathBuf};

use anyhow::{Context, Result as AnyResult, bail};
use thiserror::Error;

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

/// Errors produced while validating ROM bytes and address mappings.
#[derive(Debug, Error)]
pub enum RomError {
    #[error("U18 ROM is too small: got {actual:#x} bytes, need at least {minimum:#x}")]
    RomTooSmall { actual: usize, minimum: usize },
    #[error("truncated big-endian word at file offset {pos:#x} (ROM length {len:#x})")]
    TruncatedWord { pos: usize, len: usize },
    #[error("6809 address {addr:#06x} is outside the ROM windows 0x4000..=0xffff")]
    InvalidAddress { addr: u16 },
    #[error(
        "6809 address {addr:#06x} maps outside the ROM (offset {offset:#x}, length {rom_len:#x})"
    )]
    AddressOutOfRange {
        addr: u16,
        offset: usize,
        rom_len: usize,
    },
    #[error("invalid bank selector {selector:#04x}")]
    InvalidBankSelector { selector: u8 },
    #[error("bank selector {selector:#04x} selects {chip:?}, but only U18 is available")]
    UnavailableChip { selector: u8, chip: RomChip },
    #[error("bank selector {selector:#04x} selects a page outside the {rom_len:#x}-byte ROM")]
    InvalidBankPage { selector: u8, rom_len: usize },
    #[error("arithmetic overflow while computing {context}")]
    ArithmeticOverflow { context: &'static str },
    #[error("failed to read {chip} ROM '{path}': {source}")]
    ReadRom {
        chip: &'static str,
        path: PathBuf,
        #[source]
        source: std::io::Error,
    },
    #[error("CVSD table pointer {pointer:#06x} is invalid: {source}")]
    InvalidCvsdTablePointer {
        pointer: u16,
        #[source]
        source: Box<RomError>,
    },
    #[error("CVSD descriptor {index} at {addr:#06x} is truncated")]
    TruncatedCvsdDescriptor { index: usize, addr: u16 },
    #[error("CVSD descriptor {index} has invalid data range {start:#06x}..{end:#06x}")]
    InvalidCvsdRange { index: usize, start: u16, end: u16 },
    #[error(
        "CVSD descriptor {index} maps outside its ROM (offset {offset:#x}, size {size:#x}, ROM length {rom_len:#x})"
    )]
    CvsdRangeOutOfBounds {
        index: usize,
        offset: usize,
        size: usize,
        rom_len: usize,
    },
    #[error("CVSD table contains no entries; check that the provided ROMs are WPC-89 sound ROMs")]
    NoCvsdEntries,
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
#[derive(Debug, Clone)]
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
pub fn read_be_u16(data: &[u8], pos: usize) -> std::result::Result<u16, RomError> {
    let bytes = data
        .get(
            pos..pos.checked_add(2).ok_or(RomError::ArithmeticOverflow {
                context: "big-endian word range",
            })?,
        )
        .ok_or(RomError::TruncatedWord {
            pos,
            len: data.len(),
        })?;
    Ok(u16::from_be_bytes([bytes[0], bytes[1]]))
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
    rom_len: usize,
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
    pub fn from_u18(u18_data: &[u8]) -> std::result::Result<Self, RomError> {
        let sbf = system_bank_offset(u18_data.len()).ok_or(RomError::RomTooSmall {
            actual: u18_data.len(),
            minimum: 0x20000,
        })?;
        let byte = |offset: usize| {
            u18_data
                .get(
                    sbf.checked_add(offset)
                        .ok_or(RomError::ArithmeticOverflow {
                            context: "ROM header byte offset",
                        })?,
                )
                .copied()
                .ok_or(RomError::AddressOutOfRange {
                    addr: 0x4000u16.saturating_add(offset as u16),
                    offset: sbf.saturating_add(offset),
                    rom_len: u18_data.len(),
                })
        };
        let word = |offset: usize| {
            let pos = sbf
                .checked_add(offset)
                .ok_or(RomError::ArithmeticOverflow {
                    context: "ROM header word offset",
                })?;
            read_be_u16(u18_data, pos)
        };

        Ok(Self {
            system_bank_file: sbf,
            rom_len: u18_data.len(),
            fm_patch_table: word(ROM_HDR_FM_PATCH_TABLE)?,
            dac_sample_table: word(ROM_HDR_DAC_SAMPLE_TABLE)?,
            fm_program_table: word(ROM_HDR_FM_PROGRAM_TABLE)?,
            voice_type_table: word(ROM_HDR_VOICE_TYPE_TABLE)?,
            max_cmd_index: byte(ROM_HDR_MAX_CMD_INDEX)?,
            cmd_dispatch_table: word(ROM_HDR_CMD_DISPATCH_TABLE)?,
            sound_program_table: word(ROM_HDR_SOUND_PROGRAM_TABLE)?,
            cvsd_sample_table: word(ROM_HDR_CVSD_SAMPLE_TABLE)?,
        })
    }

    /// Convert a 6809 ROM address to a U18 file offset.
    ///
    /// Handles both the banked window (0x4000–0xBFFF, mapped to the system
    /// bank page) and the fixed bank (0xC000–0xFFFF, always the last 16 KB
    /// of the ROM image).
    pub fn to_file_offset(&self, addr: u16) -> std::result::Result<usize, RomError> {
        let offset = if addr >= 0xC000 {
            // Fixed bank: last 16 KB of ROM, always accessible.
            self.rom_len - 0x4000 + (addr as usize - 0xC000)
        } else if addr >= 0x4000 {
            // Banked window: 0x4000–0xBFFF mapped to system bank.
            (addr as usize) - 0x4000 + self.system_bank_file
        } else {
            return Err(RomError::InvalidAddress { addr });
        };
        if offset >= self.rom_len {
            return Err(RomError::AddressOutOfRange {
                addr,
                offset,
                rom_len: self.rom_len,
            });
        }
        Ok(offset)
    }

    /// Convert an address using a raw bank-register selector.
    /// Fixed-bank addresses ignore the selector, matching the hardware.
    pub fn to_file_offset_in_bank(
        &self,
        selector: u8,
        addr: u16,
    ) -> std::result::Result<usize, RomError> {
        if addr >= 0xC000 {
            return self.to_file_offset(addr);
        }
        if addr < 0x4000 {
            return Err(RomError::InvalidAddress { addr });
        }
        let (chip, bank) =
            decode_bank_selector(selector).ok_or(RomError::InvalidBankSelector { selector })?;
        if chip != RomChip::U18 {
            return Err(RomError::UnavailableChip { selector, chip });
        }
        map_banked_address(selector, bank, addr, self.rom_len)
    }

    /// Total ROM size in bytes.
    pub fn rom_len(&self) -> usize {
        self.rom_len
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
pub fn parse_cvsd_table(roms: &RomSet) -> std::result::Result<Vec<CvsdEntry>, RomError> {
    let u18_data = read_rom(&roms.u18, "u18")?;

    let hdr = RomHeader::from_u18(&u18_data)?;
    let cvsd_table_file = hdr
        .to_file_offset(hdr.cvsd_sample_table)
        .map_err(|source| RomError::InvalidCvsdTablePointer {
            pointer: hdr.cvsd_sample_table,
            source: Box::new(source),
        })?;

    let u14_len = std::fs::metadata(&roms.u14)
        .map_err(|source| RomError::ReadRom {
            chip: "u14",
            path: roms.u14.clone(),
            source,
        })?
        .len() as usize;
    let u15_len = std::fs::metadata(&roms.u15)
        .map_err(|source| RomError::ReadRom {
            chip: "u15",
            path: roms.u15.clone(),
            source,
        })?
        .len() as usize;

    let mut entries = Vec::new();
    let mut counter = 0usize;

    loop {
        // Read the pointer to the current entry from the table.
        let entry_ptr_pos = counter
            .checked_mul(2)
            .and_then(|n| cvsd_table_file.checked_add(n))
            .ok_or(RomError::ArithmeticOverflow {
                context: "CVSD table entry offset",
            })?;
        let entry_ptr = read_be_u16(&u18_data, entry_ptr_pos)?;
        if entry_ptr < 0x4000 {
            // Values below the banked window are end-of-table sentinels.
            break;
        }
        let entry_file = hdr.to_file_offset(entry_ptr)?;
        let descriptor_end = entry_file
            .checked_add(5)
            .ok_or(RomError::ArithmeticOverflow {
                context: "CVSD descriptor range",
            })?;
        let descriptor =
            u18_data
                .get(entry_file..descriptor_end)
                .ok_or(RomError::TruncatedCvsdDescriptor {
                    index: counter,
                    addr: entry_ptr,
                })?;

        let bank_selector = descriptor[0];
        let cvsd_data_start = u16::from_be_bytes([descriptor[1], descriptor[2]]);
        let cvsd_data_end = u16::from_be_bytes([descriptor[3], descriptor[4]]);

        let (chip, bank) =
            decode_bank_selector(bank_selector).ok_or(RomError::InvalidBankSelector {
                selector: bank_selector,
            })?;

        let rom_size = match chip {
            RomChip::U14 => u14_len,
            RomChip::U15 => u15_len,
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
        if !(0x4000..=0xBFFF).contains(&cvsd_data_start)
            || cvsd_data_end <= cvsd_data_start
            || cvsd_data_end > 0xC000
        {
            return Err(RomError::InvalidCvsdRange {
                index: counter,
                start: cvsd_data_start,
                end: cvsd_data_end,
            });
        }
        let offset = map_banked_address(bank_selector, bank, cvsd_data_start, rom_size)?;
        let size = usize::from(cvsd_data_end - cvsd_data_start);
        let end = offset
            .checked_add(size)
            .ok_or(RomError::ArithmeticOverflow {
                context: "CVSD data range",
            })?;
        if end > rom_size {
            return Err(RomError::CvsdRangeOutOfBounds {
                index: counter,
                offset,
                size,
                rom_len: rom_size,
            });
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
        return Err(RomError::NoCvsdEntries);
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
pub fn decode_entry(entry: &CvsdEntry, roms: &RomSet) -> AnyResult<Vec<i8>> {
    let rom_path = match entry.chip {
        RomChip::U14 => &roms.u14,
        RomChip::U15 => &roms.u15,
        RomChip::U18 => &roms.u18,
    };

    let rom_data = std::fs::read(rom_path)
        .with_context(|| format!("failed to read ROM: {}", rom_path.display()))?;

    let end = entry
        .offset
        .checked_add(entry.size)
        .context("CVSD entry offset + size overflowed")?;
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
pub fn read_rom_header_ptr(
    u18_data: &[u8],
    system_bank_file: usize,
    header_offset: usize,
) -> std::result::Result<u16, RomError> {
    let pos = system_bank_file
        .checked_add(header_offset)
        .ok_or(RomError::ArithmeticOverflow {
            context: "ROM header pointer offset",
        })?;
    read_be_u16(u18_data, pos)
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

fn read_rom(path: &Path, chip: &'static str) -> std::result::Result<Vec<u8>, RomError> {
    std::fs::read(path).map_err(|source| RomError::ReadRom {
        chip,
        path: path.to_path_buf(),
        source,
    })
}

fn map_banked_address(
    selector: u8,
    bank: u8,
    addr: u16,
    rom_len: usize,
) -> std::result::Result<usize, RomError> {
    if !(0x4000..=0xBFFF).contains(&addr) {
        return Err(RomError::InvalidAddress { addr });
    }
    let page = usize::from(bank)
        .checked_mul(0x8000)
        .ok_or(RomError::ArithmeticOverflow {
            context: "bank page offset",
        })?;
    let image_bias = 0x100000usize
        .checked_sub(rom_len)
        .ok_or(RomError::InvalidBankPage { selector, rom_len })?;
    let page_start = page
        .checked_sub(image_bias)
        .ok_or(RomError::InvalidBankPage { selector, rom_len })?;
    let offset =
        page_start
            .checked_add(usize::from(addr - 0x4000))
            .ok_or(RomError::ArithmeticOverflow {
                context: "banked ROM address",
            })?;
    if offset >= rom_len {
        return Err(RomError::AddressOutOfRange {
            addr,
            offset,
            rom_len,
        });
    }
    Ok(offset)
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::sync::atomic::{AtomicU64, Ordering};

    static NEXT_TEMP: AtomicU64 = AtomicU64::new(0);

    fn set_word(data: &mut [u8], pos: usize, value: u16) {
        data[pos..pos + 2].copy_from_slice(&value.to_be_bytes());
    }

    fn header_fixture() -> Vec<u8> {
        let mut rom = vec![0; 0x20000];
        for (offset, address) in [
            (ROM_HDR_FM_PATCH_TABLE, 0x4100),
            (ROM_HDR_DAC_SAMPLE_TABLE, 0x4100),
            (ROM_HDR_FM_PROGRAM_TABLE, 0x4100),
            (ROM_HDR_VOICE_TYPE_TABLE, 0x4100),
            (ROM_HDR_CMD_DISPATCH_TABLE, 0x4200),
            (ROM_HDR_SOUND_PROGRAM_TABLE, 0x4300),
            (ROM_HDR_CVSD_SAMPLE_TABLE, 0x4400),
        ] {
            set_word(&mut rom, offset, address);
        }
        rom
    }

    #[test]
    fn checked_word_rejects_truncation_and_overflow() {
        assert_eq!(read_be_u16(&[0x12, 0x34], 0).unwrap(), 0x1234);
        assert!(matches!(
            read_be_u16(&[0x12], 0),
            Err(RomError::TruncatedWord { .. })
        ));
        assert!(matches!(
            read_be_u16(&[], usize::MAX),
            Err(RomError::ArithmeticOverflow { .. })
        ));
    }

    #[test]
    fn checked_address_translation_covers_window_boundaries() {
        let rom = header_fixture();
        let header = RomHeader::from_u18(&rom).unwrap();

        assert_eq!(header.to_file_offset(0x4000).unwrap(), 0);
        assert_eq!(header.to_file_offset(0xBFFF).unwrap(), 0x7FFF);
        assert_eq!(header.to_file_offset(0xC000).unwrap(), 0x1C000);
        assert_eq!(header.to_file_offset(0xFFFF).unwrap(), 0x1FFFF);
        assert!(matches!(
            header.to_file_offset(0x3FFF),
            Err(RomError::InvalidAddress { .. })
        ));
    }

    #[test]
    fn bank_translation_validates_selector_chip_and_page() {
        let rom = header_fixture();
        let header = RomHeader::from_u18(&rom).unwrap();

        assert_eq!(
            header.to_file_offset_in_bank(SYSTEM_BANK, 0x4000).unwrap(),
            0
        );
        assert_eq!(header.to_file_offset_in_bank(0, 0xC000).unwrap(), 0x1C000);
        assert!(matches!(
            header.to_file_offset_in_bank(0x7B, 0x4000),
            Err(RomError::InvalidBankPage { .. })
        ));
        assert!(matches!(
            header.to_file_offset_in_bank(0xA0, 0x4000),
            Err(RomError::UnavailableChip { .. })
        ));
        assert!(matches!(
            header.to_file_offset_in_bank(0x00, 0x4000),
            Err(RomError::InvalidBankSelector { .. })
        ));
    }

    #[test]
    fn public_cvsd_entry_range_overflow_is_rejected() {
        let temp = std::env::temp_dir().join(format!(
            "cvsd-extractor-test-{}-{}",
            std::process::id(),
            NEXT_TEMP.fetch_add(1, Ordering::Relaxed)
        ));
        std::fs::create_dir(&temp).unwrap();
        let path = temp.join("rom.bin");
        std::fs::write(&path, [0]).unwrap();
        let roms = RomSet::new(&path, &path, &path);
        let entry = CvsdEntry {
            chip: RomChip::U18,
            bank: 0,
            offset: usize::MAX,
            size: 2,
            index: 3,
        };

        let error = decode_entry(&entry, &roms).unwrap_err();
        assert!(error.to_string().contains("overflowed"));
        std::fs::remove_dir_all(temp).unwrap();
    }

    #[test]
    fn cvsd_pointer_sentinel_terminates_after_valid_entry() {
        let mut rom = header_fixture();
        set_word(&mut rom, 0x400, 0x4500);
        set_word(&mut rom, 0x402, 0x0000);
        rom[0x500..0x505].copy_from_slice(&[SYSTEM_BANK, 0x40, 0x00, 0x40, 0x01]);
        let temp = std::env::temp_dir().join(format!(
            "cvsd-extractor-test-{}-{}",
            std::process::id(),
            NEXT_TEMP.fetch_add(1, Ordering::Relaxed)
        ));
        std::fs::create_dir(&temp).unwrap();
        let u14 = temp.join("u14.bin");
        let u15 = temp.join("u15.bin");
        let u18 = temp.join("u18.bin");
        std::fs::write(&u14, &rom).unwrap();
        std::fs::write(&u15, &rom).unwrap();
        std::fs::write(&u18, &rom).unwrap();

        let entries = parse_cvsd_table(&RomSet::new(&u14, &u15, &u18)).unwrap();
        assert_eq!(entries.len(), 1);
        assert_eq!(entries[0].offset, 0);
        assert_eq!(entries[0].size, 1);
        std::fs::remove_dir_all(temp).unwrap();
    }

    #[test]
    fn invalid_cvsd_table_pointer_preserves_mapping_error() {
        let mut rom = header_fixture();
        set_word(&mut rom, ROM_HDR_CVSD_SAMPLE_TABLE, 0x3FFF);
        let header = RomHeader::from_u18(&rom).unwrap();
        let error = header.to_file_offset(header.cvsd_sample_table).unwrap_err();

        let wrapped = RomError::InvalidCvsdTablePointer {
            pointer: header.cvsd_sample_table,
            source: Box::new(error),
        };
        assert!(wrapped.to_string().contains("outside the ROM windows"));
        assert!(std::error::Error::source(&wrapped).is_some());
    }
}
