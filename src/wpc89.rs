use std::path::{Path, PathBuf};

use anyhow::{Context, Result, bail};

use crate::cvsd_chip::CvsdChip;

/// The three ROM chips on the WPC89 sound board.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RomChip {
    U14,
    U15,
    U18,
}

/// A single decoded CVSD audio entry from the ROM table.
pub struct CvsdEntry {
    /// Which ROM chip the audio data lives in.
    pub chip: RomChip,
    /// ROM bank number (bits 0–4 of the bank selector byte).
    pub bank: u8,
    /// Byte offset in the ROM file.
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
fn decode_bank_selector(bank_selector: u8) -> Option<(RomChip, u8)> {
    let chip = match bank_selector & 0xe0 {
        0xc0 => RomChip::U14,
        0xa0 => RomChip::U15,
        0x60 => RomChip::U18,
        _ => return None,
    };
    let bank = bank_selector & 0x1f;
    Some((chip, bank))
}

/// Read a big-endian u16 from `data` at byte position `pos`.
fn read_be_u16(data: &[u8], pos: usize) -> u16 {
    u16::from_be_bytes([data[pos], data[pos + 1]])
}

/// Parse the CVSD entry table from the u18 ROM and return all entries.
pub fn parse_cvsd_table(roms: &RomSet) -> Result<Vec<CvsdEntry>> {
    let u18_data = std::fs::read(&roms.u18)
        .with_context(|| format!("failed to read u18 ROM: {}", roms.u18.display()))?;

    let u18_size = u18_data.len();
    // The system ROM occupies the last 0x20000 bytes of the u18 image.
    if u18_size < 0x20000 {
        bail!(
            "u18 ROM is too small ({} bytes); expected at least 0x20000",
            u18_size
        );
    }
    let u18_rom_offset = u18_size - 0x20000;

    // The CVSD table pointer is a 16-bit big-endian value stored at ROM-relative
    // address 0x4015, which maps to file offset `u18_rom_offset + 0x15`.
    let table_ptr_file = u18_rom_offset + 0x15;
    let cvsd_table_ptr_raw = read_be_u16(&u18_data, table_ptr_file) as usize;

    // Convert from 6809 address space (0x4000-based) to file offset.
    let cvsd_table_file = cvsd_table_ptr_raw - 0x4000 + u18_rom_offset;

    let mut entries = Vec::new();
    let mut counter = 0usize;

    loop {
        // Read the pointer to the current entry from the table.
        let entry_ptr_pos = cvsd_table_file + counter * 2;
        if entry_ptr_pos + 2 > u18_data.len() {
            break;
        }
        let entry_ptr_raw = read_be_u16(&u18_data, entry_ptr_pos) as usize;
        let entry_file = entry_ptr_raw
            .wrapping_sub(0x4000)
            .wrapping_add(u18_rom_offset);

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
            RomChip::U18 => u18_size,
        };

        // Convert bank+address to a raw file offset.
        // Formula from the Python extractor:
        //   offset = rom_bank * 0x8000 - (0x100000 - rom_size) + data_start - 0x4000
        //
        // The intermediate result can be negative for small banks with small ROMs,
        // so we use i64 arithmetic here to avoid usize underflow/overflow.
        let offset_i64 = (bank as i64) * 0x8000
            + rom_size as i64
            - 0x100000
            + cvsd_data_start as i64
            - 0x4000;

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
/// Bits within each byte are read LSB-first (little-endian bit order),
/// matching the Python `bitarray(endian='little')` behaviour.
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
