// SPDX-License-Identifier: BSD-3-Clause

pub mod cvsd_chip;
pub mod sound_program;
pub mod wav;
pub mod wpc89;

use std::path::Path;

use anyhow::Result;
use tracing::{debug, info};

pub use wpc89::{CvsdEntry, RomChip, RomSet};

/// Extract all CVSD audio entries from a WPC89 ROM set and write WAV files
/// to `output_dir`.
///
/// Each WAV file is named `NNN_<chip>_<offset>_<size>.wav`
///
/// `output_rate` sets the sample rate of the written WAV files. If it differs
/// from the native WPC89 rate (22372 Hz) the audio is resampled automatically.
///
/// Returns the number of WAV files written.
pub fn extract_all(roms: &RomSet, output_dir: impl AsRef<Path>, output_rate: u32) -> Result<usize> {
    let output_dir = output_dir.as_ref();
    std::fs::create_dir_all(output_dir)?;

    let entries = wpc89::parse_cvsd_table(roms)?;

    debug!("Found {} CVSD table entries", entries.len());
    for entry in &entries {
        debug!(
            index = entry.index,
            chip = wpc89::chip_name(entry.chip),
            bank = entry.bank,
            offset = entry.offset,
            size = entry.size,
            "CVSD entry"
        );
    }

    let mut written = 0usize;
    for entry in &entries {
        let samples = wpc89::decode_entry(entry, roms)?;
        let filename = format!(
            "{:03}_{}_{}_{}.wav",
            entry.index,
            wpc89::chip_name(entry.chip),
            entry.offset,
            entry.size,
        );
        let out_path = output_dir.join(&filename);
        wav::write_wav(&out_path, &samples, output_rate)?;
        info!(
            file = %filename,
            samples = samples.len(),
            output_rate,
            "wrote WAV"
        );
        written += 1;
    }

    Ok(written)
}
