use argh::FromArgs;
use std::path::PathBuf;

use anyhow::Result;
use pinball_cvsd_patcher::{extract_all, wav::SAMPLE_RATE_HZ, RomSet};

/// Extract CVSD audio samples from WPC89 pinball sound ROMs.
#[derive(FromArgs)]
struct Args {
    /// path to the U14 sound ROM
    #[argh(option)]
    u14: PathBuf,

    /// path to the U15 sound ROM
    #[argh(option)]
    u15: PathBuf,

    /// path to the U18 sound ROM (contains the CVSD table)
    #[argh(option)]
    u18: PathBuf,

    /// output directory for extracted WAV files (default: current directory)
    #[argh(option, default = "PathBuf::from(\".\")")]
    output: PathBuf,

    /// output sample rate in Hz; audio is resampled if different from the
    /// native WPC89 rate of 22372 Hz (default: 22372)
    #[argh(option, default = "SAMPLE_RATE_HZ")]
    output_rate: u32,
}

fn main() -> Result<()> {
    tracing_subscriber::fmt()
        .with_env_filter(
            tracing_subscriber::EnvFilter::try_from_default_env()
                .unwrap_or_else(|_| tracing_subscriber::EnvFilter::new("info")),
        )
        .init();

    let args: Args = argh::from_env();
    let roms = RomSet::new(&args.u14, &args.u15, &args.u18);
    let count = extract_all(&roms, &args.output, args.output_rate)?;
    println!("Extracted {} audio samples to '{}'", count, args.output.display());
    Ok(())
}

