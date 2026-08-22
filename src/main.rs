// SPDX-License-Identifier: BSD-3-Clause

use argh::FromArgs;
use std::io::{self, Write};
use std::path::PathBuf;

use anyhow::Result;
use cvsd_extractor::{RomSet, extract_all, sound_program, wav::SAMPLE_RATE_HZ};

/// WPC-89 pinball sound ROM tool — extract CVSD audio and decode sound programs.
#[derive(FromArgs)]
struct Args {
    #[argh(subcommand)]
    command: Command,
}

#[derive(FromArgs)]
#[argh(subcommand)]
enum Command {
    Extract(ExtractArgs),
    Programs(ProgramsArgs),
}

/// Extract CVSD audio samples as WAV files.
#[derive(FromArgs)]
#[argh(subcommand, name = "extract")]
struct ExtractArgs {
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

/// Decode and display sound programs from a U18 ROM.
#[derive(FromArgs)]
#[argh(subcommand, name = "programs")]
struct ProgramsArgs {
    /// path to the U18 sound ROM
    #[argh(option)]
    u18: PathBuf,
}

fn handle_program_report(
    result: std::result::Result<(), sound_program::ProgramReportError>,
) -> std::result::Result<(), sound_program::ProgramReportError> {
    match result {
        Err(sound_program::ProgramReportError::Write(error))
            if error.kind() == io::ErrorKind::BrokenPipe =>
        {
            Ok(())
        }
        result => result,
    }
}

fn run(args: Args, out: &mut impl Write) -> Result<()> {
    match args.command {
        Command::Extract(ea) => {
            let roms = RomSet::new(&ea.u14, &ea.u15, &ea.u18);
            let count = extract_all(&roms, &ea.output, ea.output_rate)?;
            writeln!(
                out,
                "Extracted {} audio samples to '{}'",
                count,
                ea.output.display()
            )?;
        }
        Command::Programs(pa) => {
            handle_program_report(sound_program::write_program_summary(&pa.u18, out))?;
        }
    }

    Ok(())
}

fn main() -> Result<()> {
    tracing_subscriber::fmt()
        .with_env_filter(
            tracing_subscriber::EnvFilter::try_from_default_env()
                .unwrap_or_else(|_| tracing_subscriber::EnvFilter::new("info")),
        )
        .init();

    let args: Args = argh::from_env();
    let stdout = io::stdout();
    run(args, &mut stdout.lock())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn broken_pipe_during_program_output_is_success() {
        let result = handle_program_report(Err(sound_program::ProgramReportError::Write(
            io::Error::new(io::ErrorKind::BrokenPipe, "consumer exited"),
        )));

        assert!(result.is_ok());
    }

    #[test]
    fn other_program_output_errors_propagate() {
        let result = handle_program_report(Err(sound_program::ProgramReportError::Write(
            io::Error::other("output failed"),
        )));

        assert!(matches!(
            result,
            Err(sound_program::ProgramReportError::Write(error))
                if error.kind() == io::ErrorKind::Other
        ));
    }
}
