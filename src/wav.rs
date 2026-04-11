use std::path::Path;

use anyhow::{Context, Result};
use audioadapter_buffers::direct::InterleavedSlice;
use hound::{SampleFormat, WavSpec, WavWriter};
use rubato::{
    Async, FixedAsync, Indexing, Resampler, SincInterpolationParameters, SincInterpolationType,
    WindowFunction, calculate_cutoff,
};

/// Sample rate used by the WPC89 sound board.
pub const SAMPLE_RATE_HZ: u32 = 22372;

/// Write a slice of signed 8-bit PCM samples as a WAV file.
///
/// If `output_rate` differs from `SAMPLE_RATE_HZ`, the audio is resampled
/// before writing. The output is mono, 8-bit at `output_rate` Hz. `hound`
/// converts i8 to unsigned bytes by XOR with 0x80, so 0i8 (silence) maps
/// correctly to WAV value 0x80.
pub fn write_wav(path: impl AsRef<Path>, samples: &[i8], output_rate: u32) -> Result<()> {
    let resampled;
    let samples = if output_rate != SAMPLE_RATE_HZ {
        resampled = resample(samples, SAMPLE_RATE_HZ, output_rate)
            .context("failed to resample audio")?;
        &resampled[..]
    } else {
        samples
    };

    let spec = WavSpec {
        channels: 1,
        sample_rate: output_rate,
        bits_per_sample: 8,
        sample_format: SampleFormat::Int,
    };

    let path = path.as_ref();
    let mut writer = WavWriter::create(path, spec)
        .with_context(|| format!("failed to create WAV file: {}", path.display()))?;

    for &sample in samples {
        writer
            .write_sample(sample)
            .with_context(|| format!("failed to write sample to: {}", path.display()))?;
    }

    writer
        .finalize()
        .with_context(|| format!("failed to finalize WAV file: {}", path.display()))?;

    Ok(())
}

/// Resample signed 8-bit PCM audio from `from_rate` to `to_rate` Hz using a
/// sinc interpolation filter (anti-aliased, high quality).
fn resample(samples: &[i8], from_rate: u32, to_rate: u32) -> Result<Vec<i8>> {
    let ratio = to_rate as f64 / from_rate as f64;
    let nbr_input_frames = samples.len();

    let sinc_len = 128;
    let window = WindowFunction::Blackman2;
    let f_cutoff = calculate_cutoff(sinc_len, window);
    let params = SincInterpolationParameters {
        sinc_len,
        f_cutoff,
        interpolation: SincInterpolationType::Linear,
        oversampling_factor: 256,
        window,
    };

    let mut resampler =
        Async::<f32>::new_sinc(ratio, 2.0, &params, 1024, 1, FixedAsync::Input)
            .map_err(|e| anyhow::anyhow!("failed to create resampler: {e}"))?;

    let float_in: Vec<f32> = samples.iter().map(|&s| s as f32 / 128.0).collect();

    let output_needed = resampler.process_all_needed_output_len(nbr_input_frames);
    let mut float_out = vec![0.0f32; output_needed];

    let input_adapter = InterleavedSlice::new(&float_in, 1, nbr_input_frames)
        .map_err(|e| anyhow::anyhow!("input adapter error: {e}"))?;
    let mut output_adapter = InterleavedSlice::new_mut(&mut float_out, 1, output_needed)
        .map_err(|e| anyhow::anyhow!("output adapter error: {e}"))?;

    let resampler_delay = resampler.output_delay();

    let mut indexing = Indexing {
        input_offset: 0,
        output_offset: 0,
        active_channels_mask: None,
        partial_len: None,
    };

    let mut input_frames_left = nbr_input_frames;
    let mut input_frames_next = resampler.input_frames_next();

    while input_frames_left >= input_frames_next {
        let (nbr_in, nbr_out) = resampler
            .process_into_buffer(&input_adapter, &mut output_adapter, Some(&indexing))
            .map_err(|e| anyhow::anyhow!("resampler error: {e}"))?;
        indexing.input_offset += nbr_in;
        indexing.output_offset += nbr_out;
        input_frames_left -= nbr_in;
        input_frames_next = resampler.input_frames_next();
    }

    if input_frames_left > 0 {
        indexing.partial_len = Some(input_frames_left);
        let (_nbr_in, nbr_out) = resampler
            .process_into_buffer(&input_adapter, &mut output_adapter, Some(&indexing))
            .map_err(|e| anyhow::anyhow!("resampler tail error: {e}"))?;
        indexing.output_offset += nbr_out;
    }

    // The first `resampler_delay` output frames compensate for the filter's group
    // delay and must be skipped. The expected number of output frames follows
    // from the ratio.
    let nbr_output_frames = (nbr_input_frames as f64 * ratio).ceil() as usize;
    let start = resampler_delay;
    let end = (start + nbr_output_frames).min(float_out.len());

    Ok(float_out[start..end]
        .iter()
        .map(|&v| (v * 127.0).clamp(-128.0, 127.0) as i8)
        .collect())
}
