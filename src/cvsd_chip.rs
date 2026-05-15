// SPDX-License-Identifier: BSD-3-Clause

// Chip-logic simulation of the Harris HC55536 CVSD decoder.
//
// Implements `process_bit_HC555XX` from the PinMAME reference (hc55516.c),
// using the 10- and 12-bit fixed-point registers revealed by the 2020 decap
// analysis of the physical chip.  Unlike the floating-point version, this
// implementation has the chip's inherent 10-bit integrator clipping, so no
// loudness compression is required.
//
// Parameters are for the HC55516/HC55536/HC55564 family (from hc55516.c lines
// 1300–1316).  WPC89 sound boards use the HC55536 variant.

// HC55536 syllabic filter parameters (decap-derived, hc55516.c lines 1310-1316)
const CHARGE_MASK: i32 = 0xFC0;
const CHARGE_SHIFT: u32 = 6;
const CHARGE_ADD: i32 = 0xFC1;
const DECAY_SHIFT: u32 = 4;
const SHIFTMASK: u8 = 0x07;
/// Syllabic filter register initial value (pre-charged at power-on).
const SYL_REG_INIT: i32 = 0x3F;

/// Clip a value to the 10-bit signed range (-512..=511).
#[inline]
fn clip10bits(v: i32) -> i32 {
    v.clamp(-512, 511)
}

/// Sign-extend a 10-bit value into a full i32.
#[inline]
fn signext10bits(v: i32) -> i32 {
    let v = v & 0x3FF;
    if v & 0x200 != 0 { v | !0x3FF } else { v }
}

pub struct CvsdChip {
    /// 12-bit syllabic filter register (0..=0xFFF).
    syl_reg: i32,
    /// 10-bit signed integrator register (-512..=511).
    integrator: i32,
    /// 3-bit shift register holding the last 3 bits.
    shiftreg: u8,
    /// PCM sample captured at the correct pipeline stage (after decay,
    /// before syl-charge), ready to be returned by `to_pcm_i8`.
    last_sample: i8,
}

impl CvsdChip {
    pub fn new() -> Self {
        CvsdChip {
            syl_reg: SYL_REG_INIT,
            integrator: 0,
            shiftreg: 0,
            last_sample: 0,
        }
    }

    /// Process a single CVSD bit through the chip pipeline.
    ///
    /// Pipeline order (matches `process_bit_HC555XX` in hc55516.c):
    /// 1. Shift bit into shiftreg
    /// 2. Decay syllabic filter toward max (common step)
    /// 3. Boost syllabic filter on non-coincidence
    /// 4. Decay integrator
    /// 5. **Capture sample here** (this is the output point on the real chip)
    /// 6. Charge integrator from syllabic filter (sign from current bit)
    pub fn process_bit(&mut self, bit: bool) {
        // 1. Shift bit into the 3-bit shift register.
        self.shiftreg = ((self.shiftreg << 1) | (bit as u8)) & SHIFTMASK;

        // 2. Syllabic filter decay: move toward 0xFFF (maximum charge).
        //    Floating-point equivalent: syl *= 63/64
        self.syl_reg += (!self.syl_reg & CHARGE_MASK) >> CHARGE_SHIFT;

        // 3. Extra charge on non-coincidence (last 3 bits not all identical).
        if self.shiftreg != 0 && self.shiftreg != SHIFTMASK {
            self.syl_reg += CHARGE_ADD;
        }

        // Keep syllabic filter in 12-bit range.
        self.syl_reg &= 0xFFF;

        // 4. Integrator decay: multiply by (2^DECAY_SHIFT - 1) / 2^DECAY_SHIFT.
        //    Floating-point equivalent: integrator *= 15/16
        let decay_sum = signext10bits(((!self.integrator >> DECAY_SHIFT) + 1) & 0x3FF);
        self.integrator = clip10bits(self.integrator + decay_sum);

        // 5. Capture the sample at this pipeline stage.
        //    Scale 10-bit signed (-512..511) → 16-bit signed (-32768..32767)
        //    using the same formula as the C reference.
        let sample16 = (self.integrator << 6) | (((self.integrator & 0x3FF) ^ 0x200) >> 4);
        // Take the top 8 bits of the 16-bit value → i8 range, no compression needed.
        self.last_sample = (sample16 >> 8) as i8;

        // 6. Charge integrator from syllabic filter; sign determined by current bit.
        let mut charge = self.syl_reg >> 6;
        if charge < 2 {
            charge = 2;
        }
        if self.shiftreg & 1 != 0 {
            charge = -charge;
        }
        self.integrator = clip10bits(self.integrator + charge);
    }

    /// Return the signed 8-bit PCM sample for the last processed bit.
    ///
    /// Pass this directly to `hound::WavWriter::write_sample`; hound XORs
    /// with 0x80 to produce the unsigned 0–255 range the WAV format requires.
    pub fn to_pcm_i8(&self) -> i8 {
        self.last_sample
    }
}

impl Default for CvsdChip {
    fn default() -> Self {
        Self::new()
    }
}
