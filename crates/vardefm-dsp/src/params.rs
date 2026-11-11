/// Waveform parameters for one VardeFM mode.
#[derive(Clone, Debug)]
pub struct OfdmParams {
    pub name: &'static str,
    /// Audio sample rate in Hz.
    pub fs: f32,
    pub nfft: usize,
    /// Cyclic prefix length in samples.
    pub ncp: usize,
    /// FFT bin of the lowest carrier.
    pub first_bin: usize,
    pub num_carriers: usize,
    /// Every n-th carrier is a pilot (the last carrier is always a pilot too).
    pub pilot_spacing: usize,
    /// Spectral tilt applied at TX, in dB per octave above the lowest carrier.
    /// Negative values pre-compensate for the radio's pre-emphasis. The receiver's
    /// matched filter expects preambles with this tilt, so both ends must agree on it:
    /// it is part of the mode, not a per-station setting.
    pub tilt_db_per_octave: f32,
    /// Reference output RMS level (1.0 = full scale; peaks are clipped at full scale).
    /// Used when the caller does not choose a level, e.g. the `sim` and `tx-wav` tools.
    pub tx_rms: f32,
    /// Output RMS level per speed level (index = level, see vardefm-modem). Robust
    /// levels go out louder: they gain more from the extra deviation than they lose to
    /// clipping. Header-only bursts use level 0.
    pub level_tx_rms: [f32; 7],
    /// Highest SNR (dB) the receiver reports on a burst at each level when the channel
    /// is clean: the floor set by clipping at that level's drive (and the radio).
    /// With `level_tx_rms` it lets the link predict another level's SNR from a burst's.
    pub level_snr_ceiling_db: [f32; 7],
    /// Low-pass the received audio at this frequency before the Schmidl-Cox search for
    /// preamble A (the matched filter does not use it).
    /// On the flat data port, FM noise rises with frequency, so the lower part of the
    /// band gives a much cleaner detection metric.
    pub detect_lowpass_hz: Option<f32>,
}

/// Per-level drive, measured in the FM channel simulator (`level_table` benchmark).
const NARROW_LEVEL_RMS: [f32; 7] = [0.30, 0.30, 0.30, 0.30, 0.30, 0.25, 0.25];
const WIDE_LEVEL_RMS: [f32; 7] = [0.50, 0.50, 0.50, 0.40, 0.40, 0.30, 0.30];
/// SNR ceilings per level, from the same benchmark at 40 dB carrier-to-noise.
const NARROW_SNR_CEILING_DB: [f32; 7] = [22.0, 22.0, 22.0, 22.0, 22.0, 26.5, 26.5];
const WIDE_SNR_CEILING_DB: [f32; 7] = [17.5, 17.5, 17.5, 23.5, 23.5, 29.0, 29.0];

impl OfdmParams {
    /// Mic/speaker audio through any FM radio: about 420-2670 Hz.
    pub fn narrow() -> Self {
        Self {
            name: "narrow",
            fs: 48_000.0,
            nfft: 1024,
            ncp: 128,
            first_bin: 9,
            num_carriers: 49,
            pilot_spacing: 6,
            tilt_db_per_octave: 0.0,
            tx_rms: 0.25,
            level_tx_rms: NARROW_LEVEL_RMS,
            level_snr_ceiling_db: NARROW_SNR_CEILING_DB,
            detect_lowpass_hz: None,
        }
    }

    /// Flat audio through the radio's 9600 data port: about 280-6000 Hz.
    pub fn wide() -> Self {
        Self {
            name: "wide",
            fs: 48_000.0,
            nfft: 512,
            ncp: 64,
            first_bin: 3,
            num_carriers: 62,
            pilot_spacing: 6,
            // Software pre-emphasis: the data port has none, and FM noise rises with frequency.
            tilt_db_per_octave: 3.0,
            tx_rms: 0.3,
            level_tx_rms: WIDE_LEVEL_RMS,
            level_snr_ceiling_db: WIDE_SNR_CEILING_DB,
            detect_lowpass_hz: Some(2500.0),
        }
    }

    /// Every mode. Tests that calibrate or check something per mode loop over this.
    pub fn all() -> [Self; 2] {
        [Self::narrow(), Self::wide()]
    }

    pub fn by_name(name: &str) -> Option<Self> {
        Self::all().into_iter().find(|p| p.name == name)
    }

    pub fn symbol_len(&self) -> usize {
        self.nfft + self.ncp
    }

    pub fn symbol_duration(&self) -> f32 {
        self.symbol_len() as f32 / self.fs
    }

    pub fn bin_hz(&self) -> f32 {
        self.fs / self.nfft as f32
    }

    pub fn carrier_freq(&self, k: usize) -> f32 {
        (self.first_bin + k) as f32 * self.bin_hz()
    }
}
