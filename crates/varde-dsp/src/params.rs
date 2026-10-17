/// Waveform parameters for one Varde mode.
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
    /// Negative values pre-compensate for the radio's pre-emphasis.
    pub tilt_db_per_octave: f32,
    /// Output RMS level (1.0 = full scale; peaks are clipped at full scale).
    pub tx_rms: f32,
    /// Low-pass the received audio at this frequency before searching for preamble A.
    /// On the flat data port, FM noise rises with frequency, so the lower part of the
    /// band gives a much cleaner detection metric.
    pub detect_lowpass_hz: Option<f32>,
}

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
            detect_lowpass_hz: Some(2500.0),
        }
    }

    pub fn by_name(name: &str) -> Option<Self> {
        match name {
            "narrow" => Some(Self::narrow()),
            "wide" => Some(Self::wide()),
            _ => None,
        }
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
