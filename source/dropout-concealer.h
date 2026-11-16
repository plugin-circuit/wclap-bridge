#pragma once
// Conceals skipped audio blocks without added latency (packet loss concealment, for music).
//
// When a block can't be processed (a single-threaded WCLAP whose instance is busy, realtime-lock.h), outputting
// silence clicks twice: at the cut and at the return. Instead, each channel's recent output is continued with a
// linear-prediction (autoregressive) model fitted with Burg's method, the approach of Opus' PLC and of networked
// music PLC (Sacchetto, Rottondi, Bianco), and faded out; when real audio returns it is crossfaded in from the still
// running prediction ("merge", as WebRTC NetEQ calls it). Only the past is used, so there's no latency; the model is
// fitted once per gap, in a block in which the plugin didn't run anyway.
//
// Real-time safe: all memory is allocated in prepare(); played()/conceal() don't allocate, lock or call the OS.

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <vector>

namespace wclap_bridge {

class DropoutConcealer {
	static constexpr double kPi = 3.14159265358979323846;

public:
	static constexpr int kOrder = 32;          // prediction order
	static constexpr double kHistorySeconds = 0.05;
	static constexpr double kFitSeconds = 0.025; // samples the model is fitted on
	static constexpr double kFadeSeconds = 0.03; // the prediction fades out over this (raised cosine)
	static constexpr double kMergeSeconds = 0.003; // crossfade back into real audio

	// Allocates. Not on the audio thread.
	void prepare(uint32_t maxChannels, double sampleRate) {
		sampleRate_ = sampleRate > 0 ? sampleRate : 48000;
		history_ = std::max<uint32_t>(uint32_t(sampleRate_ * kHistorySeconds), 4 * kOrder);
		fitLength_ = std::clamp<uint32_t>(uint32_t(sampleRate_ * kFitSeconds), 2 * kOrder, history_);
		fadeLength_ = std::max<uint32_t>(1, uint32_t(sampleRate_ * kFadeSeconds));
		mergeLength_ = std::max<uint32_t>(1, uint32_t(sampleRate_ * kMergeSeconds));
		fallbackDecay_ = std::exp(std::log(1e-3) / (0.005 * sampleRate_)); // -60 dB in 5 ms
		channels_.assign(maxChannels, Channel{});
		for (auto &c : channels_) {
			c.history.assign(history_, 0.0f);
			c.coeffs.assign(kOrder + 1, 0.0);
			c.state.assign(kOrder, 0.0);
		}
		fitF_.assign(history_, 0.0);
		fitB_.assign(history_, 0.0);
		concealedSamples_ = 0;
		concealing_ = false;
		mergePending_ = false;
	}
	uint32_t maxChannels() const { return uint32_t(channels_.size()); }

	// --- per block: call for every channel, then endBlock() ---

	// A processed block: if the previous one was concealed, crossfade from the prediction into this one; then record it.
	template <class Sample>
	void played(uint32_t channel, Sample *data, uint32_t frames) {
		if (channel >= channels_.size() || !data) return;
		auto &c = channels_[channel];
		if (mergePending_ && c.valid) {
			const uint32_t n = std::min(frames, mergeLength_);
			for (uint32_t i = 0; i < n; ++i) {
				const double predicted = predictNext(c) * fadeGain(concealedSamples_ + i);
				const double w = 0.5 - 0.5 * std::cos(kPi * (i + 1) / double(n + 1)); // 0 -> 1
				data[i] = Sample(w * double(data[i]) + (1.0 - w) * predicted);
			}
		}
		record(c, data, frames);
	}

	// A skipped block: continue each channel's waveform from its recent output, fading out.
	template <class Sample>
	void conceal(uint32_t channel, Sample *data, uint32_t frames) {
		if (!data) return;
		if (channel >= channels_.size()) {
			std::fill_n(data, frames, Sample(0));
			return;
		}
		auto &c = channels_[channel];
		if (!concealing_) fit(c); // the first skipped block of a gap: fit the model on what was played
		for (uint32_t i = 0; i < frames; ++i) {
			const uint64_t t = concealedSamples_ + i;
			data[i] = c.valid && t < fadeLength_ ? Sample(predictNext(c) * fadeGain(t)) : Sample(0);
		}
	}

	void endBlock(bool concealed, uint32_t frames) {
		if (concealed) {
			concealing_ = true;
			concealedSamples_ += frames;
			mergePending_ = true;
		} else {
			// a gap is over (merged, or there was none): the next one starts its fade from the beginning
			concealing_ = false;
			mergePending_ = false;
			concealedSamples_ = 0;
		}
	}

private:
	struct Channel {
		std::vector<float> history; // ring of the last history_ output samples
		uint32_t write = 0;
		std::vector<double> coeffs; // a[0..order], a[0] = 1: x[n] = -sum a[i] x[n-i]
		std::vector<double> state;  // the last kOrder samples, newest first
		double peak = 0;
		bool valid = false;         // a usable model (else silence)
		double fallback = 0;        // a model that diverged: its last good sample, decaying to zero
	};

	double sampleRate_ = 48000;
	uint32_t history_ = 0, fitLength_ = 0, fadeLength_ = 1, mergeLength_ = 1;
	double fallbackDecay_ = 0.99;
	std::vector<Channel> channels_;
	std::vector<double> fitF_, fitB_; // Burg scratch, shared (channels are fitted one after another)
	uint64_t concealedSamples_ = 0;
	bool concealing_ = false, mergePending_ = false;

	template <class Sample>
	void record(Channel &c, const Sample *data, uint32_t frames) {
		for (uint32_t i = 0; i < frames; ++i) {
			c.history[c.write] = float(data[i]);
			if (++c.write == history_) c.write = 0;
		}
	}
	float historyAt(const Channel &c, uint32_t ago) const { // ago = 1: the newest sample
		return c.history[(c.write + history_ - ago) % history_];
	}

	double fadeGain(uint64_t t) const {
		if (t >= fadeLength_) return 0.0;
		return 0.5 + 0.5 * std::cos(kPi * double(t) / double(fadeLength_)); // 1 -> 0
	}

	double predictNext(Channel &c) {
		if (!c.valid) return c.fallback *= fallbackDecay_;
		double y = 0;
		for (int i = 1; i <= kOrder; ++i) y -= c.coeffs[i] * c.state[i - 1];
		// a model that still diverges (numerically; Burg's is stable in exact arithmetic) is dropped: from its last
		// good sample the output decays to zero within a few ms instead of jumping
		if (!std::isfinite(y) || std::fabs(y) > 1.5 * c.peak + 1e-9) {
			c.valid = false;
			c.fallback = c.state[0];
			return c.fallback *= fallbackDecay_;
		}
		for (int i = kOrder - 1; i > 0; --i) c.state[i] = c.state[i - 1];
		c.state[0] = y;
		return y;
	}

	// Burg's method on the last fitLength_ samples. Each reflection coefficient is computed from direct sums,
	// k = -2 sum(f b) / sum(f^2 + b^2), so |k| <= 1 (Cauchy-Schwarz) and the model is stable; the recursive
	// denominator update of the textbook form drifts in floating point and can produce |k| > 1.
	void fit(Channel &c) {
		const uint32_t N = fitLength_;
		double energy = 0;
		c.peak = 0;
		c.fallback = 0;
		for (uint32_t n = 0; n < N; ++n) {
			const double x = historyAt(c, N - n); // oldest first
			fitF_[n] = fitB_[n] = x;
			energy += x * x;
			c.peak = std::max(c.peak, std::fabs(x));
		}
		c.valid = false;
		if (energy < 1e-12 * N) return; // silence: nothing to continue
		std::fill(c.coeffs.begin(), c.coeffs.end(), 0.0);
		c.coeffs[0] = 1.0;
		// f[n]: forward error, b[n]: backward error; stage k pairs f[n] with b[n-1] for n = k+1 .. N-1
		for (int k = 0; k < kOrder; ++k) {
			double num = 0, den = 0;
			for (uint32_t n = uint32_t(k) + 1; n < N; ++n) {
				num += fitF_[n] * fitB_[n - 1];
				den += fitF_[n] * fitF_[n] + fitB_[n - 1] * fitB_[n - 1];
			}
			if (den <= 1e-300) break;
			const double mu = std::clamp(-2.0 * num / den, -0.9999, 0.9999);
			for (int n = 0; n <= (k + 1) / 2; ++n) {
				const double t1 = c.coeffs[n] + mu * c.coeffs[k + 1 - n];
				const double t2 = c.coeffs[k + 1 - n] + mu * c.coeffs[n];
				c.coeffs[n] = t1;
				c.coeffs[k + 1 - n] = t2;
			}
			for (uint32_t n = N - 1; n > uint32_t(k); --n) { // from the end, so b[n-1] is still this stage's
				const double f = fitF_[n], b = fitB_[n - 1];
				fitF_[n] = f + mu * b;
				fitB_[n] = b + mu * f;
			}
		}
		for (int i = 0; i < kOrder; ++i) c.state[i] = historyAt(c, uint32_t(i + 1));
		c.valid = true;
	}
};

} // namespace wclap_bridge
