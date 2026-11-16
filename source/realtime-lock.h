#pragma once
// A single-threaded WCLAP (no shared memory) has one WebAssembly instance for every thread, so its calls are
// serialised by a lock that the audio thread also needs. The audio thread must never block on it (the main thread
// may be inside a long call, and it's scheduled at a lower priority): it only tries, spinning against a deadline,
// and skips the block if the instance is still busy. See Timur Doumler, "Using locks in real-time audio processing,
// safely": the real-time side only uses try_lock/unlock, a single atomic operation each, and never sleeps.

#include <atomic>
#include <chrono>
#include <cstdint>
#include <thread>

#if defined(__x86_64__) || defined(_M_X64) || defined(__i386__) || defined(_M_IX86)
#	include <immintrin.h>
#endif

namespace wclap_bridge {

inline void cpuRelax() {
#if defined(__x86_64__) || defined(_M_X64) || defined(__i386__) || defined(_M_IX86)
	_mm_pause();
#elif defined(__aarch64__) || defined(_M_ARM64)
#	if defined(_MSC_VER) && !defined(__clang__)
	__yield();
#	else
	asm volatile("yield");
#	endif
#endif
}

// Recursive (a WCLAP call may call the host, which calls back into the WCLAP on the same thread), owned by one
// thread at a time. try_lock() and unlock() are wait-free; lock() spins, then yields, then sleeps briefly: it's only
// used off the audio thread.
class RecursiveSpinMutex {
public:
	bool try_lock() noexcept {
		const auto me = self();
		if (owner_.load(std::memory_order_acquire) == me) {
			++depth_;
			return true;
		}
		std::uintptr_t none = 0;
		if (owner_.compare_exchange_strong(none, me, std::memory_order_acquire, std::memory_order_relaxed)) {
			depth_ = 1;
			return true;
		}
		return false;
	}
	void lock() noexcept {
		for (int i = 0; !try_lock(); ++i) {
			if (i < 64) cpuRelax();
			else if (i < 512) std::this_thread::yield();
			else std::this_thread::sleep_for(std::chrono::microseconds(50));
		}
	}
	void unlock() noexcept {
		if (--depth_ == 0) owner_.store(0, std::memory_order_release);
	}

private:
	static std::uintptr_t self() noexcept {
		static thread_local char tag;
		return reinterpret_cast<std::uintptr_t>(&tag);
	}
	std::atomic<std::uintptr_t> owner_{0};
	std::uint32_t depth_ = 0; // only touched by the owner
};

// How long the audio thread may spin for a busy single-threaded instance, as a fraction of the current block's
// duration (0 = try once). Set with wclap_set_audio_wait_ratio().
inline std::atomic<float> audioWaitRatio{0.2f};
// Whether a skipped block is concealed (its audio continued from the last output and faded, dropout-concealer.h)
// instead of silent. Set with wclap_set_conceal_dropouts().
inline std::atomic<bool> concealDropouts{true};
// Blocks skipped because the instance stayed busy past the deadline, since start.
inline std::atomic<std::uint64_t> audioSkippedBlocks{0};

// Try `mutex` until `budget` has passed (by reading the clock: the audio thread never sleeps here).
template <class Mutex>
bool tryLockFor(Mutex &mutex, std::chrono::nanoseconds budget) noexcept {
	if (mutex.try_lock()) return true;
	if (budget <= std::chrono::nanoseconds::zero()) return false;
	const auto deadline = std::chrono::steady_clock::now() + budget;
	do {
		for (int i = 0; i < 32; ++i) cpuRelax();
		if (mutex.try_lock()) return true;
	} while (std::chrono::steady_clock::now() < deadline);
	return false;
}

} // namespace wclap_bridge
