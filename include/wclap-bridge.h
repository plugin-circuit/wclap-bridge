#pragma once

#ifdef __cplusplus
#include <cstdint>
extern "C" {
#else
#include <stdint.h>
#endif

// 0 means no limit
bool wclap_global_init(unsigned int timeLimitMs);
// Like `wclap_global_init()`, with Wasmtime's compilation cache configured by the TOML file at `compileCacheConfig`
// (e.g. its `directory` and `files-total-size-soft-limit`, see https://docs.wasmtime.dev/cli-cache.html).
// Null uses Wasmtime's default cache settings, as `wclap_global_init()` does.
bool wclap_global_init_with_cache(unsigned int timeLimitMs, const char *compileCacheConfig);

// Single-threaded WCLAPs (no shared memory) share one instance between the main and audio threads. When the main
// thread is inside it, the audio thread spins for at most `ratio` of the current block's duration (0 = try once,
// max 0.9), then outputs a silent block instead of waiting; that block's events are replayed in the next one.
// Any thread, any time. Default 0.2.
void wclap_set_audio_wait_ratio(float ratio);
// Whether such a skipped block is concealed: each channel continued from its recent output (linear prediction) and
// faded out, then crossfaded back into real audio, instead of cutting to silence. No added latency. Default on.
void wclap_set_conceal_dropouts(bool conceal);
// Blocks skipped that way since start (all WCLAPs).
uint64_t wclap_audio_skipped_blocks();
void wclap_global_deinit();

// Opens a WCLAP, returning an opaque identifier (or null if there was an IO failure).  Even if it returns a non-null handle, you should immediately check `wclap_get_error()`
void * wclap_open(const char *wclapDir);

// Opens a WCLAP with read-only directory `/plugin/` and optional read-write directories `/presets/`, `/cache/` and `/var/`
void * wclap_open_with_dirs(const char *wclapDir, const char *presetDir, const char *cacheDir, const char *varDir);

// Thread safe, non-blocking unless there's an error (in which case the buffer is filled, and `true` returned)
bool wclap_get_error(void *, char *buffer, uint32_t bufferCapacity);

// Closes a WCLAP using its opaque identifier.  Unlike clap_entry::deinit(), this MUST be called exactly once after the corresponding wclap_open.
// This really *shouldn't* fail - if it does, then there might be a memory leak.  Normally this would be an `abort()`-worthy bug, but we might not want to lose user data
bool wclap_close(void *);

// identical to `clap_version`
typedef struct wclap_version_triple {
	uint32_t major, minor, revision;
} wclap_version_triple_t;

// Returns a pointer to the opened WCLAP's CLAP API version
const struct wclap_version_triple * wclap_version(void *);

// The CLAP version which this bridge supports (as completely as possible)
const struct wclap_version_triple * wclap_bridge_version();

// Gets a factory (if supported by both the WCLAP and the bridge)
const void * wclap_get_factory(void *, const char *factory_id);

void wclap_set_strings(const char *pluginIdPrefix, const char *pluginNamePrefix, const char *pluginNameSuffix);

#ifdef __cplusplus
}
#endif
