#pragma once
#include <cstdint>
#include <cstddef>

// Password key-derivation shared by the locator firmware and the Android app.
//
// The locator authenticates its PreLaunchData broadcasts with a password-seeded
// checksum (see Communication::ComputePasswordAuthTag).  The seed is derived
// from the user's password with FNV-1a (32-bit), which is trivial to reproduce
// byte-for-byte in Kotlin (see the app's LocatorAuth.fnv1a32).  Both sides MUST
// stay in lockstep: FNV offset basis 0x811c9dc5, prime 0x01000193, 32-bit wrap.
//
// A derived key of 0 is reserved to mean "no password set" (open locator), so a
// real password that happens to hash to 0 is bumped to 1.
namespace PasswordKdf {

inline uint32_t Fnv1a32(const char *password) {
	uint32_t hash = 0x811c9dc5u;               // FNV offset basis
	for (const char *p = password; *p != '\0'; ++p) {
		hash ^= static_cast<uint8_t>(*p);
		hash *= 0x01000193u;                   // FNV prime (32-bit wrap)
	}
	return hash;
}

// Derive the stored password key.  An empty password clears the key (0 = open);
// a non-empty password never yields 0.
inline uint32_t DeriveKey(const char *password) {
	if (password == nullptr || password[0] == '\0')
		return 0u;
	uint32_t key = Fnv1a32(password);
	return key == 0u ? 1u : key;
}

} // namespace PasswordKdf
