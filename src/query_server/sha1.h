#pragma once

// Minimal SHA-1 + base64 for the WebSocket handshake. Public domain style.

#include <cstdint>
#include <cstring>
#include <string>

namespace wqs {
namespace sha1 {

inline uint32_t rol(uint32_t v, int s) { return (v << s) | (v >> (32 - s)); }

inline void hash(const uint8_t* data, size_t len, uint8_t out[20]) {
    uint32_t h0 = 0x67452301, h1 = 0xEFCDAB89, h2 = 0x98BADCFE, h3 = 0x10325476, h4 = 0xC3D2E1F0;
    uint8_t block[64];
    uint64_t bitlen = static_cast<uint64_t>(len) * 8;
    size_t i = 0;
    auto process = [&](const uint8_t* blk) {
        uint32_t w[80];
        for (int t = 0; t < 16; ++t)
            w[t] = (uint32_t)blk[t * 4] << 24 | (uint32_t)blk[t * 4 + 1] << 16 |
                   (uint32_t)blk[t * 4 + 2] << 8 | (uint32_t)blk[t * 4 + 3];
        for (int t = 16; t < 80; ++t) w[t] = rol(w[t - 3] ^ w[t - 8] ^ w[t - 14] ^ w[t - 16], 1);
        uint32_t a = h0, b = h1, c = h2, d = h3, e = h4;
        for (int t = 0; t < 80; ++t) {
            uint32_t f, k;
            if (t < 20) { f = (b & c) | ((~b) & d); k = 0x5A827999; }
            else if (t < 40) { f = b ^ c ^ d; k = 0x6ED9EBA1; }
            else if (t < 60) { f = (b & c) | (b & d) | (c & d); k = 0x8F1BBCDC; }
            else { f = b ^ c ^ d; k = 0xCA62C1D6; }
            uint32_t temp = rol(a, 5) + f + e + k + w[t];
            e = d; d = c; c = rol(b, 30); b = a; a = temp;
        }
        h0 += a; h1 += b; h2 += c; h3 += d; h4 += e;
    };
    for (; i + 64 <= len; i += 64) process(data + i);
    size_t rem = len - i;
    std::memset(block, 0, 64);
    std::memcpy(block, data + i, rem);
    block[rem] = 0x80;
    if (rem >= 56) {
        process(block);
        std::memset(block, 0, 64);
    }
    for (int t = 0; t < 8; ++t) block[63 - t] = static_cast<uint8_t>(bitlen >> (t * 8));
    process(block);
    uint32_t hs[5] = {h0, h1, h2, h3, h4};
    for (int t = 0; t < 5; ++t) {
        out[t * 4] = static_cast<uint8_t>(hs[t] >> 24);
        out[t * 4 + 1] = static_cast<uint8_t>(hs[t] >> 16);
        out[t * 4 + 2] = static_cast<uint8_t>(hs[t] >> 8);
        out[t * 4 + 3] = static_cast<uint8_t>(hs[t]);
    }
}

inline std::string b64(const uint8_t* data, size_t len) {
    static const char* T = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
    std::string o;
    o.reserve(((len + 2) / 3) * 4);
    for (size_t i = 0; i < len; i += 3) {
        unsigned n = data[i] << 16;
        if (i + 1 < len) n |= data[i + 1] << 8;
        if (i + 2 < len) n |= data[i + 2];
        o.push_back(T[(n >> 18) & 63]);
        o.push_back(T[(n >> 12) & 63]);
        o.push_back(i + 1 < len ? T[(n >> 6) & 63] : '=');
        o.push_back(i + 2 < len ? T[n & 63] : '=');
    }
    return o;
}

inline std::string acceptKey(const std::string& clientKey) {
    const char* magic = "258EAFA5-E914-47DA-95CA-C5AB0DC85B11";
    std::string src = clientKey + magic;
    uint8_t dig[20];
    hash(reinterpret_cast<const uint8_t*>(src.data()), src.size(), dig);
    return b64(dig, 20);
}

} // namespace sha1
} // namespace wqs
