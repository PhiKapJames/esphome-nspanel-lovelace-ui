// Dependency-free host test for the NSPanel receive frame parser.
// Build & run: g++ -std=c++17 -Wall -Wextra -I ../../components/nspanel_lovelace -o /tmp/t test_frame_parser.cpp && /tmp/t
#include <cassert>
#include <cstdint>
#include <cstdio>
#include <vector>

#include "nspanel_frame.h"

using esphome::nspanel_lovelace::FrameParser;
using esphome::nspanel_lovelace::FrameRead;
namespace nfl = esphome::nspanel_lovelace;

// Build a valid wire frame: 0x55 0xBB len.lo len.hi payload crc.lo crc.hi
static std::vector<uint8_t> frame(const std::vector<uint8_t> &payload) {
  std::vector<uint8_t> f = {0x55, 0xBB, uint8_t(payload.size() & 0xFF), uint8_t(payload.size() >> 8)};
  f.insert(f.end(), payload.begin(), payload.end());
  uint16_t crc = nfl::nspanel_crc16(f.data(), f.size());
  f.push_back(crc & 0xFF);
  f.push_back(crc >> 8);
  return f;
}

struct FeedResult {
  std::vector<std::vector<uint8_t>> messages;
  int drops = 0;
};

static FeedResult feed(FrameParser &p, const std::vector<uint8_t> &bytes) {
  FeedResult r;
  for (uint8_t b : bytes) {
    std::vector<uint8_t> msg;
    switch (p.feed(b, &msg)) {
      case FrameRead::MESSAGE:
        r.messages.push_back(msg);
        break;
      case FrameRead::DROP_RESYNC:
        r.drops++;
        break;
      default:
        break;
    }
  }
  return r;
}

int main() {
  // A valid frame is delivered with exactly its payload.
  {
    FrameParser p;
    auto r = feed(p, frame({0x01, 0x02, 0xAB}));
    assert(r.messages.size() == 1 && r.messages[0].size() == 3 && r.messages[0][2] == 0xAB);
    assert(r.drops == 0);
  }

  // Garbage before the header is skipped silently, frame still parses.
  {
    FrameParser p;
    auto r = feed(p, {0x00, 0xFF, 0x55, 0x02, 0x11});  // 0x55 not followed by 0xBB -> skipped
    assert(r.messages.empty() && r.drops == 0);
    auto bytes = frame({0x42});
    bytes.insert(bytes.begin(), 0xAA);  // noise byte in front
    r = feed(p, bytes);
    assert(r.messages.size() == 1 && r.messages[0][0] == 0x42);
  }

  // Oversized length (0xFFFF) is rejected immediately: no unbounded buffering.
  {
    FrameParser p;
    auto r = feed(p, {0x55, 0xBB, 0xFF, 0xFF});
    assert(r.drops == 1);
    assert(p.size() == 0);
    // Parser resyncs and the next valid frame is still delivered.
    r = feed(p, frame({0x01}));
    assert(r.messages.size() == 1);
  }

  // Length above the frame cap is rejected even though it fits in 16 bits.
  {
    FrameParser p;
    auto r = feed(p, {0x55, 0xBB, 0x01, 0x08});  // length 2049 > MAX_FRAME_LENGTH
    assert(r.drops == 1 && p.size() == 0);
    // Exactly at the cap is accepted (no frame delivered, just not dropped).
    r = feed(p, {0x55, 0xBB, 0x00, 0x08});  // length 2048 == cap -> accepted, still waiting
    assert(r.drops == 0);
  }

  // A 0x55 inside noise, immediately followed by a real header, still parses.
  {
    FrameParser p;
    auto bytes = frame({0x33});
    bytes.insert(bytes.begin(), 0x55);  // 0x55 0x55 0xBB ...
    auto r = feed(p, bytes);
    assert(r.messages.size() == 1 && r.messages[0][0] == 0x33);
  }

  // CRC mismatch reports an error, resets state, and a good frame follows.
  {
    FrameParser p;
    auto bytes = frame({0x07, 0x08});
    bytes.back() ^= 0xFF;  // corrupt CRC
    auto r = feed(p, bytes);
    assert(r.messages.empty() && r.drops == 0);
    assert(p.size() == 0);  // state reset, no stale bytes retained
    r = feed(p, frame({0x09}));
    assert(r.messages.size() == 1);
  }

  // feed() never grows past the cap no matter what arrives.
  {
    FrameParser p;
    for (uint32_t i = 0; i < 100000; i++)
      p.feed(uint8_t(i & 0xFF));
    assert(p.size() <= FrameParser::MAX_FRAME_LENGTH + 6);
  }

  // Empty payload frame works.
  {
    FrameParser p;
    auto r = feed(p, frame({}));
    assert(r.messages.size() == 1 && r.messages[0].empty());
  }

  // The message out-parameter is optional (callers that only count bytes).
  {
    FrameParser p;
    assert(p.feed(0x00) == FrameRead::WAITING);
    FeedResult r;
    for (uint8_t b : frame({0x01, 0x02})) {
      if (p.feed(b) == FrameRead::MESSAGE)
        r.messages.push_back({});
    }
    assert(r.messages.size() == 1);
  }

  printf("all frame parser tests passed\n");
  return 0;
}
