// Dependency-free host test for the custom-send pacer.
// Build & run: g++ -std=c++17 -Wall -Wextra -I ../../components/nspanel_lovelace -o /tmp/t test_custom_send_queue.cpp && /tmp/t
#include <cassert>
#include <cstdint>
#include <cstdio>
#include <string>

#include "custom_send_queue.h"

using esphome::nspanel_lovelace::CustomSendQueue;

static std::string pop(CustomSendQueue &q, uint32_t now) {
  std::string out;
  if (q.next(now, &out))
    return out;
  return {};
}

int main() {
  // First command goes out immediately once 75ms of uptime have passed.
  {
    CustomSendQueue q;
    assert(q.enqueue("a"));
    std::string out;
    assert(q.next(100, &out) && out == "a");
  }

  // FIFO order is preserved.
  {
    CustomSendQueue q;
    assert(q.enqueue("first"));
    assert(q.enqueue("second"));
    assert(q.enqueue("third"));
    assert(pop(q, 100) == "first");
    assert(pop(q, 175) == "second");
    assert(pop(q, 250) == "third");
    assert(q.empty());
  }

  // Nothing is sent within 75ms of the previous send; the exact boundary sends.
  {
    CustomSendQueue q;
    assert(q.enqueue("a"));
    std::string out;
    assert(q.next(1000, &out));
    assert(q.enqueue("b"));
    assert(!q.next(1074, &out));  // too early
    assert(q.next(1075, &out) && out == "b");  // exactly 75ms later
  }

  // Empty queue never produces a send, and an idle gap always allows one.
  {
    CustomSendQueue q;
    std::string out;
    assert(!q.next(5000, &out));
    assert(q.enqueue("a"));
    assert(q.next(5000, &out));  // last send was long before
  }

  // millis() wraparound: unsigned subtraction keeps pacing correct.
  {
    CustomSendQueue q;
    assert(q.enqueue("a"));
    std::string out;
    assert(q.next(0xFFFFFFF0u, &out));
    assert(q.enqueue("b"));
    assert(!q.next(0xFFFFFFF0u + 74, &out));  // still too early
    assert(q.next(0xFFFFFFF0u + 75, &out) && out == "b");  // wrapped, now due
  }

  // Overflow rejects the newest command instead of overwriting or growing.
  {
    CustomSendQueue q;
    for (size_t i = 0; i < q.max_pending(); i++)
      assert(q.enqueue("c" + std::to_string(i)));
    assert(!q.enqueue("overflow"));
    assert(q.size() == q.max_pending());
    assert(pop(q, 100000) == "c0");  // oldest untouched
  }

  // interval 0 (missed-updates workaround disabled): no pacing at all.
  {
    CustomSendQueue q;
    q.set_interval(0);
    assert(q.enqueue("a"));
    assert(q.enqueue("b"));
    assert(pop(q, 500) == "a");
    assert(pop(q, 500) == "b");  // same tick
  }

  printf("all custom send queue tests passed\n");
  return 0;
}
