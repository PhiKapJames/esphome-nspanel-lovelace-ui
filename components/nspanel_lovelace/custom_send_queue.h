#pragma once

#include <cstdint>
#include <deque>
#include <string>
#include <utility>

namespace esphome {
namespace nspanel_lovelace {

/**
 * FIFO queue with non-blocking pacing for custom send commands.
 *
 * The panel needs at least 75ms between frames on the wire
 * (https://github.com/sairon/esphome-nspanel-lovelace-ui/issues/8). Commands are
 * queued here and flushed one at a time from loop(), so the MQTT callback never
 * blocks. Pure logic (clock passed in) so it is host-testable.
 */
class CustomSendQueue {
 public:
  static constexpr uint32_t DEFAULT_INTERVAL_MS = 75;
  static constexpr size_t MAX_PENDING = 32;

  /** Set the minimum spacing between sends; 0 disables pacing. */
  void set_interval(uint32_t interval_ms) { this->interval_ms_ = interval_ms; }

  /** Queue a command. Returns false (rejecting the newest) when full. */
  bool enqueue(std::string command) {
    if (this->queue_.size() >= MAX_PENDING)
      return false;
    this->queue_.push_back(std::move(command));
    return true;
  }

  /** Pop the next command if one is queued and the interval has elapsed. */
  bool next(uint32_t now, std::string *out) {
    if (this->queue_.empty() || uint32_t(now - this->last_send_) < this->interval_ms_)
      return false;
    *out = std::move(this->queue_.front());
    this->queue_.pop_front();
    this->last_send_ = now;
    return true;
  }

  bool empty() const { return this->queue_.empty(); }
  size_t size() const { return this->queue_.size(); }
  static constexpr size_t max_pending() { return MAX_PENDING; }

 private:
  std::deque<std::string> queue_;
  uint32_t interval_ms_ = DEFAULT_INTERVAL_MS;
  uint32_t last_send_ = 0;
};

}  // namespace nspanel_lovelace
}  // namespace esphome
