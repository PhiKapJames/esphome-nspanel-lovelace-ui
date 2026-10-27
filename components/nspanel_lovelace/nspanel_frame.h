#pragma once

#include <cstdint>
#include <vector>

namespace esphome {
namespace nspanel_lovelace {

inline uint16_t nspanel_crc16(const uint8_t *data, uint16_t len) {
  uint16_t crc = 0xFFFF;
  while (len--) {
    crc ^= *data++;
    for (uint8_t i = 0; i < 8; i++) {
      if ((crc & 0x01) != 0) {
        crc >>= 1;
        crc ^= 0xA001;
      } else {
        crc >>= 1;
      }
    }
  }
  return crc;
}

enum class FrameRead : uint8_t { WAITING, MESSAGE, CHECKSUM_ERROR, DROP_RESYNC };

/**
 * Byte-by-byte parser for NSPanel UART frames (0x55 0xBB len payload crc16).
 *
 * The frame length is capped so a corrupt or hostile length field cannot grow
 * the buffer without bound; oversized frames are dropped and the parser
 * resyncs on the next 0x55 0xBB header. Pure logic, so it is host-testable.
 */
class FrameParser {
 public:
  static constexpr uint16_t MAX_FRAME_LENGTH = 2048;

  FrameRead feed(uint8_t byte, std::vector<uint8_t> *message = nullptr) {
    switch (this->state_) {
      case State::HEADER1:
        if (byte == 0x55) {
          this->buf_.push_back(byte);
          this->state_ = State::HEADER2;
        }
        return FrameRead::WAITING;  // other bytes are noise, skipped silently

      case State::HEADER2:
        if (byte == 0xBB) {
          this->buf_.push_back(byte);
          this->state_ = State::LENGTH_LO;
          return FrameRead::WAITING;
        }
        // Not a frame header; drop the partial header and re-evaluate byte
        // (a 0x55 here could start a real frame).
        this->reset();
        return this->feed(byte, message);

      case State::LENGTH_LO:
        this->buf_.push_back(byte);
        this->state_ = State::LENGTH_HI;
        return FrameRead::WAITING;

      case State::LENGTH_HI: {
        this->buf_.push_back(byte);
        this->length_ = uint16_t(this->buf_[2]) | (uint16_t(this->buf_[3]) << 8);
        if (this->length_ > MAX_FRAME_LENGTH) {
          this->reset();
          return FrameRead::DROP_RESYNC;
        }
        this->state_ = this->length_ == 0 ? State::CRC_LO : State::PAYLOAD;
        return FrameRead::WAITING;
      }

      case State::PAYLOAD:
        this->buf_.push_back(byte);
        if (this->buf_.size() == size_t(4 + this->length_))
          this->state_ = State::CRC_LO;
        return FrameRead::WAITING;

      case State::CRC_LO:
        this->buf_.push_back(byte);
        this->state_ = State::CRC_HI;
        return FrameRead::WAITING;

      case State::CRC_HI: {
        this->buf_.push_back(byte);
        // Received CRC is little endian: [lo][hi] as the last two bytes.
        uint16_t crc = uint16_t(this->buf_[this->buf_.size() - 2]) |
                       (uint16_t(this->buf_[this->buf_.size() - 1]) << 8);
        uint16_t calculated = nspanel_crc16(this->buf_.data(), uint16_t(this->buf_.size() - 2));
        if (crc != calculated) {
          this->reset();
          return FrameRead::CHECKSUM_ERROR;
        }
        if (message != nullptr)
          message->assign(this->buf_.begin() + 4, this->buf_.end() - 2);
        this->reset();
        return FrameRead::MESSAGE;
      }
    }
    return FrameRead::WAITING;
  }

  size_t size() const { return this->buf_.size(); }
  void reset() {
    this->buf_.clear();
    this->state_ = State::HEADER1;
  }

 private:
  enum class State : uint8_t { HEADER1, HEADER2, LENGTH_LO, LENGTH_HI, PAYLOAD, CRC_LO, CRC_HI };

  std::vector<uint8_t> buf_;
  State state_ = State::HEADER1;
  uint16_t length_ = 0;
};

}  // namespace nspanel_lovelace
}  // namespace esphome
