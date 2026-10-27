#include "nspanel_lovelace.h"

#include "esphome/core/application.h"
#include "esphome/core/helpers.h"
#include "esphome/core/util.h"

namespace esphome {
namespace nspanel_lovelace {

static const char *const TAG = "nspanel_lovelace";

static std::string replace_custom_send_topic_(const std::string &topic, const char *replacement) {
  const std::string marker = "CustomSend";
  const auto pos = topic.find(marker);
  if (pos == std::string::npos)
    return topic;

  std::string result = topic;
  result.replace(pos, marker.size(), replacement);
  return result;
}

void NSPanelLovelace::setup() {
  if (!this->use_missed_updates_workaround_)
    this->custom_send_queue_.set_interval(0);

  this->mqtt_->subscribe(this->send_topic_, [this](const std::string &topic, const std::string &payload) {
    // Queued and paced from loop(); a delay() here would block the whole app
    // (workaround for https://github.com/sairon/esphome-nspanel-lovelace-ui/issues/8).
    this->send_custom_command(payload);
  });

  if (this->berry_driver_version_ > 0) {
    this->mqtt_->subscribe(replace_custom_send_topic_(this->send_topic_, "GetDriverVersion"),
                           [this](const std::string &topic, const std::string &payload) {
                             this->mqtt_->publish_json(this->recv_topic_, [this](ArduinoJson::JsonObject root) {
                               root["nlui_driver_version"] = this->berry_driver_version_;
                             });
                           });

    this->mqtt_->subscribe(replace_custom_send_topic_(this->send_topic_, "FlashNextion"),
                           [this](const std::string &topic, const std::string &payload) {
                             ESP_LOGD(TAG, "FlashNextion called with URL '%s'", payload.c_str());

                             // Calling upload_tft in MQTT callback directly would crash ESPHome - using a scheduler
                             // task avoids that. Maybe there is another way?
                             App.scheduler.set_timeout(
                                 this, "nspanel_lovelace_flashnextion_upload", 100, [this, payload]() {
                                   ESP_LOGD(TAG, "Starting FlashNextion with URL '%s'", payload.c_str());
                                   this->upload_tft(payload);
                                 });
                           });
  }
}

void NSPanelLovelace::loop() {
  if (this->is_updating_ || this->reparse_mode_) {
    return;
  }

  std::vector<uint8_t> message;
  while (this->available()) {
    uint8_t d;
    this->read_byte(&d);
    switch (this->frame_parser_.feed(d, &message)) {
      case FrameRead::MESSAGE:
        this->process_command_(std::string(message.begin(), message.end()));
        break;
      case FrameRead::CHECKSUM_ERROR:
        ESP_LOGW(TAG, "Dropped frame with invalid checksum");
        break;
      case FrameRead::DROP_RESYNC:
        ESP_LOGW(TAG, "Dropped oversized frame, resyncing");
        break;
      default:
        break;
    }
  }

  // Send at most one queued command per loop() to keep the loop responsive.
  std::string command;
  if (this->custom_send_queue_.next(millis(), &command))
    this->write_custom_command_(command);
}

void NSPanelLovelace::process_command_(const std::string &message) {
  this->mqtt_->publish_json(this->recv_topic_, [message](ArduinoJson::JsonObject root){
    root["CustomRecv"] = message;
  });
  this->incoming_msg_callback_.call(message);
}

void NSPanelLovelace::add_incoming_msg_callback(std::function<void(std::string)> callback) {
  this->incoming_msg_callback_.add(std::move(callback));
}

void NSPanelLovelace::dump_config() { ESP_LOGCONFIG(TAG, "NSPanelLovelace:"); }

void NSPanelLovelace::send_nextion_command(const std::string &command) {
  ESP_LOGD(TAG, "Sending: %s", command.c_str());
  this->write_str(command.c_str());
  const uint8_t to_send[3] = {0xFF, 0xFF, 0xFF};
  this->write_array(to_send, sizeof(to_send));
}

void NSPanelLovelace::send_custom_command(const std::string &command) {
  if (this->is_updating_)
    return;  // don't execute custom commands - UI updates could spoil the upload

  if (!this->custom_send_queue_.enqueue(command))
    ESP_LOGW(TAG, "Custom command queue full, dropping command: %s", command.c_str());
}

void NSPanelLovelace::write_custom_command_(const std::string &command) {
  ESP_LOGD(TAG, "Sending custom command: %s", command.c_str());
  std::vector<uint8_t> data = {0x55, 0xBB};
  data.push_back(command.length() & 0xFF);
  data.push_back((command.length() >> 8) & 0xFF);
  data.insert(data.end(), command.begin(), command.end());
  auto crc = nspanel_crc16(data.data(), data.size());
  data.push_back(crc & 0xFF);
  data.push_back((crc >> 8) & 0xFF);
  this->write_array(data);
}

void NSPanelLovelace::soft_reset() { this->send_nextion_command("rest"); }

uint16_t NSPanelLovelace::recv_ret_string_(std::string &response, uint32_t timeout, bool recv_flag) {
  uint16_t ret;
  uint8_t c = 0;
  uint8_t nr_of_ff_bytes = 0;
  uint64_t start;
  bool exit_flag = false;
  bool ff_flag = false;

  start = millis();

  while ((timeout == 0 && this->available()) || millis() - start <= timeout) {
    if (!this->available()) {
      App.feed_wdt();
      continue;
    }

    this->read_byte(&c);
    if (c == 0xFF) {
      nr_of_ff_bytes++;
    } else {
      nr_of_ff_bytes = 0;
      ff_flag = false;
    }

    if (nr_of_ff_bytes >= 3)
      ff_flag = true;

    response += (char) c;
    if (recv_flag) {
      if (response.find(0x05) != std::string::npos) {
        exit_flag = true;
      }
    }
    App.feed_wdt();
    delay(2);

    if (exit_flag || ff_flag) {
      break;
    }
  }

  if (ff_flag)
    response = response.substr(0, response.length() - 3);  // Remove last 3 0xFF

  ret = response.length();
  return ret;
}

void NSPanelLovelace::start_reparse_mode() {
  this->send_nextion_command("DRAKJHSUYDGBNCJHGJKSHBDN");
  this->send_nextion_command("recmod=0");
  this->send_nextion_command("recmod=0");
  this->send_nextion_command("connect");
  reparse_mode_ = true;
}

void NSPanelLovelace::exit_reparse_mode() {
  this->send_nextion_command("recmod=1");
  reparse_mode_ = false;
}

void NSPanelLovelace::set_baud_rate_(int baud_rate) {
  // Reconfigure the driver already owned by this UART in place. Calling setup() again would
  // take a fresh uart_num_ from the shared counter and leave the previous driver installed.
  this->parent_->set_baud_rate(static_cast<uint32_t>(baud_rate));
  this->parent_->load_settings(false);
}

}  // namespace nspanel_lovelace
}  // namespace esphome
