#include "esphome.h"
#include "miiot_uart.h"
#include <sstream>

enum NetworkStatus {
    Offline,
    Local,
    Cloud
};

enum OperationMode {
    Auto = 0,
    Silent = 1,
    Favorite = 2,
    FanManual = 3
};

enum LedBrightness {
    Bright = 0,
    Dim = 1,
    Off = 2
};

enum SiidType {
    AirPurifier = 2,
    Environment = 3,
    Filter = 4,
    Alarm = 5,
    IndicatorLight = 6,
    PhysicalControlLocked = 7,
    MotorSpeed = 10,
    UseTime = 12,
    AQI = 13,
    RFID = 14,
    Other = 15
};

enum MiiotCommand {
  INVALID,
  ERROR,
  GET_DOWN,
  PROPERTIES_CHANGED,
  RESULT,
  MODEL,
  BLE_CONFIG,
  MCU_VERSION,
  TIME,
  NET
};

struct State {
    State() = default;

    std::string model;
    std::string mcu_version;
    std::string ble;

    NetworkStatus net_status;
    bool power;
    int fanlevel;
    OperationMode mode;
    float humidity;
    float temperature;
    float aqi;
    int filter_life_remaining;
    int filter_hours_used;
    int buzzer;
    int buzzer_volume;
    LedBrightness led_brightness;
    int led;
    bool child_lock;
    int favorite_level;
    int favorite_rpm;
    int motor_speed;
    int use_time;
    int purify_volume;
    int average_aqi;
    std::string filter_rfid_tag;
    std::string filter_rfid_product_id;
    std::string app_extra;
};

class XiaomiAirPurifier : public MiiotUart, public Switch  {
 public:
  XiaomiAirPurifier(UARTComponent *parent) : MiiotUart(parent) {}

  // TODO: Add more sensors
  TextSensor *model_name = new TextSensor();
  TextSensor *mcu_firmware = new TextSensor();
  Sensor *aqi_sensor = new Sensor();
  Sensor *temperature_sensor = new Sensor();
  Sensor *humidity_sensor = new Sensor();

  void set_properties(SiidType siid, int piid, std::string value) {
      std::ostringstream os;
      os << "down set_properties " << siid << " " << piid << " " << value;
      enqueue_message(os.str());
  }

  void write_state(bool state) override {
    this->requested_power_state_ = state;
  }

  void set_power(bool enable) {
      set_properties(SiidType::AirPurifier, 2, enable ? "true" : "false");
  }

  void set_fan_level(int level) {
    set_properties(SiidType::AirPurifier, 4, to_string(level));
  }

  void set_operation_mode(OperationMode mode) {
    set_properties(SiidType::AirPurifier, 5, to_string(static_cast<int>(mode)));
  }

  std::string get_network_status() {
      auto current_status = get_current_nw_status();
      if (current_status == this->state_.net_status)
      {
          return "";
      }

      ESP_LOGD(TAG, "Updating network status: Old=%i, New=%i", this->state_.net_status, current_status);
      this->state_.net_status = current_status;
      return network_status_to_string(this->state_.net_status);
  }

  std::string get_time() {
        std::ostringstream out;
        time_t now = time(nullptr);
        out << now;
        return out.str();
  }

  void handle_changed_properties(std::vector<std::string> args) {
      ESP_LOGD(TAG, "Properties changed: SIID: %s PIID: %s", args[0].c_str(), args[1].c_str());

      auto siid_int = atoi(args[0].c_str());
      auto piid = atoi(args[1].c_str());
      auto value = args[2];
      auto siid = static_cast<SiidType>(siid_int);

      switch(siid) {
        case SiidType::AirPurifier:
          // TODO
          break;
        case SiidType::Environment:
          switch (piid) {
              case 6:
                this->state_.aqi = static_cast<float>(atoi(value.c_str()));
                this->aqi_sensor->publish_state(this->state_.aqi);
                break;
              case 7:
                this->state_.humidity = static_cast<float>(atoi(value.c_str()));
                this->humidity_sensor->publish_state(this->state_.humidity);
                break;
              case 8:
                this->state_.temperature = static_cast<float>(atoi(value.c_str()));
                this->temperature_sensor->publish_state(this->state_.temperature);
                break;
              default:
                ESP_LOGD(TAG, "Invalid Environment PIID: %i", piid);
          }
          break;
        case SiidType::Filter:
          // TODO
          break;
        case SiidType::Alarm:
          // TODO
          break;
        case SiidType::IndicatorLight:
          // TODO
          break;
        case SiidType::PhysicalControlLocked:
          // TODO
          break;
        case SiidType::MotorSpeed:
          // TODO
          break;
        case SiidType::UseTime:
          // TODO
          break;
        case SiidType::AQI:
          // TODO
          break;
        case SiidType::RFID:
          // TODO
          break;
        case SiidType::Other:
          // TODO
          break;
        default:
          ESP_LOGD(TAG, "Invalid SIID: %i", siid);
      }
  }

  std::string get_down_update() {
    auto nw_status = get_network_status();
    if (!nw_status.empty()) {
        std::ostringstream out;
        out << "down MIIO_net_change " << nw_status;
        return out.str();
    } else if (this->requested_power_state_ != this->state_.power) {
        set_power(this->requested_power_state_);
        this->state_.power = this->requested_power_state_;
    } else if (!this->down_queue_.empty()) {
        auto data = this->down_queue_[0];
        this->down_queue_.erase(this->down_queue_.begin());
        return data;
    }

    return "down none";
  }

  MiiotCommand string_to_command(std::string command) {
    if (command == "error") return MiiotCommand::ERROR;
    if (command == "get_down") return MiiotCommand::GET_DOWN;
    if (command == "properties_changed") return MiiotCommand::PROPERTIES_CHANGED;
    if (command == "result") return MiiotCommand::RESULT;
    if (command == "model") return MiiotCommand::MODEL;
    if (command == "ble_config") return MiiotCommand::BLE_CONFIG;
    if (command == "mcu_version") return MiiotCommand::MCU_VERSION;
    if (command == "time") return MiiotCommand::TIME;
    if (command == "net") return MiiotCommand::NET;

    return MiiotCommand::INVALID;
  }

  std::string args_to_str(std::vector<std::string> args) {
        std::ostringstream out;
        for (int i=0; i < args.size(); i++)
          out << args[i] << " ";
        return out.str();
  }

  std::string handle_miiot(std::string command, std::vector<std::string> args) override {
    ESP_LOGD(TAG, "handle_miiot: command=%s, args_count=%i", command.c_str(), args.size());
    auto command_enum = string_to_command(command);

    switch (command_enum) {
        case MiiotCommand::GET_DOWN:
            return get_down_update();
        case MiiotCommand::PROPERTIES_CHANGED:
            handle_changed_properties(args);
            return "ok";
        case MiiotCommand::MODEL:
            ESP_LOGD(TAG, "Got model: %s", args_to_str(args).c_str());
            this->state_.model = args[0];
            this->model_name->publish_state(this->state_.model);
            return "ok";
        case MiiotCommand::BLE_CONFIG:
            ESP_LOGD(TAG, "BLE config: %s", args_to_str(args).c_str());
            this->state_.ble = args_to_str(args);
            return "ok";
        case MiiotCommand::MCU_VERSION:
            ESP_LOGD(TAG, "MCU Version: %s", args_to_str(args).c_str());
            this->state_.mcu_version = args_to_str(args);
            this->mcu_firmware->publish_state(this->state_.mcu_version);
            return "ok";
        case MiiotCommand::TIME:
            ESP_LOGD(TAG, "Time request: %s", args_to_str(args).c_str());
            return get_time();
        case MiiotCommand::NET:
            ESP_LOGD(TAG, "Net request");
            return network_status_to_string(get_current_nw_status());
        case MiiotCommand::RESULT:
            ESP_LOGD(TAG, "Received Result: %s", args_to_str(args).c_str());
            break;
        case MiiotCommand::ERROR:
            ESP_LOGD(TAG, "Received Error: %s", args_to_str(args).c_str());
            break;
        case MiiotCommand::INVALID:
            ESP_LOGD(TAG, "Invalid command: %s", command.c_str());
    }

    return "ok";
  }

  protected:
    const char* TAG = "xiaomi_air_purifier";
    State state_{};
    std::vector<std::string> down_queue_{};
    bool requested_power_state_{false};

  void enqueue_message(std::string message) {
      this->down_queue_.push_back(message);
  }

    NetworkStatus get_current_nw_status() {
        return NetworkStatus::Local;
    }

    std::string network_status_to_string(NetworkStatus status) {
      switch (status) {
          case NetworkStatus::Offline:
            return "offline";
          case NetworkStatus::Local:
            return "local";
          case NetworkStatus::Cloud:
            return "cloud";
      }

      return "INVALID";
  }
};