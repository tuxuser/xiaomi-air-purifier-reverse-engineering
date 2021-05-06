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

enum LedBrightnessValue {
    Bright = 0,
    Dim = 1,
    Off = 2
};

enum SiidType {
    AIR_PURIFIER = 2,
    ENVIRONMENT = 3,
    FILTER = 4,
    ALARM = 5,
    INDICATOR_LIGHT = 6,
    PHYSICAL_CONTROL_LOCKED = 7,
    MOTOR_SPEED = 10,
    USE_TIME = 12,
    AQI = 13,
    RFID = 14,
    OTHER = 15
};

enum MiiotProperty {
  Power,
  FanLevel,
  Mode,
  Humidity,
  Temperature,
  Aqi,
  FilterLifeRemaining,
  FilterHoursUsed,
  Buzzer,
  BuzzerVolume,
  LedBrightness,
  Led,
  ChildLock,
  FavoriteLevel,
  FavoriteRpm,
  MotorSpeed,
  UseTime,
  PurifyVolume,
  AverageAQI,
  FilterRFIDTag,
  FilterRFIDProductId,
  AppExtra
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
    LedBrightnessValue led_brightness;
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

struct MiiotPropertyValue {
  SiidType siid;
  int piid;
};

// Gets initialized in constructor
std::map<SiidType, std::map<int, MiiotProperty>> REV_MAPPING {};

std::map<MiiotProperty, MiiotPropertyValue> MAPPING {
  {MiiotProperty::Power, {SiidType::AIR_PURIFIER, 2}},
  {MiiotProperty::FanLevel, {SiidType::AIR_PURIFIER, 4}},
  {MiiotProperty::Mode, {SiidType::AIR_PURIFIER, 5}},
  {MiiotProperty::Humidity, {SiidType::ENVIRONMENT, 7}},
  {MiiotProperty::Temperature, {SiidType::ENVIRONMENT, 8}},
  {MiiotProperty::Aqi, {SiidType::ENVIRONMENT, 6}},
  {MiiotProperty::FilterLifeRemaining, {SiidType::FILTER, 3}},
  {MiiotProperty::FilterHoursUsed, {SiidType::FILTER, 5}},
  {MiiotProperty::Buzzer, {SiidType::ALARM, 1}},
  {MiiotProperty::BuzzerVolume, {SiidType::ALARM, 2}},
  {MiiotProperty::LedBrightness, {SiidType::INDICATOR_LIGHT, 1}},
  {MiiotProperty::Led, {SiidType::INDICATOR_LIGHT, 6}},
  {MiiotProperty::ChildLock, {SiidType::PHYSICAL_CONTROL_LOCKED, 1}},
  {MiiotProperty::FavoriteLevel, {SiidType::MOTOR_SPEED, 10}},
  {MiiotProperty::FavoriteRpm, {SiidType::MOTOR_SPEED, 7}},
  {MiiotProperty::MotorSpeed, {SiidType::MOTOR_SPEED, 8}},
  {MiiotProperty::UseTime, {SiidType::USE_TIME, 1}},
  {MiiotProperty::PurifyVolume, {SiidType::AQI, 1}},
  {MiiotProperty::AverageAQI, {SiidType::AQI, 2}},
  {MiiotProperty::FilterRFIDTag, {SiidType::RFID, 1}},
  {MiiotProperty::FilterRFIDProductId, {SiidType::RFID, 3}},
  {MiiotProperty::AppExtra, {SiidType::OTHER, 1}},
};

class XiaomiAirPurifier : public MiiotUart, public Switch  {
 public:
  XiaomiAirPurifier(UARTComponent *parent) : MiiotUart(parent) {
    // Init rev mapping
    for(const auto &x: MAPPING)
      REV_MAPPING[x.second.siid][x.second.piid] = x.first;
    
    // Request all properties
    for(const auto &x: MAPPING)
      this->get_properties(x.second);
  }

  TextSensor *model_name = new TextSensor();
  TextSensor *mcu_firmware = new TextSensor();
  TextSensor *ble_config = new TextSensor();
  TextSensor *filter_rfid_tag_sensor = new TextSensor();
  TextSensor *filter_rfid_product_id_sensor = new TextSensor();
  TextSensor *app_extra_sensor = new TextSensor();

  BinarySensor *power_sensor = new BinarySensor();
  BinarySensor *child_lock_sensor = new BinarySensor();

  Sensor *fanlevel_sensor = new Sensor();
  Sensor *mode_sensor = new Sensor();

  Sensor *aqi_sensor = new Sensor();
  Sensor *temperature_sensor = new Sensor();
  Sensor *humidity_sensor = new Sensor();

  Sensor *filterlife_remaining_sensor = new Sensor();
  Sensor *filterhours_used_sensor = new Sensor();
  Sensor *buzzer_sensor = new Sensor();
  Sensor *buzzer_volume_sensor = new Sensor();
  Sensor *led_sensor = new Sensor();
  Sensor *led_brightness_sensor = new Sensor();
  
  Sensor *favorite_level_sensor = new Sensor();
  Sensor *favorite_rpm_sensor = new Sensor();

  Sensor *motor_speed_sensor = new Sensor();
  Sensor *use_time_sensor = new Sensor();
  Sensor *purify_volume_sensor = new Sensor();
  Sensor *average_aqi_sensor = new Sensor();

  MiiotProperty get_property_value_for_siid_piid(SiidType siid_type, int piid) {
    return REV_MAPPING[siid_type][piid];
  }

  void set_properties(MiiotPropertyValue target, std::string value) {
      std::ostringstream os;
      os << "down set_properties " << target.siid << " " << target.piid << " " << value;
      enqueue_message(os.str());
  }

  void get_properties(MiiotPropertyValue target) {
      std::ostringstream os;
      os << "down get_properties " << target.siid << " " << target.piid;
      enqueue_message(os.str());
  }

  void write_state(bool state) override {
    this->requested_power_state_ = state;
  }

  void set_power(bool enable) {
      set_properties(MAPPING[MiiotProperty::Power], enable ? "true" : "false");
  }

  void set_fan_level(int level) {
    set_properties(MAPPING[MiiotProperty::FanLevel], to_string(level));
  }

  void set_operation_mode(OperationMode mode) {
    set_properties(MAPPING[MiiotProperty::Mode], to_string(static_cast<int>(mode)));
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

  void handle_changed_properties(std::vector<std::string> args, bool is_result) {
      ESP_LOGD(TAG, "Properties changed: SIID: %s PIID: %s", args[0].c_str(), args[1].c_str());

      std::string value = "";
      auto siid_int = atoi(args[0].c_str());
      auto piid = atoi(args[1].c_str());

      // Results args got integer status after siid/piid
      if (is_result) {
        int result_status = atoi(args[2].c_str());
        if (result_status != 0) {
          ESP_LOGE(TAG, "Result received with error code: %i (SIID: %i, PIID: %i)", result_status, siid_int, piid);
          return;
        }

        value = args[3];
      } else {
        value = args[2];
      } 
      
      auto siid = static_cast<SiidType>(siid_int);
      auto property_enum = get_property_value_for_siid_piid(siid, piid);

      switch(property_enum) {
        case MiiotProperty::Power:
          this->state_.power = static_cast<bool>(atoi(value.c_str()));
          this->power_sensor->publish_state(this->state_.power);
          break;
        case MiiotProperty::FanLevel:
          this->state_.fanlevel = static_cast<int>(atoi(value.c_str()));
          this->fanlevel_sensor->publish_state(this->state_.fanlevel);
          break;
        case MiiotProperty::Mode:
          this->state_.mode = static_cast<OperationMode>(atoi(value.c_str()));
          this->mode_sensor->publish_state(atoi(value.c_str()));
          break;
        case MiiotProperty::Aqi:
          this->state_.aqi = static_cast<float>(atoi(value.c_str()));
          this->aqi_sensor->publish_state(this->state_.aqi);
          break;
        case MiiotProperty::Humidity:
          this->state_.humidity = static_cast<float>(atoi(value.c_str()));
          this->humidity_sensor->publish_state(this->state_.humidity);
          break;
        case MiiotProperty::Temperature:
          this->state_.temperature = static_cast<float>(atoi(value.c_str()));
          this->temperature_sensor->publish_state(this->state_.temperature);
          break;
        case MiiotProperty::FilterLifeRemaining:
          this->state_.filter_life_remaining = static_cast<int>(atoi(value.c_str()));
          this->filterlife_remaining_sensor->publish_state(this->state_.filter_life_remaining);
          break;
        case MiiotProperty::FilterHoursUsed:
          this->state_.filter_hours_used = static_cast<int>(atoi(value.c_str()));
          this->filterhours_used_sensor->publish_state(this->state_.filter_hours_used);
          break;
        case MiiotProperty::Buzzer:
          this->state_.buzzer = static_cast<int>(atoi(value.c_str()));
          this->buzzer_sensor->publish_state(this->state_.buzzer);
          break;
        case MiiotProperty::BuzzerVolume:
          this->state_.buzzer_volume = static_cast<int>(atoi(value.c_str()));
          this->buzzer_volume_sensor->publish_state(this->state_.buzzer_volume);
          break;
        case MiiotProperty::LedBrightness:
          this->state_.led_brightness = static_cast<LedBrightnessValue>(atoi(value.c_str()));
          this->led_brightness_sensor->publish_state(this->state_.led_brightness);
          break;
        case MiiotProperty::Led:
          this->state_.led = static_cast<int>(atoi(value.c_str()));
          this->led_sensor->publish_state(this->state_.led);
          break;
        case MiiotProperty::ChildLock:
          this->state_.child_lock = static_cast<bool>(atoi(value.c_str()));
          this->child_lock_sensor->publish_state(this->state_.child_lock);
          break;
        case MiiotProperty::FavoriteLevel:
          this->state_.favorite_level = static_cast<int>(atoi(value.c_str()));
          this->favorite_level_sensor->publish_state(this->state_.favorite_level);
          break;
        case MiiotProperty::FavoriteRpm:
          this->state_.favorite_rpm = static_cast<int>(atoi(value.c_str()));
          this->favorite_rpm_sensor->publish_state(this->state_.favorite_rpm);
          break;
        case MiiotProperty::MotorSpeed:
          this->state_.motor_speed = static_cast<int>(atoi(value.c_str()));
          this->motor_speed_sensor->publish_state(this->state_.motor_speed);
          break;
        case MiiotProperty::UseTime:
          this->state_.use_time = static_cast<int>(atoi(value.c_str()));
          this->use_time_sensor->publish_state(this->state_.use_time);
          break;
        case MiiotProperty::PurifyVolume:
          this->state_.purify_volume = static_cast<int>(atoi(value.c_str()));
          this->purify_volume_sensor->publish_state(this->state_.purify_volume);
          break;
        case MiiotProperty::AverageAQI:
          this->state_.average_aqi = static_cast<int>(atoi(value.c_str()));
          this->average_aqi_sensor->publish_state(this->state_.average_aqi);
          break;
        case MiiotProperty::FilterRFIDTag:
          this->state_.filter_rfid_tag = value.c_str();
          this->filter_rfid_tag_sensor->publish_state(this->state_.filter_rfid_tag);
          break;
        case MiiotProperty::FilterRFIDProductId:
          this->state_.filter_rfid_product_id = value.c_str();
          this->filter_rfid_product_id_sensor->publish_state(this->state_.filter_rfid_product_id);
          break;
        case MiiotProperty::AppExtra:
          this->state_.app_extra = value.c_str();
          this->app_extra_sensor->publish_state(this->state_.app_extra);
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
    ESP_LOGV(TAG, "handle_miiot: command=%s, args_count=%i", command.c_str(), args.size());
    auto command_enum = string_to_command(command);

    switch (command_enum) {
        case MiiotCommand::GET_DOWN:
            return get_down_update();
        case MiiotCommand::PROPERTIES_CHANGED:
            handle_changed_properties(args, false);
            return "ok";
        case MiiotCommand::MODEL:
            ESP_LOGD(TAG, "Got model: %s", args_to_str(args).c_str());
            this->state_.model = args[0];
            this->model_name->publish_state(this->state_.model);
            return "ok";
        case MiiotCommand::BLE_CONFIG:
            ESP_LOGD(TAG, "BLE config: %s", args_to_str(args).c_str());
            this->state_.ble = args_to_str(args);
            this->ble_config->publish_state(this->state_.ble);
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
            handle_changed_properties(args, true);
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