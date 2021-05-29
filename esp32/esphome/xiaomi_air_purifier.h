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

enum MiiotProperty {
  Invalid,
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
  {MiiotProperty::Aqi, {SiidType::ENVIRONMENT, 6}},
  {MiiotProperty::Humidity, {SiidType::ENVIRONMENT, 7}},
  {MiiotProperty::Temperature, {SiidType::ENVIRONMENT, 8}},
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

class XiaomiAirPurifier : public MiiotUart  {
 public:
  XiaomiAirPurifier(UARTComponent *parent) : MiiotUart(parent) {
    // Init rev mapping
    for(const auto &x: MAPPING)
      REV_MAPPING[x.second.siid][x.second.piid] = x.first;
  }

  TextSensor *model_name = new TextSensor();
  TextSensor *mcu_firmware = new TextSensor();
  TextSensor *ble_config = new TextSensor();
  TextSensor *filter_rfid_tag_sensor = new TextSensor();
  TextSensor *filter_rfid_product_id_sensor = new TextSensor();
  TextSensor *app_extra_sensor = new TextSensor();
  TextSensor *network_status = new TextSensor();

  BinarySensor *power_sensor = new BinarySensor();
  BinarySensor *child_lock_sensor = new BinarySensor();
  BinarySensor *buzzer_sensor = new BinarySensor();
  BinarySensor *led_sensor = new BinarySensor();

  Sensor *fanlevel_sensor = new Sensor();
  Sensor *mode_sensor = new Sensor();

  Sensor *aqi_sensor = new Sensor();
  Sensor *temperature_sensor = new Sensor();
  Sensor *humidity_sensor = new Sensor();

  Sensor *filterlife_remaining_sensor = new Sensor();
  Sensor *filterhours_used_sensor = new Sensor();
  Sensor *buzzer_volume_sensor = new Sensor();
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

  void get_all_properties() {
    for(const auto &x: MAPPING)
      this->get_properties(x.second);
  }

  void set_power(bool enable) {
      set_properties(MAPPING[MiiotProperty::Power], enable ? "1" : "0");
      this->power_sensor->publish_state(enable);
  }

  void set_fan_level(float level) {
    int fanlevel_int = 0;

    if (level == 0.00)
      fanlevel_int = 0;
    else if (level == 0.33)
      fanlevel_int = 1;
    else if (level == 0.66)
      fanlevel_int = 2;
    else if (level == 1.00)
      fanlevel_int = 3;
    else {
      ESP_LOGD(TAG, "set_fan_level: Invalid level %f", level);
      return;
    }
    set_properties(MAPPING[MiiotProperty::FanLevel], to_string(fanlevel_int));
    this->fanlevel_sensor->publish_state(level);
  }

  void set_operation_mode(OperationMode mode) {
    set_properties(MAPPING[MiiotProperty::Mode], to_string(static_cast<int>(mode)));
  }

  void set_led_brightness(LedBrightnessValue value) {
    auto value_int = static_cast<int>(value);
    set_properties(MAPPING[MiiotProperty::LedBrightness], to_string(value_int));
    this->led_brightness_sensor->publish_state(value_int);
  }

  void set_buzzer(bool enable) {
    set_properties(MAPPING[MiiotProperty::Buzzer], enable ? "true" : "false");
  }

  void set_child_lock(bool enable) {
    set_properties(MAPPING[MiiotProperty::ChildLock], enable ? "true" : "false");
  }

  std::string get_network_status() {
      auto current_status = get_current_nw_status();
      auto nw_status_string = network_status_to_string(current_status);
      if (nw_status_string == this->network_status->state)
      {
          return "";
      }

      ESP_LOGD(TAG, "Updating network status: Old=%s, New=%s", this->network_status->state.c_str(), nw_status_string.c_str());
      this->network_status->publish_state(nw_status_string);
      return this->network_status->state;
  }

  std::string get_time() {
        std::ostringstream out;
        time_t now = time(nullptr);
        out << now;
        return out.str();
  }

  bool str_to_bool(std::string value) {
    if (value == "true") return true;
    else if (value == "false") return false;

    ESP_LOGE(TAG, "Failed converting %s to bool", value.c_str());
    return false;
  }

  std::string args_to_str(std::vector<std::string> args) {
        std::ostringstream out;
        for (int i=0; i < args.size(); i++)
          out << args[i] << " ";
        return out.str();
  }

  void handle_value_change(int siid_int, int piid, std::string value) {      
      auto siid = static_cast<SiidType>(siid_int);
      auto property_enum = get_property_value_for_siid_piid(siid, piid);

      switch(property_enum) {
        case MiiotProperty::Power:
          this->power_sensor->publish_state(str_to_bool(value));
          break;
        case MiiotProperty::FanLevel:
          this->fanlevel_sensor->publish_state(atoi(value.c_str()));
          break;
        case MiiotProperty::Mode:
          this->mode_sensor->publish_state(static_cast<OperationMode>(atoi(value.c_str())));
          break;
        case MiiotProperty::Aqi:
          this->aqi_sensor->publish_state(static_cast<float>(atoi(value.c_str())));
          break;
        case MiiotProperty::Humidity:
          this->humidity_sensor->publish_state(static_cast<float>(atoi(value.c_str())));
          break;
        case MiiotProperty::Temperature:
          this->temperature_sensor->publish_state(static_cast<float>(atoi(value.c_str())));
          break;
        case MiiotProperty::FilterLifeRemaining:
          this->filterlife_remaining_sensor->publish_state(static_cast<int>(atoi(value.c_str())));
          break;
        case MiiotProperty::FilterHoursUsed:
          this->filterhours_used_sensor->publish_state(static_cast<int>(atoi(value.c_str())));
          break;
        case MiiotProperty::Buzzer:
          this->buzzer_sensor->publish_state(str_to_bool(value));
          break;
        case MiiotProperty::BuzzerVolume:
          this->buzzer_volume_sensor->publish_state(static_cast<int>(atoi(value.c_str())));
          break;
        case MiiotProperty::LedBrightness:
          this->led_brightness_sensor->publish_state(static_cast<LedBrightnessValue>(atoi(value.c_str())));
          break;
        case MiiotProperty::Led:
          this->led_sensor->publish_state(str_to_bool(value));
          break;
        case MiiotProperty::ChildLock:
          this->child_lock_sensor->publish_state(str_to_bool(value));
          break;
        case MiiotProperty::FavoriteLevel:
          this->favorite_level_sensor->publish_state(static_cast<int>(atoi(value.c_str())));
          break;
        case MiiotProperty::FavoriteRpm:
          this->favorite_rpm_sensor->publish_state(static_cast<int>(atoi(value.c_str())));
          break;
        case MiiotProperty::MotorSpeed:
          this->motor_speed_sensor->publish_state(static_cast<int>(atoi(value.c_str())));
          break;
        case MiiotProperty::UseTime:
          this->use_time_sensor->publish_state(static_cast<int>(atoi(value.c_str())));
          break;
        case MiiotProperty::PurifyVolume:
          this->purify_volume_sensor->publish_state(static_cast<int>(atoi(value.c_str())));
          break;
        case MiiotProperty::AverageAQI:
          this->average_aqi_sensor->publish_state(static_cast<int>(atoi(value.c_str())));
          break;
        case MiiotProperty::FilterRFIDTag:
          this->filter_rfid_tag_sensor->publish_state(value.c_str());
          break;
        case MiiotProperty::FilterRFIDProductId:
          this->filter_rfid_product_id_sensor->publish_state(value.c_str());
          break;
        case MiiotProperty::AppExtra:
          this->app_extra_sensor->publish_state(value.c_str());
          break;
        default:
          ESP_LOGD(TAG, "Unhandled property change SIID: %i, PIID: %i, Value=%s", siid, piid, value.c_str());
      }
  }

  void handle_result(std::vector<std::string> args) {
    if (args.size() != 4) {
      return;
    }
    
    auto siid_int = atoi(args[0].c_str());
    auto piid = atoi(args[1].c_str());
    auto ret_code = atoi(args[2].c_str());
    std::string value = args[3];

    ESP_LOGD(TAG, "Result: SIID: %i PIID: %i RET: %i VALUE: %s", siid_int, piid, ret_code, value.c_str());

    handle_value_change(siid_int, piid, value);
  }

  void handle_changed_properties(std::vector<std::string> args) {
    ESP_LOGD(TAG, "Properties changed: SIID: %s PIID: %s", args[0].c_str(), args[1].c_str());

    auto siid_int = atoi(args[0].c_str());
    auto piid = atoi(args[1].c_str());
    std::string value = args[2];

    handle_value_change(siid_int, piid, value);
  }

  std::string get_down_update() {
    auto nw_status = get_network_status();
    if (!nw_status.empty()) {
        std::ostringstream out;
        out << "down MIIO_net_change " << nw_status;
        return out.str();
    } else if (!this->initial_stats_requested_) {
      ESP_LOGD(TAG, "Requesting initial stats");
      get_all_properties();
      this->initial_stats_requested_ = true;
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

  std::string handle_miiot(std::string command, std::vector<std::string> args) override {
    ESP_LOGV(TAG, "handle_miiot: command=%s, args_count=%i", command.c_str(), args.size());
    auto command_enum = string_to_command(command);

    switch (command_enum) {
        case MiiotCommand::GET_DOWN:
            return get_down_update();
        case MiiotCommand::PROPERTIES_CHANGED:
            handle_changed_properties(args);
            return "ok";
        case MiiotCommand::MODEL:
            ESP_LOGD(TAG, "Got model: %s", args_to_str(args).c_str());
            this->model_name->publish_state(args[0]);
            return "ok";
        case MiiotCommand::BLE_CONFIG:
            ESP_LOGD(TAG, "BLE config: %s", args_to_str(args).c_str());
            this->ble_config->publish_state(args_to_str(args));
            return "ok";
        case MiiotCommand::MCU_VERSION:
            ESP_LOGD(TAG, "MCU Version: %s", args_to_str(args).c_str());
            this->mcu_firmware->publish_state(args_to_str(args));
            return "ok";
        case MiiotCommand::TIME:
            ESP_LOGD(TAG, "Time request: %s", args_to_str(args).c_str());
            return get_time();
        case MiiotCommand::NET:
            ESP_LOGD(TAG, "Net request");
            return network_status_to_string(get_current_nw_status());
        case MiiotCommand::RESULT:
            ESP_LOGD(TAG, "Received Result: %s", args_to_str(args).c_str());
            handle_result(args);
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
    std::vector<std::string> down_queue_{};
    bool initial_stats_requested_{false};

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