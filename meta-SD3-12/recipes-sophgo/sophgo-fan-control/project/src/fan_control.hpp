/*
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (C) 2021-2022 YADRO.
 */

#pragma once

#include <boost/asio/io_service.hpp>
#include <boost/asio/steady_timer.hpp>
#include <boost/container/flat_map.hpp>
#include <nlohmann/json.hpp>
#include <sdbusplus/asio/object_server.hpp>

#include <filesystem>
#include <string_view>

namespace fan_control
{

struct FanConfig
{
    std::string name;
    std::string dbusName;
    std::string readPath;
    std::string writePath;
    int64_t min;
    int64_t max;
    bool state;
    FanConfig() = default;
    FanConfig(const nlohmann::json& j) {
        name = j.at("name").get<std::string>();
        dbusName = j.at("dbusName").get<std::string>();
        readPath = j.at("readPath").get<std::string>();
        writePath = j.value("writePath", "");
        min = j.at("min").get<int64_t>();
        max = j.at("max").get<int64_t>();
        state = true;
    }
};

struct TempConfig
{
    std::string name;
    std::string dbusName;
    std::string readPath;
    int minSetPoint;
    bool isInherent;
    int shutdownThreshold;
    int fanFaultThreshold;
    std::map<int, int> input;
    std::map<int, int> output;
    bool accessibility;

    TempConfig() = default;

    TempConfig(const nlohmann::json& j) {
        name = j.at("name").get<std::string>();
        dbusName = j.at("dbusName").get<std::string>();
        readPath = j.at("readPath").get<std::string>();
        minSetPoint = j.at("minSetPoint").get<int>();
        isInherent = j.at("isInherent").get<bool>();
        shutdownThreshold = j.at("shutdownThreshold").get<int>();
        fanFaultThreshold = j.at("fanFaultThreshold").get<int>();
        for (const auto& [key, value] : j.at("input").items()) {
            input[std::stoi(key)] = value.get<int>();
        }
        for (const auto& [key, value] : j.at("output").items()) {
            output[std::stoi(key)] = value.get<int>();
        }

        accessibility = true;
    }

    int findInputIndex(int data) const {
        int inputIndex = -1;
        for (const auto& [index, value] : input) {
            if (data >= value) {
                inputIndex = index;
            }
        }

        return inputIndex;
    }

    int getOutputValue(int inputIndex) const {
        if (inputIndex >= 0) {
            auto it = output.find(inputIndex);
            if (it != output.end()) {
                return it->second;
            } else {
                throw std::runtime_error("Output value not found for input index.");
            }
        }
        else {
            return minSetPoint;
        }

    }
};

/*******************************************************************************
 *  Function declaration
*/
bool check_service_exists(const char* service_name, const char* object_path);
bool is_sensor_dbus_ready(void);
bool is_needed_to_check_dbus(void);
void async_wait_sensor_dbus_ready(const boost::system::error_code& ec);
bool isAutoControlMode(void);
std::string FixupPath(std::string original);
int percentageToActual(double percentage, double baseValue);
bool containsIgnoreCase(const std::string& haystack, const std::string& needle);
void set_auto_timer(int sec);
void repeat_auto_timer(void);
int cancel_auto_timer(void);
void set_sensor_monitor_timer(int sec);
void set_detect_dbus_ready_timer(int sec);
void repeat_detect_dbus_ready_timer(int sec);
void repeat_sensor_monitor_timer(void);
int cancel_sensor_monitor_timer(void);
void setupPowerMatch(const std::shared_ptr<sdbusplus::asio::connection>& conn);
void forcePowerOff(const std::shared_ptr<sdbusplus::asio::connection>& conn);
std::map<std::string, FanConfig> buildFanDataMap(const nlohmann::json& data);
std::map<std::string, TempConfig> buildTempDataMap(const nlohmann::json& data);
static int loadConfigValues();
bool string_starts_with(const std::string& fullString, const std::string& prefix);
double sysFileRead(std::string  originalPath );
double dbusPropertyRead(std::string objectPath);
double readTempValue(std::string readPath, std::string serviceName);
double readFanTachValue(std::string readPath, std::string serviceName);
int sysPwmFileWrite(std::string originalPath,double value);
int dbusPwmWrite(std::string writePath,double value);
int writePwm(std::string writePath, double value);
int writeAllFansPwm(double value);
int writeDefaultPwm(void);
void cycle_auto_fan_control (const boost::system::error_code& ec);
void cycle_sensor_monitor(const boost::system::error_code& ec);
int setFanControlMode(std::string mode);

/*=============================================================================*/





namespace dbus
{

namespace utility
{

// clang-format off
using DbusVariantType = std::variant<
    std::vector<std::tuple<std::string, std::string, std::string>>,
    std::vector<std::string>,
    std::vector<double>,
    std::string,
    int64_t,
    uint64_t,
    double,
    int32_t,
    uint32_t,
    int16_t,
    uint16_t,
    uint8_t,
    bool,
    sdbusplus::message::unix_fd,
    std::vector<uint32_t>,
    std::vector<uint16_t>,
    sdbusplus::message::object_path,
    std::tuple<uint64_t, std::vector<std::tuple<std::string, std::string, double, uint64_t>>>,
    std::vector<std::tuple<std::string, std::string>>,
    std::vector<std::tuple<uint32_t, std::vector<uint32_t>>>,
    std::vector<std::tuple<uint32_t, size_t>>,
    std::vector<std::tuple<sdbusplus::message::object_path, std::string,
                           std::string, std::string>>
 >;
}
}





} // namespace fan_control
