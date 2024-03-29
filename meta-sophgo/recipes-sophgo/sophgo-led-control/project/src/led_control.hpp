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

namespace led_control
{

namespace fs = std::filesystem;

struct SysAlarmLed {
    std::string type;
    int index;
    bool isPowerOn; // 只对输入类型有效
    std::string dbusName;
    std::string dbusPath;
    std::string dbusIntf;
    std::map<std::string, bool> dbusProperty; // 只对输入类型有效
    int DutyOn; // 只对输出类型有效
    int Period; // 只对输出类型有效
    std::string State; // 只对输出类型有效

    SysAlarmLed() = default;
    SysAlarmLed(const nlohmann::json& j) {
        type = j.at("type").get<std::string>();
        index = j.at("index").get<int>();
        dbusName = j.at("dbusName").get<std::string>();
        dbusPath = j.at("dbusPath").get<std::string>();
        dbusIntf = j.at("dbusIntf").get<std::string>();

        if (type == "input") {
            isPowerOn = j.at("isPowerOn").get<bool>();
            // for (const auto& prop : item["dbusProperty"]) {
            //     dbusProperty.push_back(prop);
            // }
            auto props = j.at("dbusProperty").get<std::map<std::string, bool>>();
            for (const auto& prop : props) {
                dbusProperty[prop.first] = false/* prop.second */;
            }
        } else {
            DutyOn = j.at("DutyOn").get<unsigned char>();
            Period = j.at("Period").get<unsigned short>();
            State = j.at("State").get<std::string>();
        }
    }
};

/*******************************************************************************
 *  Function declaration
*/
void buildSysLedData(const nlohmann::json& data);
static int loadConfigValues();
void setupPowerMatch(const std::shared_ptr<sdbusplus::asio::connection>& conn);
void asyn_monitor_sys_state (const std::shared_ptr<sdbusplus::asio::connection>& conn);
void turn_on_sys_alarm_led(void);
void turn_off_sys_alarm_led(void);
void cycle_sys_led_control (const boost::system::error_code& ec);
void set_auto_timer(int sec);
void repeat_auto_timer(void);
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





} // namespace led_control
