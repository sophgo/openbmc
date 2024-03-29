/*
// Copyright (c) 2018-2019 Intel Corporation
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//      http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
*/
#include "led_control.hpp"
#include <systemd/sd-bus.h>
#include <sys/sysinfo.h>
#include <systemd/sd-journal.h>

#include <boost/asio/io_service.hpp>
#include <boost/asio/posix/stream_descriptor.hpp>
#include <boost/asio/steady_timer.hpp>
#include <boost/container/flat_map.hpp>
#include <boost/container/flat_set.hpp>
#include <gpiod.hpp>
#include <nlohmann/json.hpp>
#include <phosphor-logging/lg2.hpp>
#include <sdbusplus/asio/object_server.hpp>
#include <sdbusplus/bus/match.hpp>
#include <sdbusplus/bus.hpp>
#include <sdbusplus/asio/property.hpp>
#include <sdbusplus/message/types.hpp>
#include <sdbusplus/message/append.hpp>
#include <sdbusplus/message/native_types.hpp>
#include <sdbusplus/message/read.hpp>
 #include <boost/date_time/posix_time/posix_time.hpp>
 #include <boost/thread/thread.hpp>

 #include <iostream>
 #include <boost/asio.hpp>
 #include <boost/thread.hpp>
 #include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/bind/bind.hpp>
#include <boost/asio/placeholders.hpp>

#include <filesystem>
#include <fstream>
#include <string_view>

#include <boost/bind/placeholders.hpp>


#include <thread>

#include <phosphor-logging/elog-errors.hpp>
#include <phosphor-logging/elog.hpp>
#include <phosphor-logging/lg2.hpp>
#include <sdbusplus/exception.hpp>
#include <xyz/openbmc_project/Sensor/Device/error.hpp>

#include <chrono>

namespace power
{
const static constexpr char* busname = "xyz.openbmc_project.State.Host";
const static constexpr char* interface = "xyz.openbmc_project.State.Host";
const static constexpr char* path = "/xyz/openbmc_project/state/host0";
const static constexpr char* property = "CurrentHostState";
} // namespace power

namespace ledDbus
{
const static constexpr char* interface = "xyz.openbmc_project.Led.Physical";
const static constexpr char* DutyOnProperty = "DutyOn";
const static constexpr char* PeriodProperty = "Period";
const static constexpr char* StateProperty = "State";
const static constexpr char* on = "xyz.openbmc_project.Led.Physical.Action.On";
const static constexpr char* off = "xyz.openbmc_project.Led.Physical.Action.Off";
} // namespace ledDbus


namespace properties
{
constexpr const char* interface = "org.freedesktop.DBus.Properties";
constexpr const char* get = "Get";
constexpr const char* set = "Set";
}// namespace properties

namespace led_control
{

static boost::asio::io_service io;
std::shared_ptr<sdbusplus::asio::connection> conn;

bool g_isPowerOn = false;
std::atomic<int> sys_fault_count(0);

std::chrono::high_resolution_clock::time_point g_time_point;
/*******************************************************************************
 * D-bus
*/
static std::unique_ptr<sdbusplus::bus::match_t> powerMatch = nullptr;
static std::string fanControlDbusName = "xyz.openbmc_project.sophgo.LED.control";
static std::shared_ptr<sdbusplus::asio::dbus_interface> ledControlInterface;


// std::string frontpanel_warning_led_trigger="/sys/class/leds/frontpanel_warning_led/trigger"
// std::string frontpanel_warning_led_brightness="/sys/class/leds/frontpanel_warning_led/brightness"
/*=============================================================================*/

/*******************************************************************************
 * data
*/
std::map<int, SysAlarmLed> g_sysAlarmLedMap   = {};
std::map<int, std::unique_ptr<sdbusplus::bus::match_t>> g_sysAlarmMatch   = {};
/*=============================================================================*/

/*******************************************************************************
 * Timers
*/
static boost::asio::deadline_timer sysAlarmLedControlTimer(io);
/*=============================================================================*/


/*******************************************************************************
 * build data
*/
void buildSysLedData(const nlohmann::json& data)
{
    std::cout << "0000" << std::endl;
    std::cout.flush();
    std::map<int, SysAlarmLed> sysAlarmLedMap;
    for (const auto& item : data) {

        SysAlarmLed sysAlarmLed(item);
        std::cout << "index=" << sysAlarmLed.index << std::endl;
        g_sysAlarmLedMap[sysAlarmLed.index] = sysAlarmLed;
        g_sysAlarmMatch[sysAlarmLed.index] = nullptr;
    }
}
static int loadConfigValues()
{
    const std::string configFilePath = "/usr/share/sophgo-led-control/led-config.json";
    std::ifstream configFile(configFilePath.c_str());
    if (!configFile.is_open())
    {
        lg2::error("loadConfigValues: Cannot open config path \'{PATH}\'",
                   "PATH", configFilePath);
        return -1;
    }

    auto jsonData = nlohmann::json::parse(configFile, nullptr, true, true);

    if (jsonData.is_discarded())
    {
        lg2::error("Power config readings JSON parser failure");
        return -1;
    }

    auto sysLedData = jsonData["sysAlarmLed"];

    buildSysLedData(sysLedData);

    return 0;
}
/*=============================================================================*/


/*******************************************************************************
 * monitor power state
*/
void setupPowerMatch(const std::shared_ptr<sdbusplus::asio::connection>& conn)
{
    powerMatch = std::make_unique<sdbusplus::bus::match::match>(
        static_cast<sdbusplus::bus::bus&>(*conn),
        "type='signal',interface='" + std::string(properties::interface) +
            "',path='" + std::string(power::path) + "',arg0='" +
            std::string(power::interface) + "'",
        [](sdbusplus::message::message& message) {
            std::string objectName;
            boost::container::flat_map<std::string, std::variant<std::string>>
                values;
            message.read(objectName, values);
            auto findState = values.find(power::property);
            if (findState != values.end())
            {
                g_isPowerOn = std::get<std::string>(findState->second).ends_with("Running");
                if (g_isPowerOn) {
                    std::cout << "power on." << "\n";
                    std::cout.flush();
                } else {
                    std::cout << "power off." << "\n";
                    std::cout.flush();
                }
            }
        });

    conn->async_method_call(
        [](boost::system::error_code ec,
           const std::variant<std::string>& state) {
            if (ec)
            {
                std::cout << "power state get error" << "\n";
                return;
            }
            g_isPowerOn = std::get<std::string>(state).ends_with("Running");
            if (g_isPowerOn) {
                std::cout << "power on 00." << "\n";
                std::cout.flush();
            } else {
                std::cout << "power off 00." << "\n";
                std::cout.flush();
            }

        },
        power::busname, power::path, properties::interface, properties::get,
        power::interface, power::property);
}
/*=============================================================================*/



/*******************************************************************************
 * monitor sys state
*/
void asyn_monitor_sys_state (const std::shared_ptr<sdbusplus::asio::connection>& conn)
{
    for (/* const */ auto& [index, sysLedData] : g_sysAlarmLedMap) {
        if (sysLedData.type == "input") {
            g_sysAlarmMatch[index]= std::make_unique<sdbusplus::bus::match::match>(
            static_cast<sdbusplus::bus::bus&>(*conn),
            "type='signal',interface='" + std::string(properties::interface) +
                "',path='" + std::string(sysLedData.dbusPath) + "',arg0='" +
                std::string(sysLedData.dbusIntf) + "'",
            [index,&sysLedData](sdbusplus::message::message& message) {
                std::string objectName;
                bool temp_state;
                boost::container::flat_map<std::string, std::variant<bool>> values;

                if ((sysLedData.isPowerOn) && (!g_isPowerOn)) {
                    return;
                }
                message.read(objectName, values);
                for (/* const */ auto& [propName, propValue] : sysLedData.dbusProperty) {
                    auto findState = values.find(propName);
                    if (findState != values.end()) {
                        bool temp_state = std::get<bool>(findState->second);
                        if ((temp_state) && (temp_state != sysLedData.dbusProperty[propName])) {
                            sysLedData.dbusProperty[propName] = temp_state;
                            sys_fault_count++;
                            std::cout << index << " - " << propName << " property : ok->error." << std::endl;
                        } else if ((!temp_state) && (temp_state != sysLedData.dbusProperty[propName])) {
                            sysLedData.dbusProperty[propName] = temp_state;
                            sys_fault_count--;
                            std::cout << index << " - " << propName << " property : error->ok." << std::endl;
                        }

                    }
                }

            });
        }
    }
}
/*=============================================================================*/



/*******************************************************************************
 * cycle function
*/

void turn_on_sys_alarm_led(void)
{
    for (/* const */ auto& [index, sysLedData] : g_sysAlarmLedMap) {
        if (sysLedData.type == "output") {
            conn->async_method_call(
                [](const boost::system::error_code ec) {
                    if (ec)
                    {
                        std::cout << "D-Bus responses error: " << ec.message() << std::endl;
                        return;
                    }
                    // std::cout << "Turn on sys alarm led." << std::endl;
                },
                sysLedData.dbusName,
                sysLedData.dbusPath,
                properties::interface, properties::set,
                sysLedData.dbusIntf,
                ledDbus::StateProperty,
                dbus::utility::DbusVariantType{ledDbus::on});
            unsigned char temp_DutyOn = (unsigned char)sysLedData.DutyOn;
            conn->async_method_call(
                [](const boost::system::error_code ec) {
                    if (ec)
                    {
                        std::cout << "D-Bus responses error: " << ec.message() << std::endl;
                        return;
                    }
                    // std::cout << "Turn on sys alarm led." << std::endl;
                },
                sysLedData.dbusName,
                sysLedData.dbusPath,
                properties::interface, properties::set,
                sysLedData.dbusIntf,
                ledDbus::DutyOnProperty,
                dbus::utility::DbusVariantType{temp_DutyOn});

            unsigned short temp_Period = (unsigned short)sysLedData.Period;
            conn->async_method_call(
                [](const boost::system::error_code ec) {
                    if (ec)
                    {
                        std::cout << "D-Bus responses error: " << ec.message() << std::endl;
                        return;
                    }
                    // std::cout << "Turn on sys alarm led." << std::endl;
                },
                sysLedData.dbusName,
                sysLedData.dbusPath,
                properties::interface, properties::set,
                sysLedData.dbusIntf,
                ledDbus::PeriodProperty,
                dbus::utility::DbusVariantType{temp_Period});
        }
    }
}

void turn_off_sys_alarm_led(void)
{
    for (/* const */ auto& [index, sysLedData] : g_sysAlarmLedMap) {
        if (sysLedData.type == "output") {
            conn->async_method_call(
                [](const boost::system::error_code ec) {
                    if (ec)
                    {
                        std::cout << "D-Bus responses error: " << ec.message() << std::endl;
                        return;
                    }
                    // std::cout << "Turn on sys alarm led." << std::endl;
                },
                sysLedData.dbusName,
                sysLedData.dbusPath,
                properties::interface, properties::set,
                sysLedData.dbusIntf,
                ledDbus::StateProperty,
                dbus::utility::DbusVariantType{ledDbus::off});

            unsigned char temp_DutyOn = 0;
            conn->async_method_call(
                [](const boost::system::error_code ec) {
                    if (ec)
                    {
                        std::cout << "D-Bus responses error: " << ec.message() << std::endl;
                        return;
                    }
                    // std::cout << "Turn on sys alarm led." << std::endl;
                },
                sysLedData.dbusName,
                sysLedData.dbusPath,
                properties::interface, properties::set,
                sysLedData.dbusIntf,
                ledDbus::DutyOnProperty,
                dbus::utility::DbusVariantType{temp_DutyOn});
            unsigned short temp_Period = 0;
            conn->async_method_call(
                [](const boost::system::error_code ec) {
                    if (ec)
                    {
                        std::cout << "D-Bus responses error: " << ec.message() << std::endl;
                        return;
                    }
                    // std::cout << "Turn on sys alarm led." << std::endl;
                },
                sysLedData.dbusName,
                sysLedData.dbusPath,
                properties::interface, properties::set,
                sysLedData.dbusIntf,
                ledDbus::PeriodProperty,
                dbus::utility::DbusVariantType{temp_Period});
        }
    }
}

void cycle_sys_led_control (const boost::system::error_code& ec)
{
    if (sys_fault_count.load() > 0) {
        // std::cout << "sys_fault_count: " << sys_fault_count.load() << std::endl;
        turn_on_sys_alarm_led();
    } else {
        // std::cout << "sys_fault_count: " << sys_fault_count.load() << std::endl;
        turn_off_sys_alarm_led();
    }
    repeat_auto_timer();
}

void set_auto_timer(int sec)
{
    sysAlarmLedControlTimer.expires_from_now( boost::posix_time::seconds(sec));
    sysAlarmLedControlTimer.async_wait(cycle_sys_led_control);
    std::cout << "Enable cycle timer!" << std::endl;
}

void repeat_auto_timer(void)
{
    sysAlarmLedControlTimer.expires_at(sysAlarmLedControlTimer.expires_at() + boost::posix_time::seconds(3));
    sysAlarmLedControlTimer.async_wait(cycle_sys_led_control);
}
/*=============================================================================*/


} // namespace led_control





int main(int argc, char* argv[])
{
    using namespace power;
    using namespace properties;
    using namespace led_control;
    using namespace boost::placeholders;

    // Load json config file
    if (loadConfigValues() == -1)
    {
        lg2::error(" Error in Parsing...");

        return -1;
    }


    conn = std::make_shared<sdbusplus::asio::connection>(io);

    setupPowerMatch(conn);
    asyn_monitor_sys_state(conn);
    set_auto_timer(10);

    io.run();

    return 0;
}
