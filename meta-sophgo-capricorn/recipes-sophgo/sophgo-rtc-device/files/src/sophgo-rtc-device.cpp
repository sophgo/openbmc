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
#include "sophgo-rtc-device.hpp"
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

namespace properties
{
constexpr const char* interface = "org.freedesktop.DBus.Properties";
constexpr const char* get = "Get";
constexpr const char* set = "Set";
}

namespace rtc_device
{


static boost::asio::io_service io;
std::shared_ptr<sdbusplus::asio::connection> conn;

static std::string node = "0";
static const std::string appName = "sophgo-rtc-device";
bool isPowerOn = false;


std::map<std::string, std::string> deviceInfoMap = {
    {"bus",              "i2c-12"},
    {"addr",               "0x68"},
    {"compatible",       "ds1307"}
};



static std::unique_ptr<sdbusplus::bus::match_t> powerMatch = nullptr;

/*******************************************************************************
 * rtc device add and delete
*/

int addRtcDevicetoi2c(void)
{
    std::string path = "/sys/bus/i2c/devices/" + deviceInfoMap["bus"] + "/new_device";
    std::string value   =  deviceInfoMap["compatible"] + " " + deviceInfoMap["addr"];
    std::ofstream outfile(path);

    if (outfile.is_open()) {
        outfile << value << std::endl;
        outfile.close();
        std::cout << "Data written to file successfully." << std::endl;
    } else {
        std::cerr << "Unable to open file for writing." << std::endl;
        return 1;
    }

    return 0;
}

int deleteRtcDevice(void)
{
    std::string path    = "/sys/bus/i2c/devices/" + deviceInfoMap["bus"] + "/delete_device";
    std::string value   = deviceInfoMap["addr"];

    std::ofstream outfile(path);

    if (outfile.is_open()) {
        outfile << value << std::endl;
        outfile.close();
        std::cout << "Data written to file successfully." << std::endl;
    } else {
        std::cerr << "Unable to open file for writing." << std::endl;
        return 1;
    }

    return 0;
}



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
                /* isPowerOn = boost::ends_with(
                    std::get<std::string>(findState->second), "Running"); */
                isPowerOn = std::get<std::string>(findState->second).ends_with("Running");
                if (isPowerOn) {
                    std::cout << "power on." << "\n";
                    std::cout.flush();
                    deleteRtcDevice();

                } else {
                    std::cout << "power off." << "\n";
                    std::cout.flush();
                    addRtcDevicetoi2c();
                }
            }
        });

#if 1
    conn->async_method_call(
        [](boost::system::error_code ec,
           const std::variant<std::string>& state) {
            if (ec)
            {
                std::cout << "power state get error" << "\n";
                return;
            }
            isPowerOn = std::get<std::string>(state).ends_with("Running");
            if (isPowerOn) {
                std::cout << "power on 00." << "\n";
                std::cout.flush();
                deleteRtcDevice();
            } else {
                std::cout << "power off 00." << "\n";
                std::cout.flush();
                addRtcDevicetoi2c();
            }

        },
        power::busname, power::path, properties::interface, properties::get,
        power::interface, power::property);
#else
    auto bus = sdbusplus::bus::new_default();
    auto method = bus.new_method_call(power::busname,
                                        power::path,
                                        properties::interface,
                                        properties::get);
    method.append(power::interface, power::property);

    try {
        auto reply = bus.call(method);
        std::variant<std::string> value;
        reply.read(value);
        isPowerOn = std::get<std::string>(value).ends_with("Running");
        if (isPowerOn) {
            std::cout << "power on 000." << "\n";
        } else {
            std::cout << "power off 000." << "\n";
        }
    } catch (const sdbusplus::exception::exception& e) {
        std::cerr << "Failed to read the power state: " << e.what() << std::endl;
        return;
    }

#endif
}

static int loadConfigValues()
{
    const std::string configFilePath = "/usr/share/sophgo-rtc-device/rtc-device-config.json";
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
    auto deviceInfo     = jsonData["deviceInfo"];



    for (auto& [key, value] : deviceInfoMap)
    {
        if (deviceInfoMap.contains(key.c_str()))
        {
            value = deviceInfoMap[key.c_str()];
            std::cout << "key= " << key << "value" << value << std::endl;
        }
    }
    return 0;
}


} // namespace rtc_device





int main(int argc, char* argv[])
{
    using namespace power;
    using namespace rtc_device;

    conn = std::make_shared<sdbusplus::asio::connection>(io);

    // Load json config file
    if (loadConfigValues() == -1)
    {
        lg2::error(" Error in Parsing...");
        return -1;
    }

    setupPowerMatch(conn);

    io.run();

    return 0;
}
