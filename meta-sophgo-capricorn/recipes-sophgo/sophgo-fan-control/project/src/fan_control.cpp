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
#include "fan_control.hpp"
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


namespace power_control
{
const static constexpr char* busname = "xyz.openbmc_project.State.Chassis";
const static constexpr char* path = "/xyz/openbmc_project/state/chassis0";
const static constexpr char* interface = "xyz.openbmc_project.State.Chassis";
const static constexpr char* property = "RequestedPowerTransition";
const static constexpr char* powerOff = "xyz.openbmc_project.State.Chassis.Transition.Off";
} // namespace power_control

namespace properties
{
constexpr const char* interface = "org.freedesktop.DBus.Properties";
constexpr const char* get = "Get";
constexpr const char* set = "Set";
}

namespace fan_control
{

#define PWM_MAX  255
static boost::asio::io_service io;
std::shared_ptr<sdbusplus::asio::connection> conn;

static std::string node = "0";
static const std::string appName = "sophgo-fan-control";
bool isPowerOn = false;
bool isAutoStart = true;
bool isNeededCheckDbus = true;
std::atomic<int> auto_timer_flag(false);
std::atomic<int> sensor_timer_flag(false);
std::atomic<int> is_first_time_poweron(true);
std::atomic<double> g_outputRpmTar(45);
double g_def_fan_speed = 45;
double g_outputRpmPre = 0;
double g_outputRpmCru = 0;
std::string g_fanControlMode = "auto";
double g_aicard_temp = -1;

static constexpr auto platform = "/sys/devices/platform/";
namespace fs = std::filesystem;

std::chrono::high_resolution_clock::time_point g_time_point;
std::chrono::high_resolution_clock::time_point g_time_powerOn_point;
int wait_count = 0;

double Ai_templimit=127;
/*******************************************************************************
 * D-bus
*/
static std::unique_ptr<sdbusplus::bus::match_t> powerMatch = nullptr;
static std::string fanControlDbusName = "xyz.openbmc_project.FanControl";
static std::shared_ptr<sdbusplus::asio::dbus_interface> fanControlInterface;
static std::shared_ptr<sdbusplus::asio::dbus_interface> aiCardTempInterface;

const std::string sensorInterface = "xyz.openbmc_project.Sensor.Value";
const std::string sensorProperty  = "Value";

const std::string pwmControlInterface = "xyz.openbmc_project.Control.FanPwm";
const std::string pwmControlProperty  = "Target";
/*=============================================================================*/





/*******************************************************************************
 * map
*/
std::map<std::string, FanConfig> g_fanDataMap   = {};
std::map<std::string, TempConfig> g_tempDataMap = {};

std::map<std::string, int> controlParamMap = {
    {"CycleTime",         3},
    {"DutyStep",          1},
    {"FanDefSpeed",      45},
    {"FanLowSpeed",      30},
    {"FanMediumSpeed",   65},
    {"FanHighSpeed",    100},
    {"RedundantFanNum",   1}
};


std::map<std::string, std::string> controlModeMap = {
    {"low",          "FanLowSpeed"},
    {"medium",       "FanMediumSpeed"},
    {"high",         "FanHighSpeed"}
};
/*=============================================================================*/

/*******************************************************************************
 * Timers
*/
static boost::asio::deadline_timer autoControlTimer(io);
static boost::asio::deadline_timer fanStateMonitorTimer(io);
static boost::asio::deadline_timer detectDbusReadyTimer(io);
/*=============================================================================*/

/*******************************************************************************
 * dbus check
*/
bool check_service_exists(const char* service_name, const char* object_path)
{
    sd_bus *bus = nullptr;
    sd_bus_error bus_error = SD_BUS_ERROR_NULL;
    sd_bus_message *msg = nullptr;
    int ret;

    // Connect to the user or system bus
    ret = sd_bus_open_system(&bus);
    if (ret < 0) {
        std::cerr << "Failed to connect to system bus: " << strerror(-ret) << std::endl;
        goto finish;
    }


    ret = sd_bus_call_method(bus,
                             service_name, // service name
                             object_path, // object path
                             "org.freedesktop.DBus.Introspectable",
                             "Introspect",
                             &bus_error, // sd_bus_error
                             &msg, // sd_bus_message
                             ""); // input signature (empty for no input arguments)

    if (ret < 0) {
        // std::cerr << "Failed to issue method call: " << bus_error.message << std::endl;
        goto finish;
    }

finish:
    sd_bus_error_free(&bus_error);
    sd_bus_message_unref(msg);
    sd_bus_unref(bus);

    return ret < 0 ? false : true;
}



bool is_sensor_dbus_ready(void)
{
    bool isReady=false;
    for (const auto& [name, config] : g_tempDataMap) {
        if (config.isInherent) {
            if (check_service_exists(config.dbusName.c_str(), config.readPath.c_str())) {
                std::cout << "ready" << std::endl;
                isReady = true;
                break;
            }
        }
    }
    return (isReady);
}

bool is_needed_to_check_dbus(void)
{
    bool isNeeded=false;
    for (const auto& [name, config] : g_tempDataMap) {
        if (config.isInherent) {
            if (string_starts_with(config.readPath, "/xyz/openbmc_project/")) {
                isNeeded = true;
                break;
            }
        }
    }
    return (isNeeded);
}

void async_wait_sensor_dbus_ready(const boost::system::error_code& ec)
{
    if (is_first_time_poweron.load()) {
        ;
    }
    if (isNeededCheckDbus) {
        wait_count++;
        std::cout << "wait_count:" << wait_count <<std::endl;
        if (is_sensor_dbus_ready()) {
            if (isAutoControlMode()) {
                //启动定时器
                set_auto_timer(1);
            }
            set_sensor_monitor_timer(1);
        } else {
            std::cout << "repeat" <<std::endl;
            set_detect_dbus_ready_timer(3);
        }
    } else {
        if (isAutoControlMode()) {
            //启动定时器
            set_auto_timer(1);
        }
        set_sensor_monitor_timer(1);
    }
}

/*=============================================================================*/


/*******************************************************************************
 * tools
*/
bool isAutoControlMode(void)
{
    if (g_fanControlMode == "auto")
        return true;

    return false;
}


std::string FixupPath(std::string original)
{
    std::string::size_type n, x;

    /* TODO: Consider the merits of using regex for this. */
    n = original.find("**");
    x = original.find(platform);

    if ((n != std::string::npos) && (x != std::string::npos))
    {
        /* This path has some missing pieces and we support it. */
        std::string base = original.substr(0, n);
        std::string fldr;
        std::string f = original.substr(n + 2, original.size() - (n + 2));

        /* Equivalent to glob and grab 0th entry. */
        for (const auto& folder : fs::directory_iterator(base))
        {
            fldr = folder.path();
            break;
        }

        if (!fldr.length())
        {
            return original;
        }

        return fldr + f;
    }
    else
    {
        /* It'll throw an exception when we use it if it's still bad. */
        return original;
    }
}


int percentageToActual(double percentage, double baseValue)
{
    return static_cast<int>(std::round((percentage / 100.0) * baseValue));
}
bool containsIgnoreCase(const std::string& haystack, const std::string& needle)
{
    // 创建副本并转换为小写
    std::string haystackLower = haystack;
    std::string needleLower = needle;
    std::transform(haystackLower.begin(), haystackLower.end(), haystackLower.begin(), ::tolower);
    std::transform(needleLower.begin(), needleLower.end(), needleLower.begin(), ::tolower);

    // 查找小写字符串
    return haystackLower.find(needleLower) != std::string::npos;
}
/*=============================================================================*/




/*******************************************************************************
 * Timers
*/
void set_auto_timer(int sec)
{
    if (auto_timer_flag.load()) {
        std::cout << "Auto timer is already enabled!" << std::endl;
        return;
    }
    isAutoStart = true;
    autoControlTimer.expires_from_now( boost::posix_time::seconds(sec));
    autoControlTimer.async_wait(cycle_auto_fan_control);
    auto_timer_flag.store(true);
    std::cout << "Enable auto timer!" << std::endl;
}

void repeat_auto_timer(void)
{
    autoControlTimer.expires_at(autoControlTimer.expires_at() + boost::posix_time::seconds(controlParamMap["CycleTime"]));
    autoControlTimer.async_wait(cycle_auto_fan_control);
}


int cancel_auto_timer(void)
{
    boost::system::error_code ec;
    isAutoStart = true;
    autoControlTimer.cancel(ec);

    // 检查是否发生错误
    if (ec) {
        // 处理错误
        std::cerr << "Error on canceling the timer: " << ec.message() << std::endl;
        return -1;
    } else {
        // 定时器取消成功
        std::cout << "Auto timer cancelled successfully." << std::endl;
        auto_timer_flag.store(false);
        return 0;
    }
}


void set_sensor_monitor_timer(int sec)
{
    if (sensor_timer_flag.load()) {
        std::cout << "Monitor timer is already enabled!" << std::endl;
        return;
    }

    fanStateMonitorTimer.expires_from_now( boost::posix_time::seconds(sec));
    fanStateMonitorTimer.async_wait(cycle_sensor_monitor);
    sensor_timer_flag.store(true);
    std::cout << "Enable monitor timer!" << std::endl;
}

void repeat_sensor_monitor_timer(void)
{
    fanStateMonitorTimer.expires_at(fanStateMonitorTimer.expires_at() + boost::posix_time::seconds(5));
    fanStateMonitorTimer.async_wait(cycle_sensor_monitor);
}

int cancel_sensor_monitor_timer(void)
{
    boost::system::error_code ec;
    fanStateMonitorTimer.cancel(ec);

    // 检查是否发生错误
    if (ec) {
        // 处理错误
        std::cerr << "Error on canceling the timer: " << ec.message() << std::endl;
        return -1;
    } else {
        // 定时器取消成功
        std::cout << "Monitor timer cancelled successfully." << std::endl;
        sensor_timer_flag.store(false);
        return 0;
    }
}


void set_detect_dbus_ready_timer(int sec)
{
    detectDbusReadyTimer.expires_from_now( boost::posix_time::seconds(sec));
    detectDbusReadyTimer.async_wait(async_wait_sensor_dbus_ready);
}


void repeat_detect_dbus_ready_timer(int sec)
{
    detectDbusReadyTimer.expires_at(detectDbusReadyTimer.expires_at() + boost::posix_time::seconds(sec));
    detectDbusReadyTimer.async_wait(async_wait_sensor_dbus_ready);
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
                /* isPowerOn = boost::ends_with(
                    std::get<std::string>(findState->second), "Running"); */
                isPowerOn = std::get<std::string>(findState->second).ends_with("Running");
                if (isPowerOn) {
                    std::cout << "power on." << "\n";
                    std::cout.flush();
                    set_detect_dbus_ready_timer(0);
                    g_time_powerOn_point = std::chrono::high_resolution_clock::now();

                } else {
                    std::cout << "power off." << "\n";
                        //关闭定时器，按默认风速控制
                    cancel_auto_timer();
                    cancel_sensor_monitor_timer();
                    g_aicard_temp = -1;
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
                set_detect_dbus_ready_timer(0);
                g_time_powerOn_point = std::chrono::high_resolution_clock::now();
            } else {
                std::cout << "power off 00." << "\n";
                std::cout.flush();
                g_aicard_temp = -1;
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
            if (isAutoControlMode()) {
                //启动定时器
                writeDefaultPwm();
                set_auto_timer(10);
            }
            set_sensor_monitor_timer();
        } else {
            std::cout << "power off 000." << "\n";
        }
    } catch (const sdbusplus::exception::exception& e) {
        std::cerr << "Failed to read the power state: " << e.what() << std::endl;
        return;
    }

#endif
}


void forcePowerOff(const std::shared_ptr<sdbusplus::asio::connection>& conn)
{
    conn->async_method_call(
        [](const boost::system::error_code ec) {
            if (ec)
            {
                std::cout << "Force power off D-Bus responses error: " << std::endl;
                return;
            }
            std::cout << "Force power off" << std::endl;
        },
        power_control::busname,
        power_control::path,
        properties::interface, properties::set,
        power_control::interface,
        power_control::property,
        dbus::utility::DbusVariantType{power_control::powerOff});

        cancel_sensor_monitor_timer();
        cancel_auto_timer();
}


/*=============================================================================*/



/*******************************************************************************
 * build data map
*/
std::map<std::string, FanConfig> buildFanDataMap(const nlohmann::json& data)
{
    std::map<std::string, FanConfig> fanDataMap;
    for (const auto& item : data) {
        FanConfig fanConfig(item);
        fanDataMap[fanConfig.name] = fanConfig;
    }
    for (const auto& [name, config] : fanDataMap) {
        std::cout << "Name: " << name << ", ReadPath: " << config.readPath << ", WritePath: " << config.writePath
                  << ", Min: " << config.min << ", Max: " << config.max << std::endl;
    }
    return fanDataMap;
}

std::map<std::string, TempConfig> buildTempDataMap(const nlohmann::json& data)
{
    std::map<std::string, TempConfig> tempDataMap;
    for (const auto& item : data) {
        TempConfig tempConfig(item);
        tempDataMap[tempConfig.name] = tempConfig;
    }

    for (const auto& [name, config] : tempDataMap) {
        std::cout << "Name: " << name << ", ReadPath: " << config.readPath << ", minSetPoint: " << config.minSetPoint
                  << ", isInherent: " << config.isInherent << std::endl;
        std::cout << "INPUT" << std::endl;
        for (const auto& [index, value] : config.input) {
            std::cout << "index: " << index << "; " << "value" << value << std::endl;
        }
        std::cout << "OUTPUT" << std::endl;
        for (const auto& [index, value] : config.output) {
            std::cout << "index: " << index << "; " << "value" << value << std::endl;
        }
    }
    return tempDataMap;
}
/*=============================================================================*/



static int loadConfigValues()
{
    const std::string configFilePath = "/usr/share/sophgo-fan-control/fan-config.json";
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

    auto fanConfigs       = jsonData["fans"];
    auto tempConfigs      = jsonData["temps"];
    auto controlParam     = jsonData["controlParams"];

    g_fanDataMap  = buildFanDataMap(fanConfigs);
    g_tempDataMap = buildTempDataMap(tempConfigs);

    for (auto& [key, value] : controlParamMap)
    {
        if (controlParam.contains(key.c_str()))
        {
            value = controlParam[key.c_str()];
            std::cout << "key= " << key << "value" << value << std::endl;
        }
    }
    return 0;
}

/*******************************************************************************
 *  read function
*/
bool string_starts_with(const std::string& fullString, const std::string& prefix)
{
    return fullString.length() >= prefix.length() &&
           fullString.compare(0, prefix.length(), prefix) == 0;
}

double sysFileRead(std::string  originalPath )
{
    std::string readPath = FixupPath(originalPath);
    // std::cout << "sys readpath= " << readPath << std::endl;
    double value = 0;
    std::ifstream ifs;
    ifs.open(readPath);
    if (!ifs) {
        std::cerr << "Error opening file: " << readPath << std::endl;
        return -1;
    }
    ifs >> value;
    ifs.close();
    return value;
}


double dbusPropertyReadWithNoName(std::string objectPath)
{
    auto bus = sdbusplus::bus::new_default();
    double propertyValue = -1;

    auto mapper = bus.new_method_call("xyz.openbmc_project.ObjectMapper",
                                        "/xyz/openbmc_project/object_mapper",
                                        "xyz.openbmc_project.ObjectMapper",
                                        "GetObject");
    mapper.append(objectPath, std::vector<std::string>({sensorInterface}));
    // Prevent program abnormal exit when dbus service does not exist
    try {
        auto result = bus.call(mapper);

        std::map<std::string, std::vector<std::string>> objPaths;
        result.read(objPaths);
        if (objPaths.empty()) {
            std::cout << "No services found for the object path." << std::endl;
            return -1;
        }
        auto& serviceName = objPaths.begin()->first;
        // std::cout << "serviceName= " << serviceName.c_str() << " objectPath " << objectPath.c_str() << std::endl;

        auto method = bus.new_method_call(serviceName.c_str(),
                                        objectPath.c_str(),
                                        "org.freedesktop.DBus.Properties",
                                        "Get");
        // std::cout << "sensorInterface= " << sensorInterface << " sensorProperty " << sensorProperty << std::endl;
        method.append(sensorInterface, sensorProperty);

        try {
            auto reply = bus.call(method);
            std::variant<double> value;
            reply.read(value);
            propertyValue = std::get<double>(value);
        } catch (const sdbusplus::exception::exception& e) {
            std::cerr << "Failed to read the property: " << e.what() << std::endl;
            return -1;
        }
    } catch (const sdbusplus::exception::exception& e) {
            std::cerr << "Find service name error!" << std::endl;
            return -1;
    }

    return propertyValue;
}

double dbusPropertyRead(std::string objectPath, std::string serviceName)
{
    auto bus = sdbusplus::bus::new_default();
    double propertyValue = -1;

    if (serviceName != "") {
        auto method = bus.new_method_call(serviceName.c_str(),
                                        objectPath.c_str(),
                                        "org.freedesktop.DBus.Properties",
                                        "Get");
        method.append(sensorInterface, sensorProperty);

        try {
            auto reply = bus.call(method);
            std::variant<double> value;
            reply.read(value);
            propertyValue = std::get<double>(value);
        } catch (const sdbusplus::exception::exception& e) {
            // std::cerr << "Failed to read the property: " << e.what() << std::endl;
            return -1;
        }
    } else {
        propertyValue = dbusPropertyReadWithNoName(objectPath);
    }


    return propertyValue;
}

double readFanTachValue(std::string readPath, std::string serviceName)
{
    double value;
    if (string_starts_with(readPath, "/sys/")) {
        value = sysFileRead(readPath);
    } else if (string_starts_with(readPath, "/xyz/openbmc_project/")) {
        value = dbusPropertyRead(readPath, serviceName);
    }
    return value;
}


double g_aicard_temp_pre;
double readTempValue(std::string readPath, std::string serviceName)
{
    double value;
    if (string_starts_with(readPath, "/sys/")) {
        value = (sysFileRead(readPath)/1000);
    } else if (string_starts_with(readPath, "/xyz/openbmc_project/")) {
        if (string_starts_with(readPath, "/xyz/openbmc_project/sensors/temperature/AiCard_Temp")) {
            value = g_aicard_temp;
            if (g_aicard_temp_pre != g_aicard_temp) {
                std::cout << "Aicard temp : " << g_aicard_temp << "Pre" << g_aicard_temp_pre <<std::endl;
            }
            g_aicard_temp_pre = g_aicard_temp;

        } else {
            value = dbusPropertyRead(readPath, serviceName);
        }

    }
    return value;
}
/*=============================================================================*/




/*******************************************************************************
 * write
*/
int sysPwmFileWrite(std::string originalPath,double value)
{
    std::string writePath = FixupPath(originalPath);
    // std::cout << "sys writePath= " << writePath << " value= " << value << std::endl;
    std::ofstream ofs;
    ofs.open(writePath);
    if (!ofs) {
        std::cerr << "Error opening file: " << writePath << std::endl;
        return -1;
    }
    ofs << static_cast<int64_t>(value);
    ofs.close();
    return 0;
}



int dbusPwmWrite(std::string writePath,double value)
{
    auto bus = sdbusplus::bus::new_default();
    auto mapper = bus.new_method_call("xyz.openbmc_project.ObjectMapper",
                                        "/xyz/openbmc_project/object_mapper",
                                        "xyz.openbmc_project.ObjectMapper",
                                        "GetObject");
    mapper.append(writePath, std::vector<std::string>({sensorInterface}));

    try {
        auto result = bus.call(mapper);

        std::map<std::string, std::vector<std::string>> objPaths;
        result.read(objPaths);
        if (objPaths.empty()) {
            std::cout << "No services found for the object path." << std::endl;
            return -1;
        }
        auto& serviceName = objPaths.begin()->first;

        try {
            auto msg = bus.new_method_call(serviceName.c_str(), writePath.c_str(),
                                       "org.freedesktop.DBus.Properties", "Set");
            msg.append(pwmControlInterface, pwmControlProperty,
                    std::variant<double>(value));
            auto reply = bus.call(msg);
        } catch (const sdbusplus::exception::exception& e) {
            std::cerr << "Write error!" << std::endl;
            return -1;
        }
    } catch(const sdbusplus::exception::exception& e) {
        std::cerr << "Find service name error!" << std::endl;
        return -1;
    }

    return 0;
}



int writePwm(std::string writePath, double value)
{
    int res;
    if (string_starts_with(writePath, "/sys/")) {
        //sys
        res = sysPwmFileWrite(writePath, value);
    } else if (string_starts_with(writePath, "/xyz/openbmc_project/")) {
        //dbus
        res = dbusPwmWrite(writePath, value);
    }

    return res;
}

int writeAllFansPwm(double value)
{
    for (const auto& [name, config] : g_fanDataMap) {
        writePwm(config.writePath, value);
    }
    return 0;
}

int writeDefaultPwm(void)
{
    double pwm = percentageToActual(g_def_fan_speed, PWM_MAX);
    writeAllFansPwm(pwm);
    return 0;
}


/*******************************************************************************
 *  Periodic processing function
*/
void cycle_auto_fan_control (const boost::system::error_code& ec)
{
    if (ec == boost::asio::error::operation_aborted) {
        std::cout << "Auto timer is canceled!" << std::endl;
        return;
    }
    // std::cout << "g_outputRpmTar " << g_outputRpmTar.load() << std::endl;
    if ((isPowerOn) && (g_fanControlMode == "auto")) {
        if (isAutoStart) {
            g_outputRpmPre = g_outputRpmTar.load();
            g_outputRpmCru = g_outputRpmTar.load();
            isAutoStart    = false;
        } else {
            if (g_outputRpmPre < g_outputRpmTar.load()) {
                g_outputRpmCru = g_outputRpmPre + controlParamMap["DutyStep"];
            } else if (g_outputRpmPre > g_outputRpmTar.load()) {
                if ((g_outputRpmTar.load() + controlParamMap["DutyStep"]) >= g_outputRpmPre) {
                    g_outputRpmPre = g_outputRpmTar.load();
                } else {
                    g_outputRpmCru = g_outputRpmPre - controlParamMap["DutyStep"];
                }
            } else if (g_outputRpmPre == g_outputRpmTar.load()) {
                g_outputRpmCru = g_outputRpmPre;
            }
        }
        writeAllFansPwm(percentageToActual(g_outputRpmCru, PWM_MAX));
        g_outputRpmPre = g_outputRpmCru;
    } else {
        std::cout << "3s test" << std::endl;
    }
    repeat_auto_timer();
}




void cycle_sensor_monitor(const boost::system::error_code& ec)
{
    int fanFaultCount = 0;
    double tempValue = 0;
    double outputMax = 0;
    double output    = 100;

    if (ec == boost::asio::error::operation_aborted) {
        std::cout << "Sensor monitor timer is canceled!" << std::endl;
        return;
    }


    std::chrono::high_resolution_clock::time_point end = std::chrono::high_resolution_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - g_time_point);
    // std::cout << "Time taken: " << elapsed.count() << " milliseconds." << std::endl;


    if (isPowerOn) {
        for (/* const */ auto& [name, config] : g_fanDataMap) {
            double value = readFanTachValue(config.readPath,config.dbusName);
            if(value <= 0) {
                fanFaultCount ++;
                if (config.state) {
                    using FanFault = sdbusplus::xyz::openbmc_project::Sensor::Device::Error::FanFault;
                    phosphor::logging::report<FanFault>();
                    std::cout << name << " fault." << std::endl;
                    config.state = false;
                }
            } else {
                config.state = true;
            }
        }

        if (fanFaultCount) {
            outputMax = 100;
            for (/* const */ auto& [name, config] : g_tempDataMap) {
                tempValue = readTempValue(config.readPath,config.dbusName);
                if (tempValue >= config.fanFaultThreshold) {
                    std::cout << name << ": Fan fault,high temperature,force power off." << std::endl;
                    forcePowerOff(conn);
                    break;
                }
            }
        } else {
            for (/* const */ auto& [name, config] : g_tempDataMap) {
                double value = readTempValue(config.readPath,config.dbusName);
                if(value < 0) {
                    if (config.isInherent) {
                        outputMax = 100;
                        if (config.accessibility) {
                            //report log
                            using readTempError = sdbusplus::xyz::openbmc_project::Sensor::Device::Error::TempReadFailure;
                            phosphor::logging::report<readTempError>();
                            std::cout << "Read temperature " << name << " error." << std::endl;
                            config.accessibility = false;
                        }
                        // break;
                    } else {
                        config.accessibility = false;
                    }

                } else if (value >= config.shutdownThreshold) {
                    std::cout << name << " high temperature " << value << ",force power off." << std::endl;
                    outputMax = 100;
                    forcePowerOff(conn);
                    /* if (!config.state)  */{
                        config.accessibility = true;
                    }
                    break;
                } else {
                    auto index = config.findInputIndex(value);
                    output = config.getOutputValue(index);
                    outputMax = (output > outputMax) ? output : outputMax;
                    /* if (!config.state) */ {
                        config.accessibility = true;
                    }
                }
            }
        }
        g_outputRpmTar.store(outputMax);
        // std::cout << "g_outputRpmTar=" << g_outputRpmTar.load() << "outputMax=" << outputMax << std::endl;
    }
    repeat_sensor_monitor_timer();
}

/*=============================================================================*/


/*******************************************************************************
 * Set fan control mode
 * AUTO,LOW,MEDIUM,HIGH
*/
int setFanControlMode(std::string mode)
{
    if (mode == g_fanControlMode) {
        return 0;
    } else {
        if (mode != "auto") {
            auto s1 = controlModeMap.find(mode);
            if (s1 != controlModeMap.end()) {
                std::cout << "s1= " << s1->second << std::endl;
                auto s2 = controlParamMap.find(s1->second);
                if (s2 != controlParamMap.end()) {
                    std::cout << "s2= " << s2->second << std::endl;
                    double pwm = percentageToActual(s2->second, PWM_MAX);
                    g_fanControlMode = mode;
                    std::cout << "control mode = " << g_fanControlMode << "; pwm = " << pwm << std::endl;
                    cancel_auto_timer();
                    writeAllFansPwm(pwm);
                } else {
                    return -1;
                }
            } else {
                return -1;
            }
        } else {
            g_fanControlMode = mode;
            std::cout << "control mode switch to " << g_fanControlMode << std::endl;
            if(isPowerOn)
                set_auto_timer(3);
        }

    }

    return 0;
}
/*=============================================================================*/

} // namespace fan_control





int main(int argc, char* argv[])
{
    using namespace power;
    using namespace properties;
    using namespace fan_control;
    using namespace boost::placeholders;


    // Load json config file
    if (loadConfigValues() == -1)
    {
        lg2::error(" Error in Parsing...");
        return -1;
    }

    isNeededCheckDbus = is_needed_to_check_dbus();
    g_def_fan_speed   = controlParamMap["FanDefSpeed"];
    g_outputRpmTar.store(g_def_fan_speed);

    if (isNeededCheckDbus)
        std::cout << "Need check dbus!" << std::endl;
    // Request the dbus names
    conn = std::make_shared<sdbusplus::asio::connection>(io);
    conn->request_name(fanControlDbusName.c_str());
    sdbusplus::asio::object_server objectServer(conn);
    fanControlInterface = objectServer.add_interface("/xyz/openbmc_project/sophgo/fanControl", "xyz.openbmc_project.Sophgo.FanControl");
    aiCardTempInterface = objectServer.add_interface("/xyz/openbmc_project/sensors/temperature/AiCard_Temp", sensorInterface);

    fanControlInterface->register_property(
        "ControlMode",
        std::string("auto"),
        [](const std::string& requested, std::string& resp) {
            auto value = setFanControlMode(requested);
            if (value < 0) {
                throw std::invalid_argument("Unrecognized controlMode Request");
                return 0;
            }
            resp = requested;
            return 1;
        });
    double  initial_value = -1;
    aiCardTempInterface->register_property(sensorProperty, initial_value,
        [](const double& newValue, double& value) {
            std::chrono::high_resolution_clock::time_point now = std::chrono::high_resolution_clock::now();
            auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - g_time_powerOn_point);
            value = newValue;
            // 开机300s之内如果板卡温度高于100度则认为是温度读取异常，127度由8bit有符号数据限定
            if ((isPowerOn) && (((value > 100) && (elapsed.count() < 300000)) || (value > Ai_templimit))) {
                std::cout << "Dbus aicard temp error: " << value << std::endl;
            } else {
                g_aicard_temp = newValue;
            }
            return 1;
        });

    aiCardTempInterface->register_property("sg1684xExistState", 0, sdbusplus::asio::PropertyPermission::readWrite);



    fanControlInterface->initialize();
    aiCardTempInterface->initialize();

    setupPowerMatch(conn);
    writeDefaultPwm();

    io.run();

    return 0;
}
