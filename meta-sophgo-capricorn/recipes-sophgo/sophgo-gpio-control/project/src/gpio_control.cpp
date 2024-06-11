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
#include "gpio_control.hpp"

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

extern "C"
{
#include <i2c/smbus.h>
#include <linux/i2c-dev.h>
}

namespace gpio_control
{
static boost::asio::io_service io;
std::shared_ptr<sdbusplus::asio::connection> conn;

static std::string node = "0";
static const std::string appName = "sophgo-gpio-control";

enum class DbusConfigType
{
    name = 1,
    path,
    interface,
    property
};
boost::container::flat_map<DbusConfigType, std::string> dbusParams = {
    {DbusConfigType::name, "DbusName"},
    {DbusConfigType::path, "Path"},
    {DbusConfigType::interface, "Interface"},
    {DbusConfigType::property, "Property"}};

enum class ConfigType
{
    GPIO = 1,
    DBUS,
    I2C
};

enum HostFlashPort
{
    BMC = 0,
    HOST
};

enum IdentifyLedSet
{
    ON = 0,
    OFF
};

enum SolPort
{
    H0U0 = 0,
    H0U1,
    H1U0,
    H1U1
};

struct ConfigData
{
    std::string name;
    std::string lineName;
    std::string dbusName;
    std::string path;
    std::string interface;
    bool polarity;
    ConfigType type;
};




static ConfigData PSUPowerOnConfig;
static ConfigData riserCardThrottleConfig;
static ConfigData solUartV2Config;
static ConfigData solUartV3Config;
static ConfigData phyIrqConfig;
static ConfigData identifyLedGetConfig;
static ConfigData identifyLedSetConfig;
static ConfigData biosFlashSwitchConfig;


// map for storing list of gpio parameters whose config are to be read from sophgo
// gpio control json config
boost::container::flat_map<std::string, ConfigData*> powerSignalMap = {

    {"PSUPowerOn",         &PSUPowerOnConfig},
    {"riserCardThrottle",  &riserCardThrottleConfig},
    {"solUartV2",          &solUartV2Config},
    {"solUartV3",          &solUartV3Config},
    {"phyIrq",             &phyIrqConfig},
    {"identifyLedGet",     &identifyLedGetConfig},
    {"identifyLedSet",     &identifyLedSetConfig},
    {"biosFlashSwitch",    &biosFlashSwitchConfig}};



static std::string gpioDbusName = "xyz.openbmc_project.Gpio";
static std::shared_ptr<sdbusplus::asio::dbus_interface> PSUPowerOnIface;//输出
static std::shared_ptr<sdbusplus::asio::dbus_interface> riserCardIface;
static std::shared_ptr<sdbusplus::asio::dbus_interface> solUartIface;//输出
static std::shared_ptr<sdbusplus::asio::dbus_interface> phyIrqIface;//作用未知
static std::shared_ptr<sdbusplus::asio::dbus_interface> identifyLedIface;
static std::shared_ptr<sdbusplus::asio::dbus_interface> biosFlashSwitchIface;//输出




// LED flashing cycle
boost::container::flat_map<std::string, int> TimerMap = {
    {"PowerPulseMs",    200  },
    {"ForceOffPulseMs", 15000},
    {"ResetPulseMs",    500  },
    {"LedSwitchPullMs", 300  }};


// Timers
// Time holding GPIOs asserted
static boost::asio::steady_timer gpioAssertTimer(io);
static boost::asio::deadline_timer getCpldInfoTimer(io);
static boost::asio::deadline_timer syn_delaytimer(io, boost::posix_time::seconds(1));

// GPIO Lines and Event Descriptors
static gpiod::line PSUPowerOnLine;
static boost::asio::posix::stream_descriptor PSUPowerOEvent(io);
static gpiod::line riserCardThLine;
static boost::asio::posix::stream_descriptor riserCardThrottleEvent(io);
static gpiod::line solUartV2Line;
static gpiod::line solUartV3Line;
static gpiod::line phyIrqLine;
static gpiod::line identifyLedGetLine;
static boost::asio::posix::stream_descriptor identifyLedGetEvent(io);
static gpiod::line identifyLedSetLine;

static gpiod::line biosFlashSwitchLine;



static bool setGPIOOutput(const std::string& name, const int value,
                          gpiod::line& gpioLine)
{
    // Find the GPIO line
    gpioLine = gpiod::find_line(name);
    if (!gpioLine)
    {
        lg2::error("Failed to find the {GPIO_NAME} line", "GPIO_NAME", name);
        return false;
    }

    // Request GPIO output to specified value
    try
    {
        gpioLine.request({appName, gpiod::line_request::DIRECTION_OUTPUT, {}},
                         value);
    }
    catch (const std::exception& e)
    {
        lg2::error("Failed to request {GPIO_NAME} output: {ERROR}", "GPIO_NAME",
                   name, "ERROR", e);
        return false;
    }

    lg2::info("{GPIO_NAME} set to {GPIO_VALUE}", "GPIO_NAME", name,
              "GPIO_VALUE", value);


    gpioLine.release();

    return true;
}

static bool setGPIOOutputNotRelease(const std::string& name, const int value,
                          gpiod::line& gpioLine)
{
    // Find the GPIO line
    gpioLine = gpiod::find_line(name);
    if (!gpioLine)
    {
        lg2::error("Failed to find the {GPIO_NAME} line", "GPIO_NAME", name);
        return false;
    }

    // Request GPIO output to specified value
    try
    {
        gpioLine.request({appName, gpiod::line_request::DIRECTION_OUTPUT, {}},
                         value);
    }
    catch (const std::exception& e)
    {
        lg2::error("Failed to request {GPIO_NAME} output: {ERROR}", "GPIO_NAME",
                   name, "ERROR", e);
        return false;
    }

    lg2::info("{GPIO_NAME} set to {GPIO_VALUE}", "GPIO_NAME", name,
              "GPIO_VALUE", value);

    return true;
}


static int loadConfigValues()
{
    const std::string configFilePath = "/usr/share/sophgo-gpio-control/gpio-config.json";
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
    auto gpios    = jsonData["gpio_configs"];
    auto timers   = jsonData["timing_configs"];

    ConfigData* tempGpioData;

    for (nlohmann::json& gpioConfig : gpios)
    {
        if (!gpioConfig.contains("Name"))
        {
            lg2::error("The 'Name' field must be defined in Json file");
            return -1;
        }

        // Iterate through the powersignal map to check if the gpio json config
        // entry is valid
        std::string gpioName = gpioConfig["Name"];
        auto signalMapIter = powerSignalMap.find(gpioName);
        if (signalMapIter == powerSignalMap.end())
        {
            lg2::error(
                "{GPIO_NAME} is not a recognized power-control signal name",
                "GPIO_NAME", gpioName);
            return -1;
        }

        // assign the power signal name to the corresponding structure reference
        // from map then fillup the structure with coressponding json config
        // value
        tempGpioData = signalMapIter->second;
        tempGpioData->name = gpioName;

        if (!gpioConfig.contains("Type"))
        {
            lg2::error("The \'Type\' field must be defined in Json file");
            return -1;
        }

        std::string signalType = gpioConfig["Type"];
        if (signalType == "GPIO")
        {
            tempGpioData->type = ConfigType::GPIO;
        }
        else if (signalType == "DBUS")
        {
            tempGpioData->type = ConfigType::DBUS;
        }
        else
        {
            lg2::error("{TYPE} is not a recognized power-control signal type",
                       "TYPE", signalType);
            return -1;
        }

        if (tempGpioData->type == ConfigType::GPIO)
        {
            if (gpioConfig.contains("LineName"))
            {
                tempGpioData->lineName = gpioConfig["LineName"];
            }
            else
            {
                lg2::error(
                    "The \'LineName\' field must be defined for GPIO configuration");
                return -1;
            }
            if (gpioConfig.contains("Polarity"))
            {
                std::string polarity = gpioConfig["Polarity"];
                if (polarity == "ActiveLow")
                {
                    tempGpioData->polarity = false;
                }
                else if (polarity == "ActiveHigh")
                {
                    tempGpioData->polarity = true;
                }
                else
                {
                    lg2::error(
                        "Polarity defined but not properly setup. Please only ActiveHigh or ActiveLow. Currently set to {POLARITY}",
                        "POLARITY", polarity);
                    return -1;
                }
            }
            else
            {
                lg2::error("Polarity field not found for {GPIO_NAME}",
                           "GPIO_NAME", tempGpioData->lineName);
                return -1;
            }
        }
        else
        {
            // if dbus based gpio config is defined read and update the dbus
            // params corresponding to the gpio config instance
            for (auto& [key, dbusParamName] : dbusParams)
            {
                if (!gpioConfig.contains(dbusParamName))
                {
                    lg2::error(
                        "The {DBUS_NAME} field must be defined for Dbus configuration ",
                        "DBUS_NAME", dbusParamName);
                    return -1;
                }
            }
            tempGpioData->dbusName =
                gpioConfig[dbusParams[DbusConfigType::name]];
            tempGpioData->path = gpioConfig[dbusParams[DbusConfigType::path]];
            tempGpioData->interface =
                gpioConfig[dbusParams[DbusConfigType::interface]];
            tempGpioData->lineName =
                gpioConfig[dbusParams[DbusConfigType::property]];
        }
    }

     // read and store the timer values from json config to Timer Map
    for (auto& [key, timerValue] : TimerMap)
    {
        if (timers.contains(key.c_str()))
        {
            timerValue = timers[key.c_str()];
        }
    }

    return 0;
}




static int checkGpioLineName()
{
    if ((PSUPowerOnConfig.lineName.empty())           || \
       (riserCardThrottleConfig.lineName.empty())    || \
       (solUartV2Config.lineName.empty())            || \
       (solUartV3Config.lineName.empty())            || \
       (phyIrqConfig.lineName.empty())               || \
       (identifyLedGetConfig.lineName.empty())       || \
       (identifyLedSetConfig.lineName.empty())       || \
       (biosFlashSwitchConfig.lineName.empty()))
    {
        return -1;
    }

    return 0;


}









static void waitForGPIOEvent(const std::string& name,
                             const std::function<void(bool)>& eventHandler,
                             gpiod::line& line,
                             boost::asio::posix::stream_descriptor& event)
{
    event.async_wait(boost::asio::posix::stream_descriptor::wait_read,
                     [&name, eventHandler, &line,
                      &event](const boost::system::error_code ec) {
                         if (ec)
                         {
                             lg2::error(
                                 "{GPIO_NAME} fd handler error: {ERROR_MSG}",
                                 "GPIO_NAME", name, "ERROR_MSG", ec.message());
                             // TODO: throw here to force power-control to
                             // restart?
                             return;
                         }
                         gpiod::line_event line_event = line.event_read();
                         eventHandler(line_event.event_type ==
                                      gpiod::line_event::RISING_EDGE);
                         waitForGPIOEvent(name, eventHandler, line, event);
                     });
}

static bool requestGPIOEvents(
    const std::string& name, const std::function<void(bool)>& handler,
    gpiod::line& gpioLine,
    boost::asio::posix::stream_descriptor& gpioEventDescriptor)
{
    // Find the GPIO line
    lg2::info("requestGPIOEvents : {GPIO_NAME} ", "GPIO_NAME", name);
    gpioLine = gpiod::find_line(name);
    if (!gpioLine)
    {
        lg2::error("Failed to find the {GPIO_NAME} line", "GPIO_NAME", name);
        return false;
    }

    try
    {
        gpioLine.request({appName, gpiod::line_request::EVENT_BOTH_EDGES, {}});
    }
    catch (const std::exception& e)
    {
        lg2::error("Failed to request events for {GPIO_NAME}: {ERROR}",
                   "GPIO_NAME", name, "ERROR", e);
        return false;
    }

    int gpioLineFd = gpioLine.event_get_fd();
    if (gpioLineFd < 0)
    {
        lg2::error("Failed to get {GPIO_NAME} fd", "GPIO_NAME", name);
        return false;
    }

    gpioEventDescriptor.assign(gpioLineFd);

    waitForGPIOEvent(name, handler, gpioLine, gpioEventDescriptor);
    return true;
}



static void identifyLedStateHandler(bool state)
{
    identifyLedIface->set_property("identifyLedState", state);
}


enum class SolUartPort
{
    CPU_UART0,
    CPU_UART1,
};

static void transitionSolUarttPort(SolUartPort port)
{
    switch (port)
    {
        case SolUartPort::CPU_UART0:
            setGPIOOutput(solUartV2Config.lineName,0,solUartV2Line);
            setGPIOOutput(solUartV3Config.lineName,0,solUartV3Line);
            break;
        case SolUartPort::CPU_UART1:
            setGPIOOutput(solUartV2Config.lineName,0,solUartV2Line);
            setGPIOOutput(solUartV3Config.lineName,1,solUartV3Line);
            break;
        default:
            break;
    }
}


enum class BiosFlashMaster
{
    BMC,
    HOST,
};

static void transitionBiosFlash(BiosFlashMaster port)
{
    switch (port)
    {
        case BiosFlashMaster::BMC:
            setGPIOOutput(biosFlashSwitchConfig.lineName,0,biosFlashSwitchLine);
            break;
        case BiosFlashMaster::HOST:
            setGPIOOutput(biosFlashSwitchConfig.lineName,1,biosFlashSwitchLine);
            break;
        default:
            break;
    }
}

static int setGPIOOutputForMs(const ConfigData& config, const int value,
                              const int durationMs,gpiod::line gpioLine)
{
    ;
    if (!setGPIOOutputNotRelease(config.lineName, value, gpioLine))
    {
        return -1;
    }
    const std::string name = config.lineName;

    gpioAssertTimer.expires_after(std::chrono::milliseconds(durationMs));
    gpioAssertTimer.async_wait(
        [gpioLine, value, name](const boost::system::error_code ec) {
            // Set the GPIO line back to the opposite value
            gpioLine.set_value(!value);
            lg2::info("{GPIO_NAME} released", "GPIO_NAME", name);
            if (ec)
            {
                // operation_aborted is expected if timer is canceled before
                // completion.
                if (ec != boost::asio::error::operation_aborted)
                {
                    lg2::error("{GPIO_NAME} async_wait failed: {ERROR_MSG}",
                               "GPIO_NAME", name, "ERROR_MSG", ec.message());
                }
            }
        });
    return 0;
}



static void switchIdentifyLed()
{
    setGPIOOutputForMs(identifyLedSetConfig, IdentifyLedSet::ON, TimerMap["LedSwitchPullMs"], identifyLedSetLine);
}


} // namespace gpio_control





int main(int argc, char* argv[])
{
    using namespace gpio_control;
    using namespace boost::placeholders;


    // Load GPIO's through json config file
    if (loadConfigValues() == -1)
    {
        lg2::error(" Error in Parsing...");
        return -1;
    }
    //check gpio lineName
    if (checkGpioLineName() == -1)
    {
        lg2::error(" MIssing gpio line name ...");
        return -1;
    }

    // Request the dbus names
    auto gpioBus = std::make_shared<sdbusplus::asio::connection>(io);
    gpioBus->request_name(gpioDbusName.c_str());
    sdbusplus::asio::object_server objectServer(gpioBus);
    riserCardIface       = objectServer.add_interface("/xyz/openbmc_project/gpio/riser", "xyz.openbmc_project.Gpio.RiserCard");
    PSUPowerOnIface      = objectServer.add_interface("/xyz/openbmc_project/gpio/psupower", "xyz.openbmc_project.Gpio.PsuPOwerOn");
    solUartIface         = objectServer.add_interface("/xyz/openbmc_project/gpio/soluart", "xyz.openbmc_project.Gpio.SolUart");
    identifyLedIface     = objectServer.add_interface("/xyz/openbmc_project/gpio/identifyLed", "xyz.openbmc_project.Gpio.identifyLed");
    biosFlashSwitchIface = objectServer.add_interface("/xyz/openbmc_project/gpio/biosflash", "xyz.openbmc_project.Gpio.BiosFlash");

    // Request GPIO events 初始化过程中先获取一次在位状态


    //identifyLed
    if (!requestGPIOEvents(identifyLedGetConfig.lineName, identifyLedStateHandler,
                            identifyLedGetLine, identifyLedGetEvent))
    {
        return -1;
    }
    identifyLedIface->register_property("identifyLedState",
                    (identifyLedGetLine.get_value() == identifyLedGetConfig.polarity));

    //Initialize out gpio
    if(!setGPIOOutput(solUartV2Config.lineName, 0, solUartV2Line))
    {
        return -1;
    }
    if(!setGPIOOutput(solUartV3Config.lineName, 0, solUartV3Line))
    {
        return -1;
    }
    if(!setGPIOOutput(identifyLedSetConfig.lineName, IdentifyLedSet::OFF, identifyLedSetLine))
    {
        return -1;
    }


    // Request DBUS events
    solUartIface->register_property(
        "SolUartPortTransition",
        std::string("CPU0UART0"),
        [](const std::string& requested, std::string& resp) {
            if (requested == "CPU0UART0")
            {
                transitionSolUarttPort(SolUartPort::CPU_UART0);
            }
            else if (requested == "CPU0UART1")
            {
                transitionSolUarttPort(SolUartPort::CPU_UART1);
            }
            resp = requested;
            return 1;
        });
    biosFlashSwitchIface->register_property(
        "BiosFlashTransition",
        std::string("host"),
        [](const std::string& requested, std::string& resp) {
            if (requested == "host")
            {
                transitionBiosFlash(BiosFlashMaster::HOST);
            }
            else if (requested == "bmc")
            {
                transitionBiosFlash(BiosFlashMaster::BMC);
            }
            resp = requested;
            return 1;
        });

    // identifyLedIface->register_method("identifyLedSwitch", switchIdentifyLed);
    identifyLedIface->register_property(
        "identifyLedSwitch",
        bool(1),
        [](const bool requested, bool resp) {
            switchIdentifyLed();
            resp = requested;
            return 1;
        });


    riserCardIface->initialize();
    PSUPowerOnIface->initialize();
    solUartIface->initialize();
    identifyLedIface->initialize();
    biosFlashSwitchIface->initialize();
    io.run();

    return 0;
}
