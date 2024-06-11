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
#include "power_control.hpp"

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
#define SOPHGO_CPLD
}

namespace power_control
{
static boost::asio::io_service io;
std::shared_ptr<sdbusplus::asio::connection> conn;
PersistentState appState;
PowerRestoreController powerRestore(io);

static std::string node = "0";
static const std::string appName = "power-control";

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




static ConfigData powerOutConfig;
static ConfigData powerOkConfig;
static ConfigData resetOutConfig;
static ConfigData nmiOutConfig;
static ConfigData sioPwrGoodConfig;
static ConfigData sioOnControlConfig;
static ConfigData sioS5Config;
static ConfigData postCompleteConfig;
static ConfigData powerButtonConfig;
static ConfigData resetButtonConfig;
static ConfigData idButtonConfig;
static ConfigData nmiButtonConfig;
static ConfigData slotPowerConfig;


struct InfoCpld
{
    std::string    deviceName;
    int            i2cFile;
    unsigned char  deviceAddr;
    unsigned char  PWControlReg;//power control register
    unsigned char  CPUAPWStateReg;
    unsigned char  CPUBPWStateReg;
    unsigned char  CPUOPWStateReg;
    unsigned char  OPWStateReg;
    unsigned char  PSUStateReg;
    unsigned char  EFUSStateReg;
    unsigned char  resetControlAReg;
    unsigned char  resetControlBReg;
};
struct InfoCpld cpldInfo;

std::mutex     i2cMutex;

struct CPUAPWStateRegInfo
{
    char PG_VPP_A1      :1;
    char PG_VPP_A0      :1;
    char PG_VDDQ_A1     :1;
    char PG_VDDQ_A0     :1;
    char PG_VTT_A1      :1;
    char PG_VTT_A0      :1;
    char PG_PCIE_PHY_A  :1;
    char PG_VDDC_A      :1;
};
struct CPUAPWStateRegInfo CPUA_pw_state;

struct CPUBPWStateRegInfo
{
    char PG_VPP_A1      :1;
    char PG_VPP_A0      :1;
    char PG_VDDQ_A1     :1;
    char PG_VDDQ_A0     :1;
    char PG_VTT_A1      :1;
    char PG_VTT_A0      :1;
    char PG_PCIE_PHY_A  :1;
    char PG_VDDC_A      :1;
};
struct CPUBPWStateRegInfo CPUB_pw_state;



struct CPUOPWStateRegInfo
{
    char PG_VDD_3V3      :1;
    char PG_VDD_1V8      :1;
    char PG_PCIE_H_B     :1;
    char PG_DDR_PHY_B    :1;
    char PG_PCIE_H_A     :1;
    char PG_DDR_PHY_A    :1;
    char RESERVED        :2;
};
struct CPUOPWStateRegInfo CPUO_pw_state;

struct OPWStateRegInfo
{
    char PG_PCIE_SW_B   :1;
    char PG_PCIE_SW_A   :1;
    char PG_3V3_RISER   :1;
    char PG_BP_5V       :1;
    char PG_5V_AUX      :1;
    char PG_VDD_5V      :1;
    char RESERVED       :2;
};
struct OPWStateRegInfo OPW_state;

struct PSUStateRegInfo
{
    char PSU1_PRSNT_N   :1;
    char PSU0_PRSNT_N   :1;
    char PSU1_ALERT_N   :1;
    char PSU0_ALERT_N   :1;
    char PSU1_PWROK     :1;
    char PSU0_PWROK     :1;
    char RESERVED       :2;
};
struct PSUStateRegInfo PSU_state;


struct EFUSStateRegInfo
{
    char NCP0_GOK   :1;
    char NCP0_D_OC  :1;
    char NCP1_GOK   :1;
    char NCP1_D_OC  :1;
    char RESVERED   :4;
};
struct EFUSStateRegInfo EFUS_state;


static bool PWRemoteControlIsEnabled = false;



// map for storing list of gpio parameters whose config are to be read from x86
// power control json config
boost::container::flat_map<std::string, ConfigData*> powerSignalMap = {
    {"PowerOut", &powerOutConfig},
    {"PowerOk", &powerOkConfig},
    {"ResetOut", &resetOutConfig},
    {"NMIOut", &nmiOutConfig},
    {"SioPowerGood", &sioPwrGoodConfig},
    {"SioOnControl", &sioOnControlConfig},
    {"SIOS5", &sioS5Config},
    {"PostComplete", &postCompleteConfig},
    {"PowerButton", &powerButtonConfig},
    {"ResetButton", &resetButtonConfig},
    {"IdButton", &idButtonConfig},
    {"NMIButton", &nmiButtonConfig},
    {"SlotPower", &slotPowerConfig}};

static std::string hostDbusName = "xyz.openbmc_project.State.Host";
static std::string chassisDbusName = "xyz.openbmc_project.State.Chassis";
static std::string osDbusName = "xyz.openbmc_project.State.OperatingSystem";
static std::string buttonDbusName = "xyz.openbmc_project.Chassis.Buttons";
static std::string nmiDbusName = "xyz.openbmc_project.Control.Host.NMI";
static std::string rstCauseDbusName =
    "xyz.openbmc_project.Control.Host.RestartCause";
static std::shared_ptr<sdbusplus::asio::dbus_interface> hostIface;
static std::shared_ptr<sdbusplus::asio::dbus_interface> chassisIface;
#ifdef CHASSIS_SYSTEM_RESET
static std::shared_ptr<sdbusplus::asio::dbus_interface> chassisSysIface;
static std::shared_ptr<sdbusplus::asio::dbus_interface> chassisSlotIface;
#endif
static std::shared_ptr<sdbusplus::asio::dbus_interface> powerButtonIface;
static std::shared_ptr<sdbusplus::asio::dbus_interface> resetButtonIface;
static std::shared_ptr<sdbusplus::asio::dbus_interface> nmiButtonIface;
static std::shared_ptr<sdbusplus::asio::dbus_interface> osIface;
static std::shared_ptr<sdbusplus::asio::dbus_interface> idButtonIface;
static std::shared_ptr<sdbusplus::asio::dbus_interface> nmiOutIface;
static std::shared_ptr<sdbusplus::asio::dbus_interface> restartCauseIface;

static gpiod::line powerButtonMask;
static gpiod::line resetButtonMask;
static bool nmiButtonMasked = false;

// This map contains all timer values that are to be read from json config
boost::container::flat_map<std::string, int> TimerMap = {
    {"PowerPulseMs", 200},
    {"ForceOffPulseMs", 15000},
    {"ResetPulseMs", 500},
#ifndef SOPHGO_CPLD
    {"PowerCycleMs", 5000},
#else
    {"PowerCycleMs", 50},
#endif
    {"SioPowerGoodWatchdogMs", 1000},
#ifndef SOPHGO_CPLD
    {"PsPowerOKWatchdogMs", 8000},
#else
    {"PsPowerOKWatchdogMs", (30 * 300)},
#endif
#ifndef SOPHGO_CPLD
    {"GracefulPowerOffS", (5 * 60)},
#else
    {"GracefulPowerOffS", (3 * 300)},
#endif

    {"WarmResetCheckMs", 500},
    {"PowerOffSaveMs", 7000},
    {"SlotPowerCycleMs", 200},
    {"CpldReadCycle", 300}};

static bool nmiEnabled = true;
#ifndef SOPHGO_CPLD
static bool sioEnabled = true;
#else
static bool sioEnabled = false;
#endif
// Timers
// Time holding GPIOs asserted
static boost::asio::steady_timer gpioAssertTimer(io);
// Time between off and on during a power cycle
static boost::asio::steady_timer powerCycleTimer(io);
// Time OS gracefully powering off
static boost::asio::steady_timer gracefulPowerOffTimer(io);
// Time the warm reset check
static boost::asio::steady_timer warmResetCheckTimer(io);
// Time power supply power OK assertion on power-on
static boost::asio::steady_timer psPowerOKWatchdogTimer(io);
// Time SIO power good assertion on power-on
static boost::asio::steady_timer sioPowerGoodWatchdogTimer(io);
// Time power-off state save for power loss tracking
static boost::asio::steady_timer powerStateSaveTimer(io);
// POH timer
static boost::asio::steady_timer pohCounterTimer(io);
// Time when to allow restart cause updates
static boost::asio::steady_timer restartCauseTimer(io);
static boost::asio::steady_timer slotPowerCycleTimer(io);


static boost::asio::deadline_timer getCpldInfoTimer(io);
static boost::asio::deadline_timer syn_delaytimer(io, boost::posix_time::seconds(1));

// GPIO Lines and Event Descriptors
static gpiod::line psPowerOKLine;
static boost::asio::posix::stream_descriptor psPowerOKEvent(io);
static gpiod::line sioPowerGoodLine;
static boost::asio::posix::stream_descriptor sioPowerGoodEvent(io);
static gpiod::line sioOnControlLine;
static boost::asio::posix::stream_descriptor sioOnControlEvent(io);
static gpiod::line sioS5Line;
static boost::asio::posix::stream_descriptor sioS5Event(io);
static gpiod::line powerButtonLine;
static boost::asio::posix::stream_descriptor powerButtonEvent(io);
static gpiod::line resetButtonLine;
static boost::asio::posix::stream_descriptor resetButtonEvent(io);
static gpiod::line nmiButtonLine;
static boost::asio::posix::stream_descriptor nmiButtonEvent(io);
static gpiod::line idButtonLine;
static boost::asio::posix::stream_descriptor idButtonEvent(io);
static gpiod::line postCompleteLine;
static boost::asio::posix::stream_descriptor postCompleteEvent(io);
static gpiod::line nmiOutLine;
static gpiod::line slotPowerLine;

static constexpr uint8_t beepPowerFail = 8;


enum class SetCpldPowerState
{
    off,
    on,
    clear,
};

//cpld function declaration
int  set_cpld_pw_on_off(SetCpldPowerState state);
void cpldControlStateClear(void);
int  set_default_cpld_info(void);
int  cpld_i2c_init(void);
int get_all_cpld_info(void);

static void beep(const uint8_t& beepPriority)
{
    lg2::info("Beep with priority: {BEEP_PRIORITY}", "BEEP_PRIORITY",
              beepPriority);

    conn->async_method_call(
        [](boost::system::error_code ec) {
            if (ec)
            {
                lg2::error(
                    "beep returned error with async_method_call (ec = {ERROR_MSG})",
                    "ERROR_MSG", ec.message());
                return;
            }
        },
        "xyz.openbmc_project.BeepCode", "/xyz/openbmc_project/BeepCode",
        "xyz.openbmc_project.BeepCode", "Beep", uint8_t(beepPriority));
}

enum class OperatingSystemStateStage
{
    Inactive,
    Standby,
};
static constexpr std::string_view
    getOperatingSystemStateStage(const OperatingSystemStateStage stage)
{
    switch (stage)
    {
        case OperatingSystemStateStage::Inactive:
            return "xyz.openbmc_project.State.OperatingSystem.Status.OSStatus.Inactive";
            break;
        case OperatingSystemStateStage::Standby:
            return "xyz.openbmc_project.State.OperatingSystem.Status.OSStatus.Standby";
            break;
        default:
            return "xyz.openbmc_project.State.OperatingSystem.Status.OSStatus.Inactive";
            break;
    }
};
static void setOperatingSystemState(const OperatingSystemStateStage stage)
{
    osIface->set_property("OperatingSystemState",
                          std::string(getOperatingSystemStateStage(stage)));

    lg2::info("Moving os state to {STATE} stage", "STATE",
              getOperatingSystemStateStage(stage));
}

enum class PowerState
{
    on,
    waitForPSPowerOK,
    waitForSIOPowerGood,
    off,
    transitionToOff,
    gracefulTransitionToOff,
    cycleOff,
    transitionToCycleOff,
    gracefulTransitionToCycleOff,
    checkForWarmReset,
};




static PowerState powerState;
static std::string getPowerStateName(PowerState state)
{
    switch (state)
    {
        case PowerState::on:
            return "On";
            break;
        case PowerState::waitForPSPowerOK:
            return "Wait for Power Supply Power OK";
            break;
        case PowerState::waitForSIOPowerGood:
            return "Wait for SIO Power Good";
            break;
        case PowerState::off:
            return "Off";
            break;
        case PowerState::transitionToOff:
            return "Transition to Off";
            break;
        case PowerState::gracefulTransitionToOff:
            return "Graceful Transition to Off";
            break;
        case PowerState::cycleOff:
            return "Power Cycle Off";
            break;
        case PowerState::transitionToCycleOff:
            return "Transition to Power Cycle Off";
            break;
        case PowerState::gracefulTransitionToCycleOff:
            return "Graceful Transition to Power Cycle Off";
            break;
        case PowerState::checkForWarmReset:
            return "Check for Warm Reset";
            break;
        default:
            return "unknown state: " + std::to_string(static_cast<int>(state));
            break;
    }
}
static void logStateTransition(const PowerState state)
{
    lg2::info("Host{HOST}: Moving to \"{STATE}\" state", "HOST", node, "STATE",
              getPowerStateName(state));
}

enum class Event
{
    psPowerOKAssert,
    psPowerOKDeAssert,
    sioPowerGoodAssert,
    sioPowerGoodDeAssert,
    sioS5Assert,
    sioS5DeAssert,
    pltRstAssert,
    pltRstDeAssert,
    postCompleteAssert,
    postCompleteDeAssert,
    powerButtonPressed,
    resetButtonPressed,
    powerCycleTimerExpired,
    psPowerOKWatchdogTimerExpired,
    sioPowerGoodWatchdogTimerExpired,
    gracefulPowerOffTimerExpired,
    powerOnRequest,
    powerOffRequest,
    powerCycleRequest,
    resetRequest,
    gracefulPowerOffRequest,
    gracefulPowerCycleRequest,
    warmResetDetected,
};
static std::string getEventName(Event event)
{
    switch (event)
    {
        case Event::psPowerOKAssert:
            return "power supply power OK assert";
            break;
        case Event::psPowerOKDeAssert:
            return "power supply power OK de-assert";
            break;
        case Event::sioPowerGoodAssert:
            return "SIO power good assert";
            break;
        case Event::sioPowerGoodDeAssert:
            return "SIO power good de-assert";
            break;
        case Event::sioS5Assert:
            return "SIO S5 assert";
            break;
        case Event::sioS5DeAssert:
            return "SIO S5 de-assert";
            break;
        case Event::pltRstAssert:
            return "PLT_RST assert";
            break;
        case Event::pltRstDeAssert:
            return "PLT_RST de-assert";
            break;
        case Event::postCompleteAssert:
            return "POST Complete assert";
            break;
        case Event::postCompleteDeAssert:
            return "POST Complete de-assert";
            break;
        case Event::powerButtonPressed:
            return "power button pressed";
            break;
        case Event::resetButtonPressed:
            return "reset button pressed";
            break;
        case Event::powerCycleTimerExpired:
            return "power cycle timer expired";
            break;
        case Event::psPowerOKWatchdogTimerExpired:
            return "power supply power OK watchdog timer expired";
            break;
        case Event::sioPowerGoodWatchdogTimerExpired:
            return "SIO power good watchdog timer expired";
            break;
        case Event::gracefulPowerOffTimerExpired:
            return "graceful power-off timer expired";
            break;
        case Event::powerOnRequest:
            return "power-on request";
            break;
        case Event::powerOffRequest:
            return "power-off request";
            break;
        case Event::powerCycleRequest:
            return "power-cycle request";
            break;
        case Event::resetRequest:
            return "reset request";
            break;
        case Event::gracefulPowerOffRequest:
            return "graceful power-off request";
            break;
        case Event::gracefulPowerCycleRequest:
            return "graceful power-cycle request";
            break;
        case Event::warmResetDetected:
            return "warm reset detected";
            break;
        default:
            return "unknown event: " + std::to_string(static_cast<int>(event));
            break;
    }
}
static void logEvent(const std::string_view stateHandler, const Event event)
{
    lg2::info("{STATE_HANDLER}: {EVENT} event received", "STATE_HANDLER",
              stateHandler, "EVENT", getEventName(event));
}

// Power state handlers
static void powerStateOn(const Event event);
static void powerStateWaitForPSPowerOK(const Event event);
static void powerStateWaitForSIOPowerGood(const Event event);
static void powerStateOff(const Event event);
static void powerStateTransitionToOff(const Event event);
static void powerStateGracefulTransitionToOff(const Event event);
static void powerStateCycleOff(const Event event);
static void powerStateTransitionToCycleOff(const Event event);
static void powerStateGracefulTransitionToCycleOff(const Event event);
static void powerStateCheckForWarmReset(const Event event);

static std::function<void(const Event)> getPowerStateHandler(PowerState state)
{
    switch (state)
    {
        case PowerState::on:
            return powerStateOn;
            break;
        case PowerState::waitForPSPowerOK:
            return powerStateWaitForPSPowerOK;
            break;
        case PowerState::waitForSIOPowerGood:
            return powerStateWaitForSIOPowerGood;
            break;
        case PowerState::off:
            return powerStateOff;
            break;
        case PowerState::transitionToOff:
            return powerStateTransitionToOff;
            break;
        case PowerState::gracefulTransitionToOff:
            return powerStateGracefulTransitionToOff;
            break;
        case PowerState::cycleOff:
            return powerStateCycleOff;
            break;
        case PowerState::transitionToCycleOff:
            return powerStateTransitionToCycleOff;
            break;
        case PowerState::gracefulTransitionToCycleOff:
            return powerStateGracefulTransitionToCycleOff;
            break;
        case PowerState::checkForWarmReset:
            return powerStateCheckForWarmReset;
            break;
        default:
            return nullptr;
            break;
    }
};

static void sendPowerControlEvent(const Event event)
{
    std::function<void(const Event)> handler = getPowerStateHandler(powerState);
    if (handler == nullptr)
    {
        lg2::error("Failed to find handler for power state: {STATE}", "STATE",
                   static_cast<int>(powerState));
        return;
    }
    handler(event);
}

static uint64_t getCurrentTimeMs()
{
    struct timespec time = {};

    if (clock_gettime(CLOCK_REALTIME, &time) < 0)
    {
        return 0;
    }
    uint64_t currentTimeMs = static_cast<uint64_t>(time.tv_sec) * 1000;
    currentTimeMs += static_cast<uint64_t>(time.tv_nsec) / 1000 / 1000;

    return currentTimeMs;
}

static constexpr std::string_view getHostState(const PowerState state)
{
    switch (state)
    {
        case PowerState::on:
        case PowerState::gracefulTransitionToOff:
        case PowerState::gracefulTransitionToCycleOff:
            return "xyz.openbmc_project.State.Host.HostState.Running";
            break;
        case PowerState::waitForPSPowerOK:
        case PowerState::waitForSIOPowerGood:
        case PowerState::off:
        case PowerState::transitionToOff:
        case PowerState::transitionToCycleOff:
        case PowerState::cycleOff:
        case PowerState::checkForWarmReset:
            return "xyz.openbmc_project.State.Host.HostState.Off";
            break;
        default:
            return "";
            break;
    }
};
static constexpr std::string_view getChassisState(const PowerState state)
{
    switch (state)
    {
        case PowerState::on:
        case PowerState::transitionToOff:
        case PowerState::gracefulTransitionToOff:
        case PowerState::transitionToCycleOff:
        case PowerState::gracefulTransitionToCycleOff:
        case PowerState::checkForWarmReset:
            return "xyz.openbmc_project.State.Chassis.PowerState.On";
            break;
        case PowerState::waitForPSPowerOK:
        case PowerState::waitForSIOPowerGood:
        case PowerState::off:
        case PowerState::cycleOff:
            return "xyz.openbmc_project.State.Chassis.PowerState.Off";
            break;
        default:
            return "";
            break;
    }
};
#ifdef CHASSIS_SYSTEM_RESET
enum class SlotPowerState
{
    on,
    off,
};
static SlotPowerState slotPowerState;
static constexpr std::string_view getSlotState(const SlotPowerState state)
{
    switch (state)
    {
        case SlotPowerState::on:
            return "xyz.openbmc_project.State.Chassis.PowerState.On";
            break;
        case SlotPowerState::off:
            return "xyz.openbmc_project.State.Chassis.PowerState.Off";
            break;
        default:
            return "";
            break;
    }
};
static void setSlotPowerState(const SlotPowerState state)
{
    slotPowerState = state;
    chassisSlotIface->set_property("CurrentPowerState",
                                   std::string(getSlotState(slotPowerState)));
    chassisSlotIface->set_property("LastStateChangeTime", getCurrentTimeMs());
}
#endif
static void savePowerState(const PowerState state)
{
    powerStateSaveTimer.expires_after(
        std::chrono::milliseconds(TimerMap["PowerOffSaveMs"]));
    powerStateSaveTimer.async_wait([state](const boost::system::error_code ec) {
        if (ec)
        {
            // operation_aborted is expected if timer is canceled before
            // completion.
            if (ec != boost::asio::error::operation_aborted)
            {
                lg2::error("Power-state save async_wait failed: {ERROR_MSG}",
                           "ERROR_MSG", ec.message());
            }
            return;
        }
        appState.set(PersistentState::Params::PowerState,
                     std::string{getChassisState(state)});
    });
}
static void setPowerState(const PowerState state)
{
    powerState = state;
    logStateTransition(state);

    hostIface->set_property("CurrentHostState",
                            std::string(getHostState(powerState)));

    chassisIface->set_property("CurrentPowerState",
                               std::string(getChassisState(powerState)));
    chassisIface->set_property("LastStateChangeTime", getCurrentTimeMs());

    // Save the power state for the restore policy
    savePowerState(state);
}

enum class RestartCause
{
    command,
    resetButton,
    powerButton,
    watchdog,
    powerPolicyOn,
    powerPolicyRestore,
    softReset,
};
static boost::container::flat_set<RestartCause> causeSet;
static std::string getRestartCause(RestartCause cause)
{
    switch (cause)
    {
        case RestartCause::command:
            return "xyz.openbmc_project.State.Host.RestartCause.IpmiCommand";
            break;
        case RestartCause::resetButton:
            return "xyz.openbmc_project.State.Host.RestartCause.ResetButton";
            break;
        case RestartCause::powerButton:
            return "xyz.openbmc_project.State.Host.RestartCause.PowerButton";
            break;
        case RestartCause::watchdog:
            return "xyz.openbmc_project.State.Host.RestartCause.WatchdogTimer";
            break;
        case RestartCause::powerPolicyOn:
            return "xyz.openbmc_project.State.Host.RestartCause.PowerPolicyAlwaysOn";
            break;
        case RestartCause::powerPolicyRestore:
            return "xyz.openbmc_project.State.Host.RestartCause.PowerPolicyPreviousState";
            break;
        case RestartCause::softReset:
            return "xyz.openbmc_project.State.Host.RestartCause.SoftReset";
            break;
        default:
            return "xyz.openbmc_project.State.Host.RestartCause.Unknown";
            break;
    }
}
static void addRestartCause(const RestartCause cause)
{
    // Add this to the set of causes for this restart
    causeSet.insert(cause);
}
static void clearRestartCause()
{
    // Clear the set for the next restart
    causeSet.clear();
}
static void setRestartCauseProperty(const std::string& cause)
{
    lg2::info("RestartCause set to {RESTART_CAUSE}", "RESTART_CAUSE", cause);
    restartCauseIface->set_property("RestartCause", cause);
}

#ifdef USE_ACBOOT
static void resetACBootProperty()
{
    if ((causeSet.contains(RestartCause::command)) ||
        (causeSet.contains(RestartCause::softReset)))
    {
        conn->async_method_call(
            [](boost::system::error_code ec) {
                if (ec)
                {
                    lg2::error("failed to reset ACBoot property");
                }
            },
            "xyz.openbmc_project.Settings",
            "/xyz/openbmc_project/control/host0/ac_boot",
            "org.freedesktop.DBus.Properties", "Set",
            "xyz.openbmc_project.Common.ACBoot", "ACBoot",
            std::variant<std::string>{"False"});
    }
}
#endif // USE_ACBOOT

static void setRestartCause()
{
    // Determine the actual restart cause based on the set of causes
    std::string restartCause =
        "xyz.openbmc_project.State.Host.RestartCause.Unknown";
    if (causeSet.contains(RestartCause::watchdog))
    {
        restartCause = getRestartCause(RestartCause::watchdog);
    }
    else if (causeSet.contains(RestartCause::command))
    {
        restartCause = getRestartCause(RestartCause::command);
    }
    else if (causeSet.contains(RestartCause::resetButton))
    {
        restartCause = getRestartCause(RestartCause::resetButton);
    }
    else if (causeSet.contains(RestartCause::powerButton))
    {
        restartCause = getRestartCause(RestartCause::powerButton);
    }
    else if (causeSet.contains(RestartCause::powerPolicyOn))
    {
        restartCause = getRestartCause(RestartCause::powerPolicyOn);
    }
    else if (causeSet.contains(RestartCause::powerPolicyRestore))
    {
        restartCause = getRestartCause(RestartCause::powerPolicyRestore);
    }
    else if (causeSet.contains(RestartCause::softReset))
    {
        restartCause = getRestartCause(RestartCause::softReset);
    }

    setRestartCauseProperty(restartCause);
}

static void systemPowerGoodFailedLog()
{
    sd_journal_send(
        "MESSAGE=PowerControl: system power good failed to assert (VR failure)",
        "PRIORITY=%i", LOG_INFO, "REDFISH_MESSAGE_ID=%s",
        "OpenBMC.0.1.SystemPowerGoodFailed", "REDFISH_MESSAGE_ARGS=%d",
        TimerMap["SioPowerGoodWatchdogMs"], NULL);
}

static void psPowerOKFailedLog()
{
    sd_journal_send(
        "MESSAGE=PowerControl: power supply power good failed to assert",
        "PRIORITY=%i", LOG_INFO, "REDFISH_MESSAGE_ID=%s",
        "OpenBMC.0.1.PowerSupplyPowerGoodFailed", "REDFISH_MESSAGE_ARGS=%d",
        TimerMap["PsPowerOKWatchdogMs"], NULL);
}

static void powerRestorePolicyLog()
{
    sd_journal_send("MESSAGE=PowerControl: power restore policy applied",
                    "PRIORITY=%i", LOG_INFO, "REDFISH_MESSAGE_ID=%s",
                    "OpenBMC.0.1.PowerRestorePolicyApplied", NULL);
}

static void powerButtonPressLog()
{
    sd_journal_send("MESSAGE=PowerControl: power button pressed", "PRIORITY=%i",
                    LOG_INFO, "REDFISH_MESSAGE_ID=%s",
                    "OpenBMC.0.1.PowerButtonPressed", NULL);
}

static void resetButtonPressLog()
{
    sd_journal_send("MESSAGE=PowerControl: reset button pressed", "PRIORITY=%i",
                    LOG_INFO, "REDFISH_MESSAGE_ID=%s",
                    "OpenBMC.0.1.ResetButtonPressed", NULL);
}

static void nmiButtonPressLog()
{
    sd_journal_send("MESSAGE=PowerControl: NMI button pressed", "PRIORITY=%i",
                    LOG_INFO, "REDFISH_MESSAGE_ID=%s",
                    "OpenBMC.0.1.NMIButtonPressed", NULL);
}

static void nmiDiagIntLog()
{
    sd_journal_send("MESSAGE=PowerControl: NMI Diagnostic Interrupt",
                    "PRIORITY=%i", LOG_INFO, "REDFISH_MESSAGE_ID=%s",
                    "OpenBMC.0.1.NMIDiagnosticInterrupt", NULL);
}

PersistentState::PersistentState()
{
    // create the power control directory if it doesn't exist
    std::error_code ec;
    if (!(std::filesystem::create_directories(powerControlDir, ec)))
    {
        if (ec.value() != 0)
        {
            lg2::error("failed to create {DIR_NAME}: {ERROR_MSG}", "DIR_NAME",
                       powerControlDir.string(), "ERROR_MSG", ec.message());
            throw std::runtime_error("Failed to create state directory");
        }
    }

    // read saved state, it's ok, if the file doesn't exists
    std::ifstream appStateStream(powerControlDir / stateFile);
    if (!appStateStream.is_open())
    {
        lg2::info("Cannot open state file \'{PATH}\'", "PATH",
                  std::string(powerControlDir / stateFile));
        stateData = nlohmann::json({});
        return;
    }
    try
    {
        appStateStream >> stateData;
        if (stateData.is_discarded())
        {
            lg2::info("Cannot parse state file \'{PATH}\'", "PATH",
                      std::string(powerControlDir / stateFile));
            stateData = nlohmann::json({});
            return;
        }
    }
    catch (const std::exception& ex)
    {
        lg2::info("Cannot read state file \'{PATH}\'", "PATH",
                  std::string(powerControlDir / stateFile));
        stateData = nlohmann::json({});
        return;
    }
}
PersistentState::~PersistentState()
{
    saveState();
}
const std::string PersistentState::get(Params parameter)
{
    auto val = stateData.find(getName(parameter));
    if (val != stateData.end())
    {
        return val->get<std::string>();
    }
    return getDefault(parameter);
}
void PersistentState::set(Params parameter, const std::string& value)
{
    stateData[getName(parameter)] = value;
    saveState();
}

const std::string PersistentState::getName(const Params parameter)
{
    switch (parameter)
    {
        case Params::PowerState:
            return "PowerState";
    }
    return "";
}
const std::string PersistentState::getDefault(const Params parameter)
{
    switch (parameter)
    {
        case Params::PowerState:
            return "xyz.openbmc_project.State.Chassis.PowerState.Off";
    }
    return "";
}
void PersistentState::saveState()
{
    std::ofstream appStateStream(powerControlDir / stateFile, std::ios::trunc);
    if (!appStateStream.is_open())
    {
        lg2::error("Cannot write state file \'{PATH}\'", "PATH",
                   std::string(powerControlDir / stateFile));
        return;
    }
    appStateStream << stateData.dump(indentationSize);
}

static constexpr char const* setingsService = "xyz.openbmc_project.Settings";
static constexpr char const* powerRestorePolicyObject =
    "/xyz/openbmc_project/control/host0/power_restore_policy";
static constexpr char const* powerRestorePolicyIface =
    "xyz.openbmc_project.Control.Power.RestorePolicy";
#ifdef USE_ACBOOT
static constexpr char const* powerACBootObject =
    "/xyz/openbmc_project/control/host0/ac_boot";
static constexpr char const* powerACBootIface =
    "xyz.openbmc_project.Common.ACBoot";
#endif // USE_ACBOOT

namespace match_rules = sdbusplus::bus::match::rules;

static int powerRestoreConfigHandler(sd_bus_message* m, void* context,
                                     sd_bus_error*)
{
    if (context == nullptr || m == nullptr)
    {
        throw std::runtime_error("Invalid match");
    }
    sdbusplus::message::message message(m);
    PowerRestoreController* powerRestore =
        static_cast<PowerRestoreController*>(context);

    if (std::string(message.get_member()) == "InterfacesAdded")
    {
        sdbusplus::message::object_path path;
        boost::container::flat_map<std::string, dbusPropertiesList> data;

        message.read(path, data);

        for (auto& [iface, properties] : data)
        {
            if ((iface == powerRestorePolicyIface)
#ifdef USE_ACBOOT
                || (iface == powerACBootIface)
#endif // USE_ACBOOT
            )
            {
                powerRestore->setProperties(properties);
            }
        }
    }
    else if (std::string(message.get_member()) == "PropertiesChanged")
    {
        std::string interfaceName;
        dbusPropertiesList propertiesChanged;

        message.read(interfaceName, propertiesChanged);

        powerRestore->setProperties(propertiesChanged);
    }
    return 1;
}

void PowerRestoreController::run()
{
    powerRestorePolicyLog();
    // this list only needs to be created once
    if (matches.empty())
    {
        matches.emplace_back(
            *conn,
            match_rules::interfacesAdded() +
                match_rules::argNpath(0, powerRestorePolicyObject) +
                match_rules::sender(setingsService),
            powerRestoreConfigHandler, this);
#ifdef USE_ACBOOT
        matches.emplace_back(*conn,
                             match_rules::interfacesAdded() +
                                 match_rules::argNpath(0, powerACBootObject) +
                                 match_rules::sender(setingsService),
                             powerRestoreConfigHandler, this);
        matches.emplace_back(*conn,
                             match_rules::propertiesChanged(powerACBootObject,
                                                            powerACBootIface) +
                                 match_rules::sender(setingsService),
                             powerRestoreConfigHandler, this);
#endif // USE_ACBOOT
    }

    // Check if it's already on DBus
    conn->async_method_call(
        [this](boost::system::error_code ec,
               const dbusPropertiesList properties) {
            if (ec)
            {
                return;
            }
            setProperties(properties);
        },
        setingsService, powerRestorePolicyObject,
        "org.freedesktop.DBus.Properties", "GetAll", powerRestorePolicyIface);

#ifdef USE_ACBOOT
    // Check if it's already on DBus
    conn->async_method_call(
        [this](boost::system::error_code ec,
               const dbusPropertiesList properties) {
            if (ec)
            {
                return;
            }
            setProperties(properties);
        },
        setingsService, powerACBootObject, "org.freedesktop.DBus.Properties",
        "GetAll", powerACBootIface);
#endif
}

void PowerRestoreController::setProperties(const dbusPropertiesList& props)
{
    for (auto& [property, propValue] : props)
    {
        if (property == "PowerRestorePolicy")
        {
            const std::string* value = std::get_if<std::string>(&propValue);
            if (value == nullptr)
            {
                lg2::error("Unable to read Power Restore Policy");
                continue;
            }
            powerRestorePolicy = *value;
        }
        else if (property == "PowerRestoreDelay")
        {
            const uint64_t* value = std::get_if<uint64_t>(&propValue);
            if (value == nullptr)
            {
                lg2::error("Unable to read Power Restore Delay");
                continue;
            }
            powerRestoreDelay = *value / 1000000; // usec to sec
        }
#ifdef USE_ACBOOT
        else if (property == "ACBoot")
        {
            const std::string* value = std::get_if<std::string>(&propValue);
            if (value == nullptr)
            {
                lg2::error("Unable to read AC Boot status");
                continue;
            }
            acBoot = *value;
        }
#endif // USE_ACBOOT
    }
    invokeIfReady();
}

void PowerRestoreController::invokeIfReady()
{
    if ((powerRestorePolicy.empty()) || (powerRestoreDelay < 0))
    {
        return;
    }
#ifdef USE_ACBOOT
    if (acBoot.empty() || acBoot == "Unknown")
    {
        return;
    }
#endif

    matches.clear();
    if (!timerFired)
    {
        // Calculate the delay from now to meet the requested delay
        // Subtract the approximate uboot time
        static constexpr const int ubootSeconds = 20;
        int delay = powerRestoreDelay - ubootSeconds;
        // Subtract the time since boot
        struct sysinfo info = {};
        if (sysinfo(&info) == 0)
        {
            delay -= info.uptime;
        }

        if (delay > 0)
        {
            powerRestoreTimer.expires_after(std::chrono::seconds(delay));
            lg2::info("Power Restore delay of {DELAY} seconds started", "DELAY",
                      delay);
            powerRestoreTimer.async_wait([this](const boost::system::error_code
                                                    ec) {
                if (ec)
                {
                    // operation_aborted is expected if timer is canceled before
                    // completion.
                    if (ec == boost::asio::error::operation_aborted)
                    {
                        return;
                    }
                    lg2::error(
                        "power restore policy async_wait failed: {ERROR_MSG}",
                        "ERROR_MSG", ec.message());
                }
                else
                {
                    lg2::info("Power Restore delay timer expired");
                }
                invoke();
            });
            timerFired = true;
        }
        else
        {
            invoke();
        }
    }
}

void PowerRestoreController::invoke()
{
    // we want to run Power Restore only once
    if (policyInvoked)
    {
        return;
    }
    policyInvoked = true;

    lg2::info("Invoking Power Restore Policy {POLICY}", "POLICY",
              powerRestorePolicy);
    if (powerRestorePolicy ==
        "xyz.openbmc_project.Control.Power.RestorePolicy.Policy.AlwaysOn")
    {
        sendPowerControlEvent(Event::powerOnRequest);
        setRestartCauseProperty(getRestartCause(RestartCause::powerPolicyOn));
    }
    else if (powerRestorePolicy ==
             "xyz.openbmc_project.Control.Power.RestorePolicy.Policy.Restore")
    {
        if (wasPowerDropped())
        {
            lg2::info("Power was dropped, restoring Host On state");
            sendPowerControlEvent(Event::powerOnRequest);
            setRestartCauseProperty(
                getRestartCause(RestartCause::powerPolicyRestore));
        }
        else
        {
            lg2::info("No power drop, restoring Host Off state");
        }
    }
    // We're done with the previous power state for the restore policy, so store
    // the current state
    savePowerState(powerState);
}

bool PowerRestoreController::wasPowerDropped()
{
    std::string state = appState.get(PersistentState::Params::PowerState);
    return state == "xyz.openbmc_project.State.Chassis.PowerState.On";
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
    return true;
}

static int setMaskedGPIOOutputForMs(gpiod::line& maskedGPIOLine,
                                    const std::string& name, const int value,
                                    const int durationMs)
{
    // Set the masked GPIO line to the specified value
    maskedGPIOLine.set_value(value);
    lg2::info("{GPIO_NAME} set to {GPIO_VALUE}", "GPIO_NAME", name,
              "GPIO_VALUE", value);
    gpioAssertTimer.expires_after(std::chrono::milliseconds(durationMs));
    gpioAssertTimer.async_wait(
        [maskedGPIOLine, value, name](const boost::system::error_code ec) {
            // Set the masked GPIO line back to the opposite value
            maskedGPIOLine.set_value(!value);
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

static int setGPIOOutputForMs(const ConfigData& config, const int value,
                              const int durationMs)
{
    // If the requested GPIO is masked, use the mask line to set the output
    if (powerButtonMask && config.lineName == powerOutConfig.lineName)
    {
        return setMaskedGPIOOutputForMs(powerButtonMask, config.lineName, value,
                                        durationMs);
    }
    if (resetButtonMask && config.lineName == resetOutConfig.lineName)
    {
        return setMaskedGPIOOutputForMs(resetButtonMask, config.lineName, value,
                                        durationMs);
    }

    // No mask set, so request and set the GPIO normally
    gpiod::line gpioLine;
    if (!setGPIOOutput(config.lineName, value, gpioLine))
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

static int assertGPIOForMs(const ConfigData& config, const int durationMs)
{
    return setGPIOOutputForMs(config, config.polarity, durationMs);
}

static void powerOn()
{
#ifndef SOPHGO_CPLD
    assertGPIOForMs(powerOutConfig, TimerMap["PowerPulseMs"]);
#else
    set_cpld_pw_on_off(SetCpldPowerState::on);
#endif
}
#ifdef CHASSIS_SYSTEM_RESET
static int slotPowerOn()
{
    if (power_control::slotPowerState != power_control::SlotPowerState::on)
    {

        slotPowerLine.set_value(1);

        if (slotPowerLine.get_value() > 0)
        {
            setSlotPowerState(SlotPowerState::on);
            lg2::info("Slot Power is switched On\n");
        }
        else
        {
            return -1;
        }
    }
    else
    {
        lg2::info("Slot Power is already in 'On' state\n");
        return -1;
    }
    return 0;
}
static int slotPowerOff()
{
    if (power_control::slotPowerState != power_control::SlotPowerState::off)
    {
        slotPowerLine.set_value(0);

        if (!(slotPowerLine.get_value() > 0))
        {
            setSlotPowerState(SlotPowerState::off);
            setPowerState(PowerState::off);
            lg2::info("Slot Power is switched Off\n");
        }
        else
        {
            return -1;
        }
    }
    else
    {
        lg2::info("Slot Power is already in 'Off' state\n");
        return -1;
    }
    return 0;
}
static void slotPowerCycle()
{
    lg2::info("Slot Power Cycle started\n");
    slotPowerOff();
    slotPowerCycleTimer.expires_after(
        std::chrono::milliseconds(TimerMap["SlotPowerCycleMs"]));
    slotPowerCycleTimer.async_wait([](const boost::system::error_code ec) {
        if (ec)
        {
            if (ec != boost::asio::error::operation_aborted)
            {
                lg2::error(
                    "Slot Power cycle timer async_wait failed: {ERROR_MSG}",
                    "ERROR_MSG", ec.message());
            }
            lg2::info("Slot Power cycle timer canceled\n");
            return;
        }
        lg2::info("Slot Power cycle timer completed\n");
        slotPowerOn();
        lg2::info("Slot Power Cycle Completed\n");
    });
}
#endif
static void gracefulPowerOff()
{
#ifndef SOPHGO_CPLD
    assertGPIOForMs(powerOutConfig, TimerMap["PowerPulseMs"]);
#else
    set_cpld_pw_on_off(SetCpldPowerState::off);
#endif
}

static void forcePowerOff()
{
#ifndef SOPHGO_CPLD
    if (assertGPIOForMs(powerOutConfig, TimerMap["ForceOffPulseMs"]) < 0)
    {
        return;
    }
#else
    if (set_cpld_pw_on_off(SetCpldPowerState::off) < 0)
    {
        return;
    }
#endif


    // If the force off timer expires, then the power-button override failed
    gpioAssertTimer.async_wait([](const boost::system::error_code ec) {
        if (ec)
        {
            // operation_aborted is expected if timer is canceled before
            // completion.
            if (ec != boost::asio::error::operation_aborted)
            {
                lg2::error("Force power off async_wait failed: {ERROR_MSG}",
                           "ERROR_MSG", ec.message());
            }
            return;
        }

        lg2::error("Power-button override failed. Not sure what to do now.");
    });
}

static void reset()
{
#ifndef SOPHGO_CPLD
    assertGPIOForMs(resetOutConfig, TimerMap["ResetPulseMs"]);
#else
    return;
#endif
}

static void gracefulPowerOffTimerStart()
{
    lg2::info("Graceful power-off timer started");
    gracefulPowerOffTimer.expires_after(
        std::chrono::seconds(TimerMap["GracefulPowerOffS"]));
    gracefulPowerOffTimer.async_wait([](const boost::system::error_code ec) {
        if (ec)
        {
            // operation_aborted is expected if timer is canceled before
            // completion.
            if (ec != boost::asio::error::operation_aborted)
            {
                lg2::error("Graceful power-off async_wait failed: {ERROR_MSG}",
                           "ERROR_MSG", ec.message());
            }
            lg2::info("Graceful power-off timer canceled");
            return;
        }
        lg2::info("Graceful power-off timer completed");
        sendPowerControlEvent(Event::gracefulPowerOffTimerExpired);
    });
}

static void powerCycleTimerStart()
{
    lg2::info("Power-cycle timer started");
    powerCycleTimer.expires_after(
        std::chrono::milliseconds(TimerMap["PowerCycleMs"]));
    powerCycleTimer.async_wait([](const boost::system::error_code ec) {
        if (ec)
        {
            // operation_aborted is expected if timer is canceled before
            // completion.
            if (ec != boost::asio::error::operation_aborted)
            {
                lg2::error("Power-cycle async_wait failed: {ERROR_MSG}",
                           "ERROR_MSG", ec.message());
            }
            lg2::info("Power-cycle timer canceled");
            return;
        }
        lg2::info("Power-cycle timer completed");
        sendPowerControlEvent(Event::powerCycleTimerExpired);
    });
}

static void psPowerOKWatchdogTimerStart()
{
    lg2::info("power supply power OK watchdog timer started");
    psPowerOKWatchdogTimer.expires_after(
        std::chrono::milliseconds(TimerMap["PsPowerOKWatchdogMs"]));
    psPowerOKWatchdogTimer.async_wait([](const boost::system::error_code ec) {
        if (ec)
        {
            // operation_aborted is expected if timer is canceled before
            // completion.
            if (ec != boost::asio::error::operation_aborted)
            {
                lg2::error(
                    "power supply power OK watchdog async_wait failed: {ERROR_MSG}",
                    "ERROR_MSG", ec.message());
            }
            lg2::info("power supply power OK watchdog timer canceled");
            return;
        }
        lg2::info("power supply power OK watchdog timer expired");
        sendPowerControlEvent(Event::psPowerOKWatchdogTimerExpired);
    });
}

static void warmResetCheckTimerStart()
{
    lg2::info("Warm reset check timer started");
    warmResetCheckTimer.expires_after(
        std::chrono::milliseconds(TimerMap["WarmResetCheckMs"]));
    warmResetCheckTimer.async_wait([](const boost::system::error_code ec) {
        if (ec)
        {
            // operation_aborted is expected if timer is canceled before
            // completion.
            if (ec != boost::asio::error::operation_aborted)
            {
                lg2::error("Warm reset check async_wait failed: {ERROR_MSG}",
                           "ERROR_MSG", ec.message());
            }
            lg2::info("Warm reset check timer canceled");
            return;
        }
        lg2::info("Warm reset check timer completed");
        sendPowerControlEvent(Event::warmResetDetected);
    });
}

static void pohCounterTimerStart()
{
    lg2::info("POH timer started");
    // Set the time-out as 1 hour, to align with POH command in ipmid
    pohCounterTimer.expires_after(std::chrono::hours(1));
    pohCounterTimer.async_wait([](const boost::system::error_code& ec) {
        if (ec)
        {
            // operation_aborted is expected if timer is canceled before
            // completion.
            if (ec != boost::asio::error::operation_aborted)
            {
                lg2::error("POH timer async_wait failed: {ERROR_MSG}",
                           "ERROR_MSG", ec.message());
            }
            lg2::info("POH timer canceled");
            return;
        }

        if (getHostState(powerState) !=
            "xyz.openbmc_project.State.Host.HostState.Running")
        {
            return;
        }

        conn->async_method_call(
            [](boost::system::error_code ec,
               const std::variant<uint32_t>& pohCounterProperty) {
                if (ec)
                {
                    lg2::error("error getting poh counter");
                    return;
                }
                const uint32_t* pohCounter =
                    std::get_if<uint32_t>(&pohCounterProperty);
                if (pohCounter == nullptr)
                {
                    lg2::error("unable to read poh counter");
                    return;
                }

                conn->async_method_call(
                    [](boost::system::error_code ec) {
                        if (ec)
                        {
                            lg2::error("failed to set poh counter");
                        }
                    },
                    "xyz.openbmc_project.Settings",
                    "/xyz/openbmc_project/state/chassis0",
                    "org.freedesktop.DBus.Properties", "Set",
                    "xyz.openbmc_project.State.PowerOnHours", "POHCounter",
                    std::variant<uint32_t>(*pohCounter + 1));
            },
            "xyz.openbmc_project.Settings",
            "/xyz/openbmc_project/state/chassis0",
            "org.freedesktop.DBus.Properties", "Get",
            "xyz.openbmc_project.State.PowerOnHours", "POHCounter");

        pohCounterTimerStart();
    });
}

static void currentHostStateMonitor()
{
    if (getHostState(powerState) ==
        "xyz.openbmc_project.State.Host.HostState.Running")
    {
        pohCounterTimerStart();
        // Clear the restart cause set for the next restart
        clearRestartCause();
    }
    else
    {
        pohCounterTimer.cancel();
        // Set the restart cause set for this restart
        setRestartCause();
    }

    static auto match = sdbusplus::bus::match::match(
        *conn,
        "type='signal',member='PropertiesChanged', "
        "interface='org.freedesktop.DBus.Properties', "
        "arg0='xyz.openbmc_project.State.Host'",
        [](sdbusplus::message::message& message) {
            std::string intfName;
            std::map<std::string, std::variant<std::string>> properties;

            try
            {
                message.read(intfName, properties);
            }
            catch (const std::exception& e)
            {
                lg2::error("Unable to read host state: {ERROR}", "ERROR", e);
                return;
            }
            if (properties.empty())
            {
                lg2::error("ERROR: Empty PropertiesChanged signal received");
                return;
            }

            // We only want to check for CurrentHostState
            if (properties.begin()->first != "CurrentHostState")
            {
                return;
            }
            std::string* currentHostState =
                std::get_if<std::string>(&(properties.begin()->second));
            if (currentHostState == nullptr)
            {
                lg2::error("{PROPERTY} property invalid", "PROPERTY",
                           properties.begin()->first);
                return;
            }

            if (*currentHostState ==
                "xyz.openbmc_project.State.Host.HostState.Running")
            {
                pohCounterTimerStart();
                // Clear the restart cause set for the next restart
                clearRestartCause();
                sd_journal_send("MESSAGE=Host system DC power is on",
                                "PRIORITY=%i", LOG_INFO,
                                "REDFISH_MESSAGE_ID=%s",
                                "OpenBMC.0.1.DCPowerOn", NULL);
            }
            else
            {
                pohCounterTimer.cancel();
                // POST_COMPLETE GPIO event is not working in some platforms
                // when power state is changed to OFF. This resulted in
                // 'OperatingSystemState' to stay at 'Standby', even though
                // system is OFF. Set 'OperatingSystemState' to 'Inactive'
                // if HostState is trurned to OFF.
                setOperatingSystemState(OperatingSystemStateStage::Inactive);

                // Set the restart cause set for this restart
                setRestartCause();
#ifdef USE_ACBOOT
                resetACBootProperty();
#endif // USE_ACBOOT
                sd_journal_send("MESSAGE=Host system DC power is off",
                                "PRIORITY=%i", LOG_INFO,
                                "REDFISH_MESSAGE_ID=%s",
                                "OpenBMC.0.1.DCPowerOff", NULL);
            }
        });
}

static void sioPowerGoodWatchdogTimerStart()
{
    lg2::info("SIO power good watchdog timer started");
    sioPowerGoodWatchdogTimer.expires_after(
        std::chrono::milliseconds(TimerMap["SioPowerGoodWatchdogMs"]));
    sioPowerGoodWatchdogTimer.async_wait([](const boost::system::error_code
                                                ec) {
        if (ec)
        {
            // operation_aborted is expected if timer is canceled before
            // completion.
            if (ec != boost::asio::error::operation_aborted)
            {
                lg2::error(
                    "SIO power good watchdog async_wait failed: {ERROR_MSG}",
                    "ERROR_MSG", ec.message());
            }
            lg2::info("SIO power good watchdog timer canceled");
            return;
        }
        lg2::info("SIO power good watchdog timer completed");
        sendPowerControlEvent(Event::sioPowerGoodWatchdogTimerExpired);
    });
}



static void powerStateOn(const Event event)
{
    logEvent(__FUNCTION__, event);
    switch (event)
    {
        case Event::psPowerOKDeAssert:
            setPowerState(PowerState::off);
            // DC power is unexpectedly lost, beep
            beep(beepPowerFail);
            break;
        case Event::sioS5Assert:
            setPowerState(PowerState::transitionToOff);
            addRestartCause(RestartCause::softReset);
            break;
#if USE_PLT_RST
        case Event::pltRstAssert:
#else
        case Event::postCompleteDeAssert:
#endif
            setPowerState(PowerState::checkForWarmReset);
            addRestartCause(RestartCause::softReset);
            warmResetCheckTimerStart();
            break;
        case Event::powerButtonPressed:
            setPowerState(PowerState::gracefulTransitionToOff);
            gracefulPowerOffTimerStart();
            break;
        case Event::powerOffRequest:
            setPowerState(PowerState::transitionToOff);
            forcePowerOff();
            break;
        case Event::gracefulPowerOffRequest:
            setPowerState(PowerState::gracefulTransitionToOff);
            gracefulPowerOffTimerStart();
            gracefulPowerOff();
            break;
        case Event::powerCycleRequest:
            setPowerState(PowerState::transitionToCycleOff);
            forcePowerOff();
            break;
        case Event::gracefulPowerCycleRequest:
            setPowerState(PowerState::gracefulTransitionToCycleOff);
            gracefulPowerOffTimerStart();
            gracefulPowerOff();
            break;
        case Event::resetRequest:
            reset();
            break;
        default:
            lg2::info("No action taken.");
            break;
    }
}

static void powerStateWaitForPSPowerOK(const Event event)
{
    logEvent(__FUNCTION__, event);
    switch (event)
    {
        case Event::psPowerOKAssert:
        {
            // Cancel any GPIO assertions held during the transition
            gpioAssertTimer.cancel();
            psPowerOKWatchdogTimer.cancel();
            if (sioEnabled == true)
            {
                sioPowerGoodWatchdogTimerStart();
                setPowerState(PowerState::waitForSIOPowerGood);
            }
            else
            {
                setPowerState(PowerState::on);
            }

            cpldControlStateClear();
            break;
        }
        case Event::psPowerOKWatchdogTimerExpired:
            setPowerState(PowerState::off);
            psPowerOKFailedLog();
            cpldControlStateClear();
            break;
        case Event::sioPowerGoodAssert:
            psPowerOKWatchdogTimer.cancel();
            setPowerState(PowerState::on);
            break;
        default:
            lg2::info("No action taken.");
            break;
    }
}

static void powerStateWaitForSIOPowerGood(const Event event)
{
    logEvent(__FUNCTION__, event);
    switch (event)
    {
        case Event::sioPowerGoodAssert:
            sioPowerGoodWatchdogTimer.cancel();
            setPowerState(PowerState::on);
            break;
        case Event::sioPowerGoodWatchdogTimerExpired:
            setPowerState(PowerState::off);
            systemPowerGoodFailedLog();
            break;
        default:
            lg2::info("No action taken.");
            break;
    }
}

static void powerStateOff(const Event event)
{
    logEvent(__FUNCTION__, event);
    switch (event)
    {
        case Event::psPowerOKAssert:
        {
            if (sioEnabled == true)
            {
                sioPowerGoodWatchdogTimerStart();
                setPowerState(PowerState::waitForSIOPowerGood);
            }
            else
            {
                setPowerState(PowerState::on);
            }
            break;
        }
        case Event::sioS5DeAssert:
            setPowerState(PowerState::waitForPSPowerOK);
            break;
        case Event::sioPowerGoodAssert:
            setPowerState(PowerState::on);
            break;
        case Event::powerButtonPressed:
            psPowerOKWatchdogTimerStart();
            setPowerState(PowerState::waitForPSPowerOK);
            break;
        case Event::powerOnRequest:
            psPowerOKWatchdogTimerStart();
            setPowerState(PowerState::waitForPSPowerOK);
            powerOn();
            break;
        default:
            lg2::info("No action taken.");
            break;
    }
}

static void powerStateTransitionToOff(const Event event)
{
    logEvent(__FUNCTION__, event);
    switch (event)
    {
        case Event::psPowerOKDeAssert:
            // Cancel any GPIO assertions held during the transition
            gpioAssertTimer.cancel();
            setPowerState(PowerState::off);
            break;
        default:
            lg2::info("No action taken.");
            break;
    }
}

static void powerStateGracefulTransitionToOff(const Event event)
{
    logEvent(__FUNCTION__, event);
    switch (event)
    {
        case Event::psPowerOKDeAssert:
            gracefulPowerOffTimer.cancel();
            setPowerState(PowerState::off);
            //clear cpld control
            cpldControlStateClear();
            break;
        case Event::gracefulPowerOffTimerExpired:
            setPowerState(PowerState::on);
            break;
        case Event::powerOffRequest:
            gracefulPowerOffTimer.cancel();
            setPowerState(PowerState::transitionToOff);
            forcePowerOff();
            break;
        case Event::powerCycleRequest:
            gracefulPowerOffTimer.cancel();
            setPowerState(PowerState::transitionToCycleOff);
            forcePowerOff();
            break;
        case Event::resetRequest:
            gracefulPowerOffTimer.cancel();
            setPowerState(PowerState::on);
            reset();
            break;
        default:
            lg2::info("No action taken.");
            break;
    }
}

static void powerStateCycleOff(const Event event)
{
    logEvent(__FUNCTION__, event);
    switch (event)
    {
        case Event::psPowerOKAssert:
        {
            powerCycleTimer.cancel();
            if (sioEnabled == true)
            {
                sioPowerGoodWatchdogTimerStart();
                setPowerState(PowerState::waitForSIOPowerGood);
            }
            else
            {
                setPowerState(PowerState::on);
            }
            break;
        }
        case Event::sioS5DeAssert:
            powerCycleTimer.cancel();
            setPowerState(PowerState::waitForPSPowerOK);
            break;
        case Event::powerButtonPressed:
            powerCycleTimer.cancel();
            psPowerOKWatchdogTimerStart();
            setPowerState(PowerState::waitForPSPowerOK);
            break;
        case Event::powerCycleTimerExpired:
            psPowerOKWatchdogTimerStart();
            setPowerState(PowerState::waitForPSPowerOK);
            powerOn();
            break;
        default:
            lg2::info("No action taken.");
            break;
    }
}

static void powerStateTransitionToCycleOff(const Event event)
{
    logEvent(__FUNCTION__, event);
    switch (event)
    {
        case Event::psPowerOKDeAssert:
            // Cancel any GPIO assertions held during the transition
            gpioAssertTimer.cancel();
            setPowerState(PowerState::cycleOff);
            powerCycleTimerStart();
            break;
        default:
            lg2::info("No action taken.");
            break;
    }
}

static void powerStateGracefulTransitionToCycleOff(const Event event)
{
    logEvent(__FUNCTION__, event);
    switch (event)
    {
        case Event::psPowerOKDeAssert:
            gracefulPowerOffTimer.cancel();
            setPowerState(PowerState::cycleOff);
            powerCycleTimerStart();
            break;
        case Event::gracefulPowerOffTimerExpired:
            setPowerState(PowerState::on);
            break;
        case Event::powerOffRequest:
            gracefulPowerOffTimer.cancel();
            setPowerState(PowerState::transitionToOff);
            forcePowerOff();
            break;
        case Event::powerCycleRequest:
            gracefulPowerOffTimer.cancel();
            setPowerState(PowerState::transitionToCycleOff);
            forcePowerOff();
            break;
        case Event::resetRequest:
            gracefulPowerOffTimer.cancel();
            setPowerState(PowerState::on);
            reset();
            break;
        default:
            lg2::info("No action taken.");
            break;
    }
}

static void powerStateCheckForWarmReset(const Event event)
{
    logEvent(__FUNCTION__, event);
    switch (event)
    {
        case Event::sioS5Assert:
            warmResetCheckTimer.cancel();
            setPowerState(PowerState::transitionToOff);
            break;
        case Event::warmResetDetected:
            setPowerState(PowerState::on);
            break;
        case Event::psPowerOKDeAssert:
            warmResetCheckTimer.cancel();
            setPowerState(PowerState::off);
            // DC power is unexpectedly lost, beep
            beep(beepPowerFail);
            break;
        default:
            lg2::info("No action taken.");
            break;
    }
}

static void psPowerOKHandler(bool state)
{
    Event powerControlEvent =
        state ? Event::psPowerOKAssert : Event::psPowerOKDeAssert;
    sendPowerControlEvent(powerControlEvent);
}

static void sioPowerGoodHandler(bool state)
{
    Event powerControlEvent =
        state ? Event::sioPowerGoodAssert : Event::sioPowerGoodDeAssert;
    sendPowerControlEvent(powerControlEvent);
}

static void sioOnControlHandler(bool state)
{
    lg2::info("SIO_ONCONTROL value changed: {VALUE}", "VALUE",
              static_cast<int>(state));
}

static void sioS5Handler(bool state)
{
    Event powerControlEvent = state ? Event::sioS5DeAssert : Event::sioS5Assert;
    sendPowerControlEvent(powerControlEvent);
}

static void powerButtonHandler(bool state)
{
    powerButtonIface->set_property("ButtonPressed", !state);
    if (!state)
    {
        powerButtonPressLog();
        if (!powerButtonMask)
        {
            sendPowerControlEvent(Event::powerButtonPressed);
            addRestartCause(RestartCause::powerButton);
        }
        else
        {
            lg2::info("power button press masked");
        }
    }
}

static void resetButtonHandler(bool state)
{
    resetButtonIface->set_property("ButtonPressed", !state);
    if (!state)
    {
        resetButtonPressLog();
        if (!resetButtonMask)
        {
            sendPowerControlEvent(Event::resetButtonPressed);
            addRestartCause(RestartCause::resetButton);
        }
        else
        {
            lg2::info("reset button press masked");
        }
    }
}

#ifdef CHASSIS_SYSTEM_RESET
static constexpr auto systemdBusname = "org.freedesktop.systemd1";
static constexpr auto systemdPath = "/org/freedesktop/systemd1";
static constexpr auto systemdInterface = "org.freedesktop.systemd1.Manager";
static constexpr auto systemTargetName = "chassis-system-reset.target";

void systemReset()
{
    conn->async_method_call(
        [](boost::system::error_code ec) {
            if (ec)
            {
                lg2::error("Failed to call chassis system reset: {ERR}", "ERR",
                           ec.message());
            }
        },
        systemdBusname, systemdPath, systemdInterface, "StartUnit",
        systemTargetName, "replace");
}
#endif

static void nmiSetEnableProperty(bool value)
{
    conn->async_method_call(
        [](boost::system::error_code ec) {
            if (ec)
            {
                lg2::error("failed to set NMI source");
            }
        },
        "xyz.openbmc_project.Settings",
        "/xyz/openbmc_project/Chassis/Control/NMISource",
        "org.freedesktop.DBus.Properties", "Set",
        "xyz.openbmc_project.Chassis.Control.NMISource", "Enabled",
        std::variant<bool>{value});
}

static void nmiReset(void)
{
    static constexpr const uint8_t value = 1;
    const static constexpr int nmiOutPulseTimeMs = 200;

    lg2::info("NMI out action");
    nmiOutLine.set_value(value);
    lg2::info("{GPIO_NAME} set to {GPIO_VALUE}", "GPIO_NAME",
              nmiOutConfig.lineName, "GPIO_VALUE", value);
    gpioAssertTimer.expires_after(std::chrono::milliseconds(nmiOutPulseTimeMs));
    gpioAssertTimer.async_wait([](const boost::system::error_code ec) {
        // restore the NMI_OUT GPIO line back to the opposite value
        nmiOutLine.set_value(!value);
        lg2::info("{GPIO_NAME} released", "GPIO_NAME", nmiOutConfig.lineName);
        if (ec)
        {
            // operation_aborted is expected if timer is canceled before
            // completion.
            if (ec != boost::asio::error::operation_aborted)
            {
                lg2::error("{GPIO_NAME} async_wait failed: {ERROR_MSG}",
                           "GPIO_NAME", nmiOutConfig.lineName, "ERROR_MSG",
                           ec.message());
            }
        }
    });
    // log to redfish
    nmiDiagIntLog();
    lg2::info("NMI out action completed");
    // reset Enable Property
    nmiSetEnableProperty(false);
}

static void nmiSourcePropertyMonitor(void)
{
    lg2::info("NMI Source Property Monitor");

    static std::unique_ptr<sdbusplus::bus::match::match> nmiSourceMatch =
        std::make_unique<sdbusplus::bus::match::match>(
            *conn,
            "type='signal',interface='org.freedesktop.DBus.Properties',"
            "member='PropertiesChanged',"
            "arg0namespace='xyz.openbmc_project.Chassis.Control.NMISource'",
            [](sdbusplus::message::message& msg) {
                std::string interfaceName;
                boost::container::flat_map<std::string,
                                           std::variant<bool, std::string>>
                    propertiesChanged;
                std::string state;
                bool value = true;
                try
                {
                    msg.read(interfaceName, propertiesChanged);
                    if (propertiesChanged.begin()->first == "Enabled")
                    {
                        value =
                            std::get<bool>(propertiesChanged.begin()->second);
                        lg2::info(
                            "NMI Enabled propertiesChanged value: {VALUE}",
                            "VALUE", value);
                        nmiEnabled = value;
                        if (nmiEnabled)
                        {
                            nmiReset();
                        }
                    }
                }
                catch (const std::exception& e)
                {
                    lg2::error("Unable to read NMI source: {ERROR}", "ERROR",
                               e);
                    return;
                }
            });
}

static void setNmiSource()
{
    conn->async_method_call(
        [](boost::system::error_code ec) {
            if (ec)
            {
                lg2::error("failed to set NMI source");
            }
        },
        "xyz.openbmc_project.Settings",
        "/xyz/openbmc_project/Chassis/Control/NMISource",
        "org.freedesktop.DBus.Properties", "Set",
        "xyz.openbmc_project.Chassis.Control.NMISource", "BMCSource",
        std::variant<std::string>{
            "xyz.openbmc_project.Chassis.Control.NMISource.BMCSourceSignal.FpBtn"});
    // set Enable Property
    nmiSetEnableProperty(true);
}

static void nmiButtonHandler(bool state)
{
    nmiButtonIface->set_property("ButtonPressed", !state);
    if (!state)
    {
        nmiButtonPressLog();
        if (nmiButtonMasked)
        {
            lg2::info("NMI button press masked");
        }
        else
        {
            setNmiSource();
        }
    }
}

static void idButtonHandler(bool state)
{
    idButtonIface->set_property("ButtonPressed", !state);
}

static void pltRstHandler(bool pltRst)
{
    if (pltRst)
    {
        sendPowerControlEvent(Event::pltRstDeAssert);
    }
    else
    {
        sendPowerControlEvent(Event::pltRstAssert);
    }
}

[[maybe_unused]] static void hostMiscHandler(sdbusplus::message::message& msg)
{
    std::string interfaceName;
    boost::container::flat_map<std::string, std::variant<bool>>
        propertiesChanged;
    try
    {
        msg.read(interfaceName, propertiesChanged);
    }
    catch (const std::exception& e)
    {
        lg2::error("Unable to read Host Misc status: {ERROR}", "ERROR", e);
        return;
    }
    if (propertiesChanged.empty())
    {
        lg2::error("ERROR: Empty Host.Misc PropertiesChanged signal received");
        return;
    }

    for (auto& [property, value] : propertiesChanged)
    {
        if (property == "ESpiPlatformReset")
        {
            bool* pltRst = std::get_if<bool>(&value);
            if (pltRst == nullptr)
            {
                lg2::error("{PROPERTY} property invalid", "PROPERTY", property);
                return;
            }
            pltRstHandler(*pltRst);
        }
    }
}

static void postCompleteHandler(bool state)
{
    if (!state)
    {
        sendPowerControlEvent(Event::postCompleteAssert);
        setOperatingSystemState(OperatingSystemStateStage::Standby);
    }
    else
    {
        sendPowerControlEvent(Event::postCompleteDeAssert);
        setOperatingSystemState(OperatingSystemStateStage::Inactive);
    }
}

static int loadConfigValues()
{
    const std::string configFilePath =
        "/usr/share/sophgo-power-control/power-config-host" + power_control::node +
        ".json";
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
    auto cpld     = jsonData["i2c_config"];

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

    //get cpld info from json config
    nlohmann::json& i2cConfig = cpld;
    cpldInfo.deviceName = i2cConfig["deviceName"];
    if(cpldInfo.deviceName.empty())
    {
        lg2::error("get i2c deviceName from json config failed!");
        return -1;
    }
    else
    {
        lg2::info("I2C deviceName : \'{NAME}\'", "NAME", cpldInfo.deviceName);
    }
    cpldInfo.deviceAddr = i2cConfig["deviceAddr"];
    lg2::info("I2C deviceAddr : \'{ADDR}\'", "ADDR", cpldInfo.deviceAddr);
    cpldInfo.PWControlReg = i2cConfig["PWControlReg"];
    lg2::info("I2C PWControlReg : \'{ADDR}\'", "ADDR", cpldInfo.PWControlReg);
    cpldInfo.CPUAPWStateReg = i2cConfig["CPUAPWStateReg"];
    lg2::info("I2C CPUAPWStateReg : \'{ADDR}\'", "ADDR", cpldInfo.CPUAPWStateReg);
    cpldInfo.CPUBPWStateReg = i2cConfig["CPUBPWStateReg"];
    lg2::info("I2C CPUBPWStateReg : \'{ADDR}\'", "ADDR", cpldInfo.CPUBPWStateReg);
    cpldInfo.CPUOPWStateReg = i2cConfig["CPUOPWStateReg"];
    lg2::info("I2C CPUOPWStateReg : \'{ADDR}\'", "ADDR", cpldInfo.CPUOPWStateReg);
    cpldInfo.OPWStateReg = i2cConfig["OPWStateReg"];
    lg2::info("I2C OPWStateReg : \'{ADDR}\'", "ADDR", cpldInfo.OPWStateReg);
    cpldInfo.PSUStateReg = i2cConfig["PSUStateReg"];
    lg2::info("I2C PSUStateReg : \'{ADDR}\'", "ADDR", cpldInfo.PSUStateReg);
    cpldInfo.EFUSStateReg = i2cConfig["EFUSStateReg"];
    lg2::info("I2C EFUSStateReg : \'{ADDR}\'", "ADDR", cpldInfo.EFUSStateReg);
    cpldInfo.resetControlAReg = i2cConfig["resetControlAReg"];
    lg2::info("I2C resetControlAReg : \'{ADDR}\'", "ADDR", cpldInfo.resetControlAReg);
    cpldInfo.resetControlBReg = i2cConfig["resetControlBReg"];
    lg2::info("I2C resetControlBReg : \'{ADDR}\'", "ADDR", cpldInfo.resetControlBReg);

    return 0;
}

static bool getDbusMsgGPIOState(sdbusplus::message::message& msg,
                                const std::string& lineName, bool& value)
{
    std::string thresholdInterface;
    std::string event;
    boost::container::flat_map<std::string, std::variant<bool>>
        propertiesChanged;
    try
    {
        msg.read(thresholdInterface, propertiesChanged);
        if (propertiesChanged.empty())
        {
            return false;
        }

        event = propertiesChanged.begin()->first;
        if (event.empty() || event != lineName)
        {
            return false;
        }

        value = std::get<bool>(propertiesChanged.begin()->second);
        return true;
    }
    catch (const std::exception& e)
    {
        lg2::error(
            "exception while reading dbus property \'{DBUS_NAME}\': {ERROR}",
            "DBUS_NAME", lineName, "ERROR", e);
        return false;
    }
}

static sdbusplus::bus::match::match
    dbusGPIOMatcher(const ConfigData& cfg, std::function<void(bool)> onMatch)
{
    auto pulseEventMatcherCallback =
        [&cfg, onMatch](sdbusplus::message::message& msg) {
            bool value = false;
            if (!getDbusMsgGPIOState(msg, cfg.lineName, value))
            {
                return;
            }
            onMatch(value);
        };

    return sdbusplus::bus::match::match(
        static_cast<sdbusplus::bus::bus&>(*conn),
        "type='signal',interface='org.freedesktop.DBus.Properties',member='"
        "PropertiesChanged',arg0='" +
            cfg.interface + "'",
        std::move(pulseEventMatcherCallback));
}

int getProperty(ConfigData& configData)
{
    auto method = conn->new_method_call(
        configData.dbusName.c_str(), configData.path.c_str(),
        "org.freedesktop.DBus.Properties", "Get");
    method.append(configData.interface.c_str(), configData.lineName.c_str());

    auto reply = conn->call(method);
    if (reply.is_method_error())
    {
        lg2::error(
            "Error reading {PROPERTY} D-Bus property on interface {INTERFACE} and path {PATH}",
            "PROPERTY", configData.lineName, "INTERFACE", configData.interface,
            "PATH", configData.path);
        return -1;
    }
    std::variant<bool> resp;
    reply.read(resp);
    auto respValue = std::get_if<bool>(&resp);
    if (!respValue)
    {
        lg2::error("Error: {PROPERTY} D-Bus property is not the expected type",
                   "PROPERTY", configData.lineName);
        return -1;
    }
    return (*respValue);
}


int set_default_cpld_info(void)
{
    cpldInfo.deviceName       = std::string("/dev/i2c-8");
    cpldInfo.deviceAddr       = 0x57;
    cpldInfo.PWControlReg     = 0x02;
    cpldInfo.CPUAPWStateReg   = 0x03;
    cpldInfo.CPUBPWStateReg   = 0x04;
    cpldInfo.CPUOPWStateReg   = 0x05;
    cpldInfo.OPWStateReg      = 0x06;
    cpldInfo.PSUStateReg      = 0x07;
    cpldInfo.EFUSStateReg     = 0x08;
    cpldInfo.resetControlAReg = 0x09;
    cpldInfo.resetControlBReg = 0x0A;
}

int cpld_i2c_init(void)
{
    cpldInfo.i2cFile = open(cpldInfo.deviceName.c_str(), O_RDWR);
    if (cpldInfo.i2cFile < 0)
    {
        lg2::info("unable to open i2c device ");
        return -1;
    }
    else
    {
        lg2::info("i2cFile : {FD}","FD",cpldInfo.i2cFile);
    }
    if (ioctl(cpldInfo.i2cFile, I2C_SLAVE_FORCE, cpldInfo.deviceAddr) < 0)
    {
        lg2::error("unable to set device address");
        close(cpldInfo.i2cFile);
        return -1;
    }


    return 1;
}


static int get_cpld_info(char addr, char reg, char *value)
{
    struct i2c_rdwr_ioctl_data cpld_data;
    unsigned char   tempBuffer0[4] = {0,0,0,0};
    unsigned char   tempBuffer1[4] = {0,0,0,0};

    cpld_data.nmsgs = 2;

    cpld_data.msgs=(struct i2c_msg*)malloc(cpld_data.nmsgs*sizeof(struct i2c_msg));
    if(!cpld_data.msgs)
    {
         lg2::error("malloc error");
        return -1;
    }

    (cpld_data.msgs[0]).len      = 1;
    (cpld_data.msgs[0]).addr     = addr;
    (cpld_data.msgs[0]).flags    = 0;//write
    (cpld_data.msgs[0]).buf      = tempBuffer0;//数据地址
    (cpld_data.msgs[0]).buf[0]   = reg;//数据地址
    (cpld_data.msgs[1]).len      = 1;//读出的数据
    (cpld_data.msgs[1]).addr     = addr;// e2prom 设备地址
    (cpld_data.msgs[1]).flags    = I2C_M_RD;//read
    (cpld_data.msgs[1]).buf      = tempBuffer1;//存放返回值的地址。
    (cpld_data.msgs[1]).buf[0]   = 0;//初始化读缓冲
    i2cMutex.lock();
    int ret=ioctl(cpldInfo.i2cFile,I2C_RDWR,(unsigned long)&cpld_data);
    i2cMutex.unlock();
    if(ret<0)
    {
        lg2::error("i2c read error addr: {ADDR}, reg: {REG}", "ADDR", addr, "REG", reg);
        free(cpld_data.msgs);
        return -1;
    }
    else
    {
        *value = (cpld_data.msgs[1]).buf[0];
        free(cpld_data.msgs);
        return 1;
    }
}


int set_cpld_pw_on_off(SetCpldPowerState state)
{
    //0-off 1-on
    struct i2c_rdwr_ioctl_data cpld_data;
    unsigned char   tempBuffer0[4];

    std::cout << "set_cpld_pw_on_off:" << boost::this_thread::get_id() <<std:: endl;

    cpld_data.nmsgs=1;

    cpld_data.msgs=(struct i2c_msg*)malloc(cpld_data.nmsgs*sizeof(struct i2c_msg));
    if(!cpld_data.msgs)
    {
         lg2::error("malloc error");
        return -1;
    }

    (cpld_data.msgs[0]).len    = 2;
    (cpld_data.msgs[0]).addr   = cpldInfo.deviceAddr;//设备地址
    (cpld_data.msgs[0]).flags  = 0; //write
    (cpld_data.msgs[0]).buf    = tempBuffer0;
    (cpld_data.msgs[0]).buf[0] = cpldInfo.PWControlReg;// 写入目标的地址
    if(state == SetCpldPowerState::on)
    {
        (cpld_data.msgs[0]).buf[1] = 0x81;//the data to write
    }
    else if(state == SetCpldPowerState::off)
    {
        (cpld_data.msgs[0]).buf[1] = 0x80;//the data to write
    }
    else if(state == SetCpldPowerState::clear)
    {
        (cpld_data.msgs[0]).buf[1] = 0x00;//the data to write
    }
    i2cMutex.lock();
    int ret=ioctl(cpldInfo.i2cFile,I2C_RDWR,(unsigned long)&cpld_data);
    i2cMutex.unlock();
    if(ret<0)
    {
        lg2::error("i2c write error !");
        free(cpld_data.msgs);
        return -1;
    }
    else
    {
        free(cpld_data.msgs);
        return 1;
    }
}

int get_all_cpld_info(void)
{
    char tempValue;
    if(get_cpld_info(cpldInfo.deviceAddr, cpldInfo.CPUAPWStateReg, &tempValue) < 0)
    {
        return -1;
    }
    else
    {
        CPUA_pw_state = (struct CPUAPWStateRegInfo)(tempValue);
    }

    if(get_cpld_info(cpldInfo.deviceAddr, cpldInfo.CPUBPWStateReg, &tempValue) < 0)
    {
        return -1;
    }
    else
    {
        CPUB_pw_state = (struct CPUBPWStateRegInfo)(tempValue);
    }

    if(get_cpld_info(cpldInfo.deviceAddr, cpldInfo.CPUOPWStateReg, &tempValue) < 0)
    {
        return -1;
    }
    else
    {
        CPUO_pw_state = (struct CPUOPWStateRegInfo)(tempValue);
    }


    if(get_cpld_info(cpldInfo.deviceAddr, cpldInfo.OPWStateReg, &tempValue) < 0)
    {
        return -1;
    }
    else
    {
        OPW_state = (struct OPWStateRegInfo)(tempValue);
    }

    if(get_cpld_info(cpldInfo.deviceAddr, cpldInfo.PSUStateReg, &tempValue) < 0)
    {
        return -1;
    }
    else
    {
        PSU_state = (struct PSUStateRegInfo)(tempValue);
    }

    if(get_cpld_info(cpldInfo.deviceAddr, cpldInfo.EFUSStateReg, &tempValue) < 0)
    {
        return -1;
    }
    else
    {
        EFUS_state = (struct EFUSStateRegInfo)(tempValue);
    }


    return 1;
}






static std::string_view getCPUAPWState(void)
{
    std::string str0 = "V";
#if 0
    char tempValue;

    if(get_cpld_info(cpldInfo.deviceAddr, cpldInfo.CPUAPWStateReg, &tempValue) < 0)
    {
        ;
    }
    else
    {
        CPUA_pw_state = (struct CPUAPWStateRegInfo)(tempValue);
    }
#endif
    if(CPUA_pw_state.PG_VPP_A1)
        str0 += "0";
    if(CPUA_pw_state.PG_VPP_A0)
        str0 += "1";
    if(CPUA_pw_state.PG_VDDQ_A1)
        str0 += "2";
    if(CPUA_pw_state.PG_VDDQ_A0)
        str0 += "3";
    if(CPUA_pw_state.PG_VTT_A1)
        str0 += "4";
    if(CPUA_pw_state.PG_VTT_A0)
        str0 += "5";
    if(CPUA_pw_state.PG_PCIE_PHY_A)
        str0 += "6";
    if(CPUA_pw_state.PG_VDDC_A)
        str0 += "7";
    return str0;
}




static std::string_view getCPUBPWState(void)
{
    std::string str0 = "V";
#if 0
    char tempValue;
    if(get_cpld_info(cpldInfo.deviceAddr, cpldInfo.CPUBPWStateReg, &tempValue) < 0)
    {
        ;
    }
    else
    {
        CPUB_pw_state = (struct CPUBPWStateRegInfo)(tempValue);
    }
#endif
    if(CPUB_pw_state.PG_VPP_A1)
        str0 += "0";
    if(CPUB_pw_state.PG_VPP_A0)
        str0 += "1";
    if(CPUB_pw_state.PG_VDDQ_A1)
        str0 += "2";
    if(CPUB_pw_state.PG_VDDQ_A0)
        str0 += "3";
    if(CPUB_pw_state.PG_VTT_A1)
        str0 += "4";
    if(CPUB_pw_state.PG_VTT_A0)
        str0 += "5";
    if(CPUB_pw_state.PG_PCIE_PHY_A)
        str0 += "6";
    if(CPUB_pw_state.PG_VDDC_A)
        str0 += "7";

    return str0;
}


static std::string_view getCPUOPWState(void)
{

    std::string str0 = "V";
#if 0
    char tempValue;
    if(get_cpld_info(cpldInfo.deviceAddr, cpldInfo.CPUOPWStateReg, &tempValue) < 0)
    {
        //return -1;
    }
    else
    {
        CPUO_pw_state = (struct CPUOPWStateRegInfo)(tempValue);
    }
#endif
    if(CPUO_pw_state.PG_VDD_3V3)
        str0 += "0";
    if(CPUO_pw_state.PG_VDD_1V8)
        str0 += "1";
    if(CPUO_pw_state.PG_PCIE_H_B)
        str0 += "2";
    if(CPUO_pw_state.PG_DDR_PHY_B)
        str0 += "3";
    if(CPUO_pw_state.PG_PCIE_H_A)
        str0 += "4";
    if(CPUO_pw_state.PG_DDR_PHY_A)
        str0 += "5";

    return str0;
}



static std::string_view getOtherPWState(void)
{
    std::string str0 = "V";
#if 0
    char tempValue;
    if(get_cpld_info(cpldInfo.deviceAddr, cpldInfo.OPWStateReg, &tempValue) < 0)
    {
        //return -1;
    }
    else
    {
        OPW_state = (struct OPWStateRegInfo)(tempValue);
    }
#endif
    if(OPW_state.PG_PCIE_SW_B)
        str0 += "0";
    if(OPW_state.PG_PCIE_SW_A)
        str0 += "1";
    if(OPW_state.PG_3V3_RISER)
        str0 += "2";
    if(OPW_state.PG_BP_5V)
        str0 += "3";
    if(OPW_state.PG_5V_AUX)
        str0 += "4";
    if(OPW_state.PG_VDD_5V)
        str0 += "5";

    return str0;
}


static std::string_view getPSUState(void)
{
    std::string str0 = "V";
#if 0
    char tempValue;
    if(get_cpld_info(cpldInfo.deviceAddr, cpldInfo.PSUStateReg, &tempValue) < 0)
    {
        //return -1;
    }
    else
    {
        PSU_state = (struct PSUStateRegInfo)(tempValue);
    }
#endif
    if(PSU_state.PSU1_PRSNT_N)
        str0 += "0";
    if(PSU_state.PSU0_PRSNT_N)
        str0 += "1";
    if(PSU_state.PSU1_ALERT_N)
        str0 += "2";
    if(PSU_state.PSU0_ALERT_N)
        str0 += "3";
    if(PSU_state.PSU1_PWROK)
        str0 += "4";
    if(PSU_state.PSU0_PWROK)
        str0 += "5";

    return str0;
}

static std::string_view getEFUSState(void)
{
    std::string str0 = "V";
#if 0
    char tempValue;
    if(get_cpld_info(cpldInfo.deviceAddr, cpldInfo.EFUSStateReg, &tempValue) < 0)
    {
        //return -1;
    }
    else
    {
        EFUS_state = (struct EFUSStateRegInfo)(tempValue);
    }
#endif
    if(EFUS_state.NCP0_GOK)
        str0 += "0";
    if(EFUS_state.NCP0_D_OC)
        str0 += "1";
    if(EFUS_state.NCP1_GOK)
        str0 += "2";
    if(EFUS_state.NCP1_D_OC)
        str0 += "3";

    return str0;
}




static void setCpldInfoDbusProperty(void)
{
    hostIface->set_property("CurrentCPUAPWState",
                        std::string(getCPUAPWState()));
    hostIface->set_property("CurrentCPUBPWState",
                        std::string(getCPUBPWState()));
    hostIface->set_property("CurrentCPUOtherPWState",
                        std::string(getCPUOPWState()));
    hostIface->set_property("CurrentOtherPWState",
                        std::string(getOtherPWState()));
    hostIface->set_property("CurrentPSUState",
                        std::string(getPSUState()));
    hostIface->set_property("CurrentEFUState",
                        std::string(getEFUSState()));
}



bool check_power_state(void)
{
    //ON和OFF的标准是什么,暂定只判断CPU A和B
    if((getCPUAPWState() == "V01234567") &&
       (getCPUBPWState() == "V01234567"))
    {
        return true;
    }
    else
    {
        return false;
    }
}


void cycle_get_all_cpld_info_thread (void)
{
    while(1)
    {
        get_all_cpld_info();
        setCpldInfoDbusProperty();
        std::cout << "cycle_get_all_cpld_info_thread:" << boost::this_thread::get_id() <<std:: endl;
        if (causeSet.contains(RestartCause::command) &&
            ((powerState == PowerState::gracefulTransitionToOff)  ||   //判断是否正在进行cmd关机流程
            (powerState == PowerState::transitionToCycleOff)     ||   //判断是否正在进行开机状态下的cmd重启流程
            (powerState == PowerState::gracefulTransitionToCycleOff)))//判断是否正在进行开机状态下的cmd GracefulWarmReboot流程
        {
            if(!check_power_state())
            {
                lg2::info("cmd cause power off");
                psPowerOKHandler(0);
            }
        }
        //判断是否正在进行cmd开机流程
        if (causeSet.contains(RestartCause::command) && (powerState == PowerState::waitForPSPowerOK) )
        {
            lg2::info("cycle_get_all_cpld_info-0002");
            if(check_power_state())
            {
                lg2::info("cmd cause power on");
                psPowerOKHandler(1);
            }
        }

        std::this_thread::sleep_for(std::chrono::seconds(20));
    }

}

void cycle_get_all_cpld_info (const boost::system::error_code& )
{

    get_all_cpld_info();


    setCpldInfoDbusProperty();

    //lg2::info("getThreadId : {ID}","ID", boost::this_thread::get_id());
    // std::cout << "cycle_get_all_cpld_info:" << boost::this_thread::get_id() <<std:: endl;

    if (causeSet.contains(RestartCause::command) &&
        ((powerState == PowerState::gracefulTransitionToOff)  ||   //判断是否正在进行cmd关机流程
         (powerState == PowerState::transitionToCycleOff)     ||   //判断是否正在进行开机状态下的cmd重启流程
         (powerState == PowerState::gracefulTransitionToCycleOff)))//判断是否正在进行开机状态下的cmd GracefulWarmReboot流程
    {
        if(!check_power_state())
        {
            lg2::info("cmd cause power off");
            psPowerOKHandler(0);
        }

    }
    //判断是否正在进行cmd开机流程
    if (causeSet.contains(RestartCause::command) && (powerState == PowerState::waitForPSPowerOK) )
    {
        lg2::info("cycle_get_all_cpld_info-0002");
        if(check_power_state())
        {
            lg2::info("cmd cause power on");
            psPowerOKHandler(1);
        }

    }

    getCpldInfoTimer.expires_at(getCpldInfoTimer.expires_at() + boost::posix_time::milliseconds(/*TimerMap["CpldReadCycle"]*/20000));
    // getCpldInfoTimer.expires_from_now(boost::posix_time::milliseconds(/*TimerMap["CpldReadCycle"]*/20000));
    getCpldInfoTimer.async_wait(cycle_get_all_cpld_info);


}

void set_cycle_timer(void)
{
    lg2::info("set_cycle_timer-01");
    getCpldInfoTimer.expires_from_now(boost::posix_time::milliseconds(/*TimerMap["CpldReadCycle"]*/20000));
    lg2::info("set_cycle_timer-02");
    getCpldInfoTimer.async_wait(cycle_get_all_cpld_info);
    lg2::info("set_cycle_timer-03");

}



void cpldControlStateClear(void)
{
    if (causeSet.contains(RestartCause::command))
        causeSet.erase(RestartCause::command);

    set_cpld_pw_on_off(SetCpldPowerState::clear);
}



} // namespace power_control

int main(int argc, char* argv[])
{
    using namespace power_control;
    using namespace boost::placeholders;

    if (argc > 1)
    {
        node = argv[1];
    }
    lg2::info("Start Chassis power control service for host : {NODE}", "NODE",
              node);

    conn = std::make_shared<sdbusplus::asio::connection>(io);

    // Load GPIO's through json config file
    if (loadConfigValues() == -1)
    {
        lg2::error("Host{NODE}: Error in Parsing...", "NODE", node);
        set_default_cpld_info();
    }

#ifdef SOPHGO_CPLD

    while(1)
    {
        lg2::info("try to init cpld i2c");
        if(cpld_i2c_init() == 1)
            break;

        syn_delaytimer.wait();
    }
#if 0
    while (1)
    {
        lg2::info("try to get cpld info");
        if(get_all_cpld_info() == 1)
            break;

        syn_delaytimer.wait();
    }
#else
    get_all_cpld_info();
#endif

#endif
    /* Currently for single host based systems additional busname is added
    with "0" at the end of the name ex : xyz.openbmc_project.State.Host0.
    Going forward for single hosts the old bus name without zero numbering
    will be removed when all other applications adapted to the
    bus name with zero numbering (xyz.openbmc_project.State.Host0). */

    if (node == "0")
    {
        // Request all the dbus names
        conn->request_name(hostDbusName.c_str());
        conn->request_name(chassisDbusName.c_str());
        conn->request_name(osDbusName.c_str());
        conn->request_name(buttonDbusName.c_str());
        conn->request_name(nmiDbusName.c_str());
        conn->request_name(rstCauseDbusName.c_str());
    }

    hostDbusName += node;
    chassisDbusName += node;
    osDbusName += node;
    buttonDbusName += node;
    nmiDbusName += node;
    rstCauseDbusName += node;

    // Request all the dbus names
    conn->request_name(hostDbusName.c_str());
    conn->request_name(chassisDbusName.c_str());
    conn->request_name(osDbusName.c_str());
    conn->request_name(buttonDbusName.c_str());
    conn->request_name(nmiDbusName.c_str());
    conn->request_name(rstCauseDbusName.c_str());


#ifndef SOPHGO_CPLD

    if (sioPwrGoodConfig.lineName.empty() ||
        sioOnControlConfig.lineName.empty() || sioS5Config.lineName.empty())
    {
        sioEnabled = false;
        lg2::info("SIO control GPIOs not defined, disable SIO support.");
    }

    // Request PS_PWROK GPIO events
    if (powerOkConfig.type == ConfigType::GPIO)
    {
        if (!requestGPIOEvents(powerOkConfig.lineName, psPowerOKHandler,
                               psPowerOKLine, psPowerOKEvent))
        {
            return -1;
        }
    }
    else if (powerOkConfig.type == ConfigType::DBUS)
    {

        static sdbusplus::bus::match::match powerOkEventMonitor =
            power_control::dbusGPIOMatcher(powerOkConfig, psPowerOKHandler);
    }
    else
    {
        lg2::error("PowerOk name should be configured from json config file");
        return -1;
    }

    if (sioEnabled == true)
    {
        // Request SIO_POWER_GOOD GPIO events
        if (sioPwrGoodConfig.type == ConfigType::GPIO)
        {
            if (!requestGPIOEvents(sioPwrGoodConfig.lineName,
                                   sioPowerGoodHandler, sioPowerGoodLine,
                                   sioPowerGoodEvent))
            {
                return -1;
            }
        }
        else if (sioPwrGoodConfig.type == ConfigType::DBUS)
        {
            static sdbusplus::bus::match::match sioPwrGoodEventMonitor =
                power_control::dbusGPIOMatcher(sioPwrGoodConfig,
                                               sioPowerGoodHandler);
        }
        else
        {
            lg2::error(
                "sioPwrGood name should be configured from json config file");
            return -1;
        }

        // Request SIO_ONCONTROL GPIO events
        if (sioOnControlConfig.type == ConfigType::GPIO)
        {
            if (!requestGPIOEvents(sioOnControlConfig.lineName,
                                   sioOnControlHandler, sioOnControlLine,
                                   sioOnControlEvent))
            {
                return -1;
            }
        }
        else if (sioOnControlConfig.type == ConfigType::DBUS)
        {
            static sdbusplus::bus::match::match sioOnControlEventMonitor =
                power_control::dbusGPIOMatcher(sioOnControlConfig,
                                               sioOnControlHandler);
        }
        else
        {
            lg2::error(
                "sioOnControl name should be configured from jsonconfig file\n");
            return -1;
        }

        // Request SIO_S5 GPIO events
        if (sioS5Config.type == ConfigType::GPIO)
        {
            if (!requestGPIOEvents(sioS5Config.lineName, sioS5Handler,
                                   sioS5Line, sioS5Event))
            {
                return -1;
            }
        }
        else if (sioS5Config.type == ConfigType::DBUS)
        {
            static sdbusplus::bus::match::match sioS5EventMonitor =
                power_control::dbusGPIOMatcher(sioS5Config, sioS5Handler);
        }
        else
        {
            lg2::error("sioS5 name should be configured from json config file");
            return -1;
        }
    }

    // Request POWER_BUTTON GPIO events
    if (powerButtonConfig.type == ConfigType::GPIO)
    {
        if (!requestGPIOEvents(powerButtonConfig.lineName, powerButtonHandler,
                               powerButtonLine, powerButtonEvent))
        {
            return -1;
        }
    }
    else if (powerButtonConfig.type == ConfigType::DBUS)
    {
        static sdbusplus::bus::match::match powerButtonEventMonitor =
            power_control::dbusGPIOMatcher(powerButtonConfig,
                                           powerButtonHandler);
    }

    // Request RESET_BUTTON GPIO events
    if (resetButtonConfig.type == ConfigType::GPIO)
    {
        if (!requestGPIOEvents(resetButtonConfig.lineName, resetButtonHandler,
                               resetButtonLine, resetButtonEvent))
        {
            return -1;
        }
    }
    else if (resetButtonConfig.type == ConfigType::DBUS)
    {
        static sdbusplus::bus::match::match resetButtonEventMonitor =
            power_control::dbusGPIOMatcher(resetButtonConfig,
                                           resetButtonHandler);
    }

    // Request NMI_BUTTON GPIO events
    if (nmiButtonConfig.type == ConfigType::GPIO)
    {
        if (!nmiButtonConfig.lineName.empty())
        {
            requestGPIOEvents(nmiButtonConfig.lineName, nmiButtonHandler,
                              nmiButtonLine, nmiButtonEvent);
        }
    }
    else if (nmiButtonConfig.type == ConfigType::DBUS)
    {
        static sdbusplus::bus::match::match nmiButtonEventMonitor =
            power_control::dbusGPIOMatcher(nmiButtonConfig, nmiButtonHandler);
    }

    // Request ID_BUTTON GPIO events
    if (idButtonConfig.type == ConfigType::GPIO)
    {
        if (!idButtonConfig.lineName.empty())
        {
            requestGPIOEvents(idButtonConfig.lineName, idButtonHandler,
                              idButtonLine, idButtonEvent);
        }
    }
    else if (idButtonConfig.type == ConfigType::DBUS)
    {
        static sdbusplus::bus::match::match idButtonEventMonitor =
            power_control::dbusGPIOMatcher(idButtonConfig, idButtonHandler);
    }

#ifdef USE_PLT_RST
    sdbusplus::bus::match::match pltRstMatch(
        *conn,
        "type='signal',interface='org.freedesktop.DBus.Properties',member='"
        "PropertiesChanged',arg0='xyz.openbmc_project.State.Host.Misc'",
        hostMiscHandler);
#endif

    // Request POST_COMPLETE GPIO events
    if (postCompleteConfig.type == ConfigType::GPIO)
    {
        if (!requestGPIOEvents(postCompleteConfig.lineName, postCompleteHandler,
                               postCompleteLine, postCompleteEvent))
        {
            return -1;
        }
    }
    else if (postCompleteConfig.type == ConfigType::DBUS)
    {
        static sdbusplus::bus::match::match postCompleteEventMonitor =
            power_control::dbusGPIOMatcher(postCompleteConfig,
                                           postCompleteHandler);
    }
    else
    {
        lg2::error(
            "postComplete name should be configured from json config file");
        return -1;
    }

    // initialize NMI_OUT GPIO.
    if (!nmiOutConfig.lineName.empty())
    {
        setGPIOOutput(nmiOutConfig.lineName, 0, nmiOutLine);
    }

    // Initialize POWER_OUT and RESET_OUT GPIO.
    gpiod::line line;
    if (!powerOutConfig.lineName.empty())
    {
        if (!setGPIOOutput(powerOutConfig.lineName, !powerOutConfig.polarity,
                           line))
        {
            return -1;
        }
    }
    else
    {
        lg2::error("powerOut name should be configured from json config file");
        return -1;
    }

    if (!resetOutConfig.lineName.empty())
    {
        if (!setGPIOOutput(resetOutConfig.lineName, !resetOutConfig.polarity,
                           line))
        {
            return -1;
        }
    }
    else
    {
        lg2::error("ResetOut name should be configured from json config file");
        return -1;
    }
    // Release line
    line.reset();


#endif //SOPHGO_CPLD




#ifndef SOPHGO_CPLD
    // Initialize the power state
    powerState = PowerState::off;
    // Check power good

    if (powerOkConfig.type == ConfigType::GPIO)
    {
        if (psPowerOKLine.get_value() > 0 ||
            (sioEnabled &&
             (sioPowerGoodLine.get_value() == sioPwrGoodConfig.polarity)))
        {
            powerState = PowerState::on;
        }
    }
    else
    {
        if (getProperty(powerOkConfig))
        {
            powerState = PowerState::on;
        }
    }
    // Check if we need to start the Power Restore policy
    if (powerState != PowerState::on)
    {
        powerRestore.run();
    }

    if (nmiOutLine)
        nmiSourcePropertyMonitor();

#else
    // Initialize the power state
    powerState = PowerState::off;
    // Check power good
    if(check_power_state())
    {
        powerState = PowerState::on;
    }
    else
    {
        powerState = PowerState::off;
    }

#endif //SOPHGO_CPLD


    lg2::info("Initializing power state.");
    logStateTransition(powerState);

    // Power Control Service
    sdbusplus::asio::object_server hostServer =
        sdbusplus::asio::object_server(conn);

    // Power Control Interface
    hostIface =
        hostServer.add_interface("/xyz/openbmc_project/state/host" + node,
                                 "xyz.openbmc_project.State.Host");

#ifndef SOPHGO_CPLD
    // Interface for IPMI/Redfish initiated host state transitions
    hostIface->register_property(
        "RequestedHostTransition",
        std::string("xyz.openbmc_project.State.Host.Transition.Off"),
        [](const std::string& requested, std::string& resp) {
            if (requested == "xyz.openbmc_project.State.Host.Transition.Off")
            {

                // if power button is masked, ignore this
                if (!powerButtonMask)
                {
                    sendPowerControlEvent(Event::gracefulPowerOffRequest);
                    addRestartCause(RestartCause::command);
                }
                else
                {
                    lg2::info("Power Button Masked.");
                    throw std::invalid_argument("Transition Request Masked");
                    return 0;
                }

                //i2c write,timeout retransmission


            }
            else if (requested ==
                     "xyz.openbmc_project.State.Host.Transition.On")
            {

                // if power button is masked, ignore this
                if (!powerButtonMask)
                {
                    sendPowerControlEvent(Event::powerOnRequest);
                    addRestartCause(RestartCause::command);
                }
                else
                {
                    lg2::info("Power Button Masked.");
                    throw std::invalid_argument("Transition Request Masked");
                    return 0;
                }




            }
            else if (requested ==
                     "xyz.openbmc_project.State.Host.Transition.Reboot")
            {

                // if power button is masked, ignore this
                if (!powerButtonMask)
                {
                    sendPowerControlEvent(Event::powerCycleRequest);
                    addRestartCause(RestartCause::command);
                }
                else
                {
                    lg2::info("Power Button Masked.");
                    throw std::invalid_argument("Transition Request Masked");
                    return 0;
                }



            }
            else if (
                requested ==
                "xyz.openbmc_project.State.Host.Transition.GracefulWarmReboot")
            {

                // if reset button is masked, ignore this
                if (!resetButtonMask)
                {
                    sendPowerControlEvent(Event::gracefulPowerCycleRequest);
                    addRestartCause(RestartCause::command);
                }
                else
                {
                    lg2::info("Reset Button Masked.");
                    throw std::invalid_argument("Transition Request Masked");
                    return 0;
                }



            }
            else if (
                requested ==
                "xyz.openbmc_project.State.Host.Transition.ForceWarmReboot")
            {

                // if reset button is masked, ignore this
                if (!resetButtonMask)
                {
                    sendPowerControlEvent(Event::resetRequest);
                    addRestartCause(RestartCause::command);
                }
                else
                {
                    lg2::info("Reset Button Masked.");
                    throw std::invalid_argument("Transition Request Masked");
                    return 0;
                }



            }
            else
            {
                lg2::error("Unrecognized host state transition request.");
                throw std::invalid_argument("Unrecognized Transition Request");
                return 0;
            }
            resp = requested;
            return 1;
        });
#else
     // Interface for IPMI/Redfish initiated host state transitions
    hostIface->register_property(
        "RequestedHostTransition",
        std::string("xyz.openbmc_project.State.Host.Transition.Off"),
        [](const std::string& requested, std::string& resp) {
            if (requested == "xyz.openbmc_project.State.Host.Transition.Off")
            {
                lg2::info("Host off");;
                sendPowerControlEvent(Event::gracefulPowerOffRequest);
                addRestartCause(RestartCause::command);
            }
            else if (requested ==
                     "xyz.openbmc_project.State.Host.Transition.On")
            {
                sendPowerControlEvent(Event::powerOnRequest);
                addRestartCause(RestartCause::command);
            }
            else if (requested ==
                     "xyz.openbmc_project.State.Host.Transition.Reboot")
            {
                sendPowerControlEvent(Event::powerCycleRequest);
                addRestartCause(RestartCause::command);
            }
            else if (
                requested ==
                "xyz.openbmc_project.State.Host.Transition.GracefulWarmReboot")
            {
                sendPowerControlEvent(Event::gracefulPowerCycleRequest);
                addRestartCause(RestartCause::command);
            }
            else if (
                requested ==
                "xyz.openbmc_project.State.Host.Transition.ForceWarmReboot")
            {
                // sendPowerControlEvent(Event::resetRequest);
                // addRestartCause(RestartCause::command);
                lg2::info("Do not support ForceWarmReboot.");
            }
            else
            {
                lg2::error("Unrecognized host state transition request.");
                throw std::invalid_argument("Unrecognized Transition Request");
                return 0;
            }
            resp = requested;
            return 1;
        });

#endif


    hostIface->register_property("CurrentHostState",
                                 std::string(getHostState(powerState)));

    hostIface->register_property("CurrentCPUAPWState",
                                 std::string(getCPUAPWState()));
    hostIface->register_property("CurrentCPUBPWState",
                                 std::string(getCPUBPWState()));
    hostIface->register_property("CurrentCPUOtherPWState",
                                 std::string(getCPUOPWState()));
    hostIface->register_property("CurrentOtherPWState",
                                 std::string(getOtherPWState()));
    hostIface->register_property("CurrentPSUState",
                                 std::string(getPSUState()));
    hostIface->register_property("CurrentEFUState",
                                 std::string(getEFUSState()));


    hostIface->initialize();


    // Chassis Control Service
    sdbusplus::asio::object_server chassisServer =
        sdbusplus::asio::object_server(conn);

    // Chassis Control Interface
    chassisIface =
        chassisServer.add_interface("/xyz/openbmc_project/state/chassis" + node,
                                    "xyz.openbmc_project.State.Chassis");

    chassisIface->register_property(
        "RequestedPowerTransition",
        std::string("xyz.openbmc_project.State.Chassis.Transition.Off"),
        [](const std::string& requested, std::string& resp) {
#ifndef SOPHGO_CPLD
            if (requested == "xyz.openbmc_project.State.Chassis.Transition.Off")
            {
                // if power button is masked, ignore this
                if (!powerButtonMask)
                {
                    sendPowerControlEvent(Event::powerOffRequest);
                    addRestartCause(RestartCause::command);
                }
                else
                {
                    lg2::info("Power Button Masked.");
                    throw std::invalid_argument("Transition Request Masked");
                    return 0;
                }
            }
            else if (requested ==
                     "xyz.openbmc_project.State.Chassis.Transition.On")
            {

                // if power button is masked, ignore this
                if (!powerButtonMask)
                {
                    sendPowerControlEvent(Event::powerOnRequest);
                    addRestartCause(RestartCause::command);
                }
                else
                {
                    lg2::info("Power Button Masked.");
                    throw std::invalid_argument("Transition Request Masked");
                    return 0;
                }
            }
            else if (requested ==
                     "xyz.openbmc_project.State.Chassis.Transition.PowerCycle")
            {

                // if power button is masked, ignore this
                if (!powerButtonMask)
                {
                    sendPowerControlEvent(Event::powerCycleRequest);
                    addRestartCause(RestartCause::command);
                }
                else
                {
                    lg2::info("Power Button Masked.");
                    throw std::invalid_argument("Transition Request Masked");
                    return 0;
                }
            }
            else
            {
                lg2::error("Unrecognized chassis state transition request.");
                throw std::invalid_argument("Unrecognized Transition Request");
                return 0;
            }
#else
            if ((requested == "xyz.openbmc_project.State.Chassis.Transition.Off") ||
                (requested == "xyz.openbmc_project.State.Chassis.Transition.On")  ||
                (requested == "xyz.openbmc_project.State.Chassis.Transition.PowerCycle"))
            {
                lg2::info("Chassis off on PowerCycle");;
            }
            else
            {
                lg2::error("Unrecognized chassis state transition request.");
                throw std::invalid_argument("Unrecognized Transition Request");
                return 0;
            }
#endif
            resp = requested;
            return 1;
        });
    chassisIface->register_property("CurrentPowerState",
                                    std::string(getChassisState(powerState)));
    chassisIface->register_property("LastStateChangeTime", getCurrentTimeMs());

    chassisIface->initialize();

#ifdef CHASSIS_SYSTEM_RESET
    // Chassis System Service
    sdbusplus::asio::object_server chassisSysServer =
        sdbusplus::asio::object_server(conn);

    // Chassis System Interface
    chassisSysIface = chassisSysServer.add_interface(
        "/xyz/openbmc_project/state/chassis_system0",
        "xyz.openbmc_project.State.Chassis");

    chassisSysIface->register_property(
        "RequestedPowerTransition",
        std::string("xyz.openbmc_project.State.Chassis.Transition.On"),
        [](const std::string& requested, std::string& resp) {
            if (requested ==
                "xyz.openbmc_project.State.Chassis.Transition.PowerCycle")
            {
                systemReset();
                addRestartCause(RestartCause::command);
            }
            else
            {
                lg2::error(
                    "Unrecognized chassis system state transition request.");
                throw std::invalid_argument("Unrecognized Transition Request");
                return 0;
            }
            resp = requested;
            return 1;
        });
    chassisSysIface->register_property(
        "CurrentPowerState", std::string(getChassisState(powerState)));
    chassisSysIface->register_property("LastStateChangeTime",
                                       getCurrentTimeMs());

    chassisSysIface->initialize();

    if (!slotPowerConfig.lineName.empty())
    {
        if (!setGPIOOutput(slotPowerConfig.lineName, 1, slotPowerLine))
        {
            return -1;
        }

        slotPowerState = SlotPowerState::off;
        if (slotPowerLine.get_value() > 0)
        {
            slotPowerState = SlotPowerState::on;
        }

        chassisSlotIface = chassisSysServer.add_interface(
            "/xyz/openbmc_project/state/chassis_system" + node,
            "xyz.openbmc_project.State.Chassis");
        chassisSlotIface->register_property(
            "RequestedPowerTransition",
            std::string("xyz.openbmc_project.State.Chassis.Transition.On"),
            [](const std::string& requested, std::string& resp) {
                if (requested ==
                    "xyz.openbmc_project.State.Chassis.Transition.On")
                {
                    slotPowerOn();
                }
                else if (requested ==
                         "xyz.openbmc_project.State.Chassis.Transition.Off")
                {
                    slotPowerOff();
                }
                else if (
                    requested ==
                    "xyz.openbmc_project.State.Chassis.Transition.PowerCycle")
                {
                    slotPowerCycle();
                }
                else
                {
                    lg2::error(
                        "Unrecognized chassis system state transition request.\n");
                    throw std::invalid_argument(
                        "Unrecognized Transition Request");
                    return 0;
                }
                resp = requested;
                return 1;
            });
        chassisSlotIface->register_property(
            "CurrentPowerState", std::string(getSlotState(slotPowerState)));
        chassisSlotIface->register_property("LastStateChangeTime",
                                            getCurrentTimeMs());
        chassisSlotIface->initialize();
    }
#endif
#ifndef SOPHGO_CPLD
    // Buttons Service
    sdbusplus::asio::object_server buttonsServer =
        sdbusplus::asio::object_server(conn);

    if (!powerButtonConfig.lineName.empty())
    {
        // Power Button Interface
        power_control::powerButtonIface = buttonsServer.add_interface(
            "/xyz/openbmc_project/chassis/buttons/power",
            "xyz.openbmc_project.Chassis.Buttons");

        powerButtonIface->register_property(
            "ButtonMasked", false, [](const bool requested, bool& current) {
                if (requested)
                {
                    if (powerButtonMask)
                    {
                        return 1;
                    }
                    if (!setGPIOOutput(powerOutConfig.lineName,
                                       !powerOutConfig.polarity,
                                       powerButtonMask))
                    {
                        throw std::runtime_error("Failed to request GPIO");
                        return 0;
                    }
                    lg2::info("Power Button Masked.");
                }
                else
                {
                    if (!powerButtonMask)
                    {
                        return 1;
                    }
                    lg2::info("Power Button Un-masked");
                    powerButtonMask.reset();
                }
                // Update the mask setting
                current = requested;
                return 1;
            });

        // Check power button state
        bool powerButtonPressed;
        if (powerButtonConfig.type == ConfigType::GPIO)
        {
            powerButtonPressed = powerButtonLine.get_value() == 0;
        }
        else
        {
            powerButtonPressed = getProperty(powerButtonConfig) == 0;
        }

        powerButtonIface->register_property("ButtonPressed",
                                            powerButtonPressed);

        powerButtonIface->initialize();
    }

    if (!resetButtonConfig.lineName.empty())
    {
        // Reset Button Interface

        resetButtonIface = buttonsServer.add_interface(
            "/xyz/openbmc_project/chassis/buttons/reset",
            "xyz.openbmc_project.Chassis.Buttons");

        resetButtonIface->register_property(
            "ButtonMasked", false, [](const bool requested, bool& current) {
                if (requested)
                {
                    if (resetButtonMask)
                    {
                        return 1;
                    }
                    if (!setGPIOOutput(resetOutConfig.lineName,
                                       !resetOutConfig.polarity,
                                       resetButtonMask))
                    {
                        throw std::runtime_error("Failed to request GPIO");
                        return 0;
                    }
                    lg2::info("Reset Button Masked.");
                }
                else
                {
                    if (!resetButtonMask)
                    {
                        return 1;
                    }
                    lg2::info("Reset Button Un-masked");
                    resetButtonMask.reset();
                }
                // Update the mask setting
                current = requested;
                return 1;
            });

        // Check reset button state
        bool resetButtonPressed;
        if (resetButtonConfig.type == ConfigType::GPIO)
        {
            resetButtonPressed = resetButtonLine.get_value() == 0;
        }
        else
        {
            resetButtonPressed = getProperty(resetButtonConfig) == 0;
        }

        resetButtonIface->register_property("ButtonPressed",
                                            resetButtonPressed);

        resetButtonIface->initialize();
    }

    if (nmiButtonLine)
    {
        // NMI Button Interface
        nmiButtonIface = buttonsServer.add_interface(
            "/xyz/openbmc_project/chassis/buttons/nmi",
            "xyz.openbmc_project.Chassis.Buttons");

        nmiButtonIface->register_property(
            "ButtonMasked", false, [](const bool requested, bool& current) {
                if (nmiButtonMasked == requested)
                {
                    // NMI button mask is already set as requested, so no change
                    return 1;
                }
                if (requested)
                {
                    lg2::info("NMI Button Masked.");
                    nmiButtonMasked = true;
                }
                else
                {
                    lg2::info("NMI Button Un-masked.");
                    nmiButtonMasked = false;
                }
                // Update the mask setting
                current = nmiButtonMasked;
                return 1;
            });

        // Check NMI button state
        bool nmiButtonPressed;
        if (nmiButtonConfig.type == ConfigType::GPIO)
        {
            nmiButtonPressed = nmiButtonLine.get_value() == 0;
        }
        else
        {
            nmiButtonPressed = getProperty(nmiButtonConfig) == 0;
        }

        nmiButtonIface->register_property("ButtonPressed", nmiButtonPressed);

        nmiButtonIface->initialize();
    }

    if (nmiOutLine)
    {
        // NMI out Service
        sdbusplus::asio::object_server nmiOutServer =
            sdbusplus::asio::object_server(conn);

        // NMI out Interface
        nmiOutIface = nmiOutServer.add_interface(
            "/xyz/openbmc_project/control/host" + node + "/nmi",
            "xyz.openbmc_project.Control.Host.NMI");
        nmiOutIface->register_method("NMI", nmiReset);
        nmiOutIface->initialize();
    }

    if (idButtonLine)
    {
        // ID Button Interface
        idButtonIface = buttonsServer.add_interface(
            "/xyz/openbmc_project/chassis/buttons/id",
            "xyz.openbmc_project.Chassis.Buttons");

        // Check ID button state
        bool idButtonPressed;
        if (idButtonConfig.type == ConfigType::GPIO)
        {
            idButtonPressed = idButtonLine.get_value() == 0;
        }
        else
        {
            idButtonPressed = getProperty(idButtonConfig) == 0;
        }

        idButtonIface->register_property("ButtonPressed", idButtonPressed);

        idButtonIface->initialize();
    }
#endif

    // OS State Service
    sdbusplus::asio::object_server osServer =
        sdbusplus::asio::object_server(conn);

    // OS State Interface
    osIface = osServer.add_interface(
        "/xyz/openbmc_project/state/os",
        "xyz.openbmc_project.State.OperatingSystem.Status");

    // Get the initial OS state based on POST complete
    //      0: Asserted, OS state is "Standby" (ready to boot)
    //      1: De-Asserted, OS state is "Inactive"
    OperatingSystemStateStage osState;
#ifndef SOPHGO_CPLD
    if (postCompleteConfig.type == ConfigType::GPIO)
    {
        osState = postCompleteLine.get_value() > 0
                      ? OperatingSystemStateStage::Inactive
                      : OperatingSystemStateStage::Standby;
    }
    else
    {
        osState = getProperty(postCompleteConfig) > 0
                      ? OperatingSystemStateStage::Inactive
                      : OperatingSystemStateStage::Standby;
    }
#else
    osState = OperatingSystemStateStage::Inactive;
#endif

    osIface->register_property(
        "OperatingSystemState",
        std::string(getOperatingSystemStateStage(osState)));

    osIface->initialize();



    // Restart Cause Service
    sdbusplus::asio::object_server restartCauseServer =
        sdbusplus::asio::object_server(conn);

    // Restart Cause Interface
    restartCauseIface = restartCauseServer.add_interface(
        "/xyz/openbmc_project/control/host" + node + "/restart_cause",
        "xyz.openbmc_project.Control.Host.RestartCause");

    restartCauseIface->register_property(
        "RestartCause",
        std::string("xyz.openbmc_project.State.Host.RestartCause.Unknown"));

    restartCauseIface->register_property(
        "RequestedRestartCause",
        std::string("xyz.openbmc_project.State.Host.RestartCause.Unknown"),
        [](const std::string& requested, std::string& resp) {
            if (requested ==
                "xyz.openbmc_project.State.Host.RestartCause.WatchdogTimer")
            {
                addRestartCause(RestartCause::watchdog);
            }
            else
            {
                throw std::invalid_argument(
                    "Unrecognized RestartCause Request");
                return 0;
            }

            lg2::info("RestartCause requested: {RESTART_CAUSE}",
                      "RESTART_CAUSE", requested);
            resp = requested;
            return 1;
        });

    restartCauseIface->initialize();



    currentHostStateMonitor();

#ifdef SOPHGO_CPLD
    //使用定时器周期读取cpld数据并更新dbus属性值
    set_cycle_timer();

    // std::thread thread2 = std::thread(cycle_get_all_cpld_info_thread);
    // std::cout << "thread2 ID:" << thread2.get_id() << std::endl;
    // //将 thread2 与主线程分离开，thread1 线程独立执行。
    // thread2.detach();
#endif

    io.run();

    return 0;
}
