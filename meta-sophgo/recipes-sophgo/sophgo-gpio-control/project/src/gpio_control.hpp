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

#include <sdbusplus/bus.hpp>


#include <sdbusplus/message.hpp>
#include <sdbusplus/asio/connection.hpp>
#include <variant>
#include <map>
#include <vector>
#include <string>

namespace gpio_control
{

/**
 * @brief Persistent State Manager
 *
 * This manager supposed to store runtime parameters that supposed to be
 * persistent over BMC reboot. It provides simple Get/Set interface and handle
 * default values, hardcoded in getDefault() method.
 * @note: currently only string parameters supported
 */
using dbusPropertiesList =
    boost::container::flat_map<std::string,
                               std::variant<std::string, uint64_t>>;


namespace MatchRules = sdbusplus::bus::match::rules;


enum class CpuFlashPort
{
    FLASH_TO_HOST,
    FLASH_TO_BMC,
};
enum class SolUartPort
{
    CPU0_UART0,
    CPU0_UART1,
    CPU1_UART0,
    CPU1_UART1,
};

enum class wdtTimerCancelFlag
{
    INTOOS,
    DBUS,
    PXEBOOT,
};


void setupPowerMatch(const std::shared_ptr<sdbusplus::asio::connection>& conn);
static int loadConfigValues();
void forcePowerRestart(const boost::system::error_code& ec);
void set_wdt_timer(int sec);
static void handleGetVersion(const boost::system::error_code& ec);
int readMtdContent(const std::string &mtdDevice, char *buffer, ssize_t &bytesRead);
int findMtdDevice(const std::string &label, std::string &mtdDevice);
static void transitionCpuAFlashPort(CpuFlashPort port);
static void transitionCpuBFlashPort(CpuFlashPort port);

void set_wdt_timer(int sec);
int cancel_wdt_timer(int flag);

void set_version_timer(int sec);
int cancel_version_timer(int flag);

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



} // namespace power_control
