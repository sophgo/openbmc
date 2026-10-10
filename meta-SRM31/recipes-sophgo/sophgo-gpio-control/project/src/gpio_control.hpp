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





} // namespace power_control
