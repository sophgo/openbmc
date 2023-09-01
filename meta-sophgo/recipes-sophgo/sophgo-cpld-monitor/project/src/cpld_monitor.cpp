#include "cpld_monitor.hpp"

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


#define CPU_A_POWER_STATE_ELEMENT_NUM 8
#define CPU_B_POWER_STATE_ELEMENT_NUM 8
#define OTHER_POWER_STATE_ELEMENT_NUM 8
#define PSU_STATE_ELEMENT_NUM 6
#define EFUSE_STATE_ELEMENT_NUM 4

#define	GET_BIT(x, bit)	((x & (1 << bit)) >> bit)

namespace cpld_monitor
{
    static boost::asio::io_service io;
    std::shared_ptr<sdbusplus::asio::connection> conn;

    struct cpldI2cDev
    {
        std::string    deviceName;
        unsigned char  deviceAddr;
        int            i2cFile;
    };
    struct cpldI2cDev g_strucpldI2cDev;

    boost::container::flat_map<std::string, int> cpldRegMap = {
        {"hardWareVersionReg", 0},
        {"cpldVersionReg",     1},
        {"softWareBuildMReg",  2},
        {"softWareBuildDReg",  3},
        {"CPUAPWStateReg",     4},
        {"CPUBPWStateReg",     5},
        {"OPWStateReg",        7},
        {"PSUStateReg",        8},
        {"EFUSStateReg",       9}
    };

    boost::container::flat_map<std::string, char> cpldStateValueMap = {
        {"hardWareVersionReg", 0},
        {"cpldVersionReg",     0},
        {"softWareBuildMReg",  0},
        {"softWareBuildDReg",  0},
        {"CPUAPWStateReg",     0},
        {"CPUBPWStateReg",     0},
        {"OPWStateReg",        0},
        {"PSUStateReg",        0},
        {"EFUSStateReg",       0}
    };


    std::mutex     i2cMutex;

    static const std::string cpuAPWStateArry[CPU_A_POWER_STATE_ELEMENT_NUM] = {
        {"PG_VPP_A1"},
        {"PG_VPP_A0"},
        {"PG_VDDQ_A1"},
        {"PG_VDDQ_A0"},
        {"PG_VTT_A1"},
        {"PG_VTT_A0"},
        {"PG_PCIE_PHY_A"},
        {"PG_VDDC_A"}
    };

    static const std::string cpuBPWStateArry[CPU_B_POWER_STATE_ELEMENT_NUM] = {
        {"PG_VPP_B1"},
        {"PG_VPP_B0"},
        {"PG_VDDQ_B1"},
        {"PG_VDDQ_B0"},
        {"PG_VTT_B1"},
        {"PG_VTT_B0"},
        {"PG_PCIE_PHY_B"},
        {"PG_VDDC_B"}
    };

    static const std::string otherPWStateArry[OTHER_POWER_STATE_ELEMENT_NUM] = {
        {"PG_VDD_3V3"},
        {"PG_VDD_1V8"},
        {"PG_PCIE_H_1V8_B"},
        {"PG_DDR_PHY_B"},
        {"PG_PCIE_H_1V8_A"},
        {"PG_DDR_PHY_A"},
        {"cpua_pwrok"},
        {"cpub_pwrok"}
    };

    static const std::string psuStateArry[PSU_STATE_ELEMENT_NUM] = {
        {"PSU1_PRSNT_N"},
        {"PSU0_PRSNT_N"},
        {"PSU1_ALERT_N"},
        {"PSU0_ALERT_N"},
        {"PSU1_PWROK"},
        {"PSU0_PWROK"}
    };

    static const std::string efuseStateArry[EFUSE_STATE_ELEMENT_NUM] = {
        {"NCP0_GOK"},
        {"NCP0_D_OC"},
        {"NCP1_GOK"},
        {"NCP1_D_OC"}
    };



    static std::string g_strucpldI2cDevDbusName = "xyz.openbmc_project.State.CPLD";
    static std::shared_ptr<sdbusplus::asio::dbus_interface> hardVersionIface;
    static std::shared_ptr<sdbusplus::asio::dbus_interface> softVersionIface;
    static std::shared_ptr<sdbusplus::asio::dbus_interface> cpuAPowerStateIface;
    static std::shared_ptr<sdbusplus::asio::dbus_interface> cpuBPowerStateIface;
    static std::shared_ptr<sdbusplus::asio::dbus_interface> otherPowerStateIface;
    static std::shared_ptr<sdbusplus::asio::dbus_interface> psuStateIface;
    static std::shared_ptr<sdbusplus::asio::dbus_interface> efuseStateIface;

    boost::container::flat_map<std::string, int> TimerMap = {
    {"CpldReadCycle", 5}};

    //timer
    static boost::asio::deadline_timer getg_strucpldI2cDevTimer(io);

    static int loadConfigValues()
    {
        const std::string configFilePath =
            "/usr/share/sophgo-cpld-monitor/sophgo-cpld-monitor.json";
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

        auto timers      = jsonData["timing_configs"];
        auto i2c_dev     = jsonData["i2c_dev"];
        auto i2c_regmap  = jsonData["i2c_regmap"];

        // read and store the timer values from json config to Timer Map
        for (auto& [key, timerValue] : TimerMap)
        {
            if (timers.contains(key.c_str()))
            {
                timerValue = timers[key.c_str()];
            }
        }

        //i2c_dev
        nlohmann::json& i2cConfig = i2c_dev;
        g_strucpldI2cDev.deviceName = i2cConfig["deviceName"];
        if(g_strucpldI2cDev.deviceName.empty())
        {
            lg2::error("get i2c deviceName from json config failed!");
            return -1;
        }
        else
        {
            lg2::info("I2C deviceName : \'{NAME}\'", "NAME", g_strucpldI2cDev.deviceName);
        }
        g_strucpldI2cDev.deviceAddr = i2cConfig["deviceAddr"];
        lg2::info("I2C deviceAddr : \'{ADDR}\'", "ADDR", g_strucpldI2cDev.deviceAddr);

        //i2c_regmap
        for (auto& [key, regValue] : cpldRegMap)
        {
            if (i2c_regmap.contains(key.c_str()))
            {
                regValue = i2c_regmap[key.c_str()];
                lg2::info("{NAME} : \'{ADDR}\'", "NAME", key.c_str(), "ADDR", regValue);
            }
        }
        return 0;
    }

    int set_i2c_default_info(void)
    {
        g_strucpldI2cDev.deviceName         = std::string("/dev/i2c-7");
        g_strucpldI2cDev.deviceAddr         = 0x57;
        return 0;
    }


    int open_i2c_device(void)
    {
        g_strucpldI2cDev.i2cFile = open(g_strucpldI2cDev.deviceName.c_str(), O_RDWR);
        if (g_strucpldI2cDev.i2cFile < 0)
        {
            lg2::info("unable to open i2c device ");
            return -1;
        }
        if (ioctl(g_strucpldI2cDev.i2cFile, I2C_SLAVE_FORCE, g_strucpldI2cDev.deviceAddr) < 0)
        {
            lg2::error("unable to set device address");
            close(g_strucpldI2cDev.i2cFile);
            return -1;
        }
        return 1;
    }

    int close_i2c_device(void)
    {
        if(g_strucpldI2cDev.i2cFile > 0)
        {
            close(g_strucpldI2cDev.i2cFile);
            return 1;
        }

        return -1;
    }

    static int read_i2c_device(char addr, char reg, char *value)
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
        int ret=ioctl(g_strucpldI2cDev.i2cFile,I2C_RDWR,(unsigned long)&cpld_data);
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


    int get_cpld_info(void)
    {
        char tempValue;

        if(open_i2c_device() == -1)
        {
            return -1;
        }

        for (auto& [key, regValue] : cpldRegMap)
        {
            if(read_i2c_device(g_strucpldI2cDev.deviceAddr,regValue, &tempValue) < 0)
            {
                close_i2c_device();
                return -1;
            }
            else
            {
                if (cpldStateValueMap.contains(key.c_str()))
                {
                    cpldStateValueMap[key.c_str()] = tempValue;
                    // lg2::info("{NAME} : {VALUE}", "NAME", key.c_str(), "VALUE",  cpldStateValueMap[key.c_str()]);
                }
            }
        }
        close_i2c_device();
        return 1;
    }

    static void setDbusProperty(void)
    {
        int i;

        for (auto& [key, value] : cpldStateValueMap)
        {
            if(!strcmp(key.c_str(), "hardWareVersionReg"))
            {
                hardVersionIface->set_property("VERSION", \
                    std::to_string(value >> 4) + "." + std::to_string(value & 0xF));

            }
            else if(!strcmp(key.c_str(), "CPUAPWStateReg"))
            {
                for(i=0; i<CPU_A_POWER_STATE_ELEMENT_NUM; i++)
                {
                    cpuAPowerStateIface->set_property(cpuAPWStateArry[i], \
                        GET_BIT(cpldStateValueMap[key.c_str()],i) ? std::string("1") : std::string("0"));
                }
            }
            else if(!strcmp(key.c_str(), "CPUBPWStateReg"))
            {
                for(i=0; i<CPU_B_POWER_STATE_ELEMENT_NUM; i++)
                {
                    cpuBPowerStateIface->set_property(cpuBPWStateArry[i], \
                        GET_BIT(cpldStateValueMap[key.c_str()],i) ? std::string("1") : std::string("0"));
                }
            }
            else if(!strcmp(key.c_str(), "OPWStateReg"))
            {
                for(i=0; i<OTHER_POWER_STATE_ELEMENT_NUM; i++)
                {
                    otherPowerStateIface->set_property(otherPWStateArry[i], \
                        GET_BIT(cpldStateValueMap[key.c_str()],i) ? std::string("1") : std::string("0"));
                }
            }
            else if(!strcmp(key.c_str(), "PSUStateReg"))
            {
                for(i=0; i<PSU_STATE_ELEMENT_NUM; i++)
                {
                    psuStateIface->set_property(psuStateArry[i], \
                        GET_BIT(cpldStateValueMap[key.c_str()],i) ? std::string("1") : std::string("0"));
                }
            }
            else if(!strcmp(key.c_str(), "PSUStateReg"))
            {
                for(i=0; i<EFUSE_STATE_ELEMENT_NUM; i++)
                {
                    efuseStateIface->set_property(psuStateArry[i], \
                        GET_BIT(cpldStateValueMap[key.c_str()],i) ? std::string("1") : std::string("0"));
                }
            }
        }

        softVersionIface->set_property("VERSION", \
            std::to_string(cpldStateValueMap["hardWareVersionReg"] >> 4) + "." +      \
            std::to_string(cpldStateValueMap["hardWareVersionReg"] & 0x0F) + "-" +    \
            std::to_string(cpldStateValueMap["cpldVersionReg"] >> 4) + "." +      \
            std::to_string(cpldStateValueMap["cpldVersionReg"] & 0x0F) + "-" +    \
            std::to_string(cpldStateValueMap["softWareBuildMReg"] >> 4) +         \
            std::to_string(cpldStateValueMap["softWareBuildMReg"] & 0x0F) + "." + \
            std::to_string(cpldStateValueMap["softWareBuildDReg"] >> 4) +         \
            std::to_string(cpldStateValueMap["softWareBuildDReg"] & 0x0F));
    }

    void cycle_get_cpld_info (const boost::system::error_code& )
    {
        get_cpld_info();
        setDbusProperty();
        getg_strucpldI2cDevTimer.expires_at(getg_strucpldI2cDevTimer.expires_at() + boost::posix_time::seconds(TimerMap["CpldReadCycle"]));
        getg_strucpldI2cDevTimer.async_wait(cycle_get_cpld_info);
    }

    void set_cycle_timer(void)
    {
        getg_strucpldI2cDevTimer.expires_from_now( boost::posix_time::seconds(TimerMap["CpldReadCycle"]));
        getg_strucpldI2cDevTimer.async_wait(cycle_get_cpld_info);
    }

}


int main(int argc, char* argv[])
{
    int i;
    using namespace cpld_monitor;
    using namespace boost::placeholders;

    set_i2c_default_info();
    if (loadConfigValues() == -1)
    {
        lg2::error("Cpld config file: Error in Parsing...");
        set_i2c_default_info();
    }


    auto cpldBus = std::make_shared<sdbusplus::asio::connection>(io);
    cpldBus->request_name(g_strucpldI2cDevDbusName.c_str());
    sdbusplus::asio::object_server objectServer(cpldBus);

    hardVersionIface =
        objectServer.add_interface("/xyz/openbmc_project/state/cpld/hardVersion",
                                 "xyz.openbmc_project.State.Cpld.Version");
        hardVersionIface->register_property("VERSION",std::string("0.0"));


    softVersionIface =
        objectServer.add_interface("/xyz/openbmc_project/state/cpld/softVersion",
                                 "xyz.openbmc_project.State.Cpld.Version");
        softVersionIface->register_property("VERSION",std::string("0.0_0.0"));


    cpuAPowerStateIface =
        objectServer.add_interface("/xyz/openbmc_project/state/cpld/cpuAPowerState",
                                 "xyz.openbmc_project.State.Cpld.CpuAPowerState");
    for(i=0; i<CPU_A_POWER_STATE_ELEMENT_NUM; i++)
    {
        cpuAPowerStateIface->register_property(cpuAPWStateArry[i],std::string("0"));
    }


    cpuBPowerStateIface =
        objectServer.add_interface("/xyz/openbmc_project/state/cpld/cpuBPowerState",
                                 "xyz.openbmc_project.State.Cpld.CpuBPowerState");
    for(i=0; i<CPU_B_POWER_STATE_ELEMENT_NUM; i++)
    {
        cpuBPowerStateIface->register_property(cpuBPWStateArry[i],std::string("0"));
    }


    otherPowerStateIface =
        objectServer.add_interface("/xyz/openbmc_project/state/cpld/otherPowerState",
                                 "xyz.openbmc_project.State.Cpld.OtherPowerState");
    for(i=0; i<OTHER_POWER_STATE_ELEMENT_NUM; i++)
    {
        otherPowerStateIface->register_property(otherPWStateArry[i],std::string("0"));
    }

    psuStateIface =
        objectServer.add_interface("/xyz/openbmc_project/state/cpld/psuState",
                                 "xyz.openbmc_project.State.Cpld.PsuState");
    for(i=0; i<PSU_STATE_ELEMENT_NUM; i++)
    {
        psuStateIface->register_property(psuStateArry[i],std::string("0"));
    }


    efuseStateIface =
        objectServer.add_interface("/xyz/openbmc_project/state/cpld/efuseState",
                                 "xyz.openbmc_project.State.Cpld.EfuseState");
    for(i=0; i<EFUSE_STATE_ELEMENT_NUM; i++)
    {
        efuseStateIface->register_property(efuseStateArry[i],std::string("0"));
    }

    hardVersionIface->initialize();
    softVersionIface->initialize();
    cpuAPowerStateIface->initialize();
    cpuBPowerStateIface->initialize();
    otherPowerStateIface->initialize();
    psuStateIface->initialize();
    efuseStateIface->initialize();


    set_cycle_timer();

    io.run();

    return 0;


}