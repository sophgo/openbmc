#!/bin/bash


# ============================================================
#  global variable
# ============================================================
I2C_RETRY_MAX=3

# riser card
RISER_BUS[0]="8"
RISER_BUS[1]="9"
RISER_BUS[2]="10"

CH_ADDR[0]="0x01"
CH_ADDR[1]="0x02"
CH_ADDR[2]="0x04"
CH_ADDR[3]="0x08"

# AI Card
SC5_CHIP_NUM=3
SC7_CHIP_NUM=8
CHIP_ADDR[0]="0x60"
CHIP_ADDR[1]="0x61"
CHIP_ADDR[2]="0x62"
CHIP_ADDR[3]="0x63"
CHIP_ADDR[4]="0x64"
CHIP_ADDR[5]="0x65"
CHIP_ADDR[6]="0x66"
CHIP_ADDR[7]="0x67"

CHIP_TEMP_REG="0x00"
BOARD_TEMP_REG="0x01"
BOARD_POWER_REG="0x02"
PWM_REG="0x03"
VENDOR_ID_REG="0x10"
HARD_VERSION_REG="0x14"
FIRMWARE_VERSION_REG="0x18"
CARD_TYPE_REG="0x1C"
SUB_VENDOR_ID_REG="0x20"
# ============================================================







# ===========================================================
# $1: i2c bus
# ret: If exists, echo "1", otherwise echo "0"
# ===========================================================
function is_riser_card_exit(){
    if [ "$#" -lt 1 ] ;then
        echo "0"
    else
        if [ $(i2cdetect -y $1 | grep 71 | wc -l) -eq 1 ] ;then
            echo "1"
        else
            echo "0"
        fi
    fi
}




# ===========================================================
# $1: i2c bus
# $2: 0x00-close
#     0x01-ch1
#     0x02-ch2
#     0x04-ch3
#     0x08-ch4
# ret: success echo 0; fail echo 1
# ===========================================================
function riser_i2c_switch_mux(){
    if [ "$#" -lt 2 ] ;then
        echo "1"
    else
        if [ $(is_riser_card_exit $1) = 1 ] ;then
            for ((rtry=0; rtry<$I2C_RETRY_MAX; rtry++))
            do
                i2cset -y -f $1 0x71 $2
                if [ $? -eq 0 ] ;then
                    break
                fi
            done
            if [ $rtry -lt $I2C_RETRY_MAX ];then
                echo "0"
            else
                echo "1"
            fi
        else
            echo "1"
        fi
    fi
}

# ===========================================================
# $1: i2c bus
# ret: success echo value; fail echo "err"
# ===========================================================
function riser_i2c_read_mux(){
    if [ "$#" -lt 1 ] ;then
        echo "err"
    else
        if [ $(is_riser_card_exit $1) = 1 ] ;then
            for ((rtry=0; rtry<$I2C_RETRY_MAX; rtry++))
            do
                val=$(i2cget -f -y $1 0x71)
                if [ $? -eq 0 ];then
                    break
                fi
            done
            if [ $rtry -lt $I2C_RETRY_MAX ];then
	            echo $val
            else
                echo "err"
            fi
        else
            echo "err"
        fi
    fi
}

# ===========================================================
# $1: i2c bus
# $2: channel no:
#     0x01-ch1
#     0x02-ch2
#     0x04-ch3
#     0x08-ch4
# $3: device addr
# $4: device register addr
# ret: success echo value; fail echo "err"
# ===========================================================
function riser_i2c_read_device_8bit(){
    if [ "$#" -lt 4 ] ;then
        echo "err"
    else
        #  Riser card present on i2c bus
        if [ $(is_riser_card_exit $1) = 1 ] ;then
            #  Switch to the specified channel
            val=$(riser_i2c_switch_mux $1 $2)
            if [ $val -eq 0 ] ;then
                for ((rtry=0; rtry<$I2C_RETRY_MAX; rtry++))
                do
                    val=$(i2cget -f -y $1 $3 $4)
                    if [ $? -eq 0 ];then
                        break
                    fi
                done
                if [ $rtry -lt $I2C_RETRY_MAX ];then
                    echo $val
                else
                    echo "err"
                fi
            else
                echo "err"
            fi
        else
            echo "err"
        fi
    fi
}


# ===========================================================
# $1: i2c bus
# $2: channel no:
#     0x01-ch1
#     0x02-ch2
#     0x04-ch3
#     0x08-ch4
# $3: device addr
# $4: device register addr
# ret: success echo value; fail echo "err"
# ===========================================================
function riser_i2c_read_device_16bit(){
    if [ "$#" -lt 4 ] ;then
        echo "err"
    else
        #  Riser card present on i2c bus
        if [ $(is_riser_card_exit $1) = 1 ] ;then
            #  Switch to the specified channel
            val=$(riser_i2c_switch_mux $1 $2)
            if [ $val -eq 0 ] ;then
                for ((rtry=0; rtry<$I2C_RETRY_MAX; rtry++))
                do
                    val=$(i2cget -f -y $1 $3 $4 w)
                    if [ $? -eq 0 ];then
                        break
                    fi
                done
                if [ $rtry -lt $I2C_RETRY_MAX ];then
                    echo $val
                else
                    echo "err"
                fi
            else
                echo "err"
            fi
        else
            echo "err"
        fi
    fi
}


# ===========================================================
# $1: i2c bus
# $2: channel no:
#     0x01-ch1
#     0x02-ch2
#     0x04-ch3
#     0x08-ch4
# $3: device addr
# $4: device base register addr
# $5: bytes num
# ret: success echo value; fail echo "err"
# ===========================================================
function riser_i2c_read_device_nbytes(){
    if [ "$#" -lt 4 ] ;then
        echo "err"
    else
        #  Riser card present on i2c bus
        if [ $(is_riser_card_exit $1) = 1 ] ;then
            #  Switch to the specified channel
            val=$(riser_i2c_switch_mux $1 $2)
            if [ $val -eq 0 ] ;then
                for ((rtry=0; rtry<$I2C_RETRY_MAX; rtry++))
                do
                    val=$(i2cget -f -y $1 $3 $4 i $5)
                    if [ $? -eq 0 ];then
                        break
                    fi
                done
                if [ $rtry -lt $I2C_RETRY_MAX ];then
                    echo $val
                else
                    echo "err"
                fi
            else
                echo "err"
            fi
        else
            echo "err"
        fi
    fi
}



# ===========================================================
# get Device ID and Vendor ID of AI card
# $1: i2c bus
# $2: slave addr
# ret：
# ===========================================================
function get_vendorId_of_AiCard(){
    if [ "$#" -lt 2 ] ;then
        echo "err"
    else
        value=$(i2cget -f -y $1 $2 $VENDOR_ID_REG i 4)
        value=$(echo $value | sed 's/0x//g' | sed 's/[[:space:]]//g')
        echo $value
    fi
}


function get_firmWare_of_AiCard(){
    if [ "$#" -lt 2 ] ;then
        echo "err"
    else
        value=$(i2cget -f -y $1 $2 $FIRMWARE_VERSION_REG i 4)
        value=$(echo $value | sed 's/0x//g' | sed 's/[[:space:]]//g')
        echo $value
    fi
}



# ===========================================================
# $1: i2c bus
# $2: channel no:
#     0x01-ch1
#     0x02-ch2
#     0x04-ch3
#     0x08-ch4
# ret: If the AI card exists, echo "SC5" or "SC7", otherwise echo "absent"
#      error echo "err"
# ===========================================================
function is_1684_card_exist(){
    if [ "$#" -lt 2 ] ;then
        echo "err"
    else
        if [ $(is_riser_card_exit $1) = 1 ] ;then
            if [ $(riser_i2c_switch_mux $1 $2) == 0 ] ;then
                if [ $(i2cdetect -y $1 | grep 60\ 61\ 62\ 63\ 64\ 65\ 66\ 67 | wc -l) == 1 ] ;then
                    if [ $(get_vendorId_of_AiCard $1 0x60) == "16861f1c" ] ;then
                        echo "SC7"
                    else
                        echo "absent"
                    fi
                elif [ $(i2cdetect -y $1 | grep 60\ 61\ 62 | wc -l) == 1 ] ;then
                    if [ $(get_vendorId_of_AiCard $1 0x60) == "16841e30" ] ;then
                        echo "SC5"
                    else
                        echo "absent"
                    fi
                else
                    echo "absent"
                fi
            else
                echo "err"
            fi
        else
            echo "absent"
        fi
    fi
}






# ===========================================================
# ret:   Maximum value of all parameters
# ===========================================================
function riser_get_max_value(){
    local l_max_value=0
    if [ $# -ge 1 ] ;then
        for arg in "$@"
        do
            if [ $arg -gt $l_max_value ] ;then
                l_max_value=$arg
            fi
        done
    fi
    echo "$l_max_value"
}


# ===========================================================
# notes:
# ret:
# ===========================================================
function riser_i2c_scan_1684(){
    local riser_bus_num=`echo "${#RISER_BUS[@]}"`
    local channel_num=`echo "${#CH_ADDR[@]}"`
    local chip_num=0

    for ((i = 0 ; i < $riser_bus_num ; i++)) ;
    do
        # echo ${RISER_BUS[$i]}
        for ((j = 0 ; j < $channel_num ; j++)) ;
        do
            # echo ${CH_ADDR[$j]}
            val=$(is_1684_card_exist ${RISER_BUS[$i]} ${CH_ADDR[$j]})
            if [ "$val" == "SC7" ] ;then
                chip_num=$SC7_CHIP_NUM
            elif [ "$val" == "SC5" ] ; then
                chip_num=$SC5_CHIP_NUM
            fi
            if [ $chip_num -gt 0 ] ;then
                echo "BUS:${RISER_BUS[$i]} CH:${CH_ADDR[$j]} $val"
                for ((q = 0 ; q < $chip_num ; q++)) ;
                do
                    tempHex=$(i2cget -f -y ${RISER_BUS[$i]} ${CHIP_ADDR[$q]} 0x00)
                    tempDec=$(echo $tempHex | printf "%d\n" $(cat))
                    boardTempHex=$(i2cget -f -y ${RISER_BUS[$i]} ${CHIP_ADDR[$q]} 0x01)
                    boardTempDec=$(echo $boardTempHex | printf "%d\n" $(cat))
                    if [ $q -eq 0 ] ;then
                        boardPowerHex=$(i2cget -f -y ${RISER_BUS[$i]} ${CHIP_ADDR[$q]} 0x02)
                        boardPowerDec=$(echo $boardPowerHex | printf "%d\n" $(cat))
                        vendorId=$(get_vendorId_of_AiCard ${RISER_BUS[$i]} ${CHIP_ADDR[$q]})
                        firmwareVersion=$(get_firmWare_of_AiCard ${RISER_BUS[$i]} ${CHIP_ADDR[$q]})
                    fi
                    echo "CHIP$q: $tempDec℃  $boardTempDec℃"
                done
                echo "Board power:     ${boardPowerDec}W"
                echo "Vendor ID:       0x${vendorId}"
                echo "Firware Version: 0x${firmwareVersion}"
                chip_num=0
            fi
        done
    done
}


# ===========================================================
# notes:  Get the maximum temperature of the AI-Card
# ret: the maximum temperature of chips
# ===========================================================

function riser_get_max_temp_of_aicard(){
    local riser_bus_num=`echo "${#RISER_BUS[@]}"`
    local channel_num=`echo "${#CH_ADDR[@]}"`
    local chip_num=0
    local temp_max=0

    for ((i = 0 ; i < $riser_bus_num ; i++)) ;
    do
        # echo ${RISER_BUS[$i]}
        for ((j = 0 ; j < $channel_num ; j++)) ;
        do
            # echo ${CH_ADDR[$j]}
            val=$(is_1684_card_exist ${RISER_BUS[$i]} ${CH_ADDR[$j]})
            if [ "$val" == "SC7" ] ;then
                chip_num=$SC7_CHIP_NUM
            elif [ "$val" == "SC5" ] ; then
                chip_num=$SC5_CHIP_NUM
            fi
            if [ $chip_num -gt 0 ] ;then
                for ((q = 0 ; q < $chip_num ; q++)) ;
                do
                    chipTempHex=$(i2cget -f -y ${RISER_BUS[$i]} ${CHIP_ADDR[$q]} 0x00)
                    chipTempDec=$(echo $chipTempHex | printf "%d\n" $(cat))
                    boardTempHex=$(i2cget -f -y ${RISER_BUS[$i]} ${CHIP_ADDR[$q]} 0x01)
                    boardTempDec=$(echo $boardTempHex | printf "%d\n" $(cat))
                    temp_max=$(riser_get_max_value $chipTempDec $boardTempDec $temp_max)
                done
                chip_num=0
            fi
        done
    done
    echo "$temp_max"
}


function is_1684_card_exist_on_all_risercard(){
    local riser_bus_num=`echo "${#RISER_BUS[@]}"`
    local channel_num=`echo "${#CH_ADDR[@]}"`
    local chip_num=0

    for ((i = 0 ; i < $riser_bus_num ; i++)) ;
    do
        # echo ${RISER_BUS[$i]}
        for ((j = 0 ; j < $channel_num ; j++)) ;
        do
            # echo ${CH_ADDR[$j]}
            val=$(is_1684_card_exist ${RISER_BUS[$i]} ${CH_ADDR[$j]})
            if [ $val == "SC7" ] || [ $val == "SC5" ] ;then
                echo "1"
                return
            fi
        done
    done
    echo "0"
}