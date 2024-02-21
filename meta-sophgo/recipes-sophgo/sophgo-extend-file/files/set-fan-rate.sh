#!/bin/bash
function write_pwm(){
    for file in `ls $1`
    do
        if [ -d $1"/"$file ]
        then
            cd $1"/"$file
            if [ -e $2 ]
            then
            echo "$3" > $2
            fi
        fi
    done
}

value=$(systemctl is-active sophgo-fan-control)

if [ "x$1" = "xauto" ] ;then
    if [ "x$value" = "xactive" ]; then
        echo "sophgo-fan-control.service has been started"
    else
        echo "start sophgo-fan-control.service"
        systemctl start sophgo-fan-control.service
    fi
else

    if [ "x$value" = "xactive" ]; then
        echo "stop sophgo-fan-control.service"
        systemctl stop sophgo-fan-control.service
    fi


    if [ $# -lt 1 ]; then
        echo "ERROR:  No command line arguments "
        exit 1
    fi

    case "$1" in
        [1-9][0-9]*)
            echo "$1 is valid."
            ;;
        *)
            echo "Usage: $0 number {0~255}"
            exit 1
            ;;
    esac

    if [[ $1 -ge 0 && $1 -le 255 ]]; then
        write_pwm /sys/class/hwmon pwm1 $1
    else
        echo "Usage: $0 number {0~255}"
        exit 1
    fi
fi



