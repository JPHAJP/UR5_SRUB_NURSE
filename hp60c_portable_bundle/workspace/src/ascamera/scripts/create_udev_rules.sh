#!/bin/bash
#
#  @file      create_udev_rules.sh
#  @brief     create udev rules for angstrong camera
#
#  Copyright (c) 2022 Angstrong Tech.Co.,Ltd
#
#  @author    Angstrong SDK develop Team
#  @date      2022/05/25
#  @version   1.0

GREEN="\e[32;1m"
NORMAL="\e[39m"
RED="\e[31m"

CUR_DIR=$(dirname "$(readlink -f "$0")")

# check for whitespace in $CUR_DIR and exit for safety reasons
grep -q "[[:space:]]" <<<"${CUR_DIR}" && { echo "\"${CUR_DIR}\" contains whitespace. Not supported. Aborting." >&2 ; exit 1 ; }

if [ $EUID -ne 0 ]; then
    echo -e "${RED}---This script requires root privileges, trying to use sudo${NORMAL}"
    sudo "$CUR_DIR/create_udev_rules.sh" "$@"
    exit $?
fi

echo "copy $CUR_DIR/angstrong-camera.rules to /etc/udev/rules.d"
cp $CUR_DIR/angstrong-camera.rules /etc/udev/rules.d/angstrong-camera.rules

# reload rules
echo "reload udev rules"
# udevadm control --reload
udevadm control --reload-rules
service udev restart
udevadm trigger

echo "reload udev rules done"
