#!/bin/bash
# /**
#  * @file      gettarget.sh
#  * @brief     get local gcc target
#  *
#  * Copyright (c) 2022 Angstrong Tech.Co.,Ltd
#  *
#  * @author    Angstrong SDK develop Team
#  * @date      2022/03/23
#  * @version   1.0

#  */

## gawk is not the default installation for ubuntu
#gcc -v 2>&1 | grep Target: | gawk '{print $2}'

gcc -v 2>&1 | grep Target: | sed 's/Target: //g'
