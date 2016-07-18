#!/bin/bash

module="fast-mpu6050"
device="mpu6050"
mode="666"

# remove stale nodes
rm -f /dev/${device}

major=$(awk "\$2 == \"$device\" {print \$1}" /proc/devices)
mknod /dev/${device} c $major 0

# give appropriate group/permissions, and change the group.
# Not all distributions have staff, some have "wheel" instead.
#group="staff"
#grep -q '^staff:' /etc/group || group="wheel"
#chgrp $group /dev/${device}[0-3]
chmod $mode /dev/${device}
