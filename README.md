(yet another prototype of)  
## Inertial Navigation System

**Version:** 2.0 2016-07-29

### Release notes:
This version fully supports quaternion 3D calculations.  
However, due to the complicated process of calibrations needed,  
this version has no demonstrative advantages in results over  
the previous plane version.  

If you still want to see something new, look at `nav/img/test6-e0-e1-diff.png`   
(or see image at the and of this page). If I manually multiply projection  
of g to axis k with some coefficient from zero to one,   
I will be able to correct trajectory. But, for every test I need   
to pick up this number exclusively.  

There is still lack of any smart filters.  

Sensors supported: Ivensense MPU-6050 only.  

HW Connection: Sensor is connected to I2C-1 Bus, interrupt line is connected  
to GPIO pin. See .dts file.  

Sensor settings are hardcoded into driver.  

### TODO:
* Introduce smart calibrations.
* Add smart filter(s).
* Export settings of sensor driver to sysfs.

### List of directories:
* `./nav` -- userspace calculation program
* `./drv` -- sensor driver
* `./ref` -- reference materials
* `./pub` -- presentation, public speech
* `./btn` -- utility for demonstrtation, *board-specific*

### Installation:
Copy mknod.sh to your board.  
Edit device tree of your board to add mpu6050 sensor support.   
Look in `bcm2708-rpi-b.dts` for reference.   

On the host computer: go to `./drv`, add you cross compiler to Makefile.  
Run  
```sh 
$ make
```
Copy fast-mpu6050.ko to `/lib/modules/<your kernel release>/extra` on your board.   

On your board:  
Run  
```sh
# depmod
```
Reboot.  
After each reboot of your board you should run  
```sh
# ./mknod.sh
```
in order to create `/dev/mpu6050` node.  

On the host: go to `./nav`, add you cross compiler to Makefile.  
Run  
```sh
$ make
```
Copy nav to your board.  

On the host:  
Run  
```sh
$ make PLATFORM=THIS
```
This makes the host version of nav. Use it to calculate trajectory.  
Warning: this will rewrite cross-version of nav binary you've  
compiled in previous step.  

### How to run program:
see `./nav/README`

---
![test6-e0-e1-diff](./nav/img/test6-e0-e1-diff.png)
