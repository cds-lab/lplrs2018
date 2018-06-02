# lplrs2018
reward system

Depends on library for the HX711 on github: https://github.com/bogde/HX711

To initiate control via serial, use command line utility `screen`:  
```
$ screen /dev/ttyACM0 115200
```

To use the MATLAB commands, MATLAB must be able to open the serial port associated with the Arduino (often `/dev/ttyACM0`).  Because `/dev/ttyACM0` is actually virtual (ACM means abstract control model) MATLAB seems to be unable to open it.  Instead, associate a device node normally referencing a hardware serial port such as `/dev/ttyS0` with `/dev/ttyACM0` by adding a java options file to the MATLAB folder:

```
$ cd /usr/local/MATLAB/R2015b/bin/glnxa64     
$ sudo echo '-Dgnu.io.rxtx.SerialPorts=/dev/ttyS0:/dev/ttyS1:/dev/USB0:/dev/ttyACM0' > java.opts
```
