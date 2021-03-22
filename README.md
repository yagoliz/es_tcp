# es_tcp: Reaching higher frequencies with RTL-SDR

**es\_tcp** is a modification of the well known [**rtl_tcp**](https://github.com/pinkavaj/rtl-sdr/blob/master/src/rtl_tcp.c) utility that can extend the frequency range of the RTL-SDR from 0-6 GHz. This project also takes inspiration (and the files under ```converter/``` files) from this other great project: [**es_sensor**](https://github.com/electrosense/es-sensor)

Tested with GQRX in Debian (sid). Note: You will need to enable the **No Limits** check box in order to reach higher frequencies than the default 1766 MHz.

Here's how the 2.4 GHz band looks like captured with an RTL-SDR:

![Wifi Band with GQRX](./img/gqrx.gif)

## Hardware requirements

To be able to reach 6 GHz you will need one of this [bad boys](https://github.com/electrosense/hardware) otherwise it will work as the regular rtl_tcp.

### Installing the udev rules
To install the udev rule and have the downconverter appear as ```/dev/esenseconv``` you can do the following:
```bash
sudo cp rules/z10-converter.rules /etc/udev/rules.d/
sudo udevadm control --reload
sudo udevadm trigger
```

## Building

Tested on debian (sid), but it should work for Ubuntu 20.04 and Debian 10. First install the necessary libraries

```bash
sudo apt install librtlsdr-dev libusb-1.0-0-dev cmake build-essential
```

Then build as you would any cmake project:
```bash
mkdir build && cd build
cmake ..
make -j$(nproc)
```

## Running

```bash
./es_tcp -h
```

```bash
Usage:  [-a Listen address]
        [-c Converter path (default: /dev/ttyACM0)      [-p Listen port (default: 1234)]
        [-f Frequency to tune to [Hz]]
        [-g Gain (default: 0 for auto)]
        [-s Sample rate in Hz (default: 2048000 Hz)]
        [-b Number of buffers (default: 15, set by library)]
        [-n Max number of linked list buffers to keep (default: 500)]
        [-d Device index (default: 0)]
        [-P PPM error (default: 0)]
        [-T Enable bias-T on GPIO PIN 0 (works for rtl-sdr.com v3 dongles)]
```

Example: Listen on port 1222 and the converter path is ```/dev/esenseconv```
```bash
./es_tcp -p 1222 -c /dev/esenseconv
```



