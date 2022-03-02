# OCABS - the Open CA Basic Service implementation

This repository contains _OCABS_, a C++ open implementation of the CA Basic Service, to be used for deployement into real hardware, running, for instance, [https://github.com/francescoraves483/OpenWrt-V2X)[OpenWrt-V2X].

This project includes a full open source implementation of the **ETSI CA Basic Service**, with reference to the standard [https://www.etsi.org/deliver/etsi_en/302600_302699/30263702/01.04.01_60/en_30263702v010401p.pdf](ETSI EN 302 637-2 V1.4.1), including the **BTP** and **GeoNetworking** layers. CAMs **version 2** are currently being managed (there is no official support for the older CAMs version 1).

As the project is still work-in-progress, it currently provides only the dissemination of CAMs, while the reception of the same messages should be managed by any other V2X service requiring the reception of CAMs.

**Important**: OCABS needs a source of PVT (Position-Velocity-Time) GNSS data through `gpsd`. Thus, a GNSS device must be available (either real, or emulated, for instance thanks to tools like `gpsfake`) and it must be connected to a `gpsd` instance.

The OCABS main help, with all the possibile options, can be displayed with `./OCABS --help`, after compiling the OCABS binary.

OCABS also supports a proposed enhancement to CAM messages (i.e., the [https://github.com/francescoraves483/EnhancedCAMs-asn1](Enhanced CAMs)), to be used in conjunction with the [https://github.com/francescoraves483/AIM-AutomotiveIntegratedMap](AIM Local Dynamic Map). The transmission of the enhanced messages may break compatibility with other existing standard-compliant stacks (e.g., the Cohda Wireless stack, the [ms-van3t](https://github.com/marcomali/ms-van3t) stack in emulation mode), and it is thus enabled only when explicitly specified through the option `--enable-enhanced-CAMs` (or, in short, `-E`) . If not enabled, standard-compliant version 2 CAMs are sent, without any modification.

When enhanced CAMs are **not** enabled, the following options (and the corresponding short versions) are **no longer** valid and thus should not be specified: `--own-public-ip`, `--own-private-ip`, `--extra-comp-dev-ip`, `--rssi-aux-update-interval`, `--aux-dev-ip`.

# Compiling OCABS

OCABS can be easily compiled on the same device in which it will be run thanks to `make`.

As the only pre-requisite, `libgps-dev` must be installed and available.

To download and build the latest version of OCABS, first, clone this repository:
```
git clone https://github.com/francescoraves483/OCABS-project
``` 
Then, `cd` into the newly created directory:
```
cd OCABS-project
```
Finally, build OCABS:
```
make
```

The OCABS binary file will be called `OCABS`.

Then, make sure that a GNSS data provider is active and `gpsd` is running (you can use the `gpsmon` tool to check if GNSS PVT data is actively being received).

You can then launch OCABS with (a stationID should be specified all the times with the `--vehicle-id` option):
```
./OCABS --vehicle-id <unsigned 32-bit integer stationID> --gnss-device <IP or URL of the device where gpsd is being run - usually localhost/127.0.0.1> --gnss-port <port to use for the connection to gpsd> --interface <interface from which CAMs will be disseminated>
```

If you get a permission denied error (as OCABS is internally using *raw sockets*), you can try launching again OCABS with `sudo` or from the `root` user.

# Cross-compiling OCABS for OpenWrt

OCABS is also though to be easily cross-compiled for any embedded platform or router running OpenWrt.

Cross-compilation also leverages `make`. However, it requires a few additional steps, after cloning the repository.

First of all, an OpenWrt toolchain must be available in the device used for cross-compilation, with the `PATH` and `STAGING_DIR` environment variables being properly set. You can find additional information on how to set up the OpenWrt toolchain [https://openwrt.org/docs/guide-developer/toolchain/crosscompile](here). When performing the build procedure steps, select the `libgps` package with `make menuconfig`.

Then, open the OCABS `Makefile` and update lines 3 and 4 (this needs to be done only once per development PC):
- Line 3 (`OPENWRT_STAGING_DIR`) should be updated to point to the directory where the OpenWrt toolchain `./usr/include` is located (you can start from the already available path to properly update this line)
- Line 4 (`OPENWRT_LIBGPS_VER`) should be updated with the libgps version which came with the OpenWrt toolchain installation (currently, it should be kept to `3.23`, but it may change in the future)

After performing these modifications, `cd` into the OCABS project main directory:
```
cd OCABS-project
```
Finally, build OCABS with the `compileAPU` target:
```
make compileAPU
```
As in the previous case, a binary file named `OCABS` will be generated. The executable can be then transferred to the device running OpenWrt and launched with:
```
./OCABS --vehicle-id <unsigned 32-bit integer stationID> --gnss-device <IP or URL of the device where gpsd is being run - usually localhost/127.0.0.1> --gnss-port <port to use for the connection to gpsd> --interface <interface from which CAMs will be disseminated>
``` 

You can also make OCABS run as a service, by creating two new files inside OpenWrt: 
- `/etc/init.d/OCABS`, to create a new service
- `/etc/config/OCABS`, to configure the OCABS service

A sample of the two files is reported below:

## `/etc/init.d/OCABS` sample

This sample shows how to create the OCABS service through an `init.d` script. More details are available [https://openwrt.org/docs/techref/initscripts](here) and [https://openwrt.org/docs/guide-developer/procd-init-script-example](here).

For a basic working service setup (without Enhanced CAMs), you can freely copy the whole content reported below. This sample `init.d` assumes that the OCABS executable has been placed inside a directory named `/root/OCABS` (see the `procd_set_param command` line).

```
#!/bin/sh /etc/rc.common
 
USE_PROCD=1
 
START=99
STOP=01
 
CONFIGURATION=OCABS
 
start_service() {
    # Reading configuration parameters (i.e. interface and port)
    config_load "${CONFIGURATION}"
    local name
    local every
 
    config_get interface OCABS interface
	config_get gnssport OCABS gnssport
	config_get vehicleID OCABS vehicleID
	config_get gpsdaddress OCABS gpsdaddress
 
    procd_open_instance
 
    # pass config to script on start
    procd_set_param command /bin/bash -c "cd /root/OCABS && ./OCABS --interface $interface --gnss-port $gnssport --vehicle-id $vehicleID --gnss-device $gpsdaddress"
    procd_set_param file /etc/config/OCABS
    procd_set_param stdout 1
    procd_set_param stderr 1
	procd_set_param respawn ${respawn_threshold:-3600} ${respawn_timeout:-60} ${respawn_retry:-0}
    procd_close_instance
}
```

## `/etc/config/OCABS` sample

This sample shows how to create an OCABS configuration file, after setting up the proper `init.d` script.

The configuration file options should match the ones reported in the `config_get` lines of the `init.d` script.

```
config myservice 'OCABS'
	option interface 'wlan1'
	option gnssport '2947'
	option vehicleID '3000112'
	option gpsdaddress '127.0.0.1'
```

In general, this sample allows the user to set the dissemination interface (`interface` option), the gpsd service address (`gpsdaddress` option), to retrieve the PVT data from, the port at which the gpsd address is available (`gnssport` option) and the device station ID (`vehicleID` option). This sample is related to an OCABS service set to disseminate non-Enhanced CAMs only.

## Starting and stopping the OpenWrt service

After setting up the OCABS OpenWrt service, you can start it with:
```
service OCABS start
```
And stop it with:
```
service OCABS stop
```
The service can also be enabled to run at start-up with:
```
service OCABS enable
```
The execution at start-up can be disabled, instead, with:
```
service OCABS disable
```

When running as a service, the output of OCABS is not directly available, but it can be retrieved with:
```
logread
```


# Contact and License information

This project is licensed under a GPL-3.0 License. Please see also the `LICENSE` file for more details.

For any question, please write to _francescorav.es483@gmail.com_. Thanks!
