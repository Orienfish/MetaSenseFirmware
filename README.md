# METASENSE Ari Quality Node Firmware
The firmware works with particle photon (http://particle.io) and the custom hardware designed by UCSD MetaSense team (http://metasense.ucsd.edu).
This version of the firmware has been modified to compute part of the herarhcical NN and communicate with the rest of the smarthome.

To use the code compile and upload it to your photon using Particle CLI tools.

## License
The code developed by UCSD is provided under BSD 3 Clause Open Source License (the code is marked with a banner in the source file).
Libraries we used and adapted are provided under thier respective licences.



## Setup Photon-based MetaSense

This firmware is for MetaSense developed by UCSD SeeLab. On the board, there are all kinds of sensors: temperature, humidity, barometer, VOC, CO2, CO, OX, NO2. Check the [paper](http://seelab.ucsd.edu/papers/spie17.pdf) for more details.

### Background - Device Modes

Listening Mode - blink blue slowly - waiting to connect to Wi-Fi

Wi-Fi Network Reset - blink blue rapidly - erase existing network - hold setup button for ten seconds

DFU Mode - blink yellow - for firmware program - hold both button and release reset button first, wait until it blinks yellow

For more information, check the [official guide](https://docs.particle.io/tutorials/device-os/led/photon/#standard-modes).

### Environment Setup

Use the CLI tool in terminal is the most convenient option. Following the [installation guide](https://docs.particle.io/tutorials/developer-tools/cli/).

### Setup a Photon

1. Verify your account is working! Check [setup account](https://login.particle.io/signup?app=setup&redirect=http://setup.particle.io).

   **Trouble Shooting #1**: if you got the output "Please login", try the following commands to re-log into your account.

   ```shell
   particle logout
   particle login
   ```

   **Trouble Shooting #2**: if you need the MAC address of the photon, you can use a serial tool such as PuTTy, enter "m" and you will get the MAC address. For detailed information, check [here](https://blog.jongallant.com/2015/08/particle-photon-mac-address/).

2. Enter Wi-Fi Network Reset Mode. Then use `particle setup` to guide you through setting up your account and your device

### Compile, Flash and Serial Monitor

**Important Note**: To compile and flash program to your Photon, you only need to setup your Particle account and make sure you have network connection. Setting up Photon and register it on network is not required.

1. To compile a project in the **root** directory (not `src` directory)

   ```shell
   particle compile photon my_project_folder --saveTo firmware.bin
   ```

   For more options, check [official guide of CLI - particle compile](https://docs.particle.io/reference/developer-tools/cli/#particle-compile)

2. To flash to the device locally

   First enter DFU mode. Make sure dfu-util is installed. Then run the following command:

   ```shell
   particle flash --usb firmware.bin
   ```

3. Use a Serial Monitor (read-only) is usually very helpful during debugging. However, you will have to use other tools like PuTTY to interact over serial.

   ```shell
    particle serial monitor --port COM3
   ```

   **Important Note**: If the photon blinks green rapidly no matter what you programed, try `particle update` to update the firmware.

   

   