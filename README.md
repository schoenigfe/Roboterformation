# Roboterformation ESP-IDF Repository
__Roboterformation__ is a student project aiming to create a system of multiple mobile robots, which are able to drive in formation. Checkout the [Roboterformation ROS](https://github.com/maxdoesch/Roboterformation_ROS) repository for the ROS counterpart of this project.


## Getting Started

* Follow the [ESP-IDF Get Started Guide](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/get-started/windows-setup.html) to install and setup the Espressife IDF toolchain. 
**Note:** The minimum version required is 4.4
* run `git clone`
* run `git submodule update --init --recursive` to intialize the [esp-dsp](https://github.com/espressif/esp-dsp) submodule
* Run the export script on Windows (export.bat) or source it on Unix (source export.sh) in every shell environment before using ESP-IDF.
* Set the project target to `esp32` with `idf.py set-target esp32`
* run `idf.py menuconfig` to start the configuarion menu
    - go to *Serial flasher config* and set *Flash size* to 8MB
    - go to  *Component config -> ESP32-specific* and enable the option *Support for external, SPI-connected RAM*
    - go to *Wifi Configuration* and enter your Wifi-SSID and Password
    - go to *Socket Configuration* and enter the IP Address of the Ros Robot Bridge Server
    - save the changes and exit
* build the project `idf.py build` the firmware to the robot `idf.py flash`
* check the serial output with `idf.py monitor`
* follow the instruction in the [Roboterformation ROS](https://github.com/maxdoesch/Roboterformation_ROS) repository and check if the robot connects with the Ros Robot Bridge Server

