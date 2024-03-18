# STLINK-V3-BRIDGE Examples

## First steps

### Download STLINK-V3 Bridge

Download bridge cpp and header files from [this](https://www.st.com/en/development-tools/stlink-v3-bridge.html) and copy zip content (ST-LINK-V3-BRIDGE folder) to bridge folder.

### Download STM32CubeProgrammer

Download STM32CubeProgrammer from [this](https://www.st.com/en/development-tools/stm32cubeprog.html) and install for all users, then, copy `<INSTALLATION_DIR>/STM32CubeProgrammer/Drivers/FirmwareUpgrade/native/*` libraries to the drivers folder. the STM32CubeProgrammer installation will also be responsible for installing the udev rules for STLink V3.

### Download STSW-LINK007

Download ST link upgrade application from [this](https://www.st.com/en/development-tools/stsw-link007.html), extract and copy native folder contents to drivers repo folder.