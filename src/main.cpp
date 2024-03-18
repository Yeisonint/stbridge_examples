#include <cstdlib>
#include <stdio.h>
#include "common.hpp"

int main(int argc, char **argv){
    STlinkBridge::STlinkBridgeWrapper bridge;
    bridge.close();
    bridge.loadStlinkLibrary();
    std::map<uint32_t, STLink_DeviceInfo2T> devices_list = bridge.enumDevices();
    bridge.open();
    // GPIO
    uint8_t bridgeCom = COM_GPIO;
    Brg_GpioConfT gpioConf[BRG_GPIO_MAX_NB];
    for(int i=0; i<BRG_GPIO_MAX_NB; i++) {
        gpioConf[i].Mode = GPIO_MODE_INPUT;
        gpioConf[i].Speed = GPIO_SPEED_MEDIUM;
        gpioConf[i].Pull = GPIO_NO_PULL;
        gpioConf[i].OutputType = GPIO_OUTPUT_PUSHPULL; // unused in input mode
    }
    Brg_GpioInitT gpioParams={
        .GpioMask=BRG_GPIO_ALL,
        .ConfigNb=BRG_GPIO_MAX_NB,
        .pGpioConf = &gpioConf[0]};
    Brg_GpioValT gpioReadVal[BRG_GPIO_MAX_NB];
    bridge.configGPIO(gpioParams);
    bridge.readGPIO(BRG_GPIO_ALL,gpioReadVal[0]);
    for(int i=0; i<BRG_GPIO_MAX_NB; i++) {
        std::cout << "GPIO " << i << " Value: " << (gpioReadVal[i]==GPIO_SET?"High":"Low") << std::endl;
    }

    // I2C
    bridge.initI2C();
    bridge.close();
    return 0;
}
