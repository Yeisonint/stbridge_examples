#pragma once
#include <stdio.h>

#include <iostream>
#include <memory>
#include <map>
#include <cmath>
#include <tuple>
#ifndef __cpp_lib_format
  // std::format polyfill using fmtlib
  #include <fmt/core.h>
  namespace std {
  using fmt::format;
  }
#else
  #include <format>
#endif

#include "bridge.h"

#define RED   "\x1B[31m"
#define GRN   "\x1B[32m"
#define YEL   "\x1B[33m"
#define BLU   "\x1B[34m"
#define MAG   "\x1B[35m"
#define CYN   "\x1B[36m"
#define WHT   "\x1B[37m"
#define RESET "\x1B[0m"

namespace STlinkBridge{
    class STlinkBridgeException : public std::exception{
    private:
        std::string msg_;
    public:
        STlinkBridgeException(std::string msg):msg_(msg){};
        char *what(){
            return (char *)msg_.c_str();
        }
    };
    class STlinkBridgeWrapper{
    private:
    // Common
        Brg_StatusT brgStat_ = BRG_NO_ERR;
        STLinkIf_StatusT ifStat_ = STLINKIF_NO_ERR;
        char lib_path_[MAX_PATH];
        uint32_t i, numDevices_;
        int firstDevNotInUse_;
        STLink_DeviceInfo2T devInfo2_;
        std::shared_ptr<Brg> m_pBrg_;
        std::shared_ptr<STLinkInterface> m_pStlinkIf_;
        uint32_t StlHClkKHz_, comInputClkKHz_, com_param_;
        char m_serialNumber_[SERIAL_NUM_STR_MAX_LEN];
        std::map<uint32_t, STLink_DeviceInfo2T> devices_list_;
        void log_info_(std::string msg){
            std::cout << YEL << msg << RESET << std::endl;
        }
        void log_err_(std::string msg, bool throw_ex=true){
            std::cout << RED << msg << RESET << std::endl;
            if(throw_ex)
                throw STlinkBridgeException(msg+"\n");
        }
    
    public:
        STlinkBridgeWrapper():
        brgStat_(BRG_NO_ERR),
        ifStat_(STLINKIF_NO_ERR),
        firstDevNotInUse_(-1),
        m_pBrg_(nullptr),
        m_pStlinkIf_(nullptr)
        {};

        void close(){
            if (m_pBrg_ != nullptr){
                m_pBrg_->CloseBridge(COM_UNDEF_ALL);
                m_pBrg_->CloseStlink();
                m_pBrg_.reset();
            }
            if (m_pStlinkIf_ != nullptr){
                m_pStlinkIf_.reset();
            }
        }

        void loadStlinkLibrary(std::string lib_path = ""){
            if(m_pStlinkIf_==nullptr){
                m_pStlinkIf_ = std::make_shared<STLinkInterface>(STLINK_BRIDGE);
            }
            strcpy(lib_path_, lib_path.c_str());
            ifStat_ = m_pStlinkIf_->LoadStlinkLibrary(lib_path_);
            if( ifStat_!=STLINKIF_NO_ERR ) {
                throw STlinkBridgeException("STLinkUSBDriver library (dll) issue \n");
            }
        }

        std::map<uint32_t, STLink_DeviceInfo2T> enumDevices(){
            devices_list_.clear();
            ifStat_ = m_pStlinkIf_->EnumDevices(&numDevices_, false);
            if ((ifStat_ == STLINKIF_NO_ERR) || (ifStat_ == STLINKIF_PERMISSION_ERR)){
                log_info_(std::format("{0} bridge device found", numDevices_));
                for (i = 0; i < numDevices_; i++){
                    ifStat_ = m_pStlinkIf_->GetDeviceInfo2(i, &devInfo2_, sizeof(devInfo2_));
                    log_info_(std::format("Bridge {0} PID: 0x{1:04X} SN:{2}", i, devInfo2_.ProductId, devInfo2_.EnumUniqueId));
                    devices_list_[i]=devInfo2_;
                }
            }else if (ifStat_ == STLINKIF_CONNECT_ERR){
                log_err_("No STLink bridge device detected");
            }else{
                log_err_("enum error");
            }
            return devices_list_;
        }

        void open(uint32_t device=-1){
            if(devices_list_.size()==0){
                log_err_("Enumerate devices before call open");
            }
            if(device==-1){
                for (i = 0; i < numDevices_; i++){
                    if ((firstDevNotInUse_ == -1) && (devInfo2_.DeviceUsed == false)){
                        firstDevNotInUse_ = i;
                        memcpy(m_serialNumber_, &devInfo2_.EnumUniqueId, SERIAL_NUM_STR_MAX_LEN);
                        log_info_(std::format("Selected bridge STLink SN: {0}", m_serialNumber_));
                    }
                }
            }else{
                if(devices_list_.count(device)==0){
                    log_err_("Device index not found");
                }
            }

            brgStat_ = Brg::ConvSTLinkIfToBrgStatus(ifStat_);
            if( brgStat_!=BRG_NO_ERR ) {
                log_err_("USB Connection to a given device failed");
            }
            m_pBrg_ = std::make_shared<Brg>(*m_pStlinkIf_);
            m_pBrg_->SetOpenModeExclusive(true);
            brgStat_ = m_pBrg_->OpenStlink(device==-1?firstDevNotInUse_:device);
            firstDevNotInUse_=-1;
            if (brgStat_ == BRG_NOT_SUPPORTED){
                brgStat_ = Brg::ConvSTLinkIfToBrgStatus(m_pStlinkIf_->GetDeviceInfo2(0, &devInfo2_, sizeof(devInfo2_)));
                log_err_(std::format("Bridge not supported PID: 0X{0:04x} SN:{1}", devInfo2_.ProductId, devInfo2_.EnumUniqueId));
            }
            if( brgStat_==BRG_OLD_FIRMWARE_WARNING ) {
                // Status to restore at the end if all is OK
                log_info_(std::format("BRG_OLD_FIRMWARE_WARNING: v{} B{} \n",m_pBrg_->m_Version.Major_Ver,m_pBrg_->m_Version.Bridge_Ver));
                brgStat_ = BRG_NO_ERR;
            }
            float voltage = 0;
            // T_VCC pin must be connected to target voltage on debug connector
            brgStat_ = m_pBrg_->GetTargetVoltage(&voltage);
            if( (brgStat_ != BRG_NO_ERR) || (std::roundf(voltage*100.0)/100.0 == 0.0) ) {
                log_err_("Bridge get voltage error (check if T_VCC pin is connected to target voltage on debug connector)",false);
            } else {
                log_info_(std::format("Bridge get voltage: {}v",voltage));
            }
        }

        std::tuple<uint32_t, uint32_t> getClock(uint8_t bridgeCom) {
            if(m_pBrg_==nullptr){
                log_err_("Open device first");
            }
            brgStat_ = m_pBrg_->GetClk(bridgeCom, (uint32_t*)&comInputClkKHz_, (uint32_t*)&StlHClkKHz_);
            if( brgStat_!=BRG_NO_ERR ) {
                log_err_( "Error in GetClk()" );
            }
            log_info_(std::format("input CLK: {0} KHz, ST-Link HCLK: {1} KHz",comInputClkKHz_,StlHClkKHz_));
            return {comInputClkKHz_,StlHClkKHz_};
        }
        //GPIO
        void configGPIO(Brg_GpioInitT &gpioParams){
            if(m_pBrg_==nullptr){
                log_err_("Open device first");
            }
            brgStat_ = m_pBrg_->InitGPIO(&gpioParams);
            if( brgStat_ != BRG_NO_ERR ) {
                for(int i=0;i<(sizeof(gpioParams.pGpioConf)/sizeof(Brg_GpioConfT));i++)
                log_err_(std::format("Bridge Gpio init failed (mask={}, gpio0: mode= {}, pull = {}, ...)",
                        gpioParams.GpioMask, gpioParams.pGpioConf[i].Mode, gpioParams.pGpioConf[i].Pull),false);
            }
        }
        uint8_t readGPIO(uint8_t GpioMask, Brg_GpioValT &gpioReadVal){
            if(m_pBrg_==nullptr){
                log_err_("Open device first");
            }
            uint8_t gpioErrMsk;
            brgStat_ = m_pBrg_->ReadGPIO(GpioMask, &gpioReadVal, &gpioErrMsk);
            if( (brgStat_ != BRG_NO_ERR) || (gpioErrMsk!=0) ) {
                log_err_(std::format(" Bridge Read error: {}",gpioErrMsk));
            }
            return gpioErrMsk;
        }

        uint8_t writeGPIO(uint8_t GpioMask, Brg_GpioValT &gpioWriteVal){
            if(m_pBrg_==nullptr){
                log_err_("Open device first");
            }
            uint8_t gpioErrMsk;
            brgStat_ = m_pBrg_->SetResetGPIO(GpioMask, &gpioWriteVal, &gpioErrMsk);
            if( (brgStat_ != BRG_NO_ERR) || (gpioErrMsk!=0) ) {
                log_err_(std::format(" Bridge write error: {}",gpioErrMsk));
            }
            return gpioErrMsk;
        }
        //I2C
        void initI2C(){
            printf("I2C test start\n");
            Brg_I2cInitT i2cParam;
            int freqKHz = 100; //100KHz
            uint32_t timingReg; 
            int riseTimeNs, fallTimeNs, DNF;
            bool analogFilter;
            uint16_t sizeWithoutErr = 0;
            uint8_t dataRx[3072], dataTx[3072]; //max size must be aligned with target buffer
            // uint16_t i2cSlaveAddr = 0x39>>1;// convert to 7bit address
            uint16_t i2cSlaveAddr = 0x39;// convert to 7bit address

            // I2C_FAST freqKHz: 1-400KHz I2C_FAST_PLUS: 1-1000KHz
            riseTimeNs = 0; //0-300ns
            fallTimeNs = 0; //0-300ns
            DNF = 0; // digital filter OFF
            analogFilter = true;

            brgStat_ = m_pBrg_->GetI2cTiming(I2C_FAST, freqKHz, DNF, riseTimeNs, fallTimeNs, analogFilter, &timingReg);
            // example I2C_STANDARD, I2C input CLK= 192MHz, rise/fall time (ns) = 0, analog filter on, dnf=0
            // I2C freq = 400KHz timingReg = 0x20602274
            if( brgStat_ == BRG_NO_ERR ) {
                i2cParam.TimingReg = timingReg;
                i2cParam.OwnAddr = 0; // 0  unused in I2C master mode
                i2cParam.AddrMode = I2C_ADDR_7BIT;
                i2cParam.AnFilterEn = I2C_FILTER_ENABLE;
                i2cParam.DigitalFilterEn = I2C_FILTER_DISABLE;
                i2cParam.Dnf = (uint8_t)DNF; //0
                brgStat_ = m_pBrg_->InitI2C(&i2cParam);
            } else {
                    printf("I2C timing error, timing reg: 0x%08x\n", timingReg);
            }

            // 1- send 2 bytes : 1 byte regist address,1 byte data
            dataTx[0] = '#';// register address
            dataTx[1] = 'a';// data
            dataTx[2] = 'b';// data
            dataTx[3] = 'c';// data
            dataTx[4] = 'd';// data
            dataTx[5] = 'e';// data
            dataTx[6] = 'f';// data
            dataTx[7] = 'g';// data
            brgStat_ = m_pBrg_->WriteI2C(dataTx, i2cSlaveAddr, 8, &sizeWithoutErr);
            // or brgStat_ = m_pBrg_->WriteI2C((uint8_t*)&txSize, I2C_7B_ADDR(_i2cSlaveAddr), 4, &sizeWithoutErr);
            if( brgStat_ != BRG_NO_ERR ) {
                printf("BRG Write data size error (sent %d instead of 2)\n", (int)sizeWithoutErr);
            }else{
                printf("write reg 0xff sucess\n");
            }

            // 2-send to read regist address
            dataTx[0] = 0x00;// read regist address
            brgStat_ = m_pBrg_->WriteI2C(dataTx, i2cSlaveAddr, 1, &sizeWithoutErr);
            if( brgStat_ != BRG_NO_ERR ) {
                printf("BRG Write data size error (sent %d instead of 4)\n", (int)sizeWithoutErr);
            }
            // 3- read 4bytes data
            if( brgStat_ == BRG_NO_ERR ) {
                sizeWithoutErr = 0;
                brgStat_ = m_pBrg_->ReadI2C(dataRx, i2cSlaveAddr, 8, &sizeWithoutErr);
                if( brgStat_ != BRG_NO_ERR ) {
                    printf("BRG Read back data size error (read %d instead of 4)\n", (int)sizeWithoutErr);
                } else {
                    for (int i = 0; i < 8; i++)
                    {
                        printf("reg value is : 0x%02x\n", dataRx[i]);
                    }
                }
            }

            printf("I2C test end\n");
        }

    Brg_StatusT BrgRxTxVerifData(uint8_t*pRxBuff, uint8_t*pTxBuff, uint16_t size)
    {
        int i, nb;
        uint16_t sizeWithoutErr =0;
        uint32_t txSize,rxSize=0;
        Brg_I2cAddrModeT addrMode = I2C_ADDR_7BIT;
        uint16_t i2cSlaveAddr = 0x39;

        txSize = (uint32_t) size;

        // 1- send 4 bytes = data size,
        brgStat_ = m_pBrg_->WriteI2C((uint8_t*)&txSize, i2cSlaveAddr, addrMode, 4, &sizeWithoutErr);
        // or brgStat = m_pBrg_->WriteI2C((uint8_t*)&txSize, I2C_7B_ADDR(_i2cSlaveAddr), 4, &sizeWithoutErr);
        if( brgStat_ != BRG_NO_ERR ) {
            printf("BRG Write data size error (sent %d instead of 4)\n", (int)sizeWithoutErr);
        }
        // 2- wait to receive it back
        if( brgStat_ == BRG_NO_ERR ) {
            sizeWithoutErr = 0;
            brgStat_ = m_pBrg_->ReadI2C((uint8_t*)&rxSize, i2cSlaveAddr, addrMode, 4, &sizeWithoutErr);
            // or brgStat_ = m_pBrg_->ReadI2C((uint8_t*)&rxSize, I2C_7B_ADDR(i2cSlaveAddr), 4, &sizeWithoutErr);
            if( brgStat_ != BRG_NO_ERR ) {
                printf("BRG Read back data size error (read %d instead of 4)\n", (int)sizeWithoutErr);
            } else {
                if( rxSize != txSize ) {
                    brgStat_ = BRG_VERIF_ERR;
                    printf("BRG Read back RxSize = %d different from TxSize = %d \n", (int)rxSize, (int)txSize);
                }
            }
        }
        // 3- send data size bytes
        if( brgStat_ == BRG_NO_ERR ) {
            sizeWithoutErr = 0;
            brgStat_ = m_pBrg_->WriteI2C(pTxBuff, i2cSlaveAddr, addrMode, size, &sizeWithoutErr);
            if( brgStat_ != BRG_NO_ERR ) {
                printf("BRG Write data error (sent %d instead of %d)\n", (int)sizeWithoutErr, (int)size);
            }
        }
        // 4- wait to receive same data size bytes back.
        if( brgStat_ == BRG_NO_ERR ) {
            sizeWithoutErr = 0;
            brgStat_ = m_pBrg_->ReadI2C(pRxBuff, i2cSlaveAddr, addrMode, size, &sizeWithoutErr);
            if( brgStat_ != BRG_NO_ERR ) {
                printf("BRG Read back data error (read %d instead of %d)\n", (int)sizeWithoutErr, (int)size);
            } else {
                if( memcmp(pRxBuff, pTxBuff, size) !=0 ) {
                    brgStat_ = BRG_VERIF_ERR;
                    printf("BRG ERROR Read/Write verification error(s)\n");
                }
            }
        }	
        return brgStat_;
    }
    };
}