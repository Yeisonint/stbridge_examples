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
        void readGPIO(Brg_GpioInitT &gpioParams, Brg_GpioValT &gpioReadVal){
            if(m_pBrg_==nullptr){
                log_err_("Open device first");
            }
            uint8_t gpioErrMsk;
            brgStat_ = m_pBrg_->ReadGPIO(gpioParams.GpioMask, &gpioReadVal, &gpioErrMsk);
            if( (brgStat_ != BRG_NO_ERR) || (gpioErrMsk!=0) ) {
                log_err_(std::format(" Bridge Read error: {}",gpioErrMsk));
            }
        }
    };
}