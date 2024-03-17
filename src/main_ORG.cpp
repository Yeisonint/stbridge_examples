#include <cstdlib>
#include <stdio.h>
#include "bridge.h"
int main(int argc, char** argv)
{
	Brg_StatusT brgStat = BRG_NO_ERR;
    STLinkIf_StatusT ifStat = STLINKIF_NO_ERR;
	char path[MAX_PATH];
	uint32_t i, numDevices;
	int firstDevNotInUse=-1;
	STLink_DeviceInfo2T devInfo2;
	uint32_t StlHClkKHz, comInputClkKHz;
	Brg* m_pBrg = NULL;
	STLinkInterface *m_pStlinkIf = NULL;
	char m_serialNumber[SERIAL_NUM_STR_MAX_LEN];

	// Note: cErrLog g_ErrLog; to be instanciated and initialized if used with USING_ERRORLOG
	// In case previously used, close the previous connection (not the case here)
	if( m_pBrg!=NULL )
	{
		m_pBrg->CloseBridge(COM_UNDEF_ALL);
		m_pBrg->CloseStlink();
		delete m_pBrg;
		m_pBrg = NULL;
	}
	if( m_pStlinkIf !=NULL ) // always delete STLinkInterface after Brg that is using it.
	{
		delete m_pStlinkIf;
		m_pStlinkIf = NULL;
	}

	// USB interface initialization and device detection done using STLinkInterface

	// Create USB BRIDGE interface
	m_pStlinkIf = new STLinkInterface(STLINK_BRIDGE);
	strcpy(path, "");
	// Load STLinkUSBDriver library 
	// In this example STLinkUSBdriver (dll on windows) must be copied near test executable
	ifStat = m_pStlinkIf->LoadStlinkLibrary(path);
	if( ifStat!=STLINKIF_NO_ERR ) {
		printf("STLinkUSBDriver library (dll) issue \n");
	}

	if( ifStat==STLINKIF_NO_ERR ) {
		ifStat = m_pStlinkIf->EnumDevices(&numDevices, false);
		// or brgStat = Brg::ConvSTLinkIfToBrgStatus(m_pStlinkIf->EnumDevices(&numDevices, false));
	}

	// Choose the first STLink Bridge available
	if( (ifStat==STLINKIF_NO_ERR) || (ifStat==STLINKIF_PERMISSION_ERR) ) {
		printf("%d BRIDGE device found\n", (int)numDevices);

		for( i=0; i<numDevices; i++ ) {
			ifStat = m_pStlinkIf->GetDeviceInfo2(i, &devInfo2, sizeof(devInfo2));
			printf("Bridge %d PID: 0X%04hx SN:%s\n", (int)i, (unsigned short)devInfo2.ProductId, devInfo2.EnumUniqueId);

			if( (firstDevNotInUse==-1) && (devInfo2.DeviceUsed == false) ) {
				firstDevNotInUse = i;
				memcpy(m_serialNumber, &devInfo2.EnumUniqueId, SERIAL_NUM_STR_MAX_LEN);
				printf(" SELECTED BRIDGE Stlink SN:%s\n", m_serialNumber);
			}
		}
	} else if( ifStat==STLINKIF_CONNECT_ERR ) {
		printf("No STLink BRIDGE device detected\n");
	} else {
		printf("enum error\n");
	}

	brgStat = Brg::ConvSTLinkIfToBrgStatus(ifStat);

	// USB Connection to a given device done with Brg
	if( brgStat==BRG_NO_ERR ) {
		m_pBrg = new Brg(*m_pStlinkIf);
	}

	// The firmware may not be the very last one, but it may be OK like that (just inform)
	bool bOldFirmwareWarning=false;


	// Open the STLink connection
	if( brgStat==BRG_NO_ERR ) {
		m_pBrg->SetOpenModeExclusive(true);

		brgStat = m_pBrg->OpenStlink(firstDevNotInUse);

		if( brgStat==BRG_NOT_SUPPORTED ) {
			brgStat = Brg::ConvSTLinkIfToBrgStatus(m_pStlinkIf->GetDeviceInfo2(0, &devInfo2, sizeof(devInfo2)));
			printf("BRIDGE not supported PID: 0X%04hx SN:%s\n", (unsigned short)devInfo2.ProductId, devInfo2.EnumUniqueId);
		}

		if( brgStat==BRG_OLD_FIRMWARE_WARNING ) {
			// Status to restore at the end if all is OK
			bOldFirmwareWarning = true;
			brgStat = BRG_NO_ERR;
		}
	}
	// Test Voltage command
	if( brgStat==BRG_NO_ERR ) {
		float voltage = 0;
		// T_VCC pin must be connected to target voltage on debug connector
		brgStat = m_pBrg->GetTargetVoltage(&voltage);
		if( (brgStat != BRG_NO_ERR) || (voltage == 0) ) {
			printf("BRIDGE get voltage error (check if T_VCC pin is connected to target voltage on debug connector)\n");
		} else {
			printf("BRIDGE get voltage: %f V \n", (double)voltage);
		}
	}

	if( (brgStat == BRG_NO_ERR) && (bOldFirmwareWarning == true) ) {
		// brgStat = BRG_OLD_FIRMWARE_WARNING;
		printf("BRG_OLD_FIRMWARE_WARNING: v%d B%d \n",(int)m_pBrg->m_Version.Major_Ver, (int)m_pBrg->m_Version.Bridge_Ver);
	}

	// Test GET CLOCK command
	if( brgStat==BRG_NO_ERR ) {
		uint32_t StlHClkKHz, comInputClkKHz;
		// Get the current bridge input Clk for all com:
		brgStat = m_pBrg->GetClk(COM_SPI, (uint32_t*)&comInputClkKHz, (uint32_t*)&StlHClkKHz);
		printf( "SPI input CLK: %d KHz, ST-Link HCLK: %d KHz \n", (int)comInputClkKHz, (int)StlHClkKHz);
		if( brgStat==BRG_NO_ERR ) {
			brgStat = m_pBrg->GetClk(COM_I2C, (uint32_t*)&comInputClkKHz, (uint32_t*)&StlHClkKHz);
			printf( "I2C input CLK: %d KHz, ST-Link HCLK: %d KHz \n", (int)comInputClkKHz, (int)StlHClkKHz);
		}
		if( brgStat==BRG_NO_ERR ) {
			brgStat = m_pBrg->GetClk(COM_CAN, (uint32_t*)&comInputClkKHz, (uint32_t*)&StlHClkKHz);
			printf( "CAN input CLK: %d KHz, ST-Link HCLK: %d KHz \n", (int)comInputClkKHz, (int)StlHClkKHz);
		}
		if( brgStat==BRG_NO_ERR ) {
			brgStat = m_pBrg->GetClk(COM_GPIO, (uint32_t*)&comInputClkKHz, (uint32_t*)&StlHClkKHz);
			printf( "GPIO input CLK: %d KHz, ST-Link HCLK: %d KHz \n", (int)comInputClkKHz, (int)StlHClkKHz);
		}
		if( brgStat!=BRG_NO_ERR ) {
			printf( "Error in GetClk()\n" );
		}
	}

	// Test GPIO
	if( brgStat==BRG_NO_ERR ) {
		uint8_t bridgeCom = COM_GPIO;
		Brg_GpioInitT gpioParams;
		Brg_GpioConfT gpioConf[BRG_GPIO_MAX_NB];
		Brg_GpioValT gpioReadVal[BRG_GPIO_MAX_NB];
		uint8_t gpioMsk=0, gpioErrMsk;
		int i;

		if( bridgeCom == COM_GPIO ) {
			printf("Run BRIDGE GPIO test\n");
		}else {
			brgStat = BRG_NOT_SUPPORTED;
		}

		if( brgStat == BRG_NO_ERR ) {
			gpioMsk = BRG_GPIO_ALL;
			gpioParams.GpioMask = gpioMsk; // BRG_GPIO_0 1 2 3
			gpioParams.ConfigNb = BRG_GPIO_MAX_NB; //must be BRG_GPIO_MAX_NB or 1 (if 1 pGpioConf[0] used for all gpios)
			gpioParams.pGpioConf = &gpioConf[0];
			for(i=0; i<BRG_GPIO_MAX_NB; i++) {
				gpioConf[i].Mode = GPIO_MODE_INPUT;
				gpioConf[i].Speed = GPIO_SPEED_MEDIUM;
				gpioConf[i].Pull = GPIO_PULL_UP;
				gpioConf[i].OutputType = GPIO_OUTPUT_PUSHPULL; // unused in input mode
			}
			brgStat = m_pBrg->InitGPIO(&gpioParams);
			if( brgStat != BRG_NO_ERR ) {
				printf("Bridge Gpio init failed (mask=%d, gpio0: mode= %d, pull = %d, ...)\n",
				       (int)gpioParams.GpioMask, (int)gpioConf[0].Mode, (int)gpioConf[0].Pull);
			}
		}
		if( brgStat == BRG_NO_ERR ) {
			brgStat = m_pBrg->ReadGPIO(gpioMsk, &gpioReadVal[0], &gpioErrMsk);
			if( (brgStat != BRG_NO_ERR) || (gpioErrMsk!=0) ) {
				printf(" Bridge Read error\n");
			} else {
				// verify all gpio read to 1 (input pull up)
				for(i=0; i<BRG_GPIO_MAX_NB; i++) {
					if( gpioReadVal[i] != GPIO_SET ) {
						brgStat = BRG_VERIF_ERR;
						printf(" Bridge Read Verif error ( gpio %d != SET)\n", i);
					}
				}
			}
		}

		if( brgStat == BRG_NO_ERR ) {
			printf("GPIO Test1 OK \n");
		}
	}

	// Disconnect
	if( m_pBrg!=NULL ) {
		m_pBrg->CloseBridge(COM_UNDEF_ALL);
		m_pBrg->CloseStlink();
		delete m_pBrg;
		m_pBrg = NULL;
	}
	// unload STLinkUSBdriver library
	if( m_pStlinkIf!=NULL ) {
		// always delete STLinkInterface after Brg (because Brg uses STLinkInterface)
		delete m_pStlinkIf;
		m_pStlinkIf = NULL;
	}

	if( brgStat == BRG_NO_ERR )
	{
		printf("TEST SUCCESS \n");
	} else {
		printf("TEST FAIL (Bridge error: %d) \n", (int)brgStat);
	}

	return 0;
}

