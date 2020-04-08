
#include "InitialSL.h"
#include "STRX.h"
#include "FRAM_TL_CmdDispatcher.h"
#include "SerialFlash.h"
#include "cmd_action_adcs.h"
#include "FRAM_cmd_manager.h"
#include "Temperature.h"
#include "MTQ_OBC.h"
#include "MCP23008.h"
#include "OBC_TlmGenerator.h"
#include "cmd_action_power.h"
#include "cmd_action_cmdtlm.h"
#include "cmd_item.h"
#include "NormalSL.h"
#include "SafeSL.h"
#include "InitialSL.h"
#include "cmd_action.h"
#include "PeripheralTL.h"
#include "EPS_update.h"
#include "STRX_update.h"
#include "GNSS_receiver.h"
#include "ADCS_PacketHandler.h"
#include "TlmCmdTL.h"
#include "MSN_PacketHandler.h"
#include "GS_DataHandler.h"
#include "CommonSerialReceiver.h"
#include "UVC_app.h"
#include "AppStatus.h"
#include "AnomalyHandler.h"
#include "TL_CmdDispatcher.h"
#include "RT_CmdDispatcher.h"
#include "RT_TlmDispatcher.h"
#include "NormalTL.h"
#include "Debug_Application.h"
#include "nop.h"
#include "AppRegistry.h"
#include "CommandDispatcher.h"
#include "BlockCommandDefinisions.h"
#include "ModeDefinisions.h"
#include "ModeManager.h"
#include "AppManager.h"
#include "AppInfo.h"
#include "AnomalyLogger.h"
#include "BlockCommandTable.h"
#include "TaskDispatcher.h"
#include "OBCTime.h"
#include "TimeManager.h"

#include "arduino.h"
#include "plib.h"

#include "DSPI.h"
#include "DTWI.h"
#include "InternalDevice.h"
#include "RTC.h"
#include "TimeManager.h"

/* Define the Interrupt Service Routine (ISR) */
void __attribute__((interrupt)) ISR() {
	
	TMGR_count_up();
	clearIntFlag(_TIMER_3_IRQ);
}

void set_interrupt_timer(uint32_t interval_usec) {
	
	uint32_t prescale = 64;
	uint32_t period = (interval_usec / prescale) * CPU_TICKS_PER_SECOND;
	T2CON		= 0b0000000000101000;  // ��ؽ��׶��Ēl 1:64, �����ۯ�(FCY)�Ŏg�p�A32�r�b�g����
	TMR2		= 0;                   // �^�C�}�[�̃J�E���^�[������������

	uint16_t period_l = (period & 0xFFFF);
	uint16_t period_h = (period >> 16) & 0xFFFF;
	PR2			= period_l;
	PR3			= period_h;

	setIntVector(_TIMER_3_VECTOR, ISR);
	setIntPriority(_TIMER_3_VECTOR, 4, 0);
	clearIntFlag(_TIMER_3_IRQ);
	setIntEnable(_TIMER_3_IRQ);
	T2CONbits.TON = 1;           // �^�C�}�[�J�n
}

void setup()
{
	DEBUG_Serial.begin(BITRATE_DEBUG);
	DATA_Serial.begin(BITRATE_DATA);
	STRX_Serial.begin(BITRATE_STRX);
	ADCS_Serial.begin(BITRATE_ADCS);
	MSN_Serial.begin(BITRATE_MSN);
	GNSSR_Serial.begin(BITRATE_GNSSR);
	pinMode(STRX_IO_TX_CLK, OUTPUT);   // �o�͂ɐݒ�
	pinMode(STRX_IO_TX_DATA, OUTPUT);   // �o�͂ɐݒ�


	InternalDevice_init();  //for SPI, I2C
	
	if (FRAM_Begin() == Success) { DEBUG_Serial.println("Success FRAM for Status initialize"); }
	else { DEBUG_Serial.println("Failed FRAM for Status"); }

	if (FRAM_TLC_Begin() == Success) { DEBUG_Serial.println("Success FRAM for TLC initialize"); }
	else { DEBUG_Serial.println("Failed FRAM for TLC"); }
	
	
	RTC_init();					DEBUG_Serial.println("RTC_init()");
	Serial_MUX_init();			DEBUG_Serial.println("Serial_MUX_init()");
	TlmCmd_init();				DEBUG_Serial.println("TlmCmd_init()");
	PH_init();					DEBUG_Serial.println("PH_init()");
	AL_initialize();			DEBUG_Serial.println("AL_initialize()");
	AM_initialize();			DEBUG_Serial.println("AM_initialize()");
	AR_load_initial_settings(); DEBUG_Serial.println("AR_load_initial_settings()");
	AM_initialize_all_apps();	DEBUG_Serial.println("AM_initialize_all_apps()");
	BCT_initialize();			DEBUG_Serial.println("BCT_initialize()");
	TMGR_init();				DEBUG_Serial.println("TMGR_init()");
	MM_initialize();			DEBUG_Serial.println("MM_initialize()");
	TDSP_initialize();			DEBUG_Serial.println("TDSP_initialize()");

	DEBUG_Serial.println("C&DH Flight software ver. S1: complete initialize ");
	
	set_interrupt_timer(1000);

}

void loop()
{
	while (1) 
	{ 
		TDSP_execute_pl_as_task_list();

	}

}