// 
// 
// 

#include "MCP4716.h"
#include "Wire.h"

MCP47X6::MCP47X6() {
	devAddr = MCP47X6_DEFAULT_ADDRESS;
}

/******************************************
* Specific address constructor.
* @param address I2C address
* @see MCP47X6_DEFAULT_ADDRESS
*/
MCP47X6::MCP47X6(uint8_t address) {
	devAddr = address;
}

/******************************************
* Verify the I2C connection.
* Make sure the device is connected and responds as expected.
* @return true if connection is valid, false otherwise
*/
bool MCP47X6::testConnection(void) {
	Wire.beginTransmission(devAddr);
	return (Wire.endTransmission() == 0);
}

/******************************************
* Power on and prepare for general usage.
* This device recalls the previously saved state at power up. The begin()
* function reads these values from the DAC and uses them to reinitialize
* the driver since a reset of the microcontroller
* did not necessarily restart the device.
*/
bool MCP47X6::begin() {
	// read the settings from DAC EEPROM

	// reinitialize the device from the read settings
	return writeConfigReg(config);
}

bool MCP47X6::begin(uint8_t newConfig) {
	// initialize the device from the new settings
	config = newConfig;
	return true;
}

/******************************************
* Set the configuration bits for the DAC
*/
void MCP47X6::setGain(uint8_t gain) {
	config = (config & MCP47X6_GAIN_MASK) | (gain & !MCP47X6_GAIN_MASK);
}

void MCP47X6::setVReference(uint8_t vref) {
	config = (config & MCP47X6_VREF_MASK) | (vref & !MCP47X6_VREF_MASK);
}

//void MCP47X6::setPwrDnOutRes(uint8_t pdOutR) {
//  config = (config & MCP47X6_PWRDN_MASK) | (pdOutR & !MCP47X6_PWRDN_MASK);
//}

/******************************************
* Saves current DAC settings into DAC EEPROM for use at power up
*/
bool MCP47X6::saveSettings(void) {
	// read the current volatile settings
	// write the values back to DAC EEPROM
}

/******************************************
* Set DAC output level value
* Valid range is 0x000 to 0xFFF for all parts.
* For the MCP4726 all 4096 steps are used.
* For the MCP4716 only 1024 steps are used. The two LSBs are not used.
* (i.e. value & 0xFFC)
* For the MCP4706 only 256 steps are used. The four LSBs are not used.
* (i.e. value & 0xFF0)
*/
bool MCP47X6::setOutputLevel(uint16_t level) {
	Wire.beginTransmission(devAddr);
	Wire.write((config | MCP47X6_CMD_VOLALL) & MCP47X6_PWRDN_MASK);
	Wire.write((uint8_t)((level >> 8) & 0x0F));
	Wire.write((uint8_t)(level & 0xFF));
	return (Wire.endTransmission() == 0);
}

// Special case for 8-bit device (MCP4706) - saves one byte of transfer
// and is therefore faster
bool MCP47X6::setOutputLevel(uint8_t level) {
	Wire.beginTransmission(devAddr);
	Wire.write((uint8_t)MCP47X6_CMD_VOLDAC);
	Wire.write(level);
	return (Wire.endTransmission() == 0);
}


/******************************************
* Put the DAC into a low power state
* NOTE: writing any settings or DAC output level value
* returns the DAC to the awake power state.
*/
bool MCP47X6::powerDown() {
	return writeConfigReg(config);
}

bool MCP47X6::powerDown(uint8_t pdOutR) {
	config = (config & MCP47X6_PWRDN_MASK) | (pdOutR & !MCP47X6_PWRDN_MASK);
	return writeConfigReg(config);
}

/******************************************
* Private helper function to write just the config register
*/
bool MCP47X6::writeConfigReg(uint8_t theConfig) {
	Wire.beginTransmission(devAddr);
	Wire.write(theConfig | MCP47X6_CMD_VOLCONFIG);
	return (Wire.endTransmission() == 0);
}

















//MCP4716::MCP4716(void) {};
//
//bool MCP4716::init(int address, int vref, int gain)
//{
//	_address = address;
//	_vref = vref;
//	_gain = gain;
//
//	//writeConfigReg(_config);
//	setVReference(_vref);
//	setGain(_gain);
//	writeConfigReg(_config);
//	return(1);
//}
//
//bool MCP4716::writeConfigReg(unsigned char theConfig) {
//	Wire1_beginTransmission(_address);
//	Wire1_write(theConfig | MCP47X6_CMD_VOLCONFIG);
//	Wire1_endTransmission();
//	return (1);
//}
//
//void MCP4716::setGain(unsigned char gain) {
//	_config = (_config & MCP47X6_GAIN_MASK) | (gain & !MCP47X6_GAIN_MASK);
//}
//
//void MCP4716::setVReference(unsigned char vref) {
//	_config = (_config & MCP47X6_VREF_MASK) | (vref & !MCP47X6_VREF_MASK);
//}
//
//bool MCP4716::setOutputLevel(unsigned int level) {
//	Wire1_beginTransmission(_address);
//	Wire1_write((_config | MCP47X6_CMD_VOLALL) & MCP47X6_PWRDN_MASK);
//	Wire1_write((unsigned char)((level >> 8) & 0x0F));
//	Wire1_write((unsigned char)(level & 0xFF));
//	Wire1_endTransmission();
//
//	return (1);
//}