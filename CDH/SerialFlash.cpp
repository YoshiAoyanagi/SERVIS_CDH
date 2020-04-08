// 
// 
// 

#include "SerialFlash.h"

DSPI0  flash_spi_dev;

SerialFlash::SerialFlash(char cs_pin)
{
	flash_cs = cs_pin;
	pinMode(flash_cs, OUTPUT);
}

void SerialFlash::dSPI_init(char cs_pin)
{
	
}

unsigned char  SerialFlash::dSPI_transfer(unsigned char data)
{
	return (flash_spi_dev.transfer(data));
}
void  SerialFlash::dSPI_select(void)
{
	digitalWrite(flash_cs, LOW);
}
void  SerialFlash::dSPI_deselect(void)
{
	digitalWrite(flash_cs, HIGH);
}

int SerialFlash::Begin(char cs_pin)	//FlashROM�g�p�J�n
{
	dSPI_init(cs_pin);
	int i_ret = readID();
	if (i_ret != 0)
	{
		Serial.println("SerialFlashROM failed!\n");
		return FLASH_STS_FAILED;
	}
	return FLASH_STS_IDLE;
}

int SerialFlash::readID(void)
{
	Ready();
	dSPI_select();
	int i;
	unsigned char buf[18];
	dSPI_transfer(FLASH_CMD_READ_ID);

	Serial.print("SerialFlash ID: ");

	for (i = 0; i < 18; i++)
	{
		buf[i] = dSPI_transfer(0x00);
		Serial.print(buf[i], HEX); Serial.print(" ");
	}
	Serial.print("\n");
	dSPI_deselect();

	if (buf[0] == (unsigned char)(FLASH_ID >> 16)
		&& buf[1] == (unsigned char)(FLASH_ID >> 8))
		return FLASH_STS_IDLE;
	else
		return FLASH_STS_FAILED;
}

int SerialFlash::WriteEnable(void)	//FlashRom�ւ̏������ݗL��
{
	dSPI_select();
	dSPI_transfer(FLASH_CMD_WRITE_ENABLE);
	dSPI_deselect();
	return FLASH_STS_IDLE;
}
int SerialFlash::WriteDisable(void)	//FlashRom�ւ̏������ݖ���
{
	dSPI_select();
	dSPI_transfer(FLASH_CMD_WRITE_DISABLE);
	dSPI_deselect();
	return FLASH_STS_IDLE;
}

int SerialFlash::IsBusy(void)		//�����X�e�[�^�X�`�F�b�N(1: ��ƒ��A0: �A�C�h����)
{
	int timeout = 0;

	while (1)
	{
		dSPI_select();
		dSPI_transfer(FLASH_CMD_READ_STATUS_REG);

		unsigned char i_ret = dSPI_transfer(0x00);
		dSPI_deselect();

		if (i_ret & FLASH_STS_BUSY_MASK)
			return FLASH_STS_BUSY;
		else
			return FLASH_STS_IDLE;

		timeout++;
		if (timeout > 0xFF)
			return FLASH_STS_FAILED;
	}
}

int SerialFlash::ReadData(unsigned char *buf, unsigned int sector, unsigned int inner_address, unsigned int size)
{
	unsigned int address = sector;
	//address <<= 12;
	address <<= 16;
	address += inner_address;

	Ready();

	dSPI_select();
	dSPI_transfer(FLASH_CMD_READ_DATA);

	dSPI_transfer((address >> 16) & 0xFF);	//�ǂݍ��݃A�h���X�w��(1byte)
	dSPI_transfer((address >> 8) & 0xFF);	//�ǂݍ��݃A�h���X�w��(2byte)
	dSPI_transfer((address & 0xFF));			//�ǂݍ��݃A�h���X�w��(3byte)

	int i;
	for (i = 0; i < size; i++)
	{
		buf[i] = dSPI_transfer(0x00);
	}
	dSPI_deselect();

	return FLASH_STS_IDLE;
}

int SerialFlash::PageWrite(unsigned char *write_buf, unsigned int sector, unsigned int inner_address, unsigned int size)
{
	unsigned int address = sector;
	address <<= 16;
	address += inner_address;

	Ready();
	WriteEnable();	//Enable

	dSPI_select();
	dSPI_transfer(FLASH_CMD_PAGE_PROGRAM);
	dSPI_transfer((address >> 16) & 0xFF);	//�������݃A�h���X�w��(1byte)
	dSPI_transfer((address >> 8) & 0xFF);	//�������݃A�h���X�w��(2byte)
	dSPI_transfer((address & 0xFF));			//�������݃A�h���X�w��(3byte)

	int i;
	for (i = 0; i < size; i++)
	{
		dSPI_transfer(write_buf[i]);
	}

	dSPI_deselect();

	IsBusy();

	return FLASH_STS_IDLE;
}

int SerialFlash::Reset(void)
{
	dSPI_select();
	dSPI_transfer(FLASH_CMD_RESET_ENABLE);
	dSPI_deselect();

	dSPI_select();
	dSPI_transfer(FLASH_CMD_RESET_MEMORY);
	dSPI_deselect();
	return FLASH_STS_IDLE;
}
int SerialFlash::ClearRegister()
{
	dSPI_select();
	dSPI_transfer(FLASH_CMD_CLEAR_REGISTER);
	dSPI_deselect();

	return FLASH_STS_IDLE;
}

int SerialFlash::EraseSector(unsigned int sector)
{
	unsigned int address = sector;
	//address <<= 12;
	address <<= 16;

	Ready();
	WriteEnable();
	dSPI_select();

	dSPI_transfer(FLASH_CMD_ERASE_SECTOR);
	dSPI_transfer((address >> 16) & 0xFF);	//�������݃A�h���X�w��(1byte)
	dSPI_transfer((address >> 8) & 0xFF);	//�������݃A�h���X�w��(2byte)
	dSPI_transfer((address & 0xFF));			//�������݃A�h���X�w��(3byte)

	dSPI_deselect();

	IsBusy();
	delay(10);	//�҂����Ԓ����K�v�ȉ\������

	return FLASH_STS_IDLE;
}

int SerialFlash::Suspend()
{
	dSPI_select();
	dSPI_transfer(FLASH_CMD_SUSPEND);
	dSPI_deselect();
	return FLASH_STS_IDLE;
}

int SerialFlash::Ready()
{
	dSPI_select();
	dSPI_transfer(FLASH_CMD_READ_FLAG_STATUS_REG);
	unsigned char i_ret = dSPI_transfer(0x00);
	dSPI_deselect();

	if (i_ret == FLASH_STS_READY)
		return FLASH_STS_IDLE;

	return FLASH_STS_BUSY;
}

