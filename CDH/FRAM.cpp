// 
// 
// 

#include "FRAM.h"

//FM25V10

DSPI0  fram_spi_dev;

FRAM::FRAM()
{

}

void FRAM::dSPI_init(char cs_pin)
{
	fram_cs = cs_pin;
	pinMode(fram_cs, OUTPUT);
}

unsigned char  FRAM::dSPI_transfer(unsigned char data)
{
	return (fram_spi_dev.transfer(data));
}
void  FRAM::dSPI_select(void)
{
	digitalWrite(fram_cs, LOW);
}
void  FRAM::dSPI_deselect(void)
{
	digitalWrite(fram_cs, HIGH);
}

void FRAM::WriteEnable(void)
{
	dSPI_select();
	dSPI_transfer(FRAM_WREN_ID);
	dSPI_deselect();

	return;
}

void FRAM::Begin(char cs_pin)
{
	dSPI_init(cs_pin);
}

void FRAM::WriteData(unsigned char *data, unsigned int address, unsigned int size)
{
	int i;
	for (i = 0; i < size; i++)
	{
		WriteEnable();

		dSPI_select();
		dSPI_transfer(FRAM_WRITE_ID);
		dSPI_transfer((address >> 16) & 0xFF);	//�������݃A�h���X�w��(MSB)
		dSPI_transfer((address >> 8) & 0xFF);	//�������݃A�h���X�w��(MSB)
		dSPI_transfer((address & 0xFF));		//�������݃A�h���X�w��(LSB)

		dSPI_transfer(data[i]);		//�������݃f�[�^[byte]

		dSPI_deselect();
		address++;
	}
}

void FRAM::ReadData(unsigned char *data, unsigned int address, unsigned int size)
{
	int i;
	for (i = 0; i < size; i++)
	{
		dSPI_select();
		dSPI_transfer(FRAM_READ_ID);
		dSPI_transfer((address >> 16) & 0xFF);	//�������݃A�h���X�w��(MSB)
		dSPI_transfer((address >> 8) & 0xFF);	//�ǂݍ��݃A�h���X�w��(MSB)
		dSPI_transfer((address & 0xFF));		//�ǂݍ��݃A�h���X�w��(LSB)

		data[i] = dSPI_transfer(0x00);		//�ǂݍ��݃f�[�^[byte]
		dSPI_deselect();
		address++;
	}
}

