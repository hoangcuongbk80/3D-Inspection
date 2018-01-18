#include "rotaTable.h"


rotaTable::rotaTable()
{
}

rotaTable::~rotaTable()
{
}

void rotaTable::Open()
{
	if (fnPerformaxComOpen(0, &m_hUSBDevice))
	{
		status = fnPerformaxComGetNumDevices(&lpNumDevices);
		if (status == 1)
		{
			QMessageBox msgBox;
			msgBox.setText("Connected");
			msgBox.exec();
		}
		else
		{
			QMessageBox msgBox;
			msgBox.setText("No Performax Device is connected to USB!");
			msgBox.exec();
		}
		fnPerformaxComGetProductString(0, ProductString, PERFORMAX_RETURN_SERIAL_NUMBER);
	}
	else
	{
		QMessageBox msgBox;
		msgBox.setText("Rotate Table is not available");
		msgBox.exec();
	}

}

void rotaTable::Move(char str[64])
{
	strcpy(CmdBuffer, str);

	if (fnPerformaxComSendRecv(m_hUSBDevice, CmdBuffer, 64, 64, ResponseBuffer))
	{
		for (int i = 0; i < 64; i++)
		{
			if (ResponseBuffer[i] == 0x00) // EOT - End of Transmission
				break;
		}
	}
	else
	{
		QMessageBox msgBox;
		msgBox.setText("Rotate Table cannot move");
		msgBox.exec();
	}

}
