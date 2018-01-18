#ifndef ROTATABLE_H
#define ROTATABLE_H

#include <QLibrary>
#include <QMessageBox>
#include <windows.h>
#include <cstring>

#define PERFORMAX_RETURN_SERIAL_NUMBER		0
#define PERFORMAX_RETURN_DESCRIPTION		1
#define COMND_BUF_LEN						255
#define REPLY_BUF_LEN						6000


#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000

extern "C" __declspec(dllimport) BOOL _stdcall fnPerformaxComWrite(IN HANDLE pHandle, LPVOID Buffer, DWORD dwNumBytesToWrite, OUT LPDWORD lpNumBytesWritten);
extern "C" __declspec(dllimport) BOOL _stdcall fnPerformaxComRead(IN HANDLE pHandle, LPVOID Buffer, DWORD dwNumBytesToRead, OUT LPDWORD lpNumBytesReturn);
extern "C" __declspec(dllimport) BOOL _stdcall fnPerformaxComClose(IN HANDLE pHandle);
extern "C" __declspec(dllimport) BOOL _stdcall fnPerformaxComSetTimeouts(IN DWORD dwReadTimeout, DWORD dwWriteTimeout);


extern "C" __declspec(dllimport) BOOL _stdcall fnPerformaxComGetNumDevices(LPDWORD lpNumDevices);
extern "C" __declspec(dllimport) BOOL _stdcall fnPerformaxComGetProductString(IN DWORD dwNumDevices, OUT LPVOID lpDeviceString, IN DWORD dwOptions);
extern "C" __declspec(dllimport) BOOL _stdcall fnPerformaxComOpen(IN DWORD dwDeviceNum, OUT HANDLE* pHandle);
extern "C" __declspec(dllimport) BOOL _stdcall fnPerformaxComSendRecv(IN HANDLE pHandle,
	IN LPVOID wBuffer,
	IN DWORD dwNumBytesToWrite,
	IN DWORD dwNumBytesToRead,
	OUT LPVOID rBuffer);

extern "C" __declspec(dllimport) BOOL _stdcall fnPerformaxComFlush(IN HANDLE pHandle);


class rotaTable
{
public:
	rotaTable();
	~rotaTable();

	DWORD lpNumDevices;
	char  ProductString[128];
	int   status;
	char  CmdBuffer[64], ResponseBuffer[64];
	HANDLE	m_hUSBDevice;
	void Open();
	void Move(char[64]);
};

#endif 