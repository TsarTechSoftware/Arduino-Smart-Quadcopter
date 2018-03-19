#include <iostream>
#include <cstring>
#include <afx.h>

using namespace std;

bool WriteComPort(CString PortSpecifier, CString data);

int main(){

	const int MAX_INDEX = 260;

	char data[MAX_INDEX] = { 0 };

	data[0] = 0x0A;

	cout << "Type something when you're ready..." << endl;

	cin >> data[1];

	WriteComPort("COM12", 'a');

	cout << "Uh I think data has sent..." << endl;

	cin >> data[1];

	return 0;
}

bool WriteComPort(CString PortSpecifier, CString data)
{
	DCB dcb;
	DWORD byteswritten;
	HANDLE hPort = CreateFile(
		PortSpecifier,
		GENERIC_WRITE,
		0,
		NULL,
		OPEN_EXISTING,
		0,
		NULL
		);
	if (!GetCommState(hPort, &dcb))
		return false;
	dcb.BaudRate = CBR_9600;          //9600 Baud 
	dcb.ByteSize = 8;                 //8 data bits 
	dcb.Parity = NOPARITY;            //no parity 
	dcb.StopBits = ONESTOPBIT;        //1 stop 
	if (!SetCommState(hPort, &dcb))
		return false;
	bool retVal = WriteFile(hPort, data, 1, &byteswritten, NULL);
	CloseHandle(hPort);   //close the handle 
	return retVal;
}