// File: ceSerial.h
// Description: ceSerial communication class for Windows and Linux
// WebSite: http://cool-emerald.blogspot.sg/2017/05/serial-port-programming-in-c-with.html
// MIT License (https://opensource.org/licenses/MIT)
// Copyright (c) 2018 Yan Naing Aye

// References
// https://en.wikibooks.org/wiki/Serial_Programming/termios
// http://www.silabs.com/documents/public/application-notes/an197.pdf
// https://msdn.microsoft.com/en-us/library/ff802693.aspx
// http://www.cplusplus.com/forum/unices/10491/

/* THE SOFTWARE IS PROVIDED “AS IS”, WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, 
INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR 
PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE 
FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, 
ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS 
IN THE SOFTWARE. */

// This is the ceSerial library by YAN9A, imported into OScar
// The project page is available here: https://github.com/yan9a/serial

#ifndef CESERIAL_H
#define CESERIAL_H
#include <string>
#include <cstdlib>
#include <cstdio>
#include <cstring>

#if defined(_WIN64) || defined(__WIN32__) || defined(_WIN32) || defined(WIN32) || defined(__WINDOWS__) || defined(__TOS_WIN__) || defined(__CYGWIN__)
    #define CE_WINDOWS 
#elif defined(unix) || defined(__unix) || defined(__unix__)
    #define CE_LINUX
#endif

#ifdef CE_WINDOWS
	#include <windows.h>
#endif

class ceSerial {
private:
	char rxchar;
	char rxdata[92];
	std::string port;
	long baud;
	long dsize;
	char parity;
	float stopbits;
	bool stdbaud;
#ifdef CE_WINDOWS
    HANDLE hComm; //handle
	OVERLAPPED osReader;
	OVERLAPPED osWrite;
	BOOL fWaitingOnRead;
	COMMTIMEOUTS timeouts_ori;
#else
	long fd;//serial_fd
#endif
public:
	static void Delay(unsigned long ms);
	ceSerial();
	ceSerial(std::string Device, long BaudRate, long DataSize, char ParityType, float NStopBits);
	~ceSerial();
	long Open(void);//return 0 if success
	void Close();
	char ReadChar(bool& success);//return read char if success
	char *ReadData(bool& success);
	bool WriteChar(char ch);////return success flag
	bool Write(char *data);//write null terminated string and return success flag
	bool Write(char *data,long n);
	bool SetRTS(bool value);//return success flag
	bool SetDTR(bool value);//return success flag
	bool GetCTS(bool& success);
	bool GetDSR(bool& success);
	bool GetRI(bool& success);
	bool GetCD(bool& success);
	bool IsOpened();
	void SetPortName(std::string Port); 
	std::string GetPort();
	void SetBaudRate(long baudrate);
	long GetBaudRate();
	void SetDataSize(long nbits);
	long GetDataSize();
	void SetParity(char p);
	char GetParity();
	void SetStopBits(float nbits);
	float GetStopBits();
};

#endif // CESERIAL_H
