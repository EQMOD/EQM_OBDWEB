#ifdef __cplusplus
extern "C" {
#endif

#include <stdio.h>
#include <string.h>
#include <windows.h>
#include <stdlib.h>

int RS232_OpenComport(int, int);
int RS232_PollComport(int, unsigned char *, int);
int RS232_SendByte(int, unsigned char);
int RS232_SendBuf(int, unsigned char *, int);
void RS232_CloseComport(int);
void RS232_cputs(int, const char *);
int RS232_IsCTSEnabled(int);
int RS232_IsDSREnabled(int);
void RS232_enableDTR(int);
void RS232_disableDTR(int);
void RS232_enableRTS(int);
void RS232_disableRTS(int);

#define SBEGIN  0x01
#define SDATA   0x02
#define SRSP    0x03
#define SEND    0x04
#define ERRO   0x05

FILE *pfile = NULL;
long fsize = 0;
int BlkTot = 0;
int Remain = 0;
int BlkNum = 0;
int DownloadProgress = 0;
int com;
int end = 0;

HANDLE Cport[16];
char comports[16][10]={"\\\\.\\COM1",  "\\\\.\\COM2",  "\\\\.\\COM3",  "\\\\.\\COM4",
                       "\\\\.\\COM5",  "\\\\.\\COM6",  "\\\\.\\COM7",  "\\\\.\\COM8",
                       "\\\\.\\COM9",  "\\\\.\\COM10", "\\\\.\\COM11", "\\\\.\\COM12",
                       "\\\\.\\COM13", "\\\\.\\COM14", "\\\\.\\COM15", "\\\\.\\COM16"};
char baudr[64];

void ProcessProgram(void);

/*
* argv[0]----.exe file name
* argv[1]----ComPort number
* argv[2]----file path
* argv[3]----Device
*/
int main(int arg, char *argv[])
{	
	int fLen = 0;
	int device = 0;

	printf("Copyright (c) 2013 RedBearLab.com\n");
	printf("%s version 0.5\n", argv[0]);
	if(arg < 4)
	{
		printf("Invalid parameters.\n");
		printf("Usage: %s <com number> <bin file> <device>\n", argv[0]);
		printf("Example: %s 2 abc.bin 0\n", argv[0]);
		printf(" <device>: 0 -- Default (e.g. UNO)\n");
		printf("           1 -- Leonardo\n");
		return 0;
	}

	char form[5] = ".bin";
	char format[5] = "    ";
	fLen = strlen(argv[2]);
	if(fLen < 5) 
	{
		printf("The .bin file name is invalid!\n\n");
		return 0;  // file path is not valid
	}
	format[3] = argv[2][fLen-1];
	format[2] = argv[2][fLen-2];
	format[1] = argv[2][fLen-3];
	format[0] = argv[2][fLen-4];	
	if(0 != strcmp(form, format))
	{
		printf("File format must be .bin\n\n");
		return 0;
	}

	com = atoi(argv[1]);
	device = atoi(argv[3]);
	printf("Comport : COM%d\n", com);
	printf("Bin file: %s\n", argv[2]);
	if(device == 0)
	{
		printf("Device  : Default (e.g. UNO)\n\n");		
	}
	else
	{
		printf("Device: Leonardo\n\n");
	}

	com = atoi(argv[1]) - 1;
	if(1 == RS232_OpenComport(com, 115200))
	{
		return 0;	// Open comprt error
	}
	printf("Comport open!\n");
	if(device == 0)
	{
		RS232_disableDTR(com);
		printf("<Baud:115200> <data:8> <parity:none> <stopbit:1> <DTR:off> <RTS:off>\n\n");
	}
	else
	{
		RS232_enableDTR(com);
		printf("<Baud:115200> <data:8> <parity:none> <stopbit:1> <DTR:on> <RTS:off>\n\n");
	}
	RS232_disableRTS(com);
	
	pfile = fopen(argv[2], "rb");      // read only
	if(NULL == pfile)
    {
		printf("The file doesn't exist or is occupied!\n");
		RS232_CloseComport(com);
		printf("Comport closed!\n\n");
        return 0;
    }
	printf("File open!\n");
	fseek(pfile,0,SEEK_SET);
    fseek(pfile,0,SEEK_END);
    fsize = ftell(pfile);
    fseek(pfile,0,SEEK_SET);   
	Remain = fsize % 512;	
	if(Remain != 0)
	{
		BlkTot = fsize / 512 + 1;
		printf("!!WARNING: File's size isn't the integer multiples of 512 bytes, and \n");
		printf("           the last block will be filled in up to 512 bytes with 0xFF! \n");
	}
	else
	{
		BlkTot = fsize / 512;
	}
	printf("Block total: %d\n\n", BlkTot);
    BlkNum = 0;

	printf("Enable transmission...\n");
	unsigned char buf[2] = {SBEGIN, 0};      // Enable transmission,  do not verify
	if(RS232_SendBuf(com, buf, 2) != 2)
	{
		printf("Enable failed!\n");
		fclose(pfile);
		printf("File closed!\n");
		RS232_CloseComport(com);
		printf("Comport closed!\n\n");
		return 0;
	}
	else
	{
		printf("Request sent already!\n");
	}
	
	printf("/********************************************************************/\n");
	printf("* If there is no respond last for 3s, please press \"Ctrl+C\" to exit!\n");
	printf("* And pay attention to :\n*   1. The connection between computer and Arduino;\n");
	printf("*   2. The connection between Arduino and CC2540;\n");
	printf("*   3. Whether the device you using is Leonardo or not;\n");
	printf("*   4. Other unexpected errors.\n");
	printf("/********************************************************************/\n\n");
	printf("Waiting for respond from Arduino...\n\n");
	while(!end)
	{
		ProcessProgram();
	}
	if(end == 2)
	{
		printf("Upload successfully!\n");
	}
	else
	{
		printf("Upload Failed!\n");
	}
	BlkNum = 0;
	DownloadProgress = 0;
    fclose(pfile);
	printf("File closed!\n");
	RS232_CloseComport(com);
	printf("Comport closed!\n\n");

	return 0;
}

void ProcessProgram()
{
    int len;
	unsigned char rx;
	len = RS232_PollComport(com, &rx, 1);
    if(len > 0)
    {
        switch(rx)
        {
            case SRSP:
            {
                if(BlkNum == BlkTot)
                {
                    unsigned char temp = SEND;
                   	RS232_SendByte(com, temp);
					end = 2;
                }
                else
                {
					if(BlkNum == 0)
					{
						printf("Uploading firmware...\n\n");
					}
					DownloadProgress = 1;
                    unsigned char buf[515];
                    buf[0] = SDATA;
					if((BlkNum == (BlkTot-1)) && (Remain != 0))
					{
						fread(buf+1, Remain, 1, pfile);
						int filled = 512 - Remain;
						int i = 0;
						for(i; i<filled; i++)
						{
							buf[Remain+1+i] = 0xFF;
						}
					}
					else
					{
                    	fread(buf+1, 512, 1, pfile);
					}

                    unsigned short CheckSum = 0x0000;
                    for(unsigned int i=0; i<512; i++)
                    {
                        CheckSum += (unsigned char)buf[i+1];
                    }
                    buf[513] = (CheckSum >> 8) & 0x00FF;
                    buf[514] = CheckSum & 0x00FF;
					RS232_SendBuf(com, buf, 515);
                    BlkNum++;
					printf("%d  ", BlkNum);
                }
                break;
            }

            case ERRO:
            {
                if(DownloadProgress == 1)
                {
                    end = 1;
                    printf("Verify failed!\n");
                }
                else
                {
                    end = 1;
                    printf("No chip detected!\n");
                }
                break;
            }

            default:
                break;
        }
		len = 0;
    }
}

int RS232_OpenComport(int comport_number, int baudrate)
{
  if((comport_number>15)||(comport_number<0))
  {
    printf("illegal comport number\n\n");
    return(1);
  }

  switch(baudrate)
  {
    case     110 : strcpy(baudr, "baud=110 data=8 parity=N stop=1 dtr=on rts=on");
                   break;
    case     300 : strcpy(baudr, "baud=300 data=8 parity=N stop=1 dtr=on rts=on");
                   break;
    case     600 : strcpy(baudr, "baud=600 data=8 parity=N stop=1 dtr=on rts=on");
                   break;
    case    1200 : strcpy(baudr, "baud=1200 data=8 parity=N stop=1 dtr=on rts=on");
                   break;
    case    2400 : strcpy(baudr, "baud=2400 data=8 parity=N stop=1 dtr=on rts=on");
                   break;
    case    4800 : strcpy(baudr, "baud=4800 data=8 parity=N stop=1 dtr=on rts=on");
                   break;
    case    9600 : strcpy(baudr, "baud=9600 data=8 parity=N stop=1 dtr=off rts=off");
                   break;
    case   19200 : strcpy(baudr, "baud=19200 data=8 parity=N stop=1 dtr=on rts=on");
                   break;
    case   38400 : strcpy(baudr, "baud=38400 data=8 parity=N stop=1 dtr=on rts=on");
                   break;
    case   57600 : strcpy(baudr, "baud=57600 data=8 parity=N stop=1 dtr=on rts=on");
                   break;
    case  115200 : strcpy(baudr, "baud=115200 data=8 parity=N stop=1 dtr=off rts=off");
                   break;
    case  128000 : strcpy(baudr, "baud=128000 data=8 parity=N stop=1 dtr=on rts=on");
                   break;
    case  256000 : strcpy(baudr, "baud=256000 data=8 parity=N stop=1 dtr=on rts=on");
                   break;
    case  500000 : strcpy(baudr, "baud=500000 data=8 parity=N stop=1 dtr=on rts=on");
                   break;
    case 1000000 : strcpy(baudr, "baud=1000000 data=8 parity=N stop=1 dtr=on rts=on");
                   break;
    default      : printf("invalid baudrate\n");
                   return(1);
                   break;
  }

  Cport[comport_number] = CreateFileA(comports[comport_number],
                      GENERIC_READ|GENERIC_WRITE,
                      0,                          /* no share  */
                      NULL,                       /* no security */
                      OPEN_EXISTING,
                      0,                          /* no threads */
                      NULL);                      /* no templates */

  if(Cport[comport_number]==INVALID_HANDLE_VALUE)
  {
    printf("unable to open comport!\n\n");
    return(1);
  }

  DCB port_settings;
  memset(&port_settings, 0, sizeof(port_settings));  /* clear the new struct  */
  port_settings.DCBlength = sizeof(port_settings);

  if(!BuildCommDCBA(baudr, &port_settings))
  {
    printf("unable to set comport dcb settings\n\n");
    CloseHandle(Cport[comport_number]);
    return(1);
  }

  if(!SetCommState(Cport[comport_number], &port_settings))
  {
    printf("unable to set comport cfg settings\n\n");
    CloseHandle(Cport[comport_number]);
    return(1);
  }

  COMMTIMEOUTS Cptimeouts;

  Cptimeouts.ReadIntervalTimeout         = MAXDWORD;
  Cptimeouts.ReadTotalTimeoutMultiplier  = 0;
  Cptimeouts.ReadTotalTimeoutConstant    = 0;
  Cptimeouts.WriteTotalTimeoutMultiplier = 0;
  Cptimeouts.WriteTotalTimeoutConstant   = 0;

  if(!SetCommTimeouts(Cport[comport_number], &Cptimeouts))
  {
    printf("unable to set comport time-out settings\n\n");
    CloseHandle(Cport[comport_number]);
    return(1);
  }

  return(0);
}


int RS232_PollComport(int comport_number, unsigned char *buf, int size)
{
  int n;

  if(size>4096)  size = 4096;
/* added the void pointer cast, otherwise gcc will complain about */
/* "warning: dereferencing type-punned pointer will break strict aliasing rules" */

  ReadFile(Cport[comport_number], buf, size, (LPDWORD)((void *)&n), NULL);

  return(n);
}

int RS232_SendByte(int comport_number, unsigned char byte)
{
  int n;

  WriteFile(Cport[comport_number], &byte, 1, (LPDWORD)((void *)&n), NULL);

  if(n<0)  return(1);

  return(0);
}

int RS232_SendBuf(int comport_number, unsigned char *buf, int size)
{
  int n;

  if(WriteFile(Cport[comport_number], buf, size, (LPDWORD)((void *)&n), NULL))
  {
    return(n);
  }

  return(-1);
}

void RS232_CloseComport(int comport_number)
{
  CloseHandle(Cport[comport_number]);
}

int RS232_IsCTSEnabled(int comport_number)
{
  int status;

  GetCommModemStatus(Cport[comport_number], (LPDWORD)((void *)&status));

  if(status&MS_CTS_ON) return(1);
  else return(0);
}

int RS232_IsDSREnabled(int comport_number)
{
  int status;

  GetCommModemStatus(Cport[comport_number], (LPDWORD)((void *)&status));

  if(status&MS_DSR_ON) return(1);
  else return(0);
}

void RS232_enableDTR(int comport_number)
{
  EscapeCommFunction(Cport[comport_number], SETDTR);
}

void RS232_disableDTR(int comport_number)
{
  EscapeCommFunction(Cport[comport_number], CLRDTR);
}

void RS232_enableRTS(int comport_number)
{
  EscapeCommFunction(Cport[comport_number], SETRTS);
}

void RS232_disableRTS(int comport_number)
{
  EscapeCommFunction(Cport[comport_number], CLRRTS);
}

void RS232_cputs(int comport_number, const char *text)  /* sends a string to serial port */
{
  while(*text != 0)   RS232_SendByte(comport_number, *(text++));
}

#ifdef __cplusplus
} /* extern "C" */
#endif

