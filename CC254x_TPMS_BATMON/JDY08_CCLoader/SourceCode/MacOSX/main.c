#ifdef __cplusplus
extern "C" {
#endif

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <termios.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <limits.h>

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
int BlkNum = 0;
int DownloadProgress = 0;
int end = 0;

int Cport[30], error;

struct termios new_port_settings, old_port_settings[30];

char comports[1][26]={"/dev/tty.usbmodem1431"};

void ProcessProgram(void);

/*
* argv[0]----.exe file name
* argv[1]----ComPort number
* argv[2]----file path
*/
int main(int arg, char *argv[])
{	
	int fLen = 0;
	if(arg < 3)
	{
		printf("Input parameters error, it should be:\n");
		printf("for example:CCLoader.exe [/dev/]ttyS0 abc.bin\n");
		return 0;
	}
    
    strcpy(comports[0], argv[1]);

	if(1 == RS232_OpenComport(0, 115200))
	{
		printf("Open Comport failed!\n");
        printf("Please confirm comport:\"/dev/*\"\n");
		return 0;	// Open comprt error
	}
	RS232_disableDTR(0);
	RS232_disableRTS(0);
	printf("Baud:115200 data:8 parity:none stopbit:1 DTR:off RTS:off\n");

	char form[5] = ".bin";
	char format[5] = "    ";
	fLen = strlen(argv[2]);
	if(fLen < 5) 
	{
		printf("File path is invalid!\n");
		return 0;  // file path is not valid
	}
	format[3] = argv[2][fLen-1];
	format[2] = argv[2][fLen-2];
	format[1] = argv[2][fLen-3];
	format[0] = argv[2][fLen-4];	
	if(0 != strcmp(form, format))
	{
		printf("File format must be .bin");
		return 0;
	}
	pfile = fopen(argv[2], "rb");      // read only
	if(NULL == pfile)
    {
		printf("file doesn't exist or is occupied!\n");
        return 0;
    }
	printf("File open success!\n");
	fseek(pfile,0,SEEK_SET);
    fseek(pfile,0,SEEK_END);
    fsize = ftell(pfile);
    fseek(pfile,0,SEEK_SET);
    BlkTot = fsize / 512;
	if((fsize%512) != 0)
	{
		printf("Warning: file size isn't the integer multiples of 512, last bytes will miss to be sent!\n");
	}
	printf("Block total: %d\n", BlkTot);
    BlkNum = 0;
	
	printf("Waiting for Arduino setup...\n");
	unsigned char j = 0;
	for(j=5; j>0; j--)
	{
	  printf("Remain: %d\n", j);
	  sleep(1);
	}

	printf("Enable transmission...\n");
	unsigned char buf[2] = {SBEGIN, 0};      // Enable transmission,  do not verify
	if(RS232_SendBuf(0, buf, 2) != 2)
	{
		printf("Enable failed!\n");
		fclose(pfile);
		printf("File closed!\n");
		RS232_CloseComport(0);
		printf("Comport closed!\n");
		return 0;
	}
	else
	{
		printf("Request sent already!Waiting for respond...\n");
	}
	
	while(!end)
	{
		ProcessProgram();
	}
	printf("Program successfully!\n");
	BlkNum = 0;
	DownloadProgress = 0;
    fclose(pfile);
	printf("File closed!\n");
	RS232_CloseComport(0);
	printf("Comport closed!\n");
	sleep(2);
	printf("Exit: 0\n");

	return 0;
}

void ProcessProgram()
{
    int len;
	unsigned char rx;
	len = RS232_PollComport(0, &rx, 1);
    if(len > 0)
    {
        switch(rx)
        {
            case SRSP:
            {
                if(BlkNum == BlkTot)
                {
                    unsigned char temp = SEND;
                   	RS232_SendByte(0, temp);
					end = 1;
                }
                else
                {
					if(BlkNum == 0)
					{	
						printf("Begin programming...\n");
					}
					DownloadProgress = 1;
                    unsigned char buf[515];
                    buf[0] = SDATA;
                    fread(buf+1, 512, 1, pfile);

                    unsigned short CheckSum = 0x0000;
					unsigned int i;
                    for(i=0; i<512; i++)
                    {
                        CheckSum += (unsigned char)buf[i+1];
                    }
                    buf[513] = (CheckSum >> 8) & 0x00FF;
                    buf[514] = CheckSum & 0x00FF;
                    
					RS232_SendBuf(0, buf, 515);
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
  int baudr, status;

  if((comport_number>29)||(comport_number<0))
  {
    printf("illegal comport number\n");
    return(1);
  }

  switch(baudrate)
  {
    case      50 : baudr = B50;
                   break;
    case      75 : baudr = B75;
                   break;
    case     110 : baudr = B110;
                   break;
    case     134 : baudr = B134;
                   break;
    case     150 : baudr = B150;
                   break;
    case     200 : baudr = B200;
                   break;
    case     300 : baudr = B300;
                   break;
    case     600 : baudr = B600;
                   break;
    case    1200 : baudr = B1200;
                   break;
    case    1800 : baudr = B1800;
                   break;
    case    2400 : baudr = B2400;
                   break;
    case    4800 : baudr = B4800;
                   break;
    case    9600 : baudr = B9600;
                   break;
    case   19200 : baudr = B19200;
                   break;
    case   38400 : baudr = B38400;
                   break;
    case   57600 : baudr = B57600;
                   break;
    case  115200 : baudr = B115200;
                   break;
    case  230400 : baudr = B230400;
                   break;
    default      : printf("invalid baudrate\n");
                   return(1);
                   break;
  }
  
	printf("Opening comport...\n");
	Cport[comport_number] = open(comports[comport_number], O_RDWR | O_NOCTTY | O_NDELAY);
	if(Cport[comport_number]==-1)
	{
		perror("unable to open comport ");
		return(1);
	}
	printf("Comport open!\n");

  printf("Read portsettings...\n");
  error = tcgetattr(Cport[comport_number], old_port_settings + comport_number);
  if(error==-1)
  {
    close(Cport[comport_number]);
    perror("unable to read portsettings ");
    return(1);
  }
  memset(&new_port_settings, 0, sizeof(new_port_settings));  /* clear the new struct */

  cfsetispeed(&new_port_settings, baudr);
  cfsetospeed(&new_port_settings, baudr);
  new_port_settings.c_cflag |= (CS8 | CLOCAL | CREAD);
  new_port_settings.c_iflag |= IGNPAR;
  new_port_settings.c_oflag = 0;
  new_port_settings.c_lflag = 0;
  new_port_settings.c_cc[VMIN] = 0;      /* block untill n bytes are received */
  new_port_settings.c_cc[VTIME] = 0;     /* block untill a timer expires (n * 100 mSec.) */
  printf("Changing portsettings...\n");
  error = tcsetattr(Cport[comport_number], TCSANOW, &new_port_settings);
  if(error==-1)
  {
    close(Cport[comport_number]);
    perror("unable to adjust portsettings ");
    return(1);
  }

  if(ioctl(Cport[comport_number], TIOCMGET, &status) == -1)
  {
    perror("unable to get portstatus");
    return(1);
  }

  status |= TIOCM_DTR;    /* turn on DTR */
  status |= TIOCM_RTS;    /* turn on RTS */

  if(ioctl(Cport[comport_number], TIOCMSET, &status) == -1)
  {
    perror("unable to set portstatus");
    return(1);
  }

  return(0);
}


int RS232_PollComport(int comport_number, unsigned char *buf, int size)
{
  int n;

#ifndef __STRICT_ANSI__                       /* __STRICT_ANSI__ is defined when the -ansi option is used for gcc */
  if(size>SSIZE_MAX)  size = (int)SSIZE_MAX;  /* SSIZE_MAX is defined in limits.h */
#else
  if(size>4096)  size = 4096;
#endif

  n = read(Cport[comport_number], buf, size);

  return(n);
}


int RS232_SendByte(int comport_number, unsigned char byte)
{
  int n;

  n = write(Cport[comport_number], &byte, 1);
  if(n<0)  return(1);

  return(0);
}


int RS232_SendBuf(int comport_number, unsigned char *buf, int size)
{
  return(write(Cport[comport_number], buf, size));
}


void RS232_CloseComport(int comport_number)
{
  int status;

  if(ioctl(Cport[comport_number], TIOCMGET, &status) == -1)
  {
    perror("unable to get portstatus");
  }

  status &= ~TIOCM_DTR;    
  status &= ~TIOCM_RTS;    

  if(ioctl(Cport[comport_number], TIOCMSET, &status) == -1)
  {
    perror("unable to set portstatus");
  }

  tcsetattr(Cport[comport_number], TCSANOW, old_port_settings + comport_number);
  close(Cport[comport_number]);
}

/*
Constant  Description
TIOCM_LE  DSR (data set ready/line enable)
TIOCM_DTR DTR (data terminal ready)
TIOCM_RTS RTS (request to send)
TIOCM_ST  Secondary TXD (transmit)
TIOCM_SR  Secondary RXD (receive)
TIOCM_CTS CTS (clear to send)
TIOCM_CAR DCD (data carrier detect)
TIOCM_CD  Synonym for TIOCM_CAR
TIOCM_RNG RNG (ring)
TIOCM_RI  Synonym for TIOCM_RNG
TIOCM_DSR DSR (data set ready)
*/

int RS232_IsCTSEnabled(int comport_number)
{
  int status;

  ioctl(Cport[comport_number], TIOCMGET, &status);

  if(status&TIOCM_CTS) return(1);
  else return(0);
}

int RS232_IsDSREnabled(int comport_number)
{
  int status;

  ioctl(Cport[comport_number], TIOCMGET, &status);

  if(status&TIOCM_DSR) return(1);
  else return(0);
}

void RS232_enableDTR(int comport_number)
{
  int status;

  if(ioctl(Cport[comport_number], TIOCMGET, &status) == -1)
  {
    perror("unable to get portstatus");
  }

  status |= TIOCM_DTR;    /* turn on DTR */

  if(ioctl(Cport[comport_number], TIOCMSET, &status) == -1)
  {
    perror("unable to set portstatus");
  }
}

void RS232_disableDTR(int comport_number)
{
  int status;

  if(ioctl(Cport[comport_number], TIOCMGET, &status) == -1)
  {
    perror("unable to get portstatus");
  }

  status &= ~TIOCM_DTR;    /* turn off DTR */

  if(ioctl(Cport[comport_number], TIOCMSET, &status) == -1)
  {
    perror("unable to set portstatus");
  }
}

void RS232_enableRTS(int comport_number)
{
  int status;

  if(ioctl(Cport[comport_number], TIOCMGET, &status) == -1)
  {
    perror("unable to get portstatus");
  }

  status |= TIOCM_RTS;    /* turn on RTS */

  if(ioctl(Cport[comport_number], TIOCMSET, &status) == -1)
  {
    perror("unable to set portstatus");
  }
}

void RS232_disableRTS(int comport_number)
{
  int status;

  if(ioctl(Cport[comport_number], TIOCMGET, &status) == -1)
  {
    perror("unable to get portstatus");
  }

  status &= ~TIOCM_RTS;    /* turn off RTS */

  if(ioctl(Cport[comport_number], TIOCMSET, &status) == -1)
  {
    perror("unable to set portstatus");
  }
}

#ifdef __cplusplus
} /* extern "C" */
#endif

