#include "SerialClass.h"

Serial::Serial(char *portName)
{


#if defined (_WIN32) || defined( _WIN64)

        //We're not yet connected
    this->connected = false;

    //Try to connect to the given port throuh CreateFile
    this->hSerial = CreateFile(portName,
            GENERIC_READ | GENERIC_WRITE,
            0,
            NULL,
            OPEN_EXISTING,
            FILE_ATTRIBUTE_NORMAL,
            NULL);

    //Check if the connection was successfull
    if(this->hSerial==INVALID_HANDLE_VALUE)
    {
        //If not success full display an Error
        if(GetLastError()==ERROR_FILE_NOT_FOUND){

            //Print Error if neccessary
            printf("ERROR: Handle was not attached. Reason: %s not available.\n", portName);

        }
        else
        {
            printf("ERROR!!!");
        }
    }
    else
    {
        //If connected we try to set the comm parameters
        DCB dcbSerialParams = {0};

        //Try to get the current
        if (!GetCommState(this->hSerial, &dcbSerialParams))
        {
            //If impossible, show an error
            printf("failed to get current serial parameters!");
        }
        else
        {
            //Define serial connection parameters for the arduino board
            dcbSerialParams.BaudRate=CBR_9600;
            dcbSerialParams.ByteSize=8;
            dcbSerialParams.StopBits=ONESTOPBIT;
            dcbSerialParams.Parity=NOPARITY;

             //Set the parameters and check for their proper application
             if(!SetCommState(hSerial, &dcbSerialParams))
             {
                printf("ALERT: Could not set Serial Port parameters");
             }
             else
             {
                 //If everything went fine we're connected
                 this->connected = true;
                 //We wait 2s as the arduino board will be reseting
                 Sleep(ARDUINO_WAIT_TIME);
             }
        }
    }
#endif
#ifdef __linux__
    struct termios options;                                             // Structure with the device's options

    // Open device
    fd = open(portName, O_RDWR | O_NOCTTY | O_NDELAY);                    // Open port
    //if (fd == -1) return -2;                                            // If the device is not open, return -1
    fcntl(fd, F_SETFL, FNDELAY);                                        // Open the device in nonblocking mode

    // Set parameters
    tcgetattr(fd, &options);                                            // Get the current options of the port
    bzero(&options, sizeof(options));                                   // Clear all the options
    speed_t         Speed;
    Speed = B9600;
    cfsetispeed(&options, Speed);                                       // Set the baud rate at 115200 bauds
    cfsetospeed(&options, Speed);
    options.c_cflag |= ( CLOCAL | CREAD |  CS8);                        // Configure the device : 8 bits, no parity, no control
    options.c_iflag |= ( IGNPAR | IGNBRK );
    options.c_cc[VTIME]=0;                                              // Timer unused
    options.c_cc[VMIN]=0;                                               // At least on character before satisfy reading
    tcsetattr(fd, TCSANOW, &options);                                   // Activate the settings     
#endif
}

Serial::~Serial()
{

#if defined (_WIN32) || defined( _WIN64)
	//Check if we are connected before trying to disconnect
	if(this->connected)
    {
        //We're no longer connected
        this->connected = false;
        //Close the serial handler
        CloseHandle(this->hSerial);
    }
#endif


#ifdef __linux__
	//Linux closes without checking itself, but be warned, it might begin shreking itself
	close (fd);
#endif

}

int Serial::ReadData(char *buffer, unsigned int nbChar)
{
#if defined (_WIN32) || defined(_WIN64)
    //Number of bytes we'll have read
    DWORD bytesRead;
    //Number of bytes we'll really ask to read
    unsigned int toRead;

    //Use the ClearCommError function to get status info on the Serial port
    ClearCommError(this->hSerial, &this->errors, &this->status);

    //Check if there is something to read
    if(this->status.cbInQue>0)
    {
        //If there is we check if there is enough data to read the required number
        //of characters, if not we'll read only the available characters to prevent
        //locking of the application.
        if(this->status.cbInQue>nbChar)
        {
            toRead = nbChar;
        }
        else
        {
            toRead = this->status.cbInQue;
        }

        //Try to read the require number of chars, and return the number of read bytes on success
        if(ReadFile(this->hSerial, buffer, toRead, &bytesRead, NULL) && bytesRead != 0)
        {
            return bytesRead;
        }

    }

    //If nothing has been read, or that an error was detected return -1
    return -1;
#endif
#ifdef __linux__
    unsigned int     NbByteRead=0;
    unsigned char* Ptr=(unsigned char*)buffer+NbByteRead;           // Compute the position of the current byte
    int Ret=read(fd,(void*)Ptr,nbChar-NbByteRead);              // Try to read a byte on the device
    if (Ret==-1) return -2;                                         // Error while reading
    if (Ret>0) {                                                    // One or several byte(s) has been read on the device
        NbByteRead+=Ret;                                            // Increase the number of read bytes
        if (NbByteRead>=nbChar)                                 // Success : bytes has been read
            return 1;
    }
	return 0;
#endif
}


bool Serial::WriteData(char *buffer, unsigned int nbChar)
{
#if defined (_WIN32) || defined( _WIN64)
    DWORD bytesSend;

    //Try to write the buffer on the Serial port
    if(!WriteFile(this->hSerial, (void *)buffer, nbChar, &bytesSend, 0))
    {
        //In case it don't work get comm error and return false
        ClearCommError(this->hSerial, &this->errors, &this->status);

        return false;
    }
    else
        return true;
#endif
#ifdef __linux__
    if (write (fd,buffer,nbChar)!=(ssize_t)nbChar)                              // Write data
        return -1;                                                      // Error while writing
    return 1;                                                           // Write operation successfull
#endif
}

bool Serial::IsConnected()
{
    //Simply return the connection status
    return true;//this->connected;
}

int Serial::BytesAvailable()
{
#if defined (_WIN32) || defined( _WIN64)
    //Use the ClearCommError function to get status info on the Serial port
    ClearCommError(this->hSerial, &this->errors, &this->status);
    return this->status.cbInQue;
#endif
#ifdef __linux__
    int nBytes = 0;
    ioctl(fd, FIONREAD, &nBytes);
	return nBytes;
#endif
}