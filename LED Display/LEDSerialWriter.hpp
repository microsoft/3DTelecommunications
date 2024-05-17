/** LED Serial Writer helper library **
 *  *  This file provides the helper functions (Linux) for writing to an attached Trinket M0 LED display
 *   * This code does not keep any state, state should be stored in the class calling this, and it also
 *    * stored inside the Trinket
 *     * 
 *      * This code also assumes the serial port is always /dev/ttyACM0, as set in the const defined below
 *       **/

#include <unistd.h>
#include <mutex>
#include <fcntl.h>
#include <string>
#include <iostream>
#include <termios.h>
#include <cstring>

#define MAX_WRITES 10000
#define RESPONSE_TIMEOUT_MSEC 500

namespace Trinket
{
	    /* These are the exact strings sent to the trinket as part of a state command */
	    /* So they are created here as helpers for anyone calling the SetState function to pass in */
	    const std::string Idle = "Idle";
	    const std::string Starting_Calibration = "Starting Calibration";
	    const std::string Starting_Broadcast = "Starting Broadcast";
	    const std::string Starting_BG_Capture = "Starting BG Capture";
	    const std::string Calibration_Capturing_Video = "Calibration Capturing Video";
	    const std::string Calibration_Data_Transferring = "Calibration Data Transferring";
	    const std::string Calibration_Data_Transfer_Complete = "Calibration Data Transfer Complete";
	    const std::string Calibration_Running_Calib = "Calibration Running Calib";
	    const std::string Calibration_Successful = "Calibration Successful";
	    const std::string Calibration_Failed = "Calibration Failed";
	    const std::string Broadcast_Running = "Broadcast Running";
	    const std::string Capturing_BG_Images = "Capturing BG Images";
	    const std::string Pod_Rebooting = "Pod Rebooting";
	    const std::string Broadcast_Stopping = "Broadcast Stopping";

	    // if you haven't disabled the debug serial port, the command port will be /dev/ttyACM1
	    // if you disable the debug port by editing boot.py on the Trinket, it will show up as /dev/ttyACM0
	    const std::string defaultSerialPortFilename = "/dev/ttyACM1";
	    static bool Initialized = false;
	    static int SerialPort = -1;
	    static std::mutex SerialMutex;
	    static std::string SerialPortFilename;

	    void Initialize(std::string serialPortFilename)
	    {
		    // std::cout << "Opening serial port " << serialPortFilename << std::endl;
		    SerialPortFilename = serialPortFilename;

		    struct termios tio;
		    memset(&tio, 0, sizeof(tio));
		    tio.c_cflag=CS8|CREAD|CLOCAL;
		    tio.c_cc[VMIN]=1;
		    tio.c_cc[VTIME]=5;
		    	
		    SerialPort = open(serialPortFilename.c_str(),  O_RDWR | O_NONBLOCK);
		    cfsetospeed(&tio, B9600);
		    cfsetispeed(&tio, B9600);
		    tcsetattr(SerialPort, TCSANOW, &tio);

		    if(SerialPort >= 0)
		    {
			    Initialized = true;	
			    // Make sure the read buffer is empty	    
			    char buf[64];
			    int bytesRead = read(SerialPort, buf, 64);
				if(bytesRead > 0)
				{
					//std::cout << "Buffer was not empty on start." << std::endl;
				}
		    }
		    else
		    {
			    //std::cout << "Error opening serial.  IO returned: " << SerialPort << std::endl;
		    }
	    }
	    
	    void Close()
	    {
		    Initialized = false;
		    close(SerialPort);
	    }
	    
	    inline void WriteToSerial(std::string text, std::string serialPortFilename)
	    {
		    std::lock_guard<std::mutex> lock(SerialMutex);
		    if(!Initialized)
		    {
			    Trinket::Initialize(serialPortFilename);
		    }
		    if(serialPortFilename != SerialPortFilename)
		    {
			// New port
			Trinket::Close();
			Trinket::Initialize(serialPortFilename);
		    }
		    unsigned int bytesWritten = write(SerialPort, text.c_str(), text.length());
		    bytesWritten += write(SerialPort, "\n", 1);
			if(bytesWritten < text.length())
			{
				//std::cout << "Warning:  Only wrote " << bytesWritten << " bytes.  Was trying to write " << text.length() << " bytes" << std::endl;
			}

		    char buff[64];
		    memset(buff, '\0', 64);
		    int waitMSec = 0;
		    while(true)
		    {
			    int reads = read(SerialPort, buff, 64);
			    if(reads > 0)
			    {
	    			//std::cout << "Read " << reads << " bytes: [" << buff << "]" << std::endl;
				    if(std::string(buff) == text+"\n")
				    {
				    }
				    else
				    {
					//std::cout << "Received does not match. I sent: [" << text.c_str() << "\n]" << std::endl;
				    }
				    break;
			    }
			    else
			    {
				    sleep(0.001);
				    waitMSec += 1;
				    if(waitMSec > RESPONSE_TIMEOUT_MSEC)
				    {
					    //std::cout << "No response.  Closing serial." << std::endl;
					    Trinket::Close();
					    break;
				    }
			    }
		    }
	    }
	    
	    inline void SetState(std::string podState, std::string serialPortFilename = defaultSerialPortFilename)
	    {
		    std::string command = "STATE"+podState;
		    WriteToSerial(command, serialPortFilename);
			sleep(0.01);
			WriteToSerial(command, serialPortFilename);
	    } 
	    
	    inline void SetPodNumber(int number, std::string serialPortFilename = defaultSerialPortFilename)
	    {
		    std::string command = "POD"+std::to_string(number);
		    WriteToSerial(command, serialPortFilename);
			sleep(0.01);
			WriteToSerial(command, serialPortFilename);
	    }
	    
	    inline void SetBackgroundCaptureNumber(int number, std::string serialPortFilename = defaultSerialPortFilename)
	    {
		    std::string command = "BGCount"+std::to_string(number);
		    WriteToSerial(command, serialPortFilename);
			sleep(0.01);
		    WriteToSerial(command, serialPortFilename);
	    }
	    
}

/*
int main(int argc, char** argv)
{
    const std::string testStateArray[14] = {Trinket::Idle, Trinket::Starting_Calibration, Trinket::Starting_Broadcast, Trinket::Starting_BG_Capture, Trinket::Calibration_Capturing_Video, Trinket::Calibration_Data_Transferring, Trinket::Calibration_Data_Transfer_Complete, Trinket::Calibration_Running_Calib, Trinket::Calibration_Successful, Trinket::Calibration_Failed, Trinket::Broadcast_Running, Trinket::Capturing_BG_Images, Trinket::Pod_Rebooting, Trinket::Broadcast_Stopping };

	while(true)
	{
		for(int pod = 1; pod < 11; pod++)
		{
			std::cout << "Setting pod number to " << pod << std::endl;
			Trinket::SetPodNumber(pod);
			sleep(0.25);
			for(int state = 0; state < 14; state++)
			{
				std::cout << "Setting pod state to " << testStateArray[state] << std::endl;
				Trinket::SetState(testStateArray[state]);
				sleep(0.5);
			}
			Trinket::SetBackgroundCaptureNumber(pod*10);
		}
		Trinket::Close();
	}
	Trinket::Close();
}
*/