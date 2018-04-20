/*
 * main.cpp
 *
 * Connect to a Bus Pirate, set it up in UART mode and put it in live monitor
 * mode so that the serial data can be sniffed with the data output in hex
 * format so that raw packets can be seen. It is not designed for ASCII data.
 *
 * Various serial settings of the device under test can be set.
 *
 * The command to compile it is:
 *   gcc main.cpp -o buspirate_uart_sniffer -lstdc++
 *
 * MIT License
 *
 * Copyright (c) 2018 Jon Axtell
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies
 * of the Software, and to permit persons to whom the Software is furnished to do
 * so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
 * INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A
 * PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
 * HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION
 * OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
 * SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */

#include <iostream>
#include <iomanip>
#include <cerrno>
#include <cstring>
#include <algorithm>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <termios.h>
#include <fcntl.h>
#include <time.h>
#include <sys/time.h>

int uart;
std::string port;
bool verbose;
bool livemonitor;
unsigned int timeout;
unsigned int outputwidth;
unsigned int devicebaud;
unsigned int devicedata;
char deviceparity;
unsigned int devicestop;
std::string deviceprotocol;
struct timeval programstart;
struct timeval interpackettimer;
uint32_t previouspackettime;

//#############################################################################
// Timer functions
//

//-----------------------------------------------------------------------------
// Start a timer
//
void StartTimer(struct timeval &timer)
{
    gettimeofday(&timer, NULL);
}

//-----------------------------------------------------------------------------
// Get time elapsed since the timer was started
//
unsigned int TimeElapsedInMilliseconds(struct timeval &timer)
{
    struct timeval now;

    gettimeofday(&now, NULL);
    return ((now.tv_sec - timer.tv_sec) * 1000) + ((now.tv_usec - timer.tv_usec) / 1000);
}

//-----------------------------------------------------------------------------
// Get current time
//
unsigned int TimeInMilliseconds(void)
{
    struct timeval now;

    gettimeofday(&now, NULL);
    return (now.tv_sec * 1000) + (now.tv_usec  / 1000);
}

//#############################################################################
// Set the serial port
//
// Baud can be B50/B75/B110/B134/B150/B200/B300/B600/B1200/B1800/B2400/B4800/
//              B9600/B19200/B38400/B57600/B115200/B230400/B460800/B576000/
//              B921600/B1000000/B1152000/B1500000/B2000000/B2500000/
//              B3000000/B3500000/B4000000
// Data can be CS5/CS6/CS7/CS8
// Parity can be PARENB | PARODD for odd, PARENB for even, 0 for none
// Stop is 0 for 1 stop, CSTOPB for 2 stop bits
//
bool SetInterface(int handle, int baud, int data, int parity, int stop)
{
    struct termios tty;

    if (tcgetattr(handle, &tty) < 0)
    {
        return false;
    }

    cfsetospeed(&tty, baud);
    cfsetispeed(&tty, baud);

    // disable IGNBRK for mismatched speed tests; otherwise receive break as \000 chars
    tty.c_iflag &= ~IGNBRK;				// Ignore break signal
    tty.c_iflag &= ~(IXON | IXOFF | IXANY);	// Mask out xon/xoff ctrl
    tty.c_iflag &= ~ICRNL;				// Ignore CR NL
    tty.c_lflag = 0;					// No signalling chars, no echo, no canonical processing
    tty.c_oflag = 0;					// No re-mapping, no delays
    tty.c_cc[VMIN]  = 0;				// Read doesn't block
    tty.c_cc[VTIME] = 5;				// 0.5 seconds read timeout

    tty.c_cflag |= (CLOCAL | CREAD);	// Ignore modem controls, enable reading
    tty.c_cflag &= ~CSIZE;              // Mask for for databits
    tty.c_cflag |= data;                // .. and set
    tty.c_cflag &= ~(PARENB | PARODD);	// Mask out parity ...
    tty.c_cflag |= parity;				// ... and set our parity settings
    tty.c_cflag &= ~(CSTOPB);           // Mask out stop bits
    tty.c_cflag |= stop;                // ... and set
    tty.c_cflag &= ~CRTSCTS;			// RTS/CTS not supported (no flow control)

    if (tcsetattr(handle, TCSANOW, &tty) < 0)
    {
        return false;
    }

    return true;
}

//#############################################################################
// Receive and throw away a CR LF sequence
//
bool EatCRLF(int handle)
{
    char buffer[32];

    int i = 0;
    do
    {
        if (read(handle, &buffer[i], 1) == 1)
        {
            ++i;
        }
        if (i == 32)
        {
            return false;
        }
    } while (strncmp(buffer, "\r\n", 2) != 0);
    return true;
}

//#############################################################################
// Send command to Bus Pirate
//
bool SendCommand(int handle, const char *command)
{
    char buffer[32];

    // Send the command
    if (strlen(command) > 0)
    {
        if (verbose) std::cout << "Sending command [" << command << "]" << std::endl;
        write(handle, command, strlen(command));

        // Eat the echo
        int i = 0;
        do
        {
            if (read(handle, &buffer[i], 1) == 1)
            {
                ++i;
            }
            if (i == 32)
            {
                return false;
            }
        } while (strncmp(buffer, command, strlen(command)) != 0);
    }
    write(handle, "\r", 1);

    return EatCRLF(handle);
}

//#############################################################################
// Wait for specified prompt from Bus Pirate
//
bool GetPrompt(int handle, const char *prompt)
{
    char buffer[32];
    int i = 0;
    do
    {
        if (read(handle, &buffer[i], 1) == 1)
        {
            //std::cout << (int)buffer[i] << std::endl;
            if (buffer[i] == '\n')
            {
                buffer[i] = 0;
                //std::cout << "line is " << buffer << std::endl;
                i = 0;
            }
            else
            {
                ++i;
            }
        }
    } while ((strncmp(buffer, prompt, strlen(prompt)) != 0) && (i < 32));
    if (i == 32)
    {
        return false;
    }
    return true;
}

//#############################################################################
// Convert buad rate to index used by Bus Pirate
//
int BaudRateIndex(unsigned int baud)
{
    static const unsigned int rates[] = {
        300,
        1200,
        2400,
        4800,
        9600,
        19200,
        38400,
        57600,
        115200
    };

    for (unsigned int i = 0; i < sizeof(rates) / sizeof(unsigned int); ++i)
    {
        if (baud == rates[i])
        {
            return i + 1;   // Base is 1
        }
    }
    return -1;
}

//#############################################################################
// Convert data bits and parity to index used by Bus Pirate
//
int DataParityIndex(unsigned int data, char parity)
{
    if ((data == 8) && (parity == 'N'))
    {
        return 1;
    }
    else if ((data == 8) && (parity == 'E'))
    {
        return 2;
    }
    else if ((data == 8) && (parity == 'O'))
    {
        return 3;
    }
    else if ((data == 9) && (parity == 'N'))
    {
        return 4;
    }
    return -1;
}

//#############################################################################
// Convert stop bits to index used by Bus Pirate
//
int StopBitsIndex(unsigned int stop)
{
    if (stop == 1)
    {
        return 1;
    }
    else if (stop == 2)
    {
        return 2;
    }
    return -1;
}

//#############################################################################
// Output usage/help
//
void Usage(void)
{
    std::cout << "buspirate_uart_sniffer -p <port> [options] [device options]" << std::endl;
    std::cout << std::endl;
    std::cout << "Options:" << std::endl;
    std::cout << "  -w <width>  Width of output (default 16 bytes)" << std::endl;
    std::cout << "  -t <time>   Interpacket timeout (default is 50ms)" << std::endl;
    std::cout << "  -l          Already in Live monitor mode" << std::endl;
    std::cout << "  -v          Verbose output" << std::endl;
    std::cout << std::endl;
    std::cout << "Device options:" << std::endl;
    std::cout << "  -C <protocol>  Communication protocol" << std::endl;
    std::cout << "  -B <baud>      Baudrate (default is 9600)" << std::endl;
    std::cout << "  -D <data>      Data bits (8 or 9)" << std::endl;
    std::cout << "  -P <parity>    Parity (N, E, or O)" << std::endl;
    std::cout << "  -S <stop>      Stop bits (1 or 2)" << std::endl;
    std::cout << std::endl;
    std::cout << "Default is 9600 8N1 with 50ms timeout and 16 bytes per line." << std::endl;
    std::cout << "Recognised baud rates for the device are 300, 1200, 2400, 4800" << std::endl <<
                 "9600, 19200, 38400, 57600, 115200" << std::endl;
    std::cout << "Communication protocols are RP80, ccTalk, SSP, SAS, BACTA." << std::endl <<
                 "Each packet will start on a new line. If no protocol is specified" << std::endl <<
                 "the interpacket timeout will be used." << std::endl;
}

//#############################################################################
// Process command line arguments
//
// Returns false if invalid argument, missing option, etc.
//
bool ProcessCommandLineArguments(int argc, char *argv[])
{
    bool passed = true;

    deviceprotocol = "";
    devicebaud = 9600;
    devicedata = 8;
    deviceparity = 'N';
    devicestop = 1;
    outputwidth = 16;
    timeout = 0;
    verbose = false;
    livemonitor = false;

    opterr = 0;
    int opt;
    while ((opt = getopt(argc, argv, "hvlp:w:t:C:B:D:P:S:")) != -1)
    {
        switch (opt)
        {
            case 'h':
            {
                Usage();
                return EXIT_SUCCESS;    // Exit rather than return since no other work can be done
            }
            case 'p':
            {
                port = optarg;
//                std::cout << "Port is " << port << std::endl;
                break;
            }
            case 'w':
            {
                outputwidth = strtoul(optarg, NULL, 10);
                if ((outputwidth < 8) || (outputwidth > 64))
                {
                    std::cerr << "Width is between 8 and 64" << std::endl;
                    passed = false;
                }
//                std::cout << "Width is " << outputwidth << std::endl;
                break;
            }
            case 't':
            {
                timeout = strtoul(optarg, NULL, 10);
                if (timeout > 10000)
                {
                    std::cerr << "Timeout limit is 10000ms (10s)" << std::endl;
                    passed = false;
                }
//                std::cout << "Timeout is " << timeout << std::endl;
                break;
            }
            case 'v':
            {
                verbose = true;
//                std::cout << "Verbose is " << verbose << std::endl;
                break;
            }
            case 'l':
            {
                livemonitor = true;
//                std::cout << "Live monitor mode is " << livemonitor << std::endl;
                break;
            }
            case 'C':
            {
                deviceprotocol = optarg;
                std::transform(deviceprotocol.begin(), deviceprotocol.end(), deviceprotocol.begin(), [](unsigned char c) -> unsigned char { return std::toupper(c); });
                if ((deviceprotocol.compare("RP80") != 0) && (deviceprotocol.compare("CCTALK") != 0) && (deviceprotocol.compare("SSP") != 0) && (deviceprotocol.compare("SAS") != 0) && (deviceprotocol.compare("BACTA") != 0))
                {
                    std::cerr << "Protocols are RP80, ccTalk, SSP, SAS, BACTA" << std::endl;
                    passed = false;
                }
//                std::cout << "Protocol is " << deviceprotocol << std::endl;
                break;
            }
            case 'B':
            {
                devicebaud = strtoul(optarg, NULL, 10);
                if (BaudRateIndex(devicebaud) == -1)
                {
                    std::cerr << "Baudrates are 300, 1200, 2400, 4800, 9600, 19200, 38400, 57600, 115200" << std::endl;
                    passed = false;
                }
//                std::cout << "Baud rate is " << devicebaud << std::endl;
                break;
            }
            case 'D':
            {
                devicedata = strtoul(optarg, NULL, 10);
                if ((devicedata != 8) && (devicedata != 9))
                {
                    std::cerr << "Databits is 8 or 9" << std::endl;
                    passed = false;
                }
//                std::cout << "Data bits is " << devicedata << std::endl;
                break;
            }
            case 'P':
            {
                deviceparity = toupper(optarg[0]);
                if ((deviceparity != 'N') && (deviceparity != 'E') && (deviceparity != 'O'))
                {
                    std::cerr << "Parity is None, Even, or Odd" << std::endl;
                    passed = false;
                }
//                std::cout << "Parity is " << deviceparity << std::endl;
                break;
            }
            case 'S':
            {
                devicestop = strtoul(optarg, NULL, 10);
                if (StopBitsIndex(devicestop) == -1)
                {
                    std::cerr << "Stop bits is 1 or 2" << std::endl;
                    passed = false;
                }
//                std::cout << "Stop bits is " << devicestop << std::endl;
                break;
            }
            case ':':
            {
                std::cout << "Missing argument" << std::endl << std::endl;;
                Usage();
                passed = false;
                break;
            }
            case '?':
            {
                std::cout << "Unknown option" << std::endl << std::endl;
                Usage();
                passed = false;
                break;
            }
        }
    }

    if (passed && DataParityIndex(devicedata, deviceparity) == -1)
    {
        std::cerr << "Recognised combinations of data and parity are 8N, 8E, 8O, or 9N" << std::endl << std::endl;
        Usage();
        passed = false;
    }
    if (passed && port.empty())
    {
        std::cerr << "No port specified" << std::endl << std::endl;
        Usage();
        passed = false;
    }

    return passed;
}

//#############################################################################
// Send a sequence of command to put the Bus Pirate into UART Live Monitor mode
//
bool ConfigureBusPirate(void)
{
    int retry;
    for (retry = 0; retry < 10; ++retry)
    {
        SendCommand(uart, "m 1");
        if (GetPrompt(uart, "HiZ>"))
        {
            if (verbose) std::cout << "At HiZ prompt" << std::endl;
            break;
        }
        else
        {
            if (verbose) std::cout << "Not at HiZ prompt, trying to get to main menu" << std::endl;
        }
    }
    if (retry == 10)
    {
        close(uart);
        return false;
    }

    if (verbose) std::cout << "Selecting menu" << std::endl;

    SendCommand(uart, "m");
    if (!GetPrompt(uart, "(1)>"))
    {
        close(uart);
        return false;
    }

    if (verbose) std::cout << "Selecting uart mode" << std::endl;

    SendCommand(uart, "3");
    if (!GetPrompt(uart, "(1)>"))
    {
        close(uart);
        return false;
    }

    if (verbose) std::cout << "Selecting " << devicebaud << " baud" << std::endl;

    char buffer[32];
    sprintf(buffer, "%d", BaudRateIndex(devicebaud));
    SendCommand(uart, buffer);
    if (!GetPrompt(uart, "(1)>"))
    {
        close(uart);
        return false;
    }

    if (verbose) std::cout << "Selecting " << devicedata << " data bits & " << deviceparity << " parity" << std::endl;

    sprintf(buffer, "%d", DataParityIndex(devicedata, deviceparity));
    SendCommand(uart, buffer);
    if (!GetPrompt(uart, "(1)>"))
    {
        close(uart);
        return false;
    }

    if (verbose) std::cout << "Selecting " << devicestop << " stop bits" << std::endl;

    sprintf(buffer, "%d", StopBitsIndex(devicestop));
    SendCommand(uart, buffer);
    if (!GetPrompt(uart, "(1)>"))
    {
        close(uart);
        return false;
    }

    if (verbose) std::cout << "Selecting normal polarity" << std::endl;

    SendCommand(uart, "1");
    if (!GetPrompt(uart, "(1)>"))
    {
        close(uart);
        return false;
    }

    if (verbose) std::cout << "Selecting open drain type" << std::endl;

    SendCommand(uart, "1");
    if (!GetPrompt(uart, "UART>"))
    {
        close(uart);
        return false;
    }

    if (verbose) std::cout << "In UART mode, selecting live monitor" << std::endl;

    SendCommand(uart, "(2)");
    if (!GetPrompt(uart, "Any key to exit"))
    {
        close(uart);
        return false;
    }

    EatCRLF(uart);
    return true;
}

//#############################################################################
// Parse incoming data and indicate when a RP80 packet has started
//
bool ParseRP80Packet(unsigned char ch)
{
    const uint8_t TX_STX1 = 0x7F;
    const uint8_t TX_STX2 = 0x7E;
    const uint8_t RX_STX1 = 0x7D;
    const uint8_t RX_STX2 = 0x7C;
    static enum { STATE_STX1, STATE_STX2, STATE_LEN, STATE_DATA, STATE_CHECKSUM } state = STATE_STX1;
    static unsigned char buffer[128];
    static int i = 0;
    static int len = 0;

    switch (state)
    {
        case STATE_STX1:
        {
            if ((ch == TX_STX1) || (ch == RX_STX1))
            {
                state = STATE_STX2;
                return true;
            }
            else
            {
                state = STATE_STX1;
            }
            break;
        }
        case STATE_STX2:
        {
            if ((ch == TX_STX2) || (ch == RX_STX2))
            {
                state = STATE_LEN;
            }
            else
            {
                state = STATE_STX1;
            }
            break;
        }
        case STATE_LEN:
        {
            len = ch;
            i = 0;
            state = STATE_DATA;
            break;
        }
        case STATE_DATA:
        {
            buffer[i] = ch;
            if (++i >= len)
            {
                state = STATE_CHECKSUM;
            }
            break;
        }
        case STATE_CHECKSUM:
        {
            unsigned char cksum;
            for (int i = 0; i < len; ++i)
            {
                cksum ^= buffer[i];
            }
            if (cksum == ch)
            {
                // We don't care about checksum being valid here.
            }
            state = STATE_STX1;
        }
    }
    return false;
}

//#############################################################################
// Parse incoming data and indicate when a ccTalk packet has started
//
bool ParseCCTalkPacket(unsigned char ch)
{
    (void)ch;
    return false;
}

//#############################################################################
// Parse incoming data and indicate when a SSP packet has started
//
bool ParseSSPPacket(unsigned char ch)
{
    (void)ch;
    return false;
}

//#############################################################################
// Parse incoming data and indicate when a SAS packet has started
//
bool ParseSASPacket(unsigned char ch)
{
    (void)ch;
    return false;
}

//#############################################################################
// Parse incoming data and indicate when a BACTA packet has started
//
bool ParseBACTAPacket(unsigned char ch)
{
    (void)ch;
    return false;
}

//#############################################################################
// Main superloop
//
int main(int argc, char *argv[])
{
    if (argc == 1)
    {
        Usage();
        return EXIT_SUCCESS;
    }
    if (!ProcessCommandLineArguments(argc, argv))
    {
        return EXIT_FAILURE;
    }

    uart = open(port.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
    if (uart < 0)
    {
        std::cerr << "Failed to open serial port " << port << ", " << std::strerror(errno) << std::endl;
        return EXIT_FAILURE;
    }

    if (!SetInterface(uart, B115200, CS8, 0, 0))
    {
        std::cerr << "Failed to set serial port settings on " << port << ", " << std::strerror(errno) << std::endl;
    }

    if (!livemonitor)
    {
        if (!ConfigureBusPirate())
        {
            return EXIT_FAILURE;
        }
    }

    std::cout << "In live monitor mode." << std::endl <<
                 "Use Ctrl-C to quit. The Bus Pirate will need resetting after use." << std::endl;

    char ch;
    unsigned int b;
    unsigned int i;
    StartTimer(interpackettimer);
    StartTimer(programstart);
    bool newline = true;
    while (1)
    {
        if (read(uart, &ch, 1) == 1)
        {
            // If full packet has been received or interpacket timeout reached, then we can start a new line
            if (!deviceprotocol.empty())
            {
                if (deviceprotocol.compare("RP80") == 0)
                {
                    if (ParseRP80Packet(ch))
                    {
                        newline = true;
                    }
                }
                else if (deviceprotocol.compare("CCTALK") == 0)
                {
                    if (ParseCCTalkPacket(ch))
                    {
                        newline = true;
                    }
                }
                else if (deviceprotocol.compare("SSP") == 0)
                {
                    if (ParseSSPPacket(ch))
                    {
                        newline = true;
                    }
                }
                else if (deviceprotocol.compare("SAS") == 0)
                {
                    if (ParseSASPacket(ch))
                    {
                        newline = true;
                    }
                }
                else if (deviceprotocol.compare("BACTA") == 0)
                {
                    if (ParseBACTAPacket(ch))
                    {
                        newline = true;
                    }
                }
            }
            else if (timeout != 0)
            {
                if (TimeElapsedInMilliseconds(interpackettimer) > timeout)
                {
                    newline = true;
                }
            }

            // Have we reached our width per line, start a new line anyway
            if (i > outputwidth)
            {
                newline = true;
            }

            // If a new line started because of inter packet timeout or width reached, output the byte offset
            if (newline)
            {
                newline = false;
                previouspackettime = TimeElapsedInMilliseconds(interpackettimer);
                StartTimer(interpackettimer);
                if (i != 0)
                {
                    // Start a new line ready for the byte offset
                    std::cout << std::endl;
                }
                i = 0;
                std::cout << std::setfill(' ') << std::setw(6) << previouspackettime << " " << std::hex << std::setfill('0') << std::setw(8) << b << ": " << std::dec;
            }

            // Output the byte
            std::cout << std::hex << std::setfill('0') << std::setw(2) << ((unsigned int)ch & 0xFF) << " " << std::dec << std::flush;

            // Update byte offset and position on line
            ++i;
            ++b;
        }
    }
    close(uart);
    return EXIT_SUCCESS;
}
