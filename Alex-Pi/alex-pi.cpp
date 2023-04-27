#include <stdio.h>
#include <pthread.h>
#include <semaphore.h>
#include <unistd.h>
#include <stdint.h>
#include "packet.h"
#include "serial.h"
#include "serialize.h"
#include "constants.h"
#include <ncurses.h>

/**
 * PSA *
 * All relevant printing statements within the cpp file is replaced with the print functions provided by ncurses
 * to facilitate printing on an ncurses-created window rather than to stdout
*/

// Port of Arduino
#define PORT_NAME			"/dev/ttyACM0" 
// Initialise Baud Rate
#define BAUD_RATE			B9600
// Initialise default power for Arduino
int setPower = 60;

int exitFlag=0;
sem_t _xmitSema;

// Global Window Variables for GUI
WINDOW *powerwindow;
WINDOW *informationwindow;
WINDOW *my_window;
WINDOW *messages;
WINDOW *stats;
WINDOW *welcome;

/**
 * Function refreshes the Messages Window
*/

void refreshMessages() {
	mvwprintw(messages, 1, 1, "                                      ");
	mvwprintw(messages, 2, 1, "                                      ");
	mvwprintw(messages, 3, 1, "                                      ");
	wrefresh(messages);
}

void handleError(TResult error)
{
	switch(error)
	{
		case PACKET_BAD:
			mvwprintw(messages, 1, 1, "ERROR: Bad Magic Number");
			break;

		case PACKET_CHECKSUM_BAD:
			mvwprintw(messages, 1, 1, "ERROR: Bad checksum");
			break;

		default:
			mvwprintw(messages, 1, 1, "ERROR: UNKNOWN ERROR");
	}
	wrefresh(messages);
}

/**
 * Function handles the returned colour packet from the Arduino and prints out the values it has received to the Message Window
*/

void handleColourStatus(TPacket *packet) {
	switch(packet->params[0]) {
		case 1: 
		//Red Colour Identified
			mvwprintw(messages, 2, 1, "RED IDENTIFIED");
			break;
		case 2: 
		// Green Colour Identified
			mvwprintw(messages, 2, 1, "GREEN IDENTIFIED");
			break;
		default:
		// Neither red nor green
			mvwprintw(messages, 2, 1, "NOT A VICTIM");
	}
	mvwprintw(messages, 2, 23, "HUE: %lu", packet->params[1]);
		
}

void handleStatus(TPacket *packet)
{
	mvwprintw(stats, 2, 3, "----- ALEX STATUS REPORT -----");
	mvwprintw(stats, 3, 3, "Left Forward Ticks: %d", packet->params[0]);
	mvwprintw(stats, 4, 3, "Right Forward Ticks: %d", packet->params[1]);
	mvwprintw(stats, 5, 3, "Left Reverse Ticks: %d", packet->params[2]);
	mvwprintw(stats, 6, 3, "Right Reverse Ticks: %d", packet->params[3]);
	mvwprintw(stats, 7, 3, "Left Forward Ticks Turns: %d", packet->params[4]);
	mvwprintw(stats, 8, 3, "Right Forward Ticks Turns: %d", packet->params[5]);
	mvwprintw(stats, 9, 3, "Left Reverse Ticks Turns: %d", packet->params[6]);
	mvwprintw(stats, 10, 3, "Right Reverse Ticks Turns: %d", packet->params[7]);
	mvwprintw(stats, 11, 3, "Forward Distance: %d", packet->params[8]);
	mvwprintw(stats, 12, 3, "Reverse Distance: %d", packet->params[9]);
	mvwprintw(stats, 13, 3, "------------------------------");
	wrefresh(stats);
}

void handleResponse(TPacket *packet)
{
	// The response code is stored in command
	switch(packet->command)
	{
		case RESP_OK:
			mvwprintw(messages, 1, 1, "Command OK");
		break;

		case RESP_STATUS:
			handleStatus(packet);
		break;

		case RESP_COLOUR:
			handleColourStatus(packet);
		break;

		default:
			mvwprintw(messages, 1, 1, "Arduino is confused");
	}
	wrefresh(messages);
}

void handleErrorResponse(TPacket *packet)
{
	// The error code is returned in command
	switch(packet->command)
	{
		case RESP_BAD_PACKET:
			mvwprintw(messages, 1, 1, "Arduino received bad magic number");
			break;

		case RESP_BAD_CHECKSUM:
			mvwprintw(messages, 1, 1, "Arduino received bad checksum");
			break;

		case RESP_BAD_COMMAND:
			mvwprintw(messages, 1, 1, "Arduino received bad command");
			break;

		case RESP_BAD_RESPONSE:
			mvwprintw(messages, 1, 1, "Arduino received unexpected response");
			break;

		default:
			mvwprintw(messages, 1, 1, "Arduino reports a weird error");
	}
	wrefresh(messages);
}

void handleMessage(TPacket *packet)
{
	mvwprintw(messages, 1, 1, "Message from Alex: %s", packet->data);
	wrefresh(messages);
}

void handlePacket(TPacket *packet)
{
	switch(packet->packetType)
	{
		case PACKET_TYPE_COMMAND:
			// Only we send command packets, so ignore
			break;

		case PACKET_TYPE_RESPONSE:
			handleResponse(packet);
			break;

		case PACKET_TYPE_ERROR:
			handleErrorResponse(packet);
			break;

		case PACKET_TYPE_MESSAGE:
			handleMessage(packet);
			break;
	}
}

void sendPacket(TPacket *packet)
{
	char buffer[PACKET_SIZE];
	int len = serialize(buffer, packet, sizeof(TPacket));

	serialWrite(buffer, len);
}

void *receiveThread(void *p)
{
	char buffer[PACKET_SIZE];
	int len;
	TPacket packet;
	TResult result;
	int counter=0;

	while(1)
	{
		len = serialRead(buffer);
		counter+=len;
		if(len > 0)
		{
			result = deserialize(buffer, len, &packet);

			if(result == PACKET_OK)
			{
				counter=0;
				handlePacket(&packet);
			}
			else 
				if(result != PACKET_INCOMPLETE)
				{
					mvwprintw(messages, 1, 1, "PACKET ERROR");
					handleError(result);
					wrefresh(messages);
				}
		}
	}
}

void sendCommand(char command)
{
	TPacket commandPacket;

	commandPacket.packetType = PACKET_TYPE_COMMAND;

	switch(command)
	{
		case 'w':
		case 'W':
			commandPacket.params[1] = setPower;
			commandPacket.params[0] = 0;
			commandPacket.command = COMMAND_FORWARD;
			sendPacket(&commandPacket);
			break;

		case 's':
		case 'S':
			commandPacket.params[1] = setPower;
			commandPacket.params[0] = 0;
			commandPacket.command = COMMAND_REVERSE;
			sendPacket(&commandPacket);
			break;

		case 'a':
		case 'A':
			commandPacket.params[1] = setPower;
			commandPacket.params[0] = 0;
			commandPacket.command = COMMAND_TURN_LEFT;
			sendPacket(&commandPacket);
			break;

		case 'd':
		case 'D':
			commandPacket.params[1] = setPower;
			commandPacket.params[0] = 0;
			commandPacket.command = COMMAND_TURN_RIGHT;
			sendPacket(&commandPacket);
			break;

		case 'h':
		case 'H':
			commandPacket.command = COMMAND_STOP;
			sendPacket(&commandPacket);
			break;

		case 'c':
		case 'C':
			commandPacket.command = COMMAND_CLEAR_STATS;
			commandPacket.params[0] = 0;
			sendPacket(&commandPacket);
			break;

		case 'g':
		case 'G':
			commandPacket.command = COMMAND_GET_STATS;
			sendPacket(&commandPacket);
			break;
		
		case 'o':
		case 'O':
			commandPacket.command = COMMAND_IDENTIFY_COLOUR;
            sendPacket(&commandPacket);
            break;

		default:
			commandPacket.command = COMMAND_STOP;
			sendPacket(&commandPacket);
	}
}

/**
 * Function prints the respective windows to split up data and allow simultaneous viewing of all data.
 * 
 * @param[in] win     Passes in a pointer of WINDOW type, which allows the function to modify the required window
 * @param[in] height  height of the window to be created
 * @param[in] width   width of the window to be created
 * @param[in] start_y The y starting position of the window with respect to the coordinate (0,0) from the top left hand side of the window
 * @param[in] start_x The x starting position of the window with respect to the coordinate (0,0) from the top left hand side of the window
 * @param[in] wintype Specifies which window to be created, ranges from 1-6, each having its own size and window header to be printed:
 * 					  1 - Power Window, 2 - Command Window, 3 - Command List Window, 4 - Messages Window, 5 - Get Stats Print Window, 6 - Welcome to Alex Window
 * 
 * @return A pointer of WINDOW type, which should already be modified and printed on the current screen.
*/

WINDOW * setupWindow(WINDOW *win, int height, int width, int start_y, int start_x, int wintype) {
    // Create a new window of specified dimensions
    win = newwin(height, width, start_y, start_x);
    refresh();
    box(win,0 ,0);
    if (wintype == 1) {
        wprintw(win, "Show Power");
		mvwprintw(win, 1, 1, "Current Power:");
		mvwprintw(win, 3, 7, "%d", setPower);
    } else if (wintype == 2) {
        wprintw(win, "Enter Command Here!");
    } else if (wintype == 3) {
        wprintw(win, "Command List");
        mvwprintw(win, 1, 1, "Use LS/RS to move");
        mvwprintw(win, 3, 2, "X to clear stats");
        mvwprintw(win, 4, 2, "A to get stats");
        mvwprintw(win, 5, 2, "Y to get colour");
        mvwprintw(win, 6, 2, "B to exit");
		mvwprintw(win, 7, 2, "RB to increase speed");
        mvwprintw(win, 8, 2, "LB to decrease speed");
    } else if (wintype == 4) {
		wprintw(win, "Messages");
	} else if (wintype == 5) {
		wprintw(win, "Stats");
	} else if (wintype == 6) {
		mvwprintw(win, 1, 0, "WELCOME TO ALEX!");
		mvwprintw(win, 2, 1, "Done by B04-5B");
	}
    wrefresh(win);
	refresh();
    return win;
}

/**
 * Initialises the GUI before printing windows
*/

void mainSCR() {
	// Initialises the screen, refresh it, turns off input buffer and echo
    initscr();
    refresh();
	halfdelay(2.5); //Waits for 0.4 seconds, if no input then return ERR, if have input then take in like cbreak();
    noecho();
}

/**
 * Function maps all possible key inputs and sends char to the sendCommand function for further action
*/

void userInput() {
	int ch;
	bool stopFlag = true;
	// the char 'q' is used to terminate the program.
	while ((char) ch != 'q') {
		// wgetch grabs user inputs from the window my_window. If there is no user input, it returns an ERR, else it returns the char input
		while ((ch = wgetch(my_window)) == ERR) {
            if (stopFlag == false) {
            	sendCommand('h');
            	mvwprintw(my_window, 1, 1, "I have stopped!     ");
            	stopFlag = true;
            }
        } 
        if ((char)ch == 'w' && stopFlag == true) {
			// Move Forward
            mvwprintw(my_window, 1, 1, "I am moving forward!");
			sendCommand(ch);
			stopFlag = false;
        } else if ((char)ch == 'a' && stopFlag == true) {
			// Turn Left
			mvwprintw(my_window, 1, 1, "I am turning Left!  ");
			sendCommand(ch);
			stopFlag = false;
		} else if ((char)ch == 's' && stopFlag == true) {
			// Move backwards
			mvwprintw(my_window, 1, 1, "I am reversing!     ");
			sendCommand(ch);
			stopFlag = false;
		} else if ((char)ch == 'd' && stopFlag == true) {
			// Turn Right
			mvwprintw(my_window, 1, 1, "I am turning right! ");
			sendCommand(ch);
			stopFlag = false;
		} else if ((char)ch == 'g' || (char)ch == 'o') {
			// Get Stats and Colour respectively
			sendCommand(ch);
		} else if ((char)ch == 'c') {
			// Clear stats
			werase(stats);
			wrefresh(stats);
			stats = setupWindow(stats, 15, 35, 0, 41, 5);
			sendCommand(ch);
		} else if ((char)ch == 'p') {
			// Increase power
			setPower += 5;
			mvwprintw(powerwindow, 3, 7, "%d", setPower);
			wrefresh(powerwindow);
		} else if ((char)ch == 'm') {
			// Decrease Power
			setPower -= 5;
			mvwprintw(powerwindow, 3, 7, "%d", setPower);
			wrefresh(powerwindow);
		} else if ((char)ch == 'r') {
			// Force Refresh Message Window
			refreshMessages();
		}
	}
}

int main()
{
	// Connect to the Arduino (found in serial.h)
	// (portname, baudrate, bytesize, parity, stopbits, maxattempts)
	startSerial(PORT_NAME, BAUD_RATE, 8, 'N', 1, 5);

	// Sleep for two seconds
	printf("WAITING TWO SECONDS FOR ARDUINO TO REBOOT\n");
	sleep(2);
	printf("DONE\n");

	// Spawn receiver thread
	pthread_t recv;

	pthread_create(&recv, NULL, receiveThread, NULL);

	mainSCR();

    // Setup Speed Variable Window
    powerwindow = setupWindow(powerwindow, 6, 16, 4, 0, 1);
    // Setup Information Window
    informationwindow = setupWindow(informationwindow, 10, 23, 0, 17, 3);
    // Setup Command Window
    my_window = setupWindow(my_window, 22, 76, 15, 0, 2);
	// Setup Messages Window
	messages = setupWindow(messages, 5, 40, 10, 0, 4);
	// Setup Stats Window
	stats = setupWindow(stats, 15, 35, 0, 41, 5);
	// Setup Welcome Window
	welcome = setupWindow(welcome, 4, 16, 0, 0, 6);

	// Creates a Data Packet Structure called hellopacket
	TPacket helloPacket;

	// Send a hello packet
	helloPacket.packetType = PACKET_TYPE_HELLO;
	sendPacket(&helloPacket);

	// User inputs controls via GUI created with ncurseslib
	userInput();

    refresh();
	// Terminate GUI
    endwin();
	printf("Closing connection to Arduino.\n");
	endSerial();
}
