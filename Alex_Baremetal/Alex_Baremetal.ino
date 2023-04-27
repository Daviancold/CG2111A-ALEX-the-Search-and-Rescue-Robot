#include <stdarg.h>
#include "packet.h"
#include "constants.h"
#include <serialize.h>
#include <math.h>

typedef enum
{
  STOP = 0,
  FORWARD = 1,
  BACKWARDS = 2,
  LEFT = 3,
  RIGHT = 4,
} TDirection;

volatile TDirection dir = STOP;

/*
 * Alex's configuration constants
 */

// Number of ticks per revolution from the 
// wheel encoder.

#define COUNTS_PER_REV      172

// Wheel circumference in cm.
// We will use this to calculate forward/backward distance traveled 
// by taking revs * WHEEL_CIRC

#define WHEEL_CIRC          19

// Motor control pins. You need to adjust these till
// Alex moves in the correct direction
#define LF                  (1 << 5)   // Left forward pin - OCR0B PORTD PIN 5
#define LR                  (1 << 6)   // Left reverse pin - OCR0A PORTD PIN 6
#define RF                  (1 << 2)  // Right forward pin - OCR1B PORTB PIN 2
#define RR                  (1 << 3)  // Right reverse pin - OCR2A PORTB PIN 3

// Alex's dimension in cm
#define ALEX_LENGTH         19
#define ALEX_BREADTH        11

// Alex's colour sensor pins
#define S0 (1 << 7) // PORTD PIN 7
#define S1 (1 << 0) // PORTB PIN 0
#define S2 (1 << 4) // PORTB PIN 4
#define S3 (1 << 1) // PORTB PIN 1
#define sensorOut (1 << 5) //PORTB PIN 5

// Stores frequency read by photodiodes
uint32_t redFrequency = 0;
uint32_t blueFrequency = 0;
uint32_t greenFrequency = 0;

// Alex's diagonal. We compute and store this once
float AlexDiagonal = 0.0;

// Alex's turning circumference, calculated once
float AlexCirc = 0.0;
/*
 *    Alex's State Variables
 */

// Store the ticks from Alex's left and
// right encoders.
volatile unsigned long leftForwardTicks; 
volatile unsigned long rightForwardTicks;
volatile unsigned long leftReverseTicks;
volatile unsigned long rightReverseTicks;

// Left and right encoder ticks for turning
volatile unsigned long leftForwardTicksTurns;
volatile unsigned long rightForwardTicksTurns;
volatile unsigned long leftReverseTicksTurns;
volatile unsigned long rightReverseTicksTurns;

// Store the revolutions on Alex's left
// and right wheels
volatile unsigned long leftRevs;
volatile unsigned long rightRevs;

// Forward and backward distance traveled
volatile unsigned long forwardDist;
volatile unsigned long reverseDist;

// Variable to keep track of whether we have moved a commanded distance
unsigned long deltaDist;
unsigned long newDist;

// Variable to keep track of our turning angle
unsigned long deltaTicks;
unsigned long targetTicks;

/*
 * 
 * Alex Communication Routines.
 * 
 */
 
TResult readPacket(TPacket *packet)
{
    // Reads in data from the serial port and
    // deserializes it.Returns deserialized
    // data in "packet".
    
    char buffer[PACKET_SIZE];
    int len;

    len = readSerial(buffer);

    if(len == 0)
      return PACKET_INCOMPLETE;
    else
      return deserialize(buffer, len, packet);
    
}

/**
 * Function sets up the colour sensor for usage
*/

void setup_colour_sensor(){
    // Setting the outputs
    DDRB |= (S1|S2|S3);
    DDRD |= S0;
    // Setting the sensorOut as input
    DDRB &= ~sensorOut;
    // Setting frequency scaling to 20%
    PORTD |= S0;
    PORTB &= ~S1;
}

/**
 * Reads the raw values of the colour sensor, converts it into Hue angle before sending the hue data to the RPI
*/

void sendColourStats(){
    // set the filtered photodiodes to be read
    // Read the colour output frequency
    // Remap the value of the colour frequencies from 0 to 255
    // Setting RED filtered photodiodes to be read
    PORTB &= ~(S2|S3);

    redFrequency = pulseIn(13, LOW);
    uint32_t redcolour = (uint32_t)(map(redFrequency, 309, 3600, 255, 0));
    delay(100);
    // Setting the GREEN filtered photodiodes to be read
    PORTB |= (S2|S3);

    greenFrequency = pulseIn(13, LOW);
    uint32_t greencolour = (uint32_t)(map(greenFrequency, 320, 3800, 255, 0));
    delay(100);

    // Setting BLUE filtered photodiodes to be read
    PORTB &= ~S2;
    PORTB |= S3;

    blueFrequency = pulseIn(13, LOW);
    uint32_t bluecolour = (uint32_t)(map(blueFrequency, 270, 3100, 255, 0));
    delay(100);
    // Calculating the Hue angle of the colour
    uint32_t R = redcolour;

    uint32_t B = bluecolour;
 
    uint32_t G = greencolour;
 
    double hue = 0;
    double minimum = (double)(min(R,min(B,G)));
    double maximum = (double)(max(R,max(B,G)));
    double scale = maximum - minimum;
  
    if( scale == 0)
    {
      hue = -1;
    }
    else
    {
      double Red = (((maximum - R)/6) + (scale/2))/scale;
      double Blue = (((maximum - B)/6) + (scale/2))/scale;
      double Green = (((maximum - G)/6) + (scale/2))/scale;
      
      if(R == maximum)
      {
        hue = Blue - Green;
      }   
      else if(G == maximum)
      {
        hue = (1/3) + Red - Blue;
      } 
      else if(B == maximum)
      {
        hue = (2/3) + Green - Red;
      }
      if(hue<0)
      {
        hue += 1;
      }
      if(hue>1)
      {
        hue -= 1;
      }

      hue*=360;

    }
    TPacket statusPacket;
    statusPacket.packetType = PACKET_TYPE_RESPONSE;
    statusPacket.command = RESP_COLOUR;
    if((hue >= 350.0 && hue <= 359.0) || (hue > 0.0 && hue < 5.0))  
    {
      // Colour is Red
      statusPacket.params[0] = 1;
      statusPacket.params[1] = (uint32_t)hue;
    }
    else if(hue>= 26.0 && hue<= 45.0)
    {
      // Colour is Green
      statusPacket.params[0] = 2;
      statusPacket.params[1] = (uint32_t)hue;
    }
    else
    {
      // Not a valid colour
      statusPacket.params[0] = 3;
      statusPacket.params[1] = (uint32_t)hue;
    }
  sendResponse(&statusPacket);
}

void sendStatus()
{
  // Implement code to send back a packet containing key
  // information like leftTicks, rightTicks, leftRevs, rightRevs
  // forwardDist and reverseDist
  // Use the params array to store this information, and set the
  // packetType and command files accordingly, then use sendResponse
  // to send out the packet. See sendMessage on how to use sendResponse.

  TPacket statusPacket; 
  statusPacket.packetType = PACKET_TYPE_RESPONSE; 
  statusPacket.command = RESP_STATUS; 
  statusPacket.params[0] = leftForwardTicks; 
  statusPacket.params[1] = rightForwardTicks; 
  statusPacket.params[2] = leftReverseTicks; 
  statusPacket.params[3] = rightReverseTicks; 
  statusPacket.params[4] = leftForwardTicksTurns; 
  statusPacket.params[5] = rightForwardTicksTurns; 
  statusPacket.params[6] = leftReverseTicksTurns; 
  statusPacket.params[7] = rightReverseTicksTurns; 
  statusPacket.params[8] = forwardDist; 
  statusPacket.params[9] = reverseDist; 
  sendResponse(&statusPacket);
}

void sendMessage(const char *message)
{
  // Sends text messages back to the Pi. Useful
  // for debugging.
  
  TPacket messagePacket;
  messagePacket.packetType=PACKET_TYPE_MESSAGE;
  strncpy(messagePacket.data, message, MAX_STR_LEN);
  sendResponse(&messagePacket);
}

void dbprintf(char *format, ...)
{
  va_list args;
  char buffer[128];

  va_start(args, format);
  vsprintf(buffer, format, args);
  sendMessage(buffer);
}

void sendBadPacket()
{
  // Tell the Pi that it sent us a packet with a bad
  // magic number.
  
  TPacket badPacket;
  badPacket.packetType = PACKET_TYPE_ERROR;
  badPacket.command = RESP_BAD_PACKET;
  sendResponse(&badPacket);
  
}

void sendBadChecksum()
{
  // Tell the Pi that it sent us a packet with a bad
  // checksum.
  
  TPacket badChecksum;
  badChecksum.packetType = PACKET_TYPE_ERROR;
  badChecksum.command = RESP_BAD_CHECKSUM;
  sendResponse(&badChecksum);  
}

void sendBadCommand()
{
  // Tell the Pi that we don't understand its
  // command sent to us.
  
  TPacket badCommand;
  badCommand.packetType=PACKET_TYPE_ERROR;
  badCommand.command=RESP_BAD_COMMAND;
  sendResponse(&badCommand);

}

void sendBadResponse()
{
  TPacket badResponse;
  badResponse.packetType = PACKET_TYPE_ERROR;
  badResponse.command = RESP_BAD_RESPONSE;
  sendResponse(&badResponse);
}

void sendOK()
{
  TPacket okPacket; // New struct
  okPacket.packetType = PACKET_TYPE_RESPONSE;
  okPacket.command = RESP_OK;
  sendResponse(&okPacket);  
}

void sendResponse(TPacket *packet)
{
  // Takes a packet, serializes it then sends it out
  // over the serial port.
  char buffer[PACKET_SIZE];
  int len = serialize(buffer, packet, sizeof(TPacket));
  writeSerial(buffer, len);
}


/*
 * Setup and start codes for external interrupts and 
 * pullup resistors.
 * 
 */
// Enable pull up resistors on pins 2 and 3
void enablePullups()
{
  // Use bare-metal to enable the pull-up resistors on pins
  // 2 and 3. These are pins PD2 and PD3 respectively.
  // We set bits 2 and 3 in DDRD to 0 to make them inputs. 
  DDRD &= 0b11110011;
  PIND |= 0b00001100;
  
}

// Functions to be called by INT0 and INT1 ISRs.S
void leftISR()
{
  if (dir == FORWARD) {
    leftForwardTicks++;
    forwardDist = (unsigned long) ((float) leftForwardTicks / COUNTS_PER_REV * WHEEL_CIRC);
  }
  else if (dir == BACKWARDS) {
    leftReverseTicks++;
    reverseDist = (unsigned long) ((float) leftReverseTicks / COUNTS_PER_REV * WHEEL_CIRC);
  }
  else if (dir == LEFT) {
    leftReverseTicksTurns++;
  }
  else if (dir == RIGHT) {
    leftForwardTicksTurns++;
  }
}

void rightISR()
{
  if (dir == FORWARD) {
    rightForwardTicks++;
  }
  else if (dir == BACKWARDS) {
    rightReverseTicks++;
  }
  else if (dir == LEFT) {
    rightForwardTicksTurns++;
  }
  else if (dir == RIGHT) {
    rightReverseTicksTurns++;
  }
}

// Set up the external interrupt pins INT0 and INT1
// for falling edge triggered. Use bare-metal.
void setupEINT()
{
  // Use bare-metal to configure pins 2 and 3 to be
  // falling edge triggered. Remember to enable
  // the INT0 and INT1 interrupts.
  EICRA |= 0b00001010;
  EIMSK |= 0b00000011;
}

// Implement the external interrupt ISRs below.
// INT0 ISR should call leftISR while INT1 ISR
// should call rightISR.
ISR(INT0_vect) {
  leftISR();
}
ISR(INT1_vect) {
  rightISR();
}

// Implement INT0 and INT1 ISRs above.

/*
 * Setup and start codes for serial communications
 * 
 */

void setupSerial()
{
  //set Baud rate to 9600
  UBRR0H = 0;
  UBRR0L = 103;

  UCSR0C = 0b00000110;
  UCSR0A = 0;
}

void startSerial()
{
  //start UART
  UCSR0B = 0b00011000;
}

// Read the serial port. Returns the read character in
// ch if available. Also returns TRUE if ch is valid. 

int readSerial(char *buffer)
{

  int count=0;
  while(UCSR0A & 0b10000000)
  {
    unsigned char data = UDR0;
    buffer[count++] = data;
  }


  return count;
}


// Write to the serial port.
void writeSerial(const char *buffer, int len)
{
  int count = 0;
  while(count < len)
  {
    while(!(UCSR0A & 0b00100000));
    UDR0 = buffer[count];
    count++;
  }
}

/*
 * Alex's motor drivers.
 * 
 */

// Set up Alex's motors. Right now this is empty, but
// later you will replace it with code to set up the PWMs
// to drive the motors.
void setupMotors() {
    
  TCNT0 = 0; // Timer 0
  TCNT1 = 0; // Timer 1
  TCNT2 = 0; // Timer 2
    
  DDRD |= (LF | LR); // set pins as output
  DDRB |= (RF | RR); // set pins as output

  // setup timers
  TCCR0A |= (1 << WGM00); //toggle OC0A, OC0B on match, phase correct PWM with WGM02
  OCR0A = 0;
  OCR0B = 0;
 

  // Set up Timer/Counter 1 (TCCR1A, TCCR1B, TIMSK1, OCR1A, OCR1B)
  TCCR1A |= (1 << WGM10);
  OCR1A = 0;
  OCR1B = 0;
  

  // Set up Timer/Counter 2 (TCCR2A, TCCR2B, TIMSK2, OCR2A, OCR2B)
  TCCR2A |= (1 << WGM20);
  OCR2A = 0;
  OCR2B = 0;
 

}

void startMotors() {
  TCCR0B |= 0b00000011; // start Timer/Counter 0
  TCCR1B |= 0b00000011; // start Timer/Counter 1
  TCCR2B |= 0b00000100; // start Timer/Counter 2
}





// Convert percentages to PWM values
int pwmVal(float speed)
{
  if(speed < 0.0)
    speed = 0;

  if(speed > 100.0)
    speed = 100.0;

  return (int) ((speed / 100.0) * 255.0);
}

// Move Alex forward "dist" cm at speed "speed".
// "speed" is expressed as a percentage. E.g. 50 is
// move forward at half speed.
// Specifying a distance of 0 means Alex will
// continue moving forward indefinitely.
void forward(float dist, float speed) {
  // Set distance to move
  if (dist > 0) {
    deltaDist = dist;
  } else {
    deltaDist = 9999999;
  }
  newDist = forwardDist + deltaDist;
  dir = FORWARD;

  // Set PWM values for motor control
  int val = pwmVal(speed);
  OCR0B = val; // Left forward
  TCCR0A = 0b00100001;
  OCR1B = val; // Right forward
  TCCR1A = 0b00100001;
 

}

// Reverse Alex "dist" cm at speed "speed".
// "speed" is expressed as a percentage. E.g. 50 is
// reverse at half speed.
// Specifying a distance of 0 means Alex will
// continue reversing indefinitely.
void reverse(float dist, float speed) {
  // Set distance to move
  if (dist > 0) {
    deltaDist = dist;
  } else {
    deltaDist = 9999999;
  }
  newDist = reverseDist + deltaDist;
  dir = BACKWARDS;

  // Set PWM values for motor control
  int val = pwmVal(speed);
  OCR0A = val; // Left reverse
  TCCR0A = 0b10000001;
  OCR2A = val; // Right reverse
  TCCR2A = 0b10000001;

}





// New function to estimate number of wheel icks
// needed to turn an angle
unsigned long computeDeltaTicks(float ang)
{
  // We will assume that an angular distance moved = linear distance moved in one wheel
  // revolution. This is (probably) uincorrect but simplifies calculation/.
  // # of wheel revs t0o make one full 360 turn is AlexCirc / WHEEL_CIRC
  // This is for 360 degrees. For ang degrees it will be (ang * AlexCirc) / (360 * WHEEL_CIRC)
  // To convert to ticks, we multiply by COUNTS_PER_REV.

  unsigned long ticks = (unsigned long) ((ang * AlexCirc * COUNTS_PER_REV) / (360.0 * WHEEL_CIRC));
  return ticks;
}

// Turn Alex left "ang" degrees at speed "speed".
// "speed" is expressed as a percentage. E.g. 50 is
// turn left at half speed.
// Specifying an angle of 0 degrees will cause Alex to
// turn left indefinitely.
void right(float ang, float speed) {
  // Set direction
  dir = RIGHT;

  // Set PWM value for motor control
  int val = pwmVal(speed);

  // Set target number of encoder ticks to reach
  if (ang == 0) {
    deltaTicks = 99999999;
  } else {
    deltaTicks = computeDeltaTicks(ang);
  }
  targetTicks = rightReverseTicksTurns + deltaTicks;

  // Set PWM values for motor control
  OCR2A = val; // Right reverse
  TCCR2A = 0b10000001;
  OCR0B = val; // Left forward
  TCCR0A = 0b00100001;


}

// Turn Alex right "ang" degrees at speed "speed".
// "speed" is expressed as a percentage. E.g. 50 is
// turn left at half speed.
// Specifying an angle of 0 degrees will cause Alex to
// turn right indefinitely.
void left(float ang, float speed) {
  // Set direction and PWM value for motor control
  dir = LEFT;
  int val = pwmVal(speed);

  // Compute target ticks for turn
  if (ang == 0) {
    deltaTicks = 99999999;
  } else {
    deltaTicks = computeDeltaTicks(ang);
  }
  targetTicks = leftReverseTicksTurns + deltaTicks;

  // Set PWM values for motor control
  OCR0A = val; // Left reverse
  TCCR0A = 0b10000001;
  OCR1B = val; // Right forward
  TCCR1A = 0b00100001;



}

// Stop Alex. To replace with bare-metal code later.
void stop()
{
  dir = STOP;
  TCCR0A = 0b00000001;
  TCCR1A = 0b00000001;
  TCCR2A = 0b00000001;
}

/*
 * Alex's setup and run codes
 * 
 */

// Clears all our counters
void clearCounters()
{
  leftForwardTicks=0;
  rightForwardTicks=0;
  leftReverseTicks=0;
  rightReverseTicks=0;
  leftForwardTicksTurns=0;
  rightForwardTicksTurns=0;
  leftReverseTicksTurns=0;
  rightReverseTicksTurns=0;
  leftRevs=0;
  rightRevs=0;
  forwardDist=0;
  reverseDist=0; 
}

// Clears one particular counter
void clearOneCounter(int which)
{
  clearCounters();
}
// Intialize Vincet's internal states

void initializeState()
{
  clearCounters();
}

void handleCommand(TPacket *command)
{
  switch(command->command)
  {
    // For movement commands, param[0] = distance, param[1] = speed.
    case COMMAND_FORWARD:
      sendOK();
      forward((float) command->params[0], (float) command->params[1]);
      break;
    /*
     * Implement code for other commands here.
     * 
     */
    case COMMAND_STOP:
      sendOK();
      stop();
      break;
    case COMMAND_REVERSE:
      sendOK();
      reverse((float) command->params[0], (float) command->params[1]);
      break;
    case COMMAND_TURN_RIGHT:
      sendOK();
      right((float) command->params[0], (float) command->params[1]);
      break;
    case COMMAND_TURN_LEFT:
      sendOK();
      left((float) command->params[0], (float) command->params[1]);
      break;
    case COMMAND_CLEAR_STATS:
      clearOneCounter(command->params[0]);
      sendOK();
      break;
    case COMMAND_GET_STATS:
      sendStatus();
      break;
    case COMMAND_IDENTIFY_COLOUR:
      sendColourStats();
      break;
    default:
      sendBadCommand();
  }
}

void waitForHello()
{
  int exit=0;

  while(!exit)
  {
    TPacket hello;
    TResult result;
    
    do
    {
      result = readPacket(&hello);
    } while (result == PACKET_INCOMPLETE);

    if(result == PACKET_OK)
    {
      if(hello.packetType == PACKET_TYPE_HELLO)
      {
     

        sendOK();
        exit=1;
      }
      else
        sendBadResponse();
    }
    else
      if(result == PACKET_BAD)
      {
        sendBadPacket();
      }
      else
        if(result == PACKET_CHECKSUM_BAD)
          sendBadChecksum();
  } // !exit
}

void setup() {
  // put your setup code here, to run once:
  // Compute the diagonal
  AlexDiagonal = sqrt((ALEX_LENGTH * ALEX_LENGTH) + (ALEX_BREADTH *
  ALEX_BREADTH));
  AlexCirc = PI * AlexDiagonal;
  cli();
  ADCSRA = 0; // disable ADC
  PRR = 0b00000001; // shutdown ADC
  setup_colour_sensor();
  setupEINT();
  setupSerial();
  startSerial();
  setupMotors();
  startMotors();
  enablePullups();
  initializeState();
  sei();
}

void handlePacket(TPacket *packet)
{
  switch(packet->packetType)
  {
    case PACKET_TYPE_COMMAND:
      handleCommand(packet);
      break;

    case PACKET_TYPE_RESPONSE:
      break;

    case PACKET_TYPE_ERROR:
      break;

    case PACKET_TYPE_MESSAGE:
      break;

    case PACKET_TYPE_HELLO:
      break;
  }
}

void loop() {

  // Uncomment the code below for Step 2 of Activity 3 in Week 8 Studio 2

  // forward(0, 100);

  // Uncomment the code below for Week 9 Studio 2

  // put your main code here, to run repeatedly:
  TPacket recvPacket; // This holds commands from the Pi

  TResult result = readPacket(&recvPacket);

  if(result == PACKET_OK)
    handlePacket(&recvPacket);
  else
    if(result == PACKET_BAD) {
      sendBadPacket();
    }
    else
      if(result == PACKET_CHECKSUM_BAD) {
        sendBadChecksum();
      } 

  if(deltaDist > 0)
  {
    if(dir == FORWARD) {
      if(forwardDist > newDist) {
        deltaDist=0;
        newDist=0;
        stop();
      }
    } else if (dir == BACKWARDS) {
        if(reverseDist > newDist) {
          deltaDist=0;
          newDist=0;
          stop();
        }
      } else if (dir == STOP) {
        deltaDist=0;
        newDist=0;
        stop();
      }
  }
  if(deltaTicks > 0) {
    if(dir == LEFT) {
      if(leftReverseTicksTurns >= targetTicks) {
        deltaTicks = 0;
        targetTicks = 0;
        stop();
      }
    } else if (dir == RIGHT) {
      if(rightReverseTicksTurns >= targetTicks) {
        deltaTicks = 0;
        targetTicks = 0;
        stop();
      }
    } else if (dir == STOP) {
        deltaTicks = 0;
        targetTicks = 0;
        stop();
    }
  }
}
