/*******************************************************************************
* Copyright 2017 ROBOTIS CO., LTD.
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*******************************************************************************/

/* Author: Ryu Woon Jung (Leon) */
/* Editor: Jimmy Hu             */
// Note: Read Dynamixel motor present position only

//
// *********     Protocol Combined Example      *********
//
//
// Available Dynamixel model on this example : All models using Protocol 1.0 and 2.0
// This example is tested with a Dynamixel MX-28, a Dynamixel PRO 54-200 and an USB2DYNAMIXEL
// Be sure that properties of Dynamixel MX and PRO are already set as %% MX - ID : 1 / Baudnum : 34 (Baudrate : 57600) , PRO - ID : 1 / Baudnum : 1 (Baudrate : 57600)
//

// Be aware that:
// This example configures two different control tables (especially, if it uses Dynamixel and Dynamixel PRO). It may modify critical Dynamixel parameter on the control table, if Dynamixels have wrong ID.
//

#if defined(__linux__) || defined(__APPLE__)
#include <fcntl.h>
#include <termios.h>
#define STDIN_FILENO 0
#elif defined(_WIN32) || defined(_WIN64)
#include <conio.h>
#endif

#include <stdio.h>
#include <stdlib.h>
// To use time library of C
#include <time.h>
#include "dynamixel_sdk.h"                                  // Uses Dynamixel SDK library

// Control table address for Dynamixel MX
#define ADDR_MX_TORQUE_ENABLE           24                  // Control table address is different in Dynamixel model
#define ADDR_MX_GOAL_POSITION           30
#define ADDR_MX_PRESENT_POSITION        36

// Control table address for Dynamixel PRO
#define ADDR_PRO_TORQUE_ENABLE          24
#define ADDR_PRO_GOAL_POSITION          30
#define ADDR_PRO_PRESENT_POSITION       36

// Protocol version
#define PROTOCOL_VERSION1               1.0                 // See which protocol version is used in the Dynamixel
#define PROTOCOL_VERSION2               1.0

// Default setting
#define DXL1_ID                         2                   // Dynamixel#1 ID: 2
#define BAUDRATE                        1000000
#define DEVICENAME                      "/dev/ttyUSB0"      // Check which port is being used on your controller
                                                            // ex) Windows: "COM1"   Linux: "/dev/ttyUSB0" Mac: "/dev/tty.usbserial-*"

#define TORQUE_ENABLE                   0                   // Value for enabling the torque
#define TORQUE_DISABLE                  0                   // Value for disabling the torque
#define DXL1_MINIMUM_POSITION_VALUE     600                 // Dynamixel will rotate between this value
#define DXL1_MAXIMUM_POSITION_VALUE     800                 // and this value (note that the Dynamixel would not move when the position value is out of movable range. Check e-manual about the range of the Dynamixel you use.)
#define DXL2_MINIMUM_POSITION_VALUE     600
#define DXL2_MAXIMUM_POSITION_VALUE     800
#define DXL1_MOVING_SPEED               20                  // Dynamixel MX moving speed
#define DXL2_MOVING_SPEED               20                  // Dynamixel MX moving speed

#define ESC_ASCII_VALUE                 0x1b




// Function declaration
int getch(void);
int kbhit(void);
void delay(int);

int main(void)
{
  // Initialize PortHandler Structs
  // Set the port path
  // Get methods and members of PortHandlerLinux or PortHandlerWindows
  int port_num = portHandler(DEVICENAME);

  // Initialize PacketHandler Structs
  packetHandler();

  int index = 0;
  int dxl_comm_result = COMM_TX_FAIL;             // Communication result
  int dxl1_goal_position[2] = {DXL1_MINIMUM_POSITION_VALUE, DXL1_MAXIMUM_POSITION_VALUE};         // Goal position of Dynamixel MX
  int dxl2_goal_position[2] = {DXL2_MINIMUM_POSITION_VALUE, DXL2_MAXIMUM_POSITION_VALUE};         // Goal position of Dynamixel PRO

  uint8_t dxl_error = 0;                          // Dynamixel error
  uint16_t dxl1_present_position = 0;             // Present position of Dynamixel MX
  uint16_t dxl2_present_position = 0;             // Present position of Dynamixel MX

  // Open port
  if (openPort(port_num))
  {
    printf("Succeeded to open the port!\n");
  }
  else
  {
    printf("Failed to open the port!\n");
    printf("Press any key to terminate...\n");
    getch();
    return 0;
  }

  // Set port baudrate
  if (setBaudRate(port_num, BAUDRATE))
  {
    printf("Succeeded to change the baudrate!\n");
  }
  else
  {
    printf("Failed to change the baudrate!\n");
    printf("Press any key to terminate...\n");
    getch();
    return 0;
  }
  
  
  int loopnum = 0;
  for(loopnum = 0; loopnum < 100; loopnum++)
    // Read Dynamixel#1 present position
    while(dxl1_present_position == 0)
    {
        dxl1_present_position = read2ByteTxRx(port_num, PROTOCOL_VERSION1, DXL1_ID, ADDR_MX_PRESENT_POSITION);
        if ((dxl_comm_result = getLastTxRxResult(port_num, PROTOCOL_VERSION1)) != COMM_SUCCESS)
        {
          printf("%s\n", getTxRxResult(PROTOCOL_VERSION1, dxl_comm_result));
        }
        else if ((dxl_error = getLastRxPacketError(port_num, PROTOCOL_VERSION1)) != 0)
        {
          printf("%s\n", getRxPacketError(PROTOCOL_VERSION1, dxl_error));
        }
    }
    
    sleep(1);

  
  // Disable Dynamixel#1 Torque
  write1ByteTxRx(port_num, PROTOCOL_VERSION1, DXL1_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_DISABLE);
  if ((dxl_comm_result = getLastTxRxResult(port_num, PROTOCOL_VERSION1)) != COMM_SUCCESS)
  {
    printf("%s\n", getTxRxResult(PROTOCOL_VERSION1, dxl_comm_result));
  }
  else if ((dxl_error = getLastRxPacketError(port_num, PROTOCOL_VERSION1)) != 0)
  {
    printf("%s\n", getRxPacketError(PROTOCOL_VERSION1, dxl_error));
  }

  // Close port
  closePort(port_num);

  return 0;
}

int getch(void)
{
#if defined(__linux__) || defined(__APPLE__)
  struct termios oldt, newt;
  int ch;
  tcgetattr(STDIN_FILENO, &oldt);
  newt = oldt;
  newt.c_lflag &= ~(ICANON | ECHO);
  tcsetattr(STDIN_FILENO, TCSANOW, &newt);
  ch = getchar();
  tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
  return ch;
#elif defined(_WIN32) || defined(_WIN64)
  return _getch();
#endif
}


int kbhit(void)
{
#if defined(__linux__) || defined(__APPLE__)
  struct termios oldt, newt;
  int ch;
  int oldf;

  tcgetattr(STDIN_FILENO, &oldt);
  newt = oldt;
  newt.c_lflag &= ~(ICANON | ECHO);
  tcsetattr(STDIN_FILENO, TCSANOW, &newt);
  oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
  fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);

  ch = getchar();

  tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
  fcntl(STDIN_FILENO, F_SETFL, oldf);

  if (ch != EOF)
  {
    ungetc(ch, stdin);
    return 1;
  }

  return 0;
#elif defined(_WIN32) || defined(_WIN64)
  return _kbhit();
#endif
}

void delay(int milli_seconds)
{
    // Stroing start time
    clock_t start_time = clock();
 
    // looping till required time is not acheived
    while (clock() < start_time + milli_seconds)
        ;
}
