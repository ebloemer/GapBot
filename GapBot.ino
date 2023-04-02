/*

 MSE 2202 GapBot
 Language: Arduino
 Authors: Eugen Porter and Michael Naish
 
 */

//  To program and use ESP32-S3
//   
//  File->Preferences:
//  Additional Boards Manager URLs: https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json
//
//
//  Tools->:
//  Board: "Adafruit Feather ESP32-S3 No PSRAM"
//  Upload Speed: "921600"
//  USB CDC On Boot: "Enabled"
//  USB Firmware MSC on Boot: "Disabled"
//  USB DFU On Bot: "Disabled"
//  Upload Mode:"UART0/Hardware CDC"
//  SPU Frequency: "240MHz (WiFi)"
//  Flash Mode: "QIO 80MHz"
//  Flash SIze: "4MB (32Mb)"
//  Partition Scheme: "Default 4MB with spiffs (1.2MB app/1.5MB SPIFFS)"
//  Core Debug Level: "Verbose"
//  PSRAM: 'Disabled"
//  Arduino Runs On: "Core 1"
//  Events Run On: "Core 1"
//

#include <Adafruit_NeoPixel.h>
#include <MSE2202_Lib.h>

// Uncomment keywords to enable debugging output
#define DEBUG_DRIVE_SPEED 1
#define DEBUG_ENCODER_COUNT 1

// Port pin constants

#define MODE_BUTTON         0   // GPIO0  pin 27 for Push Button 1
                                   
#define LEFT_DRIVE_MOTOR_A        35  // GPIO35 pin 28 (J35) Motor 1 A
#define LEFT_DRIVE_MOTOR_B        36  // GPIO36 pin 29 (J36) Motor 1 B
#define RIGHT_DRIVE_MOTOR_A       37  // GPIO37 pin 30 (J37) Motor 2 A
#define RIGHT_DRIVE_MOTOR_B       38  // GPIO38 pin 31 (J38) Motor 2 B
                                   
#define LEFT_ARM_MOTOR_A        6  // GPIO35 pin 28 (J35) Motor 1 A
#define LEFT_ARM_MOTOR_B        7  // GPIO36 pin 29 (J36) Motor 1 B
#define RIGHT_ARM_MOTOR_A       8  // GPIO37 pin 30 (J37) Motor 2 A
#define RIGHT_ARM_MOTOR_B       9  // GPIO38 pin 31 (J38) Motor 2 B

#define RIGHT_SERVO             41  // GPIO41 pin 34 (J41) Servo 1
#define LEFT_SERVO              42 // GPIO42 pin 35 (J42) Servo 2

#define MOTOR_ENABLE_SWITCH     46  // DIP Switch S1-5 pulls Digital pin D3 to ground when on, connected to pin 15 GPIO3 (J3)
                                // When DIP Switch S1-5 is off, J3 can be used as analog AD1-2
                                   
#define SMART_LED           21  // When DIP Switch S1-11 is on, Smart LED is connected to pin 23 GPIO21 (J21)
#define SMART_LED_COUNT     1   // Number of SMART LEDs in use

#define LIMIT_SWITCH        10  // J10 limit switch input

// Constants

const int ci_Display_Update = 50;                                            // Update interval for Smart LED in milliseconds

boolean bt_Motors_Enabled = true;                                             // Motors enabled flag
boolean bt_3_S_Time_Up = false;                                               // 3 second timer elapsed flag
boolean bt_2_S_Time_Up = false;                                               // 2 second timer elapsed flag
boolean bt_200_mS_Time_Up = false; 

unsigned char uc_Drive_Index; 
unsigned int ui_Mode_PB_Debounce;                                             // Pushbutton debounce timer count

unsigned long ul_3_Second_timer = 0;                                          // 3 second timer count in milliseconds
unsigned long ul_2_Second_timer = 0;                                          // 2 second timer count in milliseconds
unsigned long ul_200_mS_timer = 0;                                            // 200 millisecond timer count in milliseconds
unsigned long ul_Display_Time;                                                // Heartbeat LED update timer
unsigned long ul_Previous_Micros;                                             // Last microsecond count
unsigned long ul_Current_Micros;

//=====================================================================================================================
//
// IMPORTANT: The constants in this section need to be set to appropriate values for your robot. 
// You will have to experiment to determine appropriate values.

const int rearExtendedRight = 1880;                                          // Value for open position of rear arm
const int rearExtendedLeft = 500;                                        // Value for closed position of rear arm
const int rearRetractedLeft = 1880;
const int rearRetractedRight = 500;
int rearPositionLeft, rearPositionRight;


//
//=====================================================================================================================

Adafruit_NeoPixel SmartLEDs(SMART_LED_COUNT, SMART_LED, NEO_RGB + NEO_KHZ800);

unsigned int  ui_Robot_Mode_Index = 0;

unsigned char LEDBrightnessIndex = 0; 
unsigned char LEDBrightnessLevels[] = {5,15,30,45,60,75,90,105,120,135,150,165,180,195,210,225,240,255,
                                       240,225,210,195,180,165,150,135,120,105,90,75,60,45,30,15};


unsigned int  ui_Mode_Indicator[7] = {                                        // Colours for different modes
  SmartLEDs.Color(255,0,0),                                                   //   Red - Stop
  SmartLEDs.Color(255,255,0),                                                 //   Yellow - Run
  SmartLEDs.Color(0,255,0),                                                   //   Green - Complete
  SmartLEDs.Color(0,0,255),                                                   //   Blue - Test stepper
  SmartLEDs.Color(255,0,255),                                                 //   Magenta - Test shoulder servo
  SmartLEDs.Color(0,255,255),                                                 //   Cyan - Test IR receiver
  SmartLEDs.Color(255,165,0)                                                  //   Orange - empty case
};

Motion Bot = Motion();

void Indicator();

void setup()
{
   Serial.begin(9600);

   // Set up motors and encoders
   Bot.driveBegin("D1", LEFT_DRIVE_MOTOR_A, LEFT_DRIVE_MOTOR_B, RIGHT_DRIVE_MOTOR_A, RIGHT_DRIVE_MOTOR_B); // Set up motors as Drive 1
   Bot.servoBegin("S1",RIGHT_SERVO);   
   Bot.servoBegin("S2",LEFT_SERVO);

   // Set up SmartLED
   SmartLEDs.begin();                                                         // Initialize smart LEDs object (REQUIRED)
   SmartLEDs.clear();                                                         // Clear pixel
   SmartLEDs.setPixelColor(0,SmartLEDs.Color(0,0,0));                         // Set pixel colors to 'off'
   SmartLEDs.show();                                                          // Send the updated pixel colors to the hardware

   // Set up mode pushbutton
   pinMode(MOTOR_ENABLE_SWITCH, INPUT_PULLUP);                                // Set up motor enable switch with internal pullup
   ui_Mode_PB_Debounce = 0;                                                   // Reset debounce timer count
   

   pinMode(MODE_BUTTON, INPUT_PULLUP);                                        //set up mode button with internal pullup
   pinMode(LIMIT_SWITCH, INPUT_PULLUP);
  //  pinMode(LEFT_DRIVE_MOTOR_A, OUTPUT);
  //  pinMode(RIGHT_DRIVE_MOTOR_A, OUTPUT);
   pinMode(LEFT_ARM_MOTOR_A, OUTPUT);
   pinMode(RIGHT_ARM_MOTOR_A, OUTPUT);
   pinMode(LEFT_ARM_MOTOR_B, OUTPUT);
   pinMode(RIGHT_ARM_MOTOR_B, OUTPUT);
}

void loop()
{ 
   ul_Current_Micros = micros();                                              // Get current time in microseconds
   if ((ul_Current_Micros - ul_Previous_Micros) >= 1000)                      // Enter when 1 ms has elapsed
   {
      ul_Previous_Micros = ul_Current_Micros;                                 // Record current time in microseconds

      // 3 second timer, counts 3000 milliseconds
      ul_3_Second_timer = ul_3_Second_timer + 1;                              // Increment 3 second timer count
      if(ul_3_Second_timer > 3000)                                            // If 3 seconds have elapsed
      {
         ul_3_Second_timer = 0;                                               // Reset 3 second timer count
         bt_3_S_Time_Up = true;                                               // Indicate that 3 seconds have elapsed
      }
   
      // 2 second timer, counts 2000 milliseconds
      ul_2_Second_timer = ul_2_Second_timer + 1;                              // Increment 2 second timer count
      if(ul_2_Second_timer > 2000)                                            // If 2 seconds have elapsed
      {
         ul_2_Second_timer = 0;                                               // Reset 2 second timer count
         bt_2_S_Time_Up = true;                                               // Indicate that 2 seconds have elapsed
      }
   
      // 200 millisecond timer, counts 200 milliseconds
      ul_200_mS_timer = ul_200_mS_timer + 1;                                  // Increment 200 millisecond timer count
      if(ul_200_mS_timer > 200)                                               // If 200 milliseconds have elapsed
      {
         ul_200_mS_timer = 0;                                                 // Reset 200 millisecond timer count
         bt_200_mS_Time_Up = true;                                            // Indicate that 200 milliseconds have elapsed
      }

      // Mode pushbutton debounce and toggle
      if(!digitalRead(MODE_BUTTON))                                           // If pushbutton GPIO goes LOW (nominal push)
      {
         // Start debounce
         if(ui_Mode_PB_Debounce <= 25)                                        // 25 millisecond debounce time
         {
            ui_Mode_PB_Debounce = ui_Mode_PB_Debounce + 1;                    // Increment debounce timer count
            if(ui_Mode_PB_Debounce > 25)                                      // If held for at least 25 mS
            {
               ui_Mode_PB_Debounce = 1000;                                    // Change debounce timer count to 1 second
            }
         }
         if(ui_Mode_PB_Debounce >= 1000)                                      // Maintain 1 second timer count until release
         {
            ui_Mode_PB_Debounce = 1000;
         }
      }
      else                                                                    // Pushbutton GPIO goes HIGH (nominal release)
      {
         if(ui_Mode_PB_Debounce <= 26)                                        // If release occurs within debounce interval
         {
            ui_Mode_PB_Debounce = 0;                                          // Reset debounce timer count
         }
         else
         {
            ui_Mode_PB_Debounce = ui_Mode_PB_Debounce + 1;                    // Increment debounce timer count
            if(ui_Mode_PB_Debounce >= 1025)                                   // If pushbutton was released for 25 mS
            {
               ui_Mode_PB_Debounce = 0;                                       // Reset debounce timer count
               ui_Robot_Mode_Index++;                                         // Switch to next mode
               if(ui_Robot_Mode_Index > 2){
                 ui_Robot_Mode_Index = 0;
               }                // Keep mode index between 0 and 2
               ul_3_Second_timer = 0;                                         // Reset 3 second timer count
               bt_3_S_Time_Up = false;                                        // Reset 3 second timer         
            }
         }
      }
  
      // check if drive motors should be powered
      bt_Motors_Enabled = !digitalRead(MOTOR_ENABLE_SWITCH);                  // If SW1-5 is on (low signal), then motors are enabled
    
      switch(ui_Robot_Mode_Index)
      {
         case 0: // Robot stopped
         {
            digitalWrite(RIGHT_ARM_MOTOR_A, LOW);
            digitalWrite(RIGHT_ARM_MOTOR_B, LOW);
            digitalWrite(LEFT_ARM_MOTOR_A, LOW);
            digitalWrite(LEFT_ARM_MOTOR_B, LOW);
            Bot.Stop("D1");                                                   // Stop Drive 1
            bt_2_S_Time_Up = false;                                           // Reset 2 second timer flag
            break;
         }

         case 1: //Robot goes
         {
          SmartLEDs.setBrightness(LEDBrightnessLevels[17]);
          Indicator();
          rearPositionLeft = rearRetractedLeft;
          rearPositionRight = rearRetractedRight;

          //Extend da chopper
          while((rearPositionLeft > rearExtendedLeft) || (rearPositionRight < rearExtendedRight)){
            if(rearPositionRight < rearExtendedRight){
              Bot.ToPosition("S1", rearPositionRight);
              rearPositionRight += 1;
            }
            if(rearPositionLeft > rearExtendedLeft){
              Bot.ToPosition("S2", rearPositionLeft);
              rearPositionLeft -= 1;
            }
            delay(1);
          }

          //drive until end of table
          Bot.Forward("D1",255);
          while(!digitalRead(LIMIT_SWITCH)){
              delay(1);
          }
          Bot.Stop("D1");

          //extend arm
          digitalWrite(LEFT_ARM_MOTOR_A, HIGH);
          digitalWrite(RIGHT_ARM_MOTOR_A, HIGH);
          digitalWrite(LEFT_ARM_MOTOR_B, HIGH);
          digitalWrite(RIGHT_ARM_MOTOR_B, HIGH);
          delay(20000);
          digitalWrite(LEFT_ARM_MOTOR_A, LOW);
          digitalWrite(RIGHT_ARM_MOTOR_A, LOW);
          digitalWrite(LEFT_ARM_MOTOR_B, LOW);
          digitalWrite(RIGHT_ARM_MOTOR_B, LOW);

          //Fold baby fold
          while((rearPositionLeft < rearRetractedLeft) || (rearPositionRight > rearRetractedRight)){
            if(rearPositionRight > rearRetractedRight){
              Bot.ToPosition("S1", rearPositionRight);
              rearPositionRight -= 1;
            }
            if(rearPositionLeft < rearRetractedLeft){
              Bot.ToPosition("S2", rearPositionLeft);
              rearPositionLeft += 1;
            }
            delay(1);
          }

          //drive baby drive
          Bot.Forward("D1", 255);
          delay(4000);
          Bot.Stop("D1");


          //throw it in reverse terry
          Bot.Reverse("D1",255);
          delay(800);
          Bot.Stop("D1");

          ui_Robot_Mode_Index = 2;

          break;
        }
        case 2: //Robot is done
        {
          if(!digitalRead(LIMIT_SWITCH)){
              ui_Robot_Mode_Index = 2;
          }
          else{
            ui_Robot_Mode_Index = 0;
          }
        }   
      } 

    ul_Display_Time++;                                                      // Count milliseconds
    if(ul_Display_Time > ci_Display_Update)                                 // When display update period has passed
    {
        ul_Display_Time = 0;
        if(ui_Robot_Mode_Index != 1){                                                 // Reset display counter
          LEDBrightnessIndex++;                                                // Shift to next brightness level
          if(LEDBrightnessIndex > sizeof(LEDBrightnessLevels))                 // If all defined levels have been used
          {
            LEDBrightnessIndex = 0;                                           // Reset to starting brightness
          }
        }else{
          LEDBrightnessIndex = 17;
        }
        SmartLEDs.setBrightness(LEDBrightnessLevels[LEDBrightnessIndex]);    // Set brightness of heartbeat LED
        Indicator();                                                         // Update LED
    }
  }
}

// Set colour of Smart LED depending on robot mode (and update brightness)
void Indicator()
{
  SmartLEDs.setPixelColor(0, ui_Mode_Indicator[ui_Robot_Mode_Index]);         // Set pixel colors to = mode 
  SmartLEDs.show();                                                           // Send the updated pixel colors to the hardware
}

