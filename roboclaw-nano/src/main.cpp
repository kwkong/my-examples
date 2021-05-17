#include <Arduino.h>
#include <RoboClaw.h>

//---Roboclaw values---//
#define address 0x80
#define kp_fp 90.49
#define ki_fp 1.71
#define kd_fp 512.79
#define kiMax 80
#define deadzone 0
#define minimum 10
#define maximum 12000

#define roboclaw_RX 2
#define roboclaw_TX 3

SoftwareSerial roboclawSerial(roboclaw_RX, roboclaw_TX);
RoboClaw roboclaw(&roboclawSerial, 10000);

//---GLobal variables--//

//---Function definitions---//


void setup()
{
    Serial.begin(115200);

    roboclaw.begin(38400);  //Standard roboclaw baudrate

    roboclaw.ForwardM1(address,0); //stop motor
}

void loop()
{
    // if (!digitalRead(increasePin))  //move up when you press the increase pin
    // {
    //     roboclaw.SpeedAccelDeccelPositionM1(address, 0, 500, 0, (getPosition()+50), 0);
    //     delay(200);
    // }

    // if (!digitalRead(decreasePin))  //move down when you press the decrease pin
    // {
    //     roboclaw.SpeedAccelDeccelPositionM1(address, 0, 500, 0, (getPosition()-50), 0);
    //     delay(200);
    // }
}

//returns the current encoder position
int getPosition()
{
    uint8_t status1;
    bool valid1;
    int enc1 = roboclaw.ReadEncM1(address, &status1, &valid1);

    if(valid1)
    {
        return enc1;
    }
    else 
    {
        return 99;
    }
}

//brings the arm to the home position based on the limit switch
void armHome()
{
    while(digitalRead(limitSwitchPin))      //while you're above the home position, move down
    {
        roboclaw.SpeedAccelDeccelPositionM1(address, 0, 5000, 0, (getPosition()-2), 0);     //slowly decrease position as you approach home
        delay(1);
        Serial.println("HOMING DOWN");
    }
    
    roboclaw.ForwardM1(address,0);  //stop motor
    roboclaw.SetEncM1(address, 0);  //set encoder value to 0
    buzzerBeepOnce();
}