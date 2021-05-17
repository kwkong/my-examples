#include <Arduino.h>
#include <SoftwareSerial.h>
#include <RoboClaw.h>

//See limitations of Arduino SoftwareSerial
SoftwareSerial mySerial(2, 3); //RX, TX
RoboClaw roboclaw(&mySerial, 10000);

#define address 0x80

#define kp_fp 90.49
#define ki_fp 1.71
#define kd_fp 512.79
#define kiMax 80
#define deadzone 0
#define minimum 10
#define maximum 12000

#define increasePin 7
#define decreasePin 6
#define startPin 5

#define toStart 0
#define toEnd 1

unsigned long timeDelay = 0;
int mode, moving, movingOld, state = 0;
int startPos, endPos = 0;
int start = 0;

//Display Encoder and Speed for Motor 1

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


void setup()
{
    pinMode(increasePin, INPUT_PULLUP);
    pinMode(decreasePin, INPUT_PULLUP);
    pinMode(startPin, INPUT_PULLUP);

    pinMode(13, OUTPUT);

    //Open Serial and roboclaw at 38400bps
    Serial.begin(115200);

    roboclaw.begin(38400);

    Serial.println("Starting...");

    Serial.print("Initial Position: ");
    Serial.println(getPosition());
    //roboclaw.SetM1PositionPID(address, kp_fp, ki_fp, kd_fp, kiMax, deadzone, minimum, maximum);
}

void loop()
{
    //accel, speed, deccel, position, flag

    Serial.println("analogRead(A0)");
    delay(100);

    if (!digitalRead(increasePin))
    {
        roboclaw.SpeedAccelDeccelPositionM1(address, 0, 500, 0, (getPosition()+100), 0);
        delay(200);
    }

    if (!digitalRead(decreasePin))
    {
        roboclaw.SpeedAccelDeccelPositionM1(address, 0, 500, 0, (getPosition()-100), 0);
        delay(200);
    }

    if(!digitalRead(startPin))
    {
        while(!digitalRead(startPin));

        switch (mode)
        {
            case 0:
                startPos = getPosition();
                mode = 1;
                Serial.print("Start Position: ");
                Serial.println(getPosition());
                break;
            
            case 1:
                endPos = getPosition();
                mode = 2;
                Serial.print("End Position: ");
                Serial.println(getPosition());
                break;

            case 2:
                start = 1;
                mode = 3;
                Serial.println("START");
                break;

            case 3:
                start = 0;
                mode = 0;
                Serial.println("STOP");
                roboclaw.ForwardM1(address,0);
                break;

            default:
                break;
        }
    }

    if(start)
    {
        if (moving == toStart)
        {
            roboclaw.SpeedAccelDeccelPositionM1(address, 0, 1000, 0, startPos, 0);
        
            if((getPosition() > startPos+2) || (getPosition() < startPos-2))
            {
                moving = toEnd;     //change direction
                timeDelay = millis();
            }
        }
        else if (moving == toEnd)
        {
            roboclaw.SpeedAccelDeccelPositionM1(address, 0, 1000, 0, endPos, 0);
        
            if((getPosition() > endPos+2) || (getPosition() < endPos-2))
            {
                moving = toStart;   //change direction
                Serial.println(millis()-timeDelay);
            }
        } 
    }
    else
    {
        //roboclaw.ForwardM1(address,0);
        //state = 0;
    }
}