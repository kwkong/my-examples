#include <Arduino.h>
#include <LiquidCrystal_I2C.h>
#include <SoftwareSerial.h>
#include <RoboClaw.h>

//-------Pin Definitions------//
#define buzzPin 12
#define buttonEncoder 10
#define DT 9
#define CLK 8
#define increasePin 7
#define decreasePin 6
#define startPin 5
#define softRx 2
#define softTx 3
#define limitSwitchPin A0
#define pressurePin A1
#define lcdSDA A4
#define lcdSCL A5

//-------Roboclaw values------//
#define address 0x80
#define kp_fp 90.49
#define ki_fp 1.71
#define kd_fp 512.79
#define kiMax 80
#define deadzone 0
#define minimum 10
#define maximum 12000

//------Other definitions------//
#define toTarget 0
#define toReturn 1

LiquidCrystal_I2C lcd(0x3F,16,2);  // set the LCD address to 0x27 for a 16 chars and 2 line display
SoftwareSerial mySerial(softRx, softTx);
RoboClaw roboclaw(&mySerial, 10000);

//-------GLobal variables-------//
int mode, moveDirection, start = 0;
int startPos, endPos = 0;
int encBotVal, encTopVal = 0;
int homePosition = 265;
int ieRatio = 1, respRate = 100, tidalVol = 100;
int ieRatioOld = 1, respRateOld = 100, tidalVolOld = 100;
float maxPressure = 0, maxPressureOld = 0;
int targetPosition = 0;

int currentStateCLK;
int lastStateCLK;
unsigned long lastButtonPress = 0;

unsigned long pressTime = 0;
unsigned long blinkTime = 0;

//-------Function definitions--------//

void armHome();
int getPosition();
void moveUpAndDown(int _tidalVol, int _ieRatio, int _respRate);
void printValues();
float readPressure();
float readEnconder(float _counter, float _stepSize);
void displayVals();
void buzzerBeepOnce();

void setup()
{
    pinMode(CLK,INPUT);
	pinMode(DT,INPUT);
	pinMode(buttonEncoder, INPUT_PULLUP);
    lastStateCLK = digitalRead(CLK);

    pinMode(increasePin, INPUT_PULLUP);
    pinMode(decreasePin, INPUT_PULLUP);
    pinMode(startPin, INPUT_PULLUP);
    pinMode(limitSwitchPin, INPUT_PULLUP);
    pinMode(buzzPin, OUTPUT);
    pinMode(13, OUTPUT);

    Serial.begin(115200);
    roboclaw.begin(38400);

    lcd.init();                      // initialize the lcd 
    lcd.backlight();
    displayVals();

    Serial.println("Starting...");
    roboclaw.ForwardM1(address,0); //stop motor

    Serial.print("Initial Position: ");
    Serial.println(getPosition());

    buzzerBeepOnce();
    buzzerBeepOnce();

    armHome();

}

void loop()
{
    while(start == 1)   //loop here once the start button in pressed
    {
        if(!digitalRead(buttonEncoder))  //stop if the start button is pressed again
        {
            roboclaw.ForwardM1(address,0);
            digitalWrite(13, LOW);
            start = 0;
            while(!digitalRead(buttonEncoder));
            buzzerBeepOnce();
            break;
        }

        if(readPressure() > maxPressure)        //if pressure limit is exceeded
        {
            digitalWrite(13, LOW);
            start = 0;

            while(getPosition() > 0)
            {
                roboclaw.SpeedAccelDeccelPositionM1(address, 0, 1000, 0, 0, 1);
            }
            digitalWrite(buzzPin, HIGH);
            delay(2000);
            digitalWrite(buzzPin, LOW);
            break;
        }

        moveUpAndDown(tidalVol, ieRatio, respRate); 
        Serial.println(readPressure());
    }

    if(!digitalRead(buttonEncoder))  //if the encoder button is pressed
    {
        pressTime = millis();
        while(!digitalRead(buttonEncoder) && ((millis() - pressTime) < 1500));   //hold timeout of 1500ms

        if((millis() - pressTime) < 1000)   //short press
        {
            Serial.println("SHORT PRESS");
            while(!digitalRead(buttonEncoder));
            buzzerBeepOnce();
            start = 1;
            digitalWrite(13, HIGH);
            delay(200);
        }
        else    //long press
        {
            Serial.println("LONG PRESS");
            buzzerBeepOnce();
            while(!digitalRead(buttonEncoder))
            {
                if(millis()-blinkTime > 1000)
                {
                    lcd.setCursor(3,1);
                    lcd.print(" ");
                    blinkTime = millis();
                }
                else if(millis()-blinkTime > 500)
                {
                    lcd.setCursor(3,1);
                    lcd.print("<");
                }
            }

            mode = 0;
            blinkTime = millis();
            while(mode < 4)
            {
                if(mode == 0)
                {
                    Serial.println("Change ieRatio");
                    ieRatio = readEnconder(ieRatio, 1);

                    if(millis()-blinkTime > 1000)
                    {
                        lcd.setCursor(3,1);
                        lcd.print(" ");
                        blinkTime = millis();
                    }
                    else if(millis()-blinkTime > 500)
                    {
                        lcd.setCursor(3,1);
                        lcd.print("<");
                    }

                    if(ieRatio > 4)
                    {
                        ieRatio = 4;
                    }
                    else if(ieRatio < 1)
                    {
                        ieRatio = 1;
                    }
                }
                else if(mode == 1)
                {
                    Serial.println("Change respRate");
                    respRate = readEnconder(respRate, 5);

                    if(millis()-blinkTime > 1000)
                    {
                        lcd.setCursor(9,1);
                        lcd.print(" ");
                        blinkTime = millis();
                    }
                    else if(millis()-blinkTime > 500)
                    {
                        lcd.setCursor(9,1);
                        lcd.print("<");
                    }

                    if(respRate > 100)
                    {
                        respRate = 100;
                    }
                    else if(respRate < 5)
                    {
                        respRate = 5;
                    }
                }
                else if(mode == 2)
                {
                    Serial.println("Change tidalVol");
                    tidalVol = readEnconder(tidalVol, 5);

                    if(millis()-blinkTime > 1000)
                    {
                        lcd.setCursor(15,1);
                        lcd.print(" ");
                        blinkTime = millis();
                    }
                    else if(millis()-blinkTime > 500)
                    {
                        lcd.setCursor(15,1);
                        lcd.print("<");
                    }

                    if(tidalVol > 100)
                    {
                        tidalVol = 100;
                    }
                    else if(tidalVol < 5)
                    {
                        tidalVol = 5;
                    }
                }
                else if(mode == 3)
                {
                    lcd.clear();
                    lcd.setCursor(0,0);
                    lcd.print("Max pressure:");
                    lcd.setCursor(0,1);
                    lcd.print(maxPressure);

                    while(1)
                    {
                        maxPressure = readEnconder(maxPressure, 20);

                        lcd.setCursor(10,1);
                        lcd.print(readPressure());

                        if(maxPressure != maxPressureOld)
                        {
                            maxPressureOld = maxPressure;
                            lcd.clear();
                            lcd.setCursor(0,0);
                            lcd.print("Max pressure:");
                            lcd.setCursor(0,1);
                            lcd.print(maxPressure);
                        }

                        if(!digitalRead(buttonEncoder))
                        {
                            mode = 4;   //this will make it break from the mode loop
                            break;
                        }
                    }
                }

                if(!digitalRead(buttonEncoder))
                {
                    while(!digitalRead(buttonEncoder));
                    buzzerBeepOnce();
                    displayVals();
                    mode++;
                }

                if((ieRatio != ieRatioOld) || (respRate != respRateOld) || (tidalVol != tidalVolOld))
                {
                    displayVals();

                    ieRatioOld = ieRatio;
                    respRateOld = respRate;
                    tidalVolOld = tidalVol;
                } 
            }
        }
    }

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

//moves the arm up and down, arg1 is the target position to move up to, arg2 is the the speed to get to that position
void moveUpAndDown(int _tidalVol, int _ieRatio, int _respRate)
{
    int iSpeed, eSpeed, targetPos = 0;

    targetPos = map(_tidalVol, 5, 100, 600, 900);
    iSpeed = map(_respRate, 5, 100, 500, 4000);

    eSpeed = iSpeed / _ieRatio;

    if (moveDirection == toTarget)
    {
        roboclaw.SpeedAccelDeccelPositionM1(address, 0, iSpeed, 0, targetPos, 0);    //move to target 
    
        if((getPosition() > startPos+1) || (getPosition() < startPos-1))    //move to target +/- 1
        {
            moveDirection = toReturn;     //change direction
        }
    }
    else if (moveDirection == toReturn)
    {
        roboclaw.SpeedAccelDeccelPositionM1(address, 0, eSpeed, 0, 0, 0);    //go back down
    
        if((getPosition() > endPos+1) || (getPosition() < endPos-1))
        {
            moveDirection = toTarget;   //change direction
        }
    }
}

//returns the pressure from the pressure sensor
float readPressure()
{
    float pressure = 0;

    pressure = analogRead(pressurePin);

    pressure -= 260;
    pressure *= 0.5;

    

    // pressure *= 0.0052;
    // pressure += -1.3549;

    return pressure;
}

//reads rotary encoder and increments or decrements the argument based on rotation
float readEnconder(float _counter, float _stepSize)
{
    // Read the current state of CLK
    currentStateCLK = digitalRead(CLK);

    // If last and current state of CLK are different, then pulse occurred
    // React to only 1 state change to avoid double count
    if (currentStateCLK != lastStateCLK  && currentStateCLK == 1)
    {
        // If the DT state is different than the CLK state then
        // the encoder is rotating CCW so decrement
        if (digitalRead(DT) != currentStateCLK) 
        {
            _counter += _stepSize;
        } 
        else 
        {
            // Encoder is rotating CW so increment
            _counter -=_stepSize;
        }

        buzzerBeepOnce();
    }

    // Remember last CLK state
    lastStateCLK = currentStateCLK;

    return _counter;
}

void displayVals()
{
    lcd.clear();

    lcd.setCursor(0,0);
    lcd.print("I/E:");
    lcd.setCursor(0,1);
    lcd.print(ieRatio);

    lcd.setCursor(5,0);
    lcd.print("Rate:");
    lcd.setCursor(5,1);
    lcd.print(respRate);

    if(respRate == 100)
    {
        lcd.setCursor(8,1);
        lcd.print("%");
    }
    else if(respRate >= 10)
    {
        lcd.setCursor(7,1);
        lcd.print("%");
    }
    else if(respRate < 10)
    {
        lcd.setCursor(6,1);
        lcd.print("%");
    }

    lcd.setCursor(11,0);
    lcd.print("Vol:");
    lcd.setCursor(11,1);
    lcd.print(tidalVol);

    if(tidalVol == 100)
    {
        lcd.setCursor(14,1);
        lcd.print("%");
    }
    else if(tidalVol >= 10)
    {
        lcd.setCursor(13,1);
        lcd.print("%");
    }
    else if(tidalVol < 10)
    {
        lcd.setCursor(12,1);
        lcd.print("%");
    }
}

void buzzerBeepOnce()
{
    // digitalWrite(buzzPin, HIGH);
    // delay(50);
    // digitalWrite(buzzPin, LOW);
    // delay(50);
}