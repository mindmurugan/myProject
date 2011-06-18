/*
 * servoTest.cpp
 * Copyright (C) Gihun Bae gbae@purdue.edu
 *
 * servoTest.cpp is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * servoTest.cpp is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "oooarkavr/control/ServoManager.hpp"
#include "oooarkavr/systems/System.hpp"
#include "oooarkavr/control/Controller.hpp"
#include "oooarkavr/math/Matrix.hpp"
#include "oooarkavr/sensors/Em406.hpp"
#include "oooarkavr/sensors/Lea5h.hpp"
#include "oooarkavr/guidance/Guide.hpp"
#include <avr/interrupt.h>
#include "WProgram.h"

namespace oooarkavr
{
#define NUM_CHANNELS 8
#define MIN_PULSEWIDTH 900
#define MAX_PULSEWIDTH 2100



class APM_RC_Class: public System
{
public:
    int  controlPin;
    unsigned long time;
    volatile unsigned char radio_status;
    volatile unsigned int PWM_RAW[8];// = {2400,2400,2400,2400,2400,2400,2400,2400};
    Controller * controller;
    // Constructors ////////////////////////////////////////////////////////////////

    APM_RC_Class(int freq, int controlPin, Controller * controller): System(freq), controlPin(controlPin), time(0), radio_status(0), controller(controller)
    {
        for (int i =0; i<8; i++) PWM_RAW[i] = 2400;
    }
    void initialize()
    {
        // Init PWM Timer 1
        pinMode(11,OUTPUT);
        pinMode(12,OUTPUT);
        pinMode(13,OUTPUT);

        //Remember the registers not declared here remains zero by default...
        TCCR1A =((1<<WGM11)|(1<<COM1A1)|(1<<COM1B1)|(1<<COM1C1));
        //Please read page 131 of DataSheet, we are changing the registers settings of WGM11,COM1B1,COM1A1 to 1 thats all...
        TCCR1B = (1<<WGM13)|(1<<WGM12)|(1<<CS11); //Prescaler set to 8, that give us a resolution of 0.5us, read page 134 of data sheet
        OCR1A = 3000; //PB5, none
        OCR1B = 3000; //PB6, OUT2
        OCR1C = 3000; //PB7  OUT3
        ICR1 = 40000; //50hz freq...Datasheet says  (system_freq/prescaler)/target frequency. So (16000000hz/8)/50hz=40000,

        // Init PWM Timer 3
        pinMode(2,OUTPUT);
        pinMode(3,OUTPUT);
        pinMode(4,OUTPUT);
        TCCR3A =((1<<WGM31)|(1<<COM3A1)|(1<<COM3B1)|(1<<COM3C1));
        TCCR3B = (1<<WGM33)|(1<<WGM32)|(1<<CS31);
        OCR3A = 3000; //PE3, NONE
        OCR3B = 3000; //PE4, OUT7
        OCR3C = 3000; //PE5, OUT6
        ICR3 = 40000; //50hz freq

        // Init PWM Timer 4
        pinMode(6,OUTPUT);
        pinMode(7,OUTPUT);
        pinMode(8,OUTPUT);

        TCCR4A =((1<<WGM41)|(1<<COM4A1)|(1<<COM4B1)|(1<<COM4C1));
        TCCR4B = (1<<WGM43)|(1<<WGM42)|(1<<CS41);
        OCR4A = 3000; //PL3,
        OCR4B = 3000; //PL4, OUT0
        OCR4C = 3000; //PL4, OUT1
        ICR4 = 40000; //50hz freq

        // Init PPM input and PWM Timer 5
        pinMode(48, INPUT);  // ICP5 pin (PPM input)
        pinMode(44,OUTPUT);   // OCR5B
        pinMode(45,OUTPUT);   // OCR5C

        TCCR5A =((1<<WGM50)|(1<<WGM51)|(1<<COM5C1)|(1<<COM5B1)|(1<<COM5A1));
        //Prescaler set to 8, that give us a resolution of 0.5us
        // Input Capture rising edge
        TCCR5B = ((1<<WGM53)|(1<<WGM52)|(1<<CS51)|(1<<ICES5));

        OCR5A = 40000; ///50hz freq.
        OCR5B = 3000; //PH5, OUT5
        OCR5C = 3000; //PH5, OUT5

        //TCCR5B |=(1<<ICES5); //Changing edge detector (rising edge).
        //TCCR5B &=(~(1<<ICES5)); //Changing edge detector. (falling edge)
        TIMSK5 |= (1<<ICIE5); // Enable Input Capture interrupt. Timer interrupt mask
    }

    void run()
    {
        if (getState()==1)  // New radio frame? (we could use also if((millis()- timer) > 20)
        {
            time = millis();
            if (inputCh(controlPin)>1500)
            {

                outputCh(0,inputCh(0));
                outputCh(1,controller->getControl()(1));
                outputCh(2,inputCh(2));
                outputCh(3,inputCh(3));
                outputCh(4,inputCh(4));
                outputCh(5,inputCh(5));
                outputCh(6,inputCh(6));
                outputCh(7,inputCh(7));
            }
            else
            {
                for (int i=0;i<NUM_CHANNELS;i++)
                {
                    outputCh(i,inputCh(i)); // Copy input to Servos
                }
            }
        }
        else if (millis()-time > 500)
        {
            for (int i=0;i<NUM_CHANNELS;i++)
            {
                outputCh(i,1500); // Copy input to Servos
            }
        }
    }

    void outputCh(unsigned char ch, int pwm)
    {
        pwm=constrain(pwm,MIN_PULSEWIDTH,MAX_PULSEWIDTH);
        pwm<<=1;   // pwm*2;

        switch (ch)
        {
        case 0:
            OCR3B=pwm;
            break;  //pin 2
        case 1:
            OCR3C=pwm;
            break;  //pin 3
        case 2:
            OCR3A=pwm;
            break;  //pin 5, PE3
        case 3:
            OCR4A=pwm;
            break;  //pin 6
        case 4:
            OCR4B=pwm;
            break;  //pin 7
        case 5:
            OCR4C=pwm;
            break;  //pin 8
        case 6:
            OCR1A=pwm;
            break;  //pin 11, PB5
        case 7:
            OCR1B=pwm;
            break;  //pin 12
            //case 8:  OCR1C=pwm; break;  //pin 13
            //case 9:  OCR5B=pwm; break;  //pin 45
            //case 10: OCR5C=pwm; break;  //pin 44
            //case 11:  OCR5A=pwm; break;  //pin 46, PL3  //This register currently used for TOP of Timer 5
        }
    }

    int inputCh(unsigned char ch)
    {
        int result;
        int result2;

        // Because servo pulse variables are 16 bits and the interrupts are running values could be corrupted.
        // We dont want to stop interrupts to read radio channels so we have to do two readings to be sure that the value is correct...
        result =  PWM_RAW[ch]>>1;  // Because timer runs at 0.5us we need to do value/2
        result2 =  PWM_RAW[ch]>>1;
        if (result != result2)
            result =  PWM_RAW[ch]>>1;   // if the results are different we make a third reading (this should be fine)

        // Limit values to a valid range
        result = constrain(result,MIN_PULSEWIDTH,MAX_PULSEWIDTH);
        radio_status=0; // Radio channel read
        return(result);
    }

    unsigned char getState(void)
    {
        return(radio_status);
    }
};


class FeedbackController : public Controller
{
public:
    FeedbackController(int freq, Matrix<float> * feedbackGain, Vector<float> * state) : Controller(freq), state(state), feedbackGain(feedbackGain), steerLimit(25*deg2Rad)
    {

//control.setSize(feedbackGain->getRows());
        control.setSize(2);
    }
    virtual ~FeedbackController()
    {
    }
    void run()
    {

        float steerCommand = constrain(((*feedbackGain)*(*state)*(-1))(0), -steerLimit, steerLimit);
        control(1) = map(steerCommand*1e5,-steerLimit*1e5, steerLimit*1e5, 900, 2100);
        control(0) =1500;
        //state->print(Serial, "Error State: ");
        //control.print(Serial, "Control Commanded: ");
        //Serial.print("Steering Commanded: "), Serial.println(steerCommand);
    }

    //Class members
    Vector<float> * state;
    Matrix<float> * feedbackGain;
    float steerLimit;
};

class ImuUpdater : public System
{
public:
    ImuUpdater(int freq, Gps * gps, HardwareSerial & imuSerial): System(freq), imuComm(imuSerial), gps(gps) {}
    void run()
    {
        int32_t lat, lon, alt;
        Vector<float> eul(3), vNED(3);
        float c1, c2, c3, s1, s2, s3, q1, q2, q3, q4;

        lat = gps->getGeodetic().lat();
        lon = gps->getGeodetic().lon();
        alt = gps->getGeodetic().alt();
        vNED = gps->getVelNED();
        //eul= gps->getEuler();
        //c1=cos(eul(0));
        //c2=cos(eul(1));
        //c3=cos(eul(2));
        //s1=sin(eul(0));
        //s2=sin(eul(1));
        //s3=sin(eul(2));
        //q1=c1*c2*c3+s1*s2*s3;
        //q2=s1*c2*c3-c1*s2*s3;
        //q3=c1*s2*c3+s1*c2*s3;
        //q4=c1*c2*s3+s1*s2*c3;

        //lat = 1300;
        //lon = 1300;
        //alt = 1300;
        //vNED(0) = 13;
        //vNED(1) = 0;
        //vNED(2) = 0;

        imuComm.packLsb(setStateMessage);
        imuComm.packLsb(lat);
        imuComm.packLsb(lon);
        imuComm.packLsb(alt);
        imuComm.packLsb(vNED(0));
        imuComm.packLsb(vNED(1));
        imuComm.packLsb(vNED(2));
        imuComm.send();
    }
    Sirf imuComm;
    Gps * gps;
};

class PcLink : public System
{
public:
    PcLink(int freq, HardwareSerial & pcSerial, GeodeticCoord * pos, Vector<float> * vNED, Vector<float> * euler, Vector<float> * w_N, Vector<float> * errorState): System(freq), pcComm(pcSerial), pos(pos), vNED(vNED), euler(euler), w_N(w_N), errorState(errorState) {}
    void run()
    {
        // send  pc message
        pcComm.packLsb(pos->lat());
        pcComm.packLsb(pos->lon());
        pcComm.packLsb(pos->alt());
        pcComm.packLsb((*vNED)(0));
        pcComm.packLsb((*vNED)(1));
        pcComm.packLsb((*vNED)(2));
        pcComm.packLsb((*euler)(0));
        pcComm.packLsb((*euler)(1));
        pcComm.packLsb((*euler)(2));
        pcComm.packLsb((*errorState)(0));
        pcComm.packLsb((*errorState)(1));
        pcComm.packLsb((*errorState)(2));
        pcComm.packLsb((*errorState)(3));
        pcComm.send();
    }
    Sirf pcComm;
    GeodeticCoord * pos;
    Vector<float> * vNED;
    Vector<float> * euler;
    Vector<float> * w_N;
    Vector<float> * errorState;
};

class TestSystem : public System
{
public:
    TestSystem(int freq, APM_RC_Class * APM_RC, Vector<GeodeticCoord*> * flightPlan, GeodeticCoord * pos, Vector<float> * vNED, Vector<float> * euler, Vector<float> * w_N, Controller * controller, Guide * guide, Gps * gps, ImuUpdater * imuUpdater, PcLink * pcLink, HardwareSerial & imuComm) : System(freq), APM_RC(APM_RC), flightPlan(flightPlan), controller(controller), guide(guide), gps(gps), imuUpdater(imuUpdater), pcLink(pcLink), pos(pos), vNED(vNED), euler(euler), w_N(w_N), imuComm(imuComm)
    {
        addSubSystem(APM_RC);
        addSubSystem(controller);
        addSubSystem(gps);
        addSubSystem(guide);
        addSubSystem(imuUpdater);
        addSubSystem(pcLink);
    }
    virtual void run()
    {
        // receive imu message
        gps->print(Serial);
        gps->getCommProtocol()->printReceiveBuffer(Serial);
        if (imuComm.receive())
        {
            float vN, vE, vD, roll, pitch, yaw, wN, wE, wD;
            int32_t imuLat, imuLon, imuAlt;
            imuComm.unpackLsb(imuLat,0);
            imuComm.unpackLsb(imuLon,4);
            imuComm.unpackLsb(imuAlt,8);
            imuComm.unpackLsb(vN,12);
            imuComm.unpackLsb(vE,16);
            imuComm.unpackLsb(vD,20);
            imuComm.unpackLsb(roll,24);
            imuComm.unpackLsb(pitch,28);
            imuComm.unpackLsb(yaw,32);
            imuComm.unpackLsb(wN,36);
            imuComm.unpackLsb(wE,40);
            imuComm.unpackLsb(wD,44);

            //Use Gps
            //(*pos) = gps->getGeodetic();
            //(*vNED) = gps->getVelNED();
            //(*euler) = gps->getEuler();
            //if (((*euler)(2)) > PI) (*euler)(2) = -PI+((*euler)(2));

            //Use imu
            pos->set(imuLat, imuLon, imuAlt);
            (*vNED)(0) = vN;
            (*vNED)(1) = vE;
            (*vNED)(2) = vD;
            (*euler)(0) = roll;
            (*euler)(1) = pitch;
            (*euler)(2) = yaw;

            (*w_N)(0) = wN;
            (*w_N)(1) = wE;
            (*w_N)(2) = wD;

            Serial.println();
            Serial.println("USED NAV VALUES");
            Serial.print("Latitude(deg 1e7): "), Serial.println(pos->lat(), DEC);
            Serial.print("Longitude(deg 1e7): "), Serial.println(pos->lon(), DEC);
            Serial.print("Altitude MSL(m): "), Serial.println(pos->alt()/1e2, DEC);
            Serial.print("vN(m/s): "), Serial.println((*vNED)(0), DEC);
            Serial.print("vE(m/s): "), Serial.println((*vNED)(1), DEC);
            Serial.print("vD(m/s): "), Serial.println((*vNED)(2), DEC);
            Serial.print("Roll(deg): "), Serial.println((*euler)(0)*RAD_TO_DEG, DEC);
            Serial.print("Pitch(deg): "), Serial.println((*euler)(1)*RAD_TO_DEG, DEC);
            Serial.print("Yaw(deg): "), Serial.println((*euler)(2)*RAD_TO_DEG, DEC);
            Serial.println();
        }

        if (pcLink->pcComm.receive())
        {
            for (int i = 0; i < flightPlan->getSize(); i++)
            {
                Serial.print(i,DEC), Serial.print(": "), Serial.print((*flightPlan)(i)->lat(),DEC), Serial.print(" ");
                Serial.print((*flightPlan)(i)->lon(), DEC), Serial.print(" ");
                Serial.println((*flightPlan)(i)->alt(), DEC);
            }
            Serial.print("Wp list size: "), Serial.println((pcLink->pcComm.getReceiveSize()-2)/12.0, DEC);

            //lat, lon, alt combination is 12 bytes total
            flightPlan->setSize((pcLink->pcComm.getReceiveSize()-2)/12.0);
            for (int i = 0; i < ((pcLink->pcComm.getReceiveSize()-2)/12.0); i++)
            {
                int32_t lat, lon, alt;
                pcLink->pcComm.unpackLsb(lat,(i*12)+2);
                pcLink->pcComm.unpackLsb(lon,(i*12)+6);
                pcLink->pcComm.unpackLsb(alt,(i*12)+10);
                (*flightPlan)(i)->set(lat, lon, alt);
            }
            guide->setNextWpIndex(1);

            for (int i = 0; i < flightPlan->getSize(); i++)
            {
                Serial.print(i,DEC), Serial.print(": "), Serial.print((*flightPlan)(i)->lat(), DEC), Serial.print(" ");
                Serial.print((*flightPlan)(i)->lon(), DEC), Serial.print(" ");
                Serial.println((*flightPlan)(i)->alt(), DEC);
            }
        }

    }
private:
    Vector<GeodeticCoord*> * flightPlan;
    Controller * controller;
    Guide * guide;
    Gps * gps;
    ImuUpdater * imuUpdater;
    PcLink * pcLink;
    GeodeticCoord * pos;
    Vector<float> * vNED;
    Vector<float> * euler;
    Vector<float> * w_N;
    Sirf imuComm;
    APM_RC_Class * APM_RC;
};



} // namespace oooarkavr



using namespace oooarkavr;

// Feedback Controller setup
Matrix<float> feedbackGain(1,4);
Vector<float> errorState(4);
FeedbackController controller(50, &feedbackGain, &errorState);

// Guidance Setup
GeodeticCoord wp00(404304290,-869157680,18300);
GeodeticCoord wp01(404299100,-869152240,18300);
GeodeticCoord wp02(0,0,0);
GeodeticCoord wp03(0,0,0);
GeodeticCoord wp04(0,0,0);
GeodeticCoord wp05(0,0,0);
GeodeticCoord wp06(0,0,0);
GeodeticCoord wp07(0,0,0);
GeodeticCoord wp08(0,0,0);
GeodeticCoord wp09(0,0,0);
GeodeticCoord wp10(0,0,0);
GeodeticCoord wp11(0,0,0);
GeodeticCoord wp12(0,0,0);
GeodeticCoord wp13(0,0,0);
GeodeticCoord wp14(0,0,0);
GeodeticCoord wp15(0,0,0);
GeodeticCoord wp16(0,0,0);
GeodeticCoord wp17(0,0,0);
GeodeticCoord wp18(0,0,0);
GeodeticCoord wp19(0,0,0);
GeodeticCoord wp20(0,0,0);
GeodeticCoord wp21(0,0,0);
GeodeticCoord wp22(0,0,0);
GeodeticCoord wp23(0,0,0);
GeodeticCoord wp24(0,0,0);

Vector<GeodeticCoord*> flightPlan(size_t(0),size_t(30));
GeodeticCoord pos;
Vector<float> vNED(3);
Vector<float> euler(3);
Vector<float> wN(3);
Guide guide(50, &flightPlan, &errorState, &pos, &vNED, &euler, &wN);

// Subsystems setup
Em406 gps(2, Serial1);
ImuUpdater imuUpdater(1, &gps, Serial2);
PcLink pcLink(5, Serial3, &pos, &vNED, &euler, &wN, &errorState);

int controlPin = 4;
APM_RC_Class APM_RC(50, controlPin, &controller);

TestSystem ugv(13, &APM_RC, &flightPlan, &pos, &vNED, &euler, &wN, &controller, &guide, &gps, &imuUpdater, &pcLink, Serial2);

/****************************************************
   Input Capture Interrupt ICP5 => PPM signal read
 ****************************************************/
// Variable definition for Input Capture interrupt
volatile unsigned int ICR5_old;
volatile unsigned char PPM_Counter=0;
ISR(TIMER5_CAPT_vect)
{
    unsigned int Pulse;
    unsigned int Pulse_Width;

    Pulse=ICR5;
    if (Pulse<ICR5_old)     // Take care of the overflow of Timer4 (TOP=40000)
    {
        Pulse_Width=(Pulse + 40000)-ICR5_old;  //Calculating pulse
    }
    else
    {
        Pulse_Width=Pulse-ICR5_old;            //Calculating pulse
    }
    if (Pulse_Width>8000)   // SYNC pulse?
    {
        PPM_Counter=0;
    }
    else
    {
        PPM_Counter &= 0x07;  // For safety only (limit PPM_Counter to 7)
        APM_RC.PWM_RAW[PPM_Counter++]=Pulse_Width;  //Saving pulse
    }

    if (PPM_Counter >= NUM_CHANNELS)
    {
        APM_RC.radio_status = 1;
    }
    ICR5_old = Pulse;
}
char data;
void setup()
{
    Serial.begin(115200);
    Serial1.begin(115200);
    Serial2.begin(28800);
    Serial3.begin(28800);
    //gps = new Em406(2,Serial1);
    gps.initialize();

    flightPlan.push_back(&wp00);
    flightPlan.push_back(&wp01);
    flightPlan.push_back(&wp02);
    flightPlan.push_back(&wp03);
    flightPlan.push_back(&wp04);
    flightPlan.push_back(&wp05);
    flightPlan.push_back(&wp06);
    flightPlan.push_back(&wp07);
    flightPlan.push_back(&wp08);
    flightPlan.push_back(&wp09);
    flightPlan.push_back(&wp10);
    flightPlan.push_back(&wp11);
    flightPlan.push_back(&wp12);
    flightPlan.push_back(&wp13);
    flightPlan.push_back(&wp14);
    flightPlan.push_back(&wp15);
    flightPlan.push_back(&wp16);
    flightPlan.push_back(&wp17);
    flightPlan.push_back(&wp18);
    flightPlan.push_back(&wp19);
    flightPlan.push_back(&wp20);
    flightPlan.push_back(&wp21);
    flightPlan.push_back(&wp22);
    flightPlan.push_back(&wp23);
    flightPlan.push_back(&wp24);
    flightPlan.setSize(2);

    guide.setup(1);

    // Q = diag([5 0 1 10])  R = 1
    feedbackGain.setSize(1,4);
    feedbackGain(0,0) = 0.0809;
    feedbackGain(0,1) = 0.4963;
    feedbackGain(0,2) = 0.6567;
    feedbackGain(0,3) = 0.0065;

    APM_RC.initialize();
    delay(1000);
    APM_RC.time = millis();
}

void loop()
{
    ugv.update();
}
