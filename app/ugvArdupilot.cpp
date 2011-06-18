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
    volatile unsigned int Start_Pulse;
    volatile unsigned int Stop_Pulse;
    volatile unsigned int Pulse_Width;

    volatile int Test;
    volatile int Test2;
    volatile int Temp;
    volatile int Counter;
    volatile byte PPM_Counter;
    int All_PWM;

    long timer;
    long timer2;

    volatile unsigned char radio_status;
    volatile unsigned int PWM_RAW[8];// = {2400,2400,2400,2400,2400,2400,2400,2400};
    Controller * controller;
    // Constructors ////////////////////////////////////////////////////////////////

    APM_RC_Class(int freq, int controlPin, Controller * controller): System(freq), Start_Pulse(0), Stop_Pulse(0), Pulse_Width(0), Test(0), Test2(0), Temp(0), Counter(0), PPM_Counter(0), All_PWM(1500), timer(0), timer2(0), radio_status(0), controller(controller)
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

        pinMode(44,OUTPUT);
        pinMode(45,OUTPUT);
        pinMode(46,OUTPUT);

        //Init PWM Timer 5
        TCCR5A =((1<<WGM51)|(1<<COM5A1)|(1<<COM5B1)|(1<<COM5C1));
        TCCR5B = (1<<WGM53)|(1<<WGM52)|(1<<CS51); //Prescaler set to 8
        OCR5A = 3000; //PL3,
        OCR5B = 3000; //PL4, OUT0
        OCR5C = 3000; //PL5  OUT1
        ICR5 = 40000;
        //ICR5 = 43910; //So (16000000hz/8)/50hz=40000,

        //Init PWM Timer4
        /*Note that timer4 is configured to used the Input capture for PPM decoding and to pulse two servos
          OCR4A is used as the top counter*/
        pinMode(49, INPUT);
        pinMode(7,OUTPUT);
        pinMode(8,OUTPUT);
        //Remember the registers not declared here remains zero by default...
        TCCR4A =((1<<WGM40)|(1<<WGM41)|(1<<COM4C1)|(1<<COM4B1)|(1<<COM4A1));
        TCCR4B = ((1<<WGM43)|(1<<WGM42)|(1<<CS41)|(1<<ICES4)); //Prescaler set to 8, that give us a resolution of 2us, read page 134 of data sheet
        OCR4A = 40000; ///50hz freq...Datasheet says  (system_freq/prescaler)/target frequency. So (16000000hz/8)/50hz=40000,
        //must be 50hz because is the servo standard (every 20 ms, and 1hz = 1sec) 1000ms/20ms=50hz, elementary school stuff...
        OCR4B = 3000; //PH4, OUT5
        OCR4C = 3000; //PH5, OUT4

        TIMSK4 |= (1<<ICIE4); //Timer interrupt mask
        sei();
    }

    void run()
    {
        //Switch low, forward the PPM
        if (InputCh(5) <= 1250)
        {
            OutputCh(0, InputCh(0));
            OutputCh(1, InputCh(1));
            OutputCh(2, InputCh(2));
            OutputCh(3, InputCh(3));
            OutputCh(4, InputCh(4));
            OutputCh(5, InputCh(5));
            OutputCh(6, InputCh(6));
            OutputCh(7, InputCh(7));
        }
        else
        {
            //Switch High, Move servos
            if (InputCh(5) >= 1750)
            {
                if ((millis()- timer2) >= 20)
                {
                    timer2=millis();
                    if (All_PWM >2100)
                        All_PWM=900;
                    else
                        All_PWM+=20;
                }
            }
            //Switch in the midle, center all servos
            else
            {
                All_PWM=1500;
            }

            OutputCh(0, All_PWM);
            OutputCh(1, All_PWM);
            OutputCh(2, All_PWM);
            OutputCh(3, All_PWM);
            OutputCh(4, All_PWM);
            OutputCh(5, All_PWM);
            OutputCh(6, All_PWM);
            OutputCh(7, All_PWM);
        }

    }

    void OutputCh(byte ch, int pwm)
    {
        pwm=constrain(pwm,900,2100);
        pwm*=2;

        switch (ch)
        {
        case 0:
            OCR5B=pwm;
            break;  //ch0
        case 1:
            OCR5C=pwm;
            break;  //ch1
        case 2:
            OCR1B=pwm;
            break;  //ch2
        case 3:
            OCR1C=pwm;
            break;  //ch3
        case 4:
            OCR4C=pwm;
            break;  //ch4
        case 5:
            OCR4B=pwm;
            break;  //ch5
        case 6:
            OCR3C=pwm;
            break;  //ch6
        case 7:
            OCR3B=pwm;
            break;  //ch7
        case 8:
            OCR5A=pwm;
            break;  //ch8,  PL3
        case 9:
            OCR1A=pwm;
            break;  //ch9,  PB5
        case 10:
            OCR3A=pwm;
            break;  //ch10, PE3
        }

    }

    int InputCh(byte ch)
    {
        return (PWM_RAW[ch]+600)/2;
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
Em406  gps(2, Serial1);
ImuUpdater imuUpdater(1, &gps, Serial2);
PcLink pcLink(5, Serial3, &pos, &vNED, &euler, &wN, &errorState);

int controlPin = 4;
APM_RC_Class APM_RC(50, controlPin, &controller);

TestSystem ugv(13, &APM_RC, &flightPlan, &pos, &vNED, &euler, &wN, &controller, &guide, &gps, &imuUpdater, &pcLink, Serial2);

/****************************************************
  Interrupt Vector
 ****************************************************/
ISR(TIMER4_CAPT_vect)//interrupt.
{
    if (((1<<ICES4)&TCCR4B) >= 0x01)
    {
        if (APM_RC.Start_Pulse>APM_RC.Stop_Pulse) //Checking if the Stop Pulse overflow the register, if yes i normalize it.
        {
            APM_RC.Stop_Pulse+=40000; //Nomarlizing the stop pulse
        }
        APM_RC.Pulse_Width=APM_RC.Stop_Pulse-APM_RC.Start_Pulse; //Calculating pulse
        if (APM_RC.Pulse_Width>5000) //Verify if this is the sync pulse
        {
            APM_RC.PPM_Counter=0; //If yes restart the counter
        }
        else
        {
            APM_RC.PWM_RAW[APM_RC.PPM_Counter]=APM_RC.Pulse_Width; //Saving pulse.
            APM_RC.PPM_Counter++;
        }
        APM_RC.Start_Pulse=ICR4;
        TCCR4B &=(~(1<<ICES4)); //Changing edge detector.
    }
    else
    {
        APM_RC.Stop_Pulse=ICR4; //Capturing time stop of the drop edge
        TCCR4B |=(1<<ICES4); //Changing edge detector.
        //TCCR4B &=(~(1<<ICES4));
    }
    //Counter++;
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
    //APM_RC.time = millis();
}

void loop()
{
    ugv.update();
}
