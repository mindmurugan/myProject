/*
 * razor9DofImu.cpp
 * Copyright (C) James Goppert 2010 jgoppert@users.sourceforge.net
 *
 * razor9DofImu.cpp is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * razor9DofImu.cpp is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "WProgram.h"
#include <Wire.h>
#include "oooarkavr/utilities/cPlusPlus.hpp"
#include "oooarkavr/communication/Sirf.hpp"
#include "oooarkavr/utilities/Timer.hpp"
#include "oooarkavr/math/basic.hpp"
#include "oooarkavr/communication/I2c.hpp"

using namespace oooarkavr;
//  prototypes
bool followCommand();
void readGyro(const size_t & samples, const float & smooth, const float & scale, const float bias[3]);
void readMag(const size_t & samples, const float & smooth, const float & scale);
void readAccel(const size_t & samples, const float & smooth, const float & scale);
void magAlign();
void gravAlign();
void calibrate();
void navigate();
void sirfSend();
void acknowledge(bool);
void calibrateacknowledge();

// constants
const static float qNormMax = 1.001; // max norm of quaternion

// communications
Sirf sirf(Serial);

// scheduling, rate in [Hz]
const static int16_t outRate = 10; // output system
const static int16_t magRate = 5; // magnetometer reading
const static int16_t accelRate = 90; // accel reading
const static int16_t gyroRate = 90; // gyroscope reading
const static int16_t alignRate = 1; // mag/grav alignment rate
const static int16_t navRate = 90; // navigation update rate
const static int16_t commandRate = 1; // rate to look for command packets
const static float gravAlignCriteria = 0.1; //if (norm(fb) - g) < gravAlignCriterial then realign

//  measurements
const uint16_t extCommandPeriod = 1000; //how long to wait on setup for an external setup command in milliseconds
const uint16_t warmPeriod = 5000; // how long to warm in milliseconsd
const uint16_t calibrationCycles = 2000; // cyles for calibration

// specific force measurement
const static int8_t fSamples = 8; // samples/read
static float fScale = 1; // calibrated later
static float fSmooth = 0; // smoothing
static float f_B[3]={}; // specific force
static float fRaw[3]={}; // raw data mean

// magnetometer measurement
const static int8_t mSamples = 1; // samples/read
static float mScale = 1; // calibrated later
static float mSmooth = 1-10./magRate; // smoothing
static float mRaw[3]={}; // raw data mean
static float m_B[3]={}; // [] normalized mag data
// normalization of mag done to detect magnetic anamoly later

// gyro measurement
const static int8_t wSamples = 3; // samples/read
static float wScale = 0.92*deg2Rad; // scale
static float wSmooth = 0; // smoothing
static float wBias[3]={}; // calibrated later
static float wRaw[3]={}; // raw data mean
static float w_B[3]={}; // [rad/s]

// state
static int64_t lat=0; // [deg*10^14] +North
static int64_t lon=0; // [deg*10^14] +Earth
static int64_t h=0; // [meters*10^4] Elliptical
static float q_NB[4]={1,0,0,0}, v_N[3]={};
static enum ImuMode { internal, external} imuMode;

// computed values, make sure these are upated when you use them
float roll, pitch, yaw; // [rad] euler angles
float f_N[3]; // specific force in nav. frame [m/s^2]
float dVN, dVE, dVD; // acceleration in navigation frame [m/s^2]

// alias's
static float &vN=v_N[0], &vE=v_N[1], &vD=v_N[2];
static float &a=q_NB[0], &b=q_NB[1], &c=q_NB[2], &d=q_NB[3];
static float &fX=f_B[0], &fY=f_B[1], &fZ=f_B[2];
static float &mX=m_B[0], &mY=m_B[1], &mZ=m_B[2];
static float &wX=w_B[0], &wY=w_B[1], &wZ=w_B[2];
float &fN=f_N[0], &fE=f_N[1], &fD=f_N[2];

// Timers
static Timer outTimer(outRate), magTimer(magRate), alignTimer(alignRate),
accelTimer(accelRate), gyroTimer(gyroRate), navTimer(navRate), commandTimer(commandRate);

// I2c classes
static I2c mag(0x1E,big), accel(0x53,little);

// gyroscope measurement
void readGyro(const size_t & samples, const float & smooth, const float & scale, const float bias[3])
{
    int32_t wSum[3] = {};
    for (size_t i=0;i<samples;i++)
    {
        wSum[0] += -analogRead(2);
        wSum[1] += -analogRead(1);
        wSum[2] += -analogRead(0);
    }
    for (size_t i=0;i<3;i++)
    {
        wRaw[i] = float(wSum[i])/samples;
        w_B[i] = scale*(wRaw[i]-bias[i])*(1-smooth)+w_B[i]*smooth;
    }
}

// magnetometer measurement
void readMag(const size_t & samples, const float & smooth, const float & scale)
{
    int status = mag.read(0x03,3,mSamples,mRaw);
    if (status == 0)
    {
        m_B[0] = -scale*mRaw[0]*(1-smooth)+m_B[0]*smooth;
        m_B[1] = -scale*mRaw[1]*(1-smooth)+m_B[1]*smooth;
        m_B[2] = -scale*mRaw[2]*(1-smooth)+m_B[2]*smooth;
    }
    else Serial.print("readMag error: "), Serial.println(status);
}

// accelerometer measurement
void readAccel(const size_t & samples, const float & smooth, const float & scale)
{
    int status = accel.read(0x32,3,fSamples,fRaw);
    if (status == 0)
    {
        f_B[0] = scale*fRaw[0]*(1-smooth)+f_B[0]*smooth;
        f_B[1] = -scale*fRaw[1]*(1-smooth)+f_B[1]*smooth;
        f_B[2] = -scale*fRaw[2]*(1-smooth)+f_B[2]*smooth;
    }
    else Serial.print("readAccel error: "), Serial.println(status);
}

// gravitational alignment
void gravAlign()
{
    //Serial.println("Performing Gravity Alignment.");
    if (((f_B[0]*f_B[0]+f_B[1]*f_B[1]+f_B[2]*f_B[2]) - g0*g0) < gravAlignCriteria*gravAlignCriteria)
    {
        float u_N[3]={0,0,-1};
        quatVectorAlign(f_B,u_N,q_NB);
    }
}

// magnetic alignment
void magAlign()
{
    //Serial.println("Performing Magnetic Alignment.");
    float m_N[3], q_MB[4], q_MN[4];
    float n_N[3] = {-1,0,0}; // not sure why this has to be negative/ should be north
    quatRotate(q_NB,m_B,m_N);
    m_N[2] = 0; // project to avoid changing pitch/roll
    quatVectorAlign(m_N,n_N,q_MN);
    quatProd(q_MN,q_NB,q_MB);
    for (int i=0;i<4;i++) q_NB[i] = q_MB[i];
}

// sensor calibration
void calibrate()
{
    float wMean[3] = {};
    float fMean[3] = {};
    float mMean[3] = {};
    float zeroBias[3] = {};

    Serial.print("\r\nWarming for ");
    Serial.print(warmPeriod/1000.), Serial.println(" seconds.");
    Serial.println("-------------------------------------------------|");
    uint32_t warmStart = millis();
    while (millis()-warmStart<warmPeriod)
    {
        readGyro(1,0,1,zeroBias);
        readAccel(1,0,1);
        readMag(1,0,1);
    }

    Serial.println("\r\nCalibrating.");
    Serial.println("-------------------------------------------------|");
    for (size_t i=1;i<calibrationCycles;i++)
    {
        if (followCommand()) return;
        readGyro(1,0,1,zeroBias);
        readAccel(1,0,1);
        readMag(1,0,1);
        for (size_t j=0;j<3;j++)
        {
            fMean[j] += f_B[j];
            wMean[j] += w_B[j];
            mMean[j] += m_B[j];
        }
        if (i%(calibrationCycles/50)==0) Serial.print("#"), Serial.flush();
    }
    Serial.println();

    // find mean values
    for (size_t i=0;i<3;i++)
    {
        fMean[i] /= calibrationCycles;
        wMean[i] /= calibrationCycles;
        mMean[i] /= calibrationCycles;
    }

    // scalar quantities
    mScale = 1./sqrt(mMean[0]*mMean[0]+mMean[1]*mMean[1]+mMean[2]*mMean[2]);
    fScale = g0/sqrt(fMean[0]*fMean[0]+fMean[1]*fMean[1]+fMean[2]*fMean[2]);

    // vector quantities
    for (size_t i=0;i<3;i++)
    {
        // gyro
        wBias[i] = wMean[i];
        w_B[i] = 0;

        // magnetometer
        m_B[i] = mMean[i]*mScale;

        // accelerometer
        f_B[i] = fScale*fMean[i];
    }

    //output
    Serial.print("\twXBias: "), Serial.print(wBias[0]);
    Serial.print("\twYBias: "), Serial.print(wBias[1]);
    Serial.print("\twZBias: "), Serial.print(wBias[2]);
    Serial.print("\tmScale: "), Serial.println(mScale,4);
    Serial.print("\tfScale: "), Serial.println(fScale,4);
    Serial.println("\n\nCalibration Complete.\n\n");
}

// navigation equations
void navigate()
{
    // time period in microseconds
    static float dA0,dB0,dC0,dD0,dLat0,dLon0,dH0,dVN0,dVE0,dVD0;
    uint32_t dt = navTimer.getPeriod();

    // temporaries for integrations
    float cosLat = cos(deg2Rad*lat/1.e14);
    float sinLat = sin(deg2Rad*lat/1.e14);
    float dLat = vN/(r0+h/1.e4);
    float dLon = vE/cosLat/(r0+h/1.e4);
    float dH = -vD;
    float tmp = (2*omega+dLon);
    quatRotate(q_NB,f_B,f_N);
    dVD = fD - vE*tmp*cosLat-vN*dLat+g0;
    dVN = fN - vE*tmp*sinLat+dVD*tmp*cosLat;
    dVE = fE + vN*tmp*sinLat+dVD*tmp*cosLat;
    float dA = -0.5*(b*wX+c*wY+d*wZ);
    float dB = 0.5*(a*wX-d*wY+c*wZ);
    float dC = 0.5*(d*wX+a*wY-b*wZ);
    float dD = -0.5*(c*wX-b*wY-a*wZ);

    // integrate attitude
    a += (dA+dA0)/2.*dt/1.e6;
    b += (dB+dB0)/2.*dt/1.e6;
    c += (dC+dC0)/2.*dt/1.e6;
    d += (dD+dD0)/2.*dt/1.e6;

    // integrate position
    lat += (dLat+dLat0)/2.*dt*1.e8;
    lon += (dLon+dLon0)/2.*dt*1.e8;
    h   += (dH+dH0)/2.*dt/1.e2;

    // integrate velocity
    vN += (dVN+dVN0)/2.*dt/1.e6;
    vE += (dVE+dVE0)/2.*dt/1.e6;
    vD += (dVD+dVD0)/2.*dt/1.e6;

    // store state
    dA0 = dA;
    dB0 = dB;
    dC0 = dC;
    dD0 = dD;
    dLat0 = dLat;
    dLon0 = dLon;
    dH0 = dH;
    dVN0 = dVN;
    dVE0 = dVE;
    dVD0 = dVD;
}

void sirfSend()
{
    if (imuMode==internal)
    {
        sirf.packLsb(int32_t(lat/1.e7));
        sirf.packLsb(int32_t(lon/1.e7));
        sirf.packLsb(int32_t(h/1.e2));
        sirf.packLsb(vN);
        sirf.packLsb(vE);
        sirf.packLsb(vD);
        sirf.packLsb(roll);
        sirf.packLsb(pitch);
        sirf.packLsb(yaw);
        sirf.packLsb(w_B[0]);
        sirf.packLsb(w_B[1]);
        sirf.packLsb(w_B[2]);
        sirf.send();
    }
    else if (imuMode==external)
    {
        const uint8_t header [] = {0xAC,0xB8};
        sirf.pack(header,(size_t) 2);
        for (int i=0;i<3;i++) sirf.packLsb(fRaw[i]);
        for (int i=0;i<3;i++) sirf.packLsb(wRaw[i]);
        for (int i=0;i<3;i++) sirf.packLsb(mRaw[i]);
        sirf.send();
    }
}

void acknowledge(bool status)
{
    const uint8_t good [] = {0xAA,0xAA};
    const uint8_t bad [] = {0xBB,0xBB};
    if (status == 1) sirf.pack(good,(size_t) 2);
    else sirf.pack(bad, (size_t) 2);
    sirf.send();
}

bool followCommand()
{
    if (sirf.receive())
    {
        //Serial.println("Packet recieved!");
        int16_t message;
        sirf.unpackLsb(message,0);
        //Serial.print("Message = ");
        //Serial.println(message);
        if (message==setStateMessage)
        {
            imuMode=internal;
            //Serial.println("Received command: setState");
            int32_t resetLat, resetLon, resetH;
            sirf.unpackLsb(resetLat,2);
            sirf.unpackLsb(resetLon,6);
            sirf.unpackLsb(resetH,10);
            sirf.unpackLsb(v_N[0],14);
            sirf.unpackLsb(v_N[1],18);
            sirf.unpackLsb(v_N[2],22);

            lat = (int64_t) resetLat*1.e7;
            lon = (int64_t) resetLon*1.e7;
            h = (int64_t) resetH*1.e2;
            acknowledge(true);
            return true;
        }
        else if (message==selfCalibrateMessage)
        {
            //Serial.println("Received command: calibrate");
            calibrate();
            acknowledge(true);
            return true;
        }
        else if (message==alignMessage)
        {
            //Serial.println("Received command: align");
            magAlign();
            gravAlign();
            acknowledge(true);
            return true;
        }
        else if (message==setGyroBiasMessage)
        {
            //Serial.println("Received command: setGyroBias");
            sirf.unpackLsb(wBias[0],2);
            sirf.unpackLsb(wBias[1],6);
            sirf.unpackLsb(wBias[2],10);
            acknowledge(true);
            return true;
        }
        else if (message==externalCalibrateMessage)
        {
            imuMode=external;
            //Serial.println("Received command: externalCalibrate");
            acknowledge(true);
            return true;
        }
        acknowledge(false);
        return false;
    }
    return false;
}

// setup sensors, perform calibration/ alignments
void setup()
{
    Wire.begin();
    Serial.begin(38400);

    // setup imu
    imuMode=internal;

    // setup hmc5843
    mag.write(0x00,0x18); // 50 Hz
    mag.write(0x01,0x20); // gain of 1
    mag.write(0x02,0x00); // continuous

    // setup adxl345
    accel.write(0x31,0x0B); // rull res +/- 16 g
    accel.write(0x2C,0x0D); // 800 Hz
    accel.write(0x2D,0x08); // meas. mode
    accel.write(0x38,0x80); // fifo stream mode

    // calibration
    calibrate();

    // alignments
    gravAlign();
    magAlign();
    gravAlign();
    magAlign();
    Serial.print("setup complete");
}

// output of data to serial port
void output()
{
    // computations
    quatToEuler(q_NB,roll,pitch,yaw);
    //quatRotate(q_NB,f_B,f_N);

    // rates
    //Serial.print("out: "), Serial.print(outTimer.getFreq());
    //Serial.print("\tnav: "), Serial.print(navTimer.getFreq());
    //Serial.print("\taccel: "), Serial.print(accelTimer.getFreq());
    //Serial.print("\tgyro: "), Serial.print(gyroTimer.getFreq());
    //Serial.print("\tmag : "), Serial.println(magTimer.getFreq());

    // output
    //Serial.print("\tx arc [m] : "), Serial.print(r0*deg2Rad*lat/1.e14,4);
    //Serial.print("\ty arc [m] : "), Serial.print(r0*deg2Rad*lon/1.e14,4);
    //Serial.print("\tlat [deg] : "), Serial.print(lat/1.e14,8);
    //Serial.print("\tlon [deg] : "), Serial.print(lon/1.e14,8);
    //Serial.print("\th [m]: "), Serial.println(h/1.e4,8);
    //Serial.print("\troll [deg]: "), Serial.print(roll*rad2Deg,4);
    //Serial.print("\tpitch [deg]: "), Serial.print(pitch*rad2Deg,4);
    //Serial.print("\tyaw [deg]: "), Serial.println(yaw*rad2Deg,4);
    //Serial.print("\tvN [m/s] : "), Serial.print(vN);
    //Serial.print("\tvE [m/s] : "), Serial.print(vE);
    //Serial.print("\tvD [m/s]: "), Serial.println(vD);
    //Serial.print("\tdVN [m/s^2]: "), Serial.print(dVN,4);
    //Serial.print("\tdVE [m/s^2]: "), Serial.print(dVE,4);
    //Serial.print("\tdVD [m/s^2]: "), Serial.println(dVD,4);
    //Serial.print("\twX [deg/s]: "), Serial.print(wX*rad2Deg,4);
    //Serial.print("\twY [deg/s]: "), Serial.print(wY*rad2Deg,4);
    //Serial.print("\twZ [deg/s]: "), Serial.println(wZ*rad2Deg,4);
    //printArray("\tq_NB []: ",q_NB,4);

    // measurements
    //Serial.print("\tfN [m/s^2]: "), Serial.print(fN,4);
    //Serial.print("\tfE [m/s^2]: "), Serial.print(fE,4);
    //Serial.print("\tfD [m/s^2]: "), Serial.println(fD,4);
    //Serial.print("\tfX [m/s^2]: "), Serial.print(fX,4);
    //Serial.print("\tfY [m/s^2]: "), Serial.print(fY,4);
    //Serial.print("\tfZ [m/s^2]: "), Serial.println(fZ,4);
    //Serial.print("\tmX []: "), Serial.print(mX,4);
    //Serial.print("\tmY []: "), Serial.print(mY,4);
    //Serial.print("\tmZ []: "), Serial.println(mZ,4);

    // raw data
    Serial.print("\tfXRaw: "), Serial.print(fRaw[0]);
    Serial.print("\tfYRaw: "), Serial.print(fRaw[1]);
    Serial.print("\tfZRaw: "), Serial.println(fRaw[2]);
    Serial.print("\twXRaw: "), Serial.print(wRaw[0]);
    Serial.print("\twYRaw: "), Serial.print(wRaw[1]);
    Serial.print("\twZRaw: "), Serial.println(wRaw[2]);
    Serial.print("\tmXRaw: "), Serial.print(mRaw[0]);
    Serial.print("\tmYRaw: "), Serial.print(mRaw[1]);
    Serial.print("\tmZRaw: "), Serial.println(mRaw[2]);

    // memory
    //displayMemory();
    //Serial.println();

    sirfSend();
}

// update required systems at rates specified by timer class's
void loop()
{
    // put critical timings first
    if (accelTimer.isReady()) readAccel(fSamples,fSmooth,fScale);
    if (gyroTimer.isReady()) readGyro(wSamples,wSmooth,wScale,wBias);
    if (magTimer.isReady()) readMag(mSamples,mSmooth,mScale);
    if (navTimer.isReady()) navigate();
    if (outTimer.isReady()) output();
    if (alignTimer.isReady()) gravAlign(), magAlign();
    if (commandTimer.isReady()) followCommand();
}
