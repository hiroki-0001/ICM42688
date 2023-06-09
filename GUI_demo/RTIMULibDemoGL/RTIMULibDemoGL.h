////////////////////////////////////////////////////////////////////////////
//  MIT License
//
//  Copyright (c) 2023 Hiroki, richards-tech, LLC
//
//  Permission is hereby granted, free of charge, to any person obtaining a copy of
//  this software and associated documentation files (the "Software"), to deal in
//  the Software without restriction, including without limitation the rights to use,
//  copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the
//  Software, and to permit persons to whom the Software is furnished to do so,
//  subject to the following conditions:
//
//  The above copyright notice and this permission notice shall be included in all
//  copies or substantial portions of the Software.
//
//  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
//  INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A
//  PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
//  HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION
//  OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
//  SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

#ifndef _RTIMULIBDEMOGL_H
#define _RTIMULIBDEMOGL_H

#include <QMainWindow>
#include <QLabel>
#include <QCheckBox>
#include <qcombobox.h>

#include "ui_RTIMULibDemoGL.h"

#include "IMU.hpp"

//  Display type codes

#define DISPLAY_FUSION      0                               // displays fusion algorithm output
#define DISPLAY_MEASURED    1                               // measured from accels and compass
#define DISPLAY_ACCELONLY   2                               // just the accel data
#define DISPLAY_COMPASSONLY 3                               // just the compass data

class IMUView;
class IMUThread;

class RTIMULibDemoGL : public QMainWindow
{
    Q_OBJECT

public:
    RTIMULibDemoGL();
    ~RTIMULibDemoGL();

public slots:
    void onEnableGyro(int);
    void onEnableAccel(int);
    void newIMUData(const IMU_DATA&);

signals:
    void newIMU();

protected:
    void timerEvent(QTimerEvent *event);
    void closeEvent(QCloseEvent *event);

private:
    void layoutStatusBar();
    void layoutWindow();
    QLabel *getFixedPanel(QString text);

    IMUThread *m_imuThread;                                 // the thread that operates the imu

    IMU_DATA m_imuData;                                   // this holds the IMU information and funsion output

    //  Qt GUI stuff

    Ui::RTIMULibDemoGLClass ui;

    QLabel *m_fusionQPoseScalar;
    QLabel *m_fusionQPoseX;
    QLabel *m_fusionQPoseY;
    QLabel *m_fusionQPoseZ;

    QLabel *m_fusionPoseX;
    QLabel *m_fusionPoseY;
    QLabel *m_fusionPoseZ;

    QLabel *m_gyroX;
    QLabel *m_gyroY;
    QLabel *m_gyroZ;

    QLabel *m_accelX;
    QLabel *m_accelY;
    QLabel *m_accelZ;
    QLabel *m_accelMagnitude;
    QLabel *m_accelResidualX;
    QLabel *m_accelResidualY;
    QLabel *m_accelResidualZ;

    QLabel *m_compassX;
    QLabel *m_compassY;
    QLabel *m_compassZ;
    QLabel *m_compassMagnitude;

    QLabel *m_pressure;
    QLabel *m_height;
    QLabel *m_temperature;
    QLabel *m_humidity;

    QLabel *m_fusionType;
    QCheckBox *m_enableGyro;
    QCheckBox *m_enableAccel;
    QCheckBox *m_enableCompass;
    QCheckBox *m_enableDebug;

    QLabel *m_imuType;
    QLabel *m_biasStatus;
    QLabel *m_rateStatus;
    QLabel *m_calStatus;

    IMUView *m_view;
    QComboBox *m_displaySelect;

    int m_rateTimer;
    int m_displayTimer;

    int m_sampleCount;
};

#endif // _RTIMULIBDEMOGL_H

