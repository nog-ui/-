#pragma once
#include <Arduino.h>

#define LEDC_FREQ 500 // LEDC频率
#define LEDC_BIT 8    // LEDC分辨率(精度) 0-255

class Motor
{
private:
    int motorPIN1, motorPIN2; // ADC、马达控制引脚
    int channel1, channel2;   // esp32的ledc通道
public:
    Motor(int, int, int, int);
    void run(int);
    void brake();    // 马达静止（高阻）
    void flameout(); // 马达熄火（无阻力）
};

Motor::Motor(int mpin1, int mpin2, int ch1 = 0, int ch2 = 1)
{
    motorPIN1 = mpin1;
    motorPIN2 = mpin2;
    channel1 = ch1;
    channel2 = ch2;
    ledcSetup(channel1, LEDC_FREQ, LEDC_BIT);
    ledcSetup(channel2, LEDC_FREQ, LEDC_BIT);
    ledcAttachPin(motorPIN1, channel1);
    ledcAttachPin(motorPIN2, channel2);
}

// 马达运动PWM控制
void Motor::run(int pwm)
{
    pwm = constrain(pwm, -255, 255);
    if (pwm >= 0)
    {
        ledcWrite(channel1, pwm);
        ledcWrite(channel2, 0);
    }
    else
    {
        ledcWrite(channel1, 0);
        ledcWrite(channel2, -pwm);
    }
}

// 马达静止（高阻）
void Motor::brake()
{
    ledcWrite(channel1, 255);
    ledcWrite(channel2, 255);
}

// 马达熄火（无阻力）
void Motor::flameout()
{
    ledcWrite(channel1, 0);
    ledcWrite(channel2, 0);
}
