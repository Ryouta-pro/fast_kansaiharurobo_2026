#include "mbed.h"
#include <iostream>
CAN can1(PA_11, PA_12, 1e6);
CANMessage msg;
int pwm_data[3] = {20000, 20000, 20000};
const float k = 360.0 / (1024.0);
float i_control = 0;
int before_error = 0;
float dt = 0.04; // 制御周期
float p_gain = 1.0;
float i_gain = 0.0;
float d_gain = 0.0;

// リミットスイッチ読み取り関数
void read_limit(bool (&is_sw_push)[5])
{
    if (can1.read(msg); msg.id == 10)
    {
        uint8_t sw = msg.data[5]; // スイッチの値の内容
        for (int i = 0; i < 5; i++)
        {
            is_sw_push[i] = (sw >> i) & 0x01;
            // シフトで判別する位を最下位ビットへ移動し、1と論理積を行うことで各位の値を判別する。
        }
    }
}

// エンコーダー読み取り関数
void read_encoder(int16_t (&enc)[5])
{
    if (can1.read(msg); msg.id == 9)
    {
        enc[0] = msg.data[7] << 8 | msg.data[6];
    }
    if (can1.read(msg); msg.id == 10)
    {
        for (int i = 0; i < 4; i++)
        {
            enc[i + 1] = msg.data[(i * 2) + 1] << 8 | msg.data[i * 2];
        }
    }
}

// PWM計算関数
void pwm_calculation(int16_t(&pwm_value), bool btton1, bool button2, int pwm_array_index)
{
    if (btton1 || button2)
    {
        pwm_value = pwm_data[pwm_array_index] * (btton1 - button2);
    }
    else
    {
        pwm_value = 0;
    }
}
void sensor_processing(int16_t(&pwm_value), bool limit, bool up_down)
{
    if (limit && ((pwm_value > 0) == up_down))
    {
        pwm_value = 0;
    }
}
// センサー処理関数
// limit: リミットスイッチの状態
// up_down: 正負の判定(true:正の場合停止)
// void pid_tick()
// {
//     // pid_control のシグネチャに合わせて呼ぶ（グローバル変数を参照）
//     pid_control(pwm1[2], encoder_value[0], want_k);
// }
// void pid_control(int16_t(&pwm_value), int16_t encoder_value, int16_t want_angle)
// {
//     int error = want_angle - encoder_value * k;
//     float p_control = error;
//     i_control += error * dt;
//     float d_control = (error - before_error) / dt;

//     pwm_value = p_gain * p_control + i_gain * i_control + d_gain * d_control;
//     if (pwm_value > 20000)
//     {
//         pwm_value = 20000;
//     }
//     else if (pwm_value < -20000)
//     {
//         pwm_value = -20000;
//     }
//     // printf("now:%d,output:%d\n", k * encoder_value, pwm_value);
//     before_error = error;
// }