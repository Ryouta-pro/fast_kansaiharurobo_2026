// 今回いる機構
// ロジャー リミットスイッチ×2  20000 両方向 負→上がる 正→下がる 9v
// garaggaraリミットスイッチ×1 20000 両方向　負→開く 正→閉じる 5v
// 今now ロリコン×1 20000　正転 9v
#include "mbed.h"
#include <iostream>
#include "kikou.hpp"
CAN can(PA_11, PA_12, 1e6);
BufferedSerial pc(USBTX, USBRX, 115200);
CANMessage msg1;
// const float k = 360.0 / (1024.0); // 360 / (512*2)
bool limit_value[5] = {0};
int16_t encoder_value[5] = {0};
int16_t pwm1[4] = {0};
int16_t want_k = 45;
bool controller[12] = {false}; // up, down, right, left,triangle,circle, cross,  square,l,r,l2,r2,
// PID制御用Ticker              // up 0, down 1, right 2,left 3,triangle 4,circle 5,cross 6,square 7,l 8,r 9,l2 10,r2 11,
Ticker pid_starter;

// void pid_tick()
// {
//     // pid_control のシグネチャに合わせて呼ぶ（グローバル変数を参照）
//     pid_control(pwm1[2], encoder_value[0], want_k);
// }

int main()
{
    // pid_starter.attach(&pid_tick, 0.04); // 0.04 秒ごとに pid_tick を呼ぶ
    while (1)
    {
        // リミットスイッチ、エンコーダー読み取り
        read_limit(limit_value);
        read_encoder(encoder_value);
        // コントローラー入力からPWM計算
        controller[0] = true;
        controller[2] = true;
        controller[9] = true;
        pwm_calculation(pwm1[0], controller[10], controller[11], 0); // カード1,l2,r2,pwm_data配列0番
        pwm_calculation(pwm1[1], controller[8], controller[9], 1);   // カード2,l,r,pwm_data配列1番
        pwm_calculation(pwm1[2], controller[4], controller[5], 2);   // カード3,circle,cross,pwm_data配列2番
        // センサー処理
        sensor_processing(pwm1[0], limit_value[0], 1); // ロジャー下リミット
        sensor_processing(pwm1[0], limit_value[1], 0); // ロジャー上リミット
        sensor_processing(pwm1[1], limit_value[2], 1); // garaggaraリミット
        // 　ロリコン処理
        // pid_control(pwm1[3], encoder_value[0], want_k);
        // printf("enc:%d,%d,%d,%d,%d\n", encoder_value[0], encoder_value[1], encoder_value[2], encoder_value[3], encoder_value[4]);
        // printf("lim:%d,%d,%d,%d,%d\n", limit_value[0], limit_value[1], limit_value[2], limit_value[3], limit_value[4]);
        // pwmデータの確認
        printf("%d,%d,%d,%d\n", pwm1[0], pwm1[1], pwm1[2], pwm1[3]);
        // モタドラへcan送信
        CANMessage msg1(2, (const uint8_t *)pwm1, 8);
        can.write(msg1);
    }
}
