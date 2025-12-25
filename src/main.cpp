// 今回いる機構
// ロジャー リミットスイッチ×2  20000 両方向 負→上がる 正→下がる 9v
// garaggaraリミットスイッチ×1 20000 両方向　負→開く 正→閉じる 5v
// 今now ロリコン×1 20000　正転 9v

// ラッピニ r
// ベルト r
// バイスト r
// 滑車 r

// 櫓ラッピニ
// アーム r
// ベルト櫓 r

// リング棒
// リングベルト r r

#include <iostream>
#include "kikou.hpp"
#include "FIRSTPENGUIN.hpp"
#include <cmath>

CAN can(PB_12, PB_13, 1e6);
int omuni_value = 5000;
int16_t pwm1[4] = {0};
int16_t pwm[4] = {0};

constexpr uint32_t can_id = 35;    // FPのCAN通信のIDを設定
FirstPenguin penguin(can_id, can); // FPの設定
// BufferedSerial pc(USBTX, USBRX, 115200);
// BufferedSerial esp(PA_9, PA_10, 115200);
// const float k = 360.0 / (1024.0); // 360 / (512*2)
bool limit_value[5] = {0};
int16_t encoder_value[5] = {0};
// int pojisyon[3] = {0};
// bool control1ler[12] = {false}; // up, down, right, left,triangle,circle, cross,  square,l,r,l2,r2,
// PID制御用Ticker              // up 0, down 1, right 2,left 3,triangle 4,circle 5,cross 6,square 7,l 8,r 9,l2 10,r2 11,

int main()
{
    while (1)
    {
        if (controller["u"])
        {
            penguin.pwm[0] = omuni_value;
            penguin.pwm[1] = -1 * omuni_value;
            penguin.pwm[2] = -1 * omuni_value;
            penguin.pwm[3] = omuni_value;
            printf("a\n");
        }
        else if (controller["d"])
        {
            penguin.pwm[0] = -1 * omuni_value;
            penguin.pwm[1] = omuni_value;
            penguin.pwm[2] = omuni_value;
            penguin.pwm[3] = -1 * omuni_value;
            printf("b\n");
        }
        else if (controller["l"])
        {
            penguin.pwm[0] = omuni_value;
            penguin.pwm[1] = omuni_value;
            penguin.pwm[2] = -1 * omuni_value;
            penguin.pwm[3] = -1 * omuni_value;
            printf("c\n");
        }
        else if (controller["r"])
        {
            penguin.pwm[0] = -1 * omuni_value;
            penguin.pwm[1] = -1 * omuni_value;
            penguin.pwm[2] = omuni_value;
            penguin.pwm[3] = omuni_value;
            printf("d\n");
        }
        else if (controller["ci"])
        {
            penguin.pwm[0] = -1 * omuni_value;
            penguin.pwm[1] = -1 * omuni_value;
            penguin.pwm[2] = -1 * omuni_value;
            penguin.pwm[3] = -1 * omuni_value;
            printf("e\n");
        }
        else if (controller["sq"])
        {
            penguin.pwm[0] = omuni_value;
            penguin.pwm[1] = omuni_value;
            penguin.pwm[2] = omuni_value;
            penguin.pwm[3] = omuni_value;
            printf("f\n");
        }
        else
        {
            penguin.pwm[0] = 0;
            penguin.pwm[1] = 0;
            penguin.pwm[2] = 0;
            penguin.pwm[3] = 0;
            printf("g\n");
        }
        // pid_control(pwm1[0],encoder_value[0],10000);
        // コントローラー入力読み取り
        read_controller();
        // controller_check();
        // リミットスイッチ、エンコーダー読み取り
        // read_limit(limit_value);
        // read_encoder(encoder_value);
        // コントローラー入力からPWM計算
        pwm_calculation(pwm1[0], controller["L1"], controller["R1"], 15000);  // カード1,l2,r2,pwm_data配列0番
        pwm_calculation(pwm1[1], controller["ci"], controller["sq"], 5000);   // カード2,l,r,pwm_data配列1番
        pwm_calculation(pwm1[2], controller["tri"], controller["cr"], 15000); // カード3,circle,cross,pwm_data配列2番
        // センサー処理
        // sensor_processing(pwm1[0], limit_value[0], 1); // ロジャー下リミット
        // sensor_processing(pwm1[0], limit_value[1], 0); // ロジャー上リミット
        // sensor_processing(pwm1[1], limit_value[2], 1); // garaggaraリミット
        // 　ロリコン処理
        // pid_control(pwm1[3], encoder_value[0], 10000);
        // printf("enc:%d,%d,%d,%d,%d\n", encoder_value[0], encoder_value[1], encoder_value[2], encoder_value[3], encoder_value[4]);
        // printf("lim:%d,%d,%d,%d,%d\n", limit_value[0], limit_value[1], limit_value[2], limit_value[3], limit_value[4]);
        // printf("%d,%d,%d,%d\n", (int)(stick_value["lx"] * 100), (int)(stick_value["ly"] * 100), (int)(stick_value["rx"] * 100), (int)(stick_value["ry"] * 100));
        // mekanamu(penguin.pwm);
        // モタドラへcan送信
        penguin.send();
        CANMessage msg(2, (const uint8_t *)pwm1, 8);
        // CANMessage msg1(4, (const uint8_t *)pwm, 8);
        can.write(msg);
        // can.write(msg1);
    }
}
