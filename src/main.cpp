#include "mbed.h"
#include "PID.hpp"
uint8_t DATA[8] = {};
InterruptIn button(PC_8);       // リミットスイッチ
InterruptIn button_reset(PC_9); // 起動スイッチ
BufferedSerial pc(USBTX, USBRX, 115200);
CAN can(PA_11, PA_12, (int)1e6);
int DJI_ID = 0x200;

int16_t motor_1 = 0;
int16_t motor_2 = 0;
int16_t motor_3 = 0;
int16_t speed = 0; // 16ビットの最大値

int reset;
int sw;

int16_t sokudo = 0;
int16_t mokuhyou = 0;
const float kp = 0.1;
const float ki = 0.035;
const float kd = 0.0;
const float sample_time = 0.02; // 20ms sample time

PID pid_controller(kp, ki, kd, sample_time);

void Switch_Stop()
{
    printf("Stop\n");
    speed = 0; // 速度0

    DATA[0] = (speed >> 8) & 0xFF; // 上位バイト
    DATA[1] = speed & 0xFF;        // 下位バイト
    DATA[2] = (speed >> 8) & 0xFF; // 上位バイト
    DATA[3] = speed & 0xFF;        // 下位バイト
    DATA[4] = (speed >> 8) & 0xFF; // 上位バイト
    DATA[5] = speed & 0xFF;        // 下位バイト
}

int main()
{
    button.mode(PullUp);
    button_reset.mode(PullUp);

    button.fall(&Switch_Stop); // 割り込み設定

    while (1)
    {

        auto now = HighResClock::now(); // タイマー設定
        static auto pre = now;
        bool sw = button_reset.read(); // ボタン読み取り

        if (now - pre > 500ms && sw == 0)
        {
            printf("Restart\n");
            speed = 16000; // 8191; // 速度MAX
            // mokuhyou = 12543;
            // float output = pid_controller.calculate(mokuhyou, sokudo);
            // printf("speed=%d\n", output);
            // int16_t mokuhyou_int16 = static_cast<int16_t>(output);
            printf("speed=%d\n", speed);

            DATA[0] = (speed >> 8) & 0xFF; // 上位バイト
            DATA[1] = speed & 0xFF;        // 下位バイト
            DATA[2] = (speed >> 8) & 0xFF; // 上位バイト
            DATA[3] = speed & 0xFF;        // 下位バイト
            DATA[4] = (speed >> 8) & 0xFF; // 上位バイト
            DATA[5] = speed & 0xFF;        // 下位バイト
            // mokuhyou_int16 = 0;
            // sokudo = 0;
            // output = 0;
            pre = now;
        }
        CANMessage msg(DJI_ID, DATA, 8); // 送信
        can.write(msg);
    }
}
