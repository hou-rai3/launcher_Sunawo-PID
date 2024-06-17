#include "mbed.h"
#include "PID.hpp"

InterruptIn button(PC_8);       // リミットスイッチ
InterruptIn button_reset(PC_9); // 起動スイッチ
int DJI_ID = 0x200;

CAN can(PA_11, PA_12, (int)1e6);
volatile int32_t encoder_position = 0;

DigitalIn encoderA(PB_13); // A相のピン A0からA5
DigitalIn encoderB(PB_14); // B相のピン A0からA5
int counter = 0;
int counter_r = 0;
bool prevStateA = 0;
bool prevStateB = 0;
uint8_t DATA[8] = {0};
int16_t speed = 0;
// sw_stopが押されたときに実行される割り込みハンドラ
void stop_motor()
{
    speed = 0; // 速度0
    for (int i = 0; i < 8; i += 2)
    {
        DATA[i] = (speed >> 8) & 0xFF; // 上位バイト
        DATA[i + 1] = speed & 0xFF;    // 下位バイト
    }
}

void reset_can()
{
    can.reset();
}

int main()
{
    button.mode(PullUp);
    button_reset.mode(PullUp);
    BufferedSerial pc(USBTX, USBRX, 115200);
    auto pre = HighResClock::now();
    auto pre_1 = pre;

    while (1)
    {
        auto now = HighResClock::now();
        bool sw = button_reset.read();
        bool sw_stop = button.read();

        if (sw_stop == 0 && now - pre > 500ms)
        {
            printf("STOP\n");
            stop_motor();
            pre = now;
        }

        if (now - pre > 1000ms && sw == 0)
        {
            printf("FIRE\n");
            counter = 0;
            // 速度を設定する例
            speed = 3000; // 速度MAX

            // 符号付き16ビットのエンコード
            int16_t signed_speed = static_cast<int16_t>(-speed);
            DATA[0] = (signed_speed >> 8) & 0xFF; // 上位バイト
            DATA[1] = signed_speed & 0xFF;        // 下位バイト
            signed_speed = static_cast<int16_t>(speed);
            DATA[2] = (signed_speed >> 8) & 0xFF; // 上位バイト
            DATA[3] = signed_speed & 0xFF;        // 下位バイト
            signed_speed = static_cast<int16_t>(-speed);
            DATA[4] = (signed_speed >> 8) & 0xFF; // 上位バイト
            DATA[5] = signed_speed & 0xFF;        // 下位バイト
            signed_speed = static_cast<int16_t>(speed);
            DATA[6] = (signed_speed >> 8) & 0xFF; // 上位バイト
            DATA[7] = signed_speed & 0xFF;        // 下位バイト

            pre = now;   // タイマーリセット
            reset_can(); // CANコントローラをリセット
        }

        if (now - pre_1 > 30ms)
        {
            CANMessage msg(DJI_ID, DATA, 8);
            if (can.write(msg))
            {
                reset_can(); // CANコントローラをリセット
                //  printf("OK\n");
            }
            else
            {
                printf("Can't send Message\n");
                // 送信失敗時の追加デバッグ情報
                printf("CAN Bus Error Status: %d\n", can.rderror());
                printf("CAN Bus Write Error Count: %d\n", can.tderror());
                if (can.rderror() == 255 || can.tderror() == 249)
                {
                    printf("Resetting CAN controller\n");
                    reset_can(); // CANコントローラをリセット
                }
            }
            pre_1 = now;
        }
    }
}
