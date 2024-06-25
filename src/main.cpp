#include "mbed.h"
#include "PID.hpp"

CAN can(PA_11, PA_12, (int)1e6);
CANMessage msg;

uint8_t DATA[8] = {0};
int DJI_ID = 0x200;


InterruptIn button_reset(PC_9); // 起動スイッチ
bool flag = false;

int16_t speed_now = 0;
int16_t target = 0;

const float kp = 0.130;
const float ki = 0.035;
const float kd = 0.0055;
// const float kp = 0.1;
// const float ki = 0.035;
// const float kd = 0.0;
const float sample_time = 0.02; // 20ms sample time
PID pid_controller(kp, ki, kd, sample_time);

void stop_motor(int zero)
{
    printf("STOP\n");
    for (int i = 0; i < 8; i += 2)
    {
        DATA[i] = (zero >> 8) & 0xFF; // 上位バイト
        DATA[i + 1] = zero & 0xFF;    // 下位バイト
    }
}

int main()
{
    button_reset.mode(PullUp);
    BufferedSerial pc(USBTX, USBRX, 115200);
    auto pre = HighResClock::now();
    auto pre_1 = pre;

    while (1)
    {
        auto now = HighResClock::now();
        auto now_1 = HighResClock::now();
        bool sw = button_reset.read();

        if (now - pre > 1000ms && sw == 0)
        {
            printf("FIRE\n");
            target = 15000;
            float output = pid_controller.calculate(target, speed_now);
            target = static_cast<int16_t>(-output);
            DATA[0] = (target >> 8) & 0xFF; // 上位バイト
            DATA[1] = target & 0xFF;        // 下位バイト
            target = static_cast<int16_t>(output);
            DATA[2] = (target >> 8) & 0xFF; // 上位バイト
            DATA[3] = target & 0xFF;        // 下位バイト
            target = static_cast<int16_t>(-output);
            DATA[4] = (target >> 8) & 0xFF; // 上位バイト
            DATA[5] = target & 0xFF;        // 下位バイト
            target = static_cast<int16_t>(output);
            DATA[6] = (target >> 8) & 0xFF; // 上位バイト
            DATA[7] = target & 0xFF;        // 下位バイト
            speed_now = output;
            pre = now;
            can.reset(); // CANコントローラをリセット
            flag = true;
        }

        if (flag == true && now - pre > 1000ms)
        {
            stop_motor(0);
            flag = false;
        }

        if (now_1 - pre_1 > 10ms)
        {
            CANMessage msg(DJI_ID, DATA, 8);
            if (can.write(msg))
            {
                can.reset();
                // CANコントローラをリセット
                //  printf("OK\n");
            }
            else
            {
                printf("Can't send Message\n");
                printf("CAN Bus Error Status: %d\n", can.rderror());
                printf("CAN Bus Write Error Count: %d\n", can.tderror());
                if (can.rderror() == 255 || can.tderror() == 249)
                {
                    printf("Resetting CAN controller\n");
                    can.reset();
                }
            }
            pre_1 = now_1;
        }
    }
}
