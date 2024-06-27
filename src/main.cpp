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

const float kp = 0.88;
const float ki = 0.30;
const float kd = 0.001;
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
    // auto pre_1 = pre;

    while (1)
    {
        auto now = HighResClock::now();
        // auto now_1 = HighResClock::now();

        auto start_time = HighResClock::now(); // PID計算開始時刻を記録
        target = 6000;
        while (HighResClock::now() - start_time < 500ms) // 1秒間PID計算を行う
        {

            float output = pid_controller.calculate(target, speed_now);
            if (target < output)
            {
                output = target;
            }
            //  printf("output = %.0f\n", output);
            int16_t output_int16 = static_cast<int16_t>(-output);
            DATA[0] = output_int16 >> 8;   // MSB
            DATA[1] = output_int16 & 0xFF; // LSB

            int16_t output1_int16 = static_cast<int16_t>(output);
            DATA[2] = output1_int16 >> 8;   // MSB
            DATA[3] = output1_int16 & 0xFF; // LSB
            int16_t output2_int16 = static_cast<int16_t>(-output);
            DATA[4] = output2_int16 >> 8;   // MSB
            DATA[5] = output2_int16 & 0xFF; // LSB

            int16_t output3_int16 = static_cast<int16_t>(output);
            DATA[6] = output3_int16 >> 8;   // MSB
            DATA[7] = output3_int16 & 0xFF; // LSB

            CANMessage msg(DJI_ID, DATA, 8);
            can.write(msg);

            if (can.read(msg) && msg.id == 0x201)
            {
                int16_t speed_1 = (msg.data[2] << 8) | msg.data[3];
                printf("1 = %d\n", speed_1);
            }
            if (can.read(msg) && msg.id == 0x202)
            {
                int16_t speed_2 = (msg.data[2] << 8) | msg.data[3];
                printf("2 = %d\n", speed_2);
            }
            if (can.read(msg) && msg.id == 0x203)
            {
                int16_t speed_3 = (msg.data[2] << 8) | msg.data[3];
                printf("3 = %d\n", speed_3);
            }
            if (can.read(msg) && msg.id == 0x204)
            {
                int16_t speed_4 = (msg.data[2] << 8) | msg.data[3];
                printf("4 = %d\n", speed_4);
            }
            ThisThread::sleep_for(20ms);
        }
        stop_motor(0);
        pre = now;   // 次の計算のために現在時刻を更新
        flag = true; // 処理完了フラグを立てる
        break;
    }
}
