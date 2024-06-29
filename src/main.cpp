#include "mbed.h"
#include "PID.hpp"

int mokuhyou = 0;
int16_t sokudo1 = 0;
int16_t sokudo2 = 0;
int16_t sokudo3 = 0;
int16_t sokudo4 = 0;

int DJI_ID = 0x1FF;

// PID controller parameters
const float kp = 0.03;
const float ki = 0.00;
const float kd = 0.00;
const float sample_time = 0.02; // 20ms sample time

// Create PID controller
PID pid_controller_1(kp, ki, kd, sample_time);
PID pid_controller_2(kp, ki, kd, sample_time);
PID pid_controller_3(kp, ki, kd, sample_time);
PID pid_controller_4(kp, ki, kd, sample_time);

int main()
{
    BufferedSerial pc(USBTX, USBRX, 115200);
    CAN can(PA_11, PA_12, (int)1e6);
    uint8_t DATA[8] = {};

    auto pre = HighResClock::now();

    while (1)
    {
        auto now = HighResClock::now();
        CANMessage msg, msg1, msg2;
        if (pc.readable())
        {
            char buf;
            pc.read(&buf, sizeof(buf));
            if (buf == 'w')
            {
                mokuhyou = 5500;
                printf("w \n");
            }
            else if (buf == 's')
            {
                mokuhyou = 5800;
            }
            else if (buf == 'a')
            {
                mokuhyou = 6000;
            }
            else if (buf == 'z')
            {
                mokuhyou = 0;
            }
        }

        // Calculate PID output
        float output_1 = pid_controller_1.calculate(mokuhyou, sokudo1);
        float output_2 = pid_controller_2.calculate(mokuhyou, sokudo2);
        float output_3 = pid_controller_3.calculate(mokuhyou, sokudo3);
        float output_4 = pid_controller_4.calculate(mokuhyou, sokudo4);

        int16_t out_1 = static_cast<int16_t>(-output_1);
        DATA[0] = out_1 >> 8;   // MSB
        DATA[1] = out_1 & 0xFF; // LSB

        int16_t out_2 = static_cast<int16_t>(output_2);
        DATA[2] = out_2 >> 8;   // MSB
        DATA[3] = out_2 & 0xFF; // LSB

        int16_t out_3 = static_cast<int16_t>(-output_3);
        DATA[4] = out_3 >> 8;   // MSB
        DATA[5] = out_3 & 0xFF; // LSB

        int16_t out_4 = static_cast<int16_t>(output_4);
        DATA[6] = out_4 >> 8;   // MSB
        DATA[7] = out_4 & 0xFF; // LSB

        if (now - pre > 10ms)
        {
            CANMessage msg(DJI_ID, DATA, 8);
            if (can.write(msg))
            {
                printf("OK\n");
            }

            if (can.read(msg1) && msg1.id == 0x205)
            {
                sokudo1 = (msg1.data[2] << 8) | msg1.data[3];
            }
            if (can.read(msg1) && msg1.id == 0x206)
            {
                sokudo2 = (msg1.data[2] << 8) | msg1.data[3];
            }
            if (can.read(msg1) && msg1.id == 0x207)
            {
                sokudo3 = (msg1.data[2] << 8) | msg1.data[3];
            }
            if (can.read(msg2) && msg2.id == 0x208)
            {
                sokudo4 = (msg2.data[2] << 8) | msg2.data[3];
            }
            printf("sokudo1 = :%d\nsokudo2 = :%d\nsokudo3 = :%d\nsokudo4 = :%d\n", sokudo1, sokudo2, sokudo3, sokudo4);
        }
        ThisThread::sleep_for(20ms);
        pre = now;
    }
}
