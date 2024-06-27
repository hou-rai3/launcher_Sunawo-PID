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
void motor_control(int16_t target, int16_t speed_now, PID &pid_controller, CAN &can, CANMessage &msg, uint8_t *DATA, int DJI_ID, bool &flag)
{
    if (msg.id == 0x201)
    {
        speed_now = (msg.data[0] << 8) | msg.data[1];
    }
    float output = pid_controller.calculate(target, speed_now); // PID計算
    int16_t motor_speed = static_cast<int16_t>(-output);        // 出力値を速度として使用
    DATA[0] = (motor_speed >> 8) & 0xFF;                        // 上位バイト
    DATA[1] = motor_speed & 0xFF;                               // 下位バイト
    motor_speed = static_cast<int16_t>(output);
    DATA[2] = (motor_speed >> 8) & 0xFF; // 上位バイト
    DATA[3] = motor_speed & 0xFF;        // 下位バイト
    motor_speed = static_cast<int16_t>(-output);
    DATA[4] = (motor_speed >> 8) & 0xFF; // 上位バイト
    DATA[5] = motor_speed & 0xFF;        // 下位バイト
    motor_speed = static_cast<int16_t>(output);
    DATA[6] = (motor_speed >> 8) & 0xFF; // 上位バイト
    DATA[7] = motor_speed & 0xFF;        // 下位バイト
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
        bool sw = button_reset.read();

        if (now - pre > 1000ms && sw == 0)
        {
            printf("FIRE\n");
            target = 16000;                        // 目標速度を設定
            auto start_time = HighResClock::now(); // PID計算開始時刻を記録

            while (HighResClock::now() - start_time < 1000ms) // 1秒間PID計算を行う
            {
                void motor_control(int16_t target, int16_t speed_now, PID & pid_controller, CAN & can, CANMessage & msg, uint8_t * DATA, int DJI_ID, bool &flag);

                CANMessage msg(DJI_ID, DATA, 8);
                if (can.write(msg))
                {
                    printf("PID Output Sent\n");
                }
                else
                {
                    printf("Can't send Message\n");
                    can.reset();
                }

                // 速度センサーからの読み取り値を更新する場合はここで行う
                // speed_now = <センサーからの読み取り値>;

                ThisThread::sleep_for(20ms); // PID計算のサンプル時間に合わせて待機
            }

            pre = now;   // 次の計算のために現在時刻を更新
            flag = true; // 処理完了フラグを立てる
        }
    }
}
