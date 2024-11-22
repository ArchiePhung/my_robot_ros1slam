#ifndef MAIN_H
#define MAIN_H

// all things relevant to robot and manipulate robot
class Robot
{
    private:
    public:
        int dir = 0;
        // Navigate_robot();
        // ~Navigate_robot();
        void run(int dir_, int Motor_speeda, int Motor_speedb);
        void spin(int direction, int motor_speed);
        void stop();
        void run_backward();
        void run_forward();
        void turn_left();
        void turn_right();
        int gain_dir(int x, int y);

};

// all things relevant to ESP32 controller
class ESP32_Controller
{
    private:
    public:
        // ESP32_Controller();
        // ~ESP32_Controller();
        Robot* rb_ctrl;
        void pin_init();
        void pwm_pin_init();
        void launch_init();

};

#endif