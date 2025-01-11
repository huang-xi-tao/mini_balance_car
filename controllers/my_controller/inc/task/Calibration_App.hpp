#pragma once

#include "robot_likeAscento.hpp"
#include "Controller.hpp"
#include "LinearFit.hpp"

namespace Calibration_App
{
    class Base
    {
    protected:
        Base(Robot_Class &robot, const Controller_Class &ctrl,
             WB6_Parameter &param) : robot(robot),
                                     ctrl(ctrl),
                                     param(param),
                                     cmd(robot.cmd)
        {
            this->started =
                this->finish = false;
            this->step = 0;
        }

        void start(void)
        {
            this->step = 0;
            this->started = true;
            this->finish = false;
        }

        virtual void update(Cmd &cmd, const float dt) = 0;

        void proceed(void)
        {
            step++;
        }

        void reset(void)
        {
            step = 0;
        }

        const Robot_Class &robot;
        const Controller_Class &ctrl;
        WB6_Parameter &param;
        Cmd &cmd;

        uint8_t step;

    public:
        bool started;
        bool finish;
    };

    class Head_mass : public Base
    {
    public:
        Head_mass(Robot_Class &robot, const Controller_Class &ctrl, WB6_Parameter &param) : Base(robot, ctrl, param)
        {
            this->in_position_cnt = this->capture_cnt = 0;
            this->ctrl_pos = 0;
            this->calibration_error = 0;
        }

        void start(void);

        virtual void update(Cmd &cmd, const float dt);

        float ctrl_pos;
        float calibration_error;

    private:
        LinearFit<float, 100> fitData;

        bool in_position(void);
        void capture_data(void);

        uint8_t in_position_cnt;
        uint8_t capture_cnt;
    };
}
