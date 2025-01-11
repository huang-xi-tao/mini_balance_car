#include "Crash.hpp"

bool Crash_Class::Crash_checkout(float dt)
{
    switch (robot.mode)
    {
    case Robot_Class::CLASH_MODE:
        if (robot.cmd.crash_flag == 1 /*&& mabs(robot.posture_ptr->vel_forward) < 0.1*/)
        {
            robot.cmd.forward.d_val = 0;
            robot.cmd.crash_count++;
            if (robot.cmd.crash_count > 2000)
            {
                ctrl.lqr2.reset();
                ctrl.lqr_yaw.reset();
                planner.wheel_cmd_reset();
                // planner.jumpHandle.reset();
                robot.wdg.clearFlag(RC_KILL, Watchdog::CRITICAL);
                ctrl.set_d_tilt = 0;
                robot.cmd.crash_flag = 0;
                robot.cmd.crash_count = 0;
                robot.cmd.crash_restar = true;
            }
            return true;
        }

        break;

    case Robot_Class::STAND:
        if (/*robot.posture_ptr->height < 0.07f || robot.cmd.kill ||*/
            ((mabs(robot.legL_ptr->tilt) > 1.2f || mabs(robot.legR_ptr->tilt) > 1.2f)
             /* || (mabs(robot.legL_ptr->wheel_vel) > robot.legL_ptr->wheel.max_vel * 0.8f
              && mabs(robot.legR_ptr->wheel_vel) > robot.legR_ptr->wheel.max_vel * 0.8f
              && robot.posture_ptr->tilt > 0.8f)*/
             ) &&
            robot.cmd.crash_flag == 0)
        {
            robot.cmd.crashing_count++;
            if (robot.cmd.crashing_count > 200)
            {
                ctrl.transform(Robot_Class::CLASH_MODE);
                planner.wheel_cmd_reset();
                planner.jumpHandle.tilt_cmd_reset();
                ctrl.lqr2.reset();
                ctrl.lqr_yaw.reset();
                // planner.jumpHandle.reset();
                robot.cmd.crashing_count = 0;
                robot.cmd.crash_flag = 1;
                robot.cmd.crash_count = 0;
            }
            return true;
        }
        else
        {
            robot.cmd.crashing_count = 0;
            return false;
        }
        break;
        break;
    }
    return false;
}