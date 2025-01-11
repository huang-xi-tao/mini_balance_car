#include "Planner.hpp"
#include <math.h>

void Planner_Class::wheel_cmd_reset(const float forward_err, const float yaw_err)
{
    cmd.forward.val = (robot.legL_ptr->wheel_pos + robot.legR_ptr->wheel_pos) / 2.f + forward_err;
    cmd.forward.d_val = (robot.legL_ptr->wheel_vel + robot.legR_ptr->wheel_vel) / 2.f;
    cmd.forward.rev = robot.legL_ptr->wheel.rev + robot.legR_ptr->wheel.rev;

    while (cmd.forward.val > M_PI)
    {
        cmd.forward.val -= 2 * M_PI;
        cmd.forward.rev += 2;
    }

    while (cmd.forward.val < -M_PI)
    {
        cmd.forward.val += 2 * M_PI;
        cmd.forward.rev -= 2;
    }

    this->yaw_cmd_reset();
}

void Planner_Class::yaw_cmd_reset(void)
{
    cmd.yaw.val = robot.posture_ptr->yaw;
    cmd.yaw.rev = robot.posture_ptr->yaw_rev * 2;
    cmd.yaw.d_val = robot.posture_ptr->d_yaw;
}

void Planner_Class::wheel_cmd(float forward_input /*velocity m/s*/,
                              float rotation_input /*yaw, rad/s*/,
                              const float dt)
{
    float accl_limit = 0, accl_limit_minus = 0;
    if (robot.flag_ptr->balance)
    {
        accl_limit = 0.5f * robot.param_ptr->core.ACCL_LIMIT * (robot.posture_ptr->leg_len - robot.param_ptr->core.LEG_MIN_LEN) / (robot.param_ptr->core.LEG_LEN_STRAIGHT - robot.param_ptr->core.LEG_MIN_LEN) + 0.5f * robot.param_ptr->core.ACCL_LIMIT; // accelerate from 0.5a ~ 1a
        accl_limit_minus = -(accl_limit / robot.param_ptr->core.FORWARD_MAX_VEL) * cmd.forward.d_val * robot.param_ptr->core.WHEEL_RADIUS - accl_limit;
        accl_limit = -(accl_limit / robot.param_ptr->core.FORWARD_MAX_VEL) * cmd.forward.d_val * robot.param_ptr->core.WHEEL_RADIUS + accl_limit;
    }
    else
    {
        accl_limit = 0.5f * robot.param_ptr->core.ACCL_LIMIT;
        accl_limit_minus = -accl_limit;
    }
    // 31V as full
    float max_vel_psc = robot.power_ptr->Vin / 31.f;
    bound(max_vel_psc, 1.f, 0.6f);

    bound(accl_limit, robot.param_ptr->core.ACCL_LIMIT * 1.0f);
    bound(accl_limit_minus, robot.param_ptr->core.ACCL_LIMIT * 1.0f);
    // bound the rotate input
    bound(rotation_input, robot.param_ptr->core.ROTATE_MAX_VEL);
    // bound the forward input
    bound(forward_input, robot.param_ptr->core.FORWARD_MAX_VEL * max_vel_psc);

    float rotation_input_linear = rotation_input * robot.param_ptr->core.WHEEL_DISTANCE / 2.f * 2.f; // get the yaw linear speed

    bound(rotation_input_linear, robot.param_ptr->core.FORWARD_MAX_VEL * max_vel_psc); // avoid sqrt a minus number

    float max_forward_vel = sqrt(robot.param_ptr->core.FORWARD_MAX_VEL * robot.param_ptr->core.FORWARD_MAX_VEL * max_vel_psc * max_vel_psc - rotation_input_linear * rotation_input_linear); // get the maximum forward vel

    float min_forward_vel = -max_forward_vel;

    float forward_input_del = (robot.param_ptr->core.FORWARD_MAX_VEL * max_vel_psc - mabs(forward_input) + 0.5f);
    if (forward_input_del > 1.f)
    {
        forward_input_del = 1;
    }
    float rotate_limit = robot.param_ptr->core.ANG_ACCL_LIMIT *
                         (1 - 0.5f * robot.posture_ptr->height / robot.param_ptr->core.LEG_LEN_STRAIGHT) * forward_input_del;
    // slope for forward
    float max_vel_cmd = cmd.forward.d_val * robot.param_ptr->core.WHEEL_RADIUS + accl_limit * dt, // d_val :angle velocity, not linear
        min_vel_cmd = cmd.forward.d_val * robot.param_ptr->core.WHEEL_RADIUS + accl_limit_minus * dt;
    bound(forward_input, max_vel_cmd, min_vel_cmd); // m/s
    // bound the final forward input
    bound(forward_input, max_forward_vel, min_forward_vel);

    // slope for yaw
    float max_ang_cmd = cmd.yaw.d_val + rotate_limit * dt,
          min_ang_cmd = cmd.yaw.d_val - rotate_limit * dt;

    //(robot.param_ptr->core.FORWARD_MAX_VEL,3)   (x1,y1)
    //(robot.param_ptr->core.FORWARD_MAX_VEL / 2.f,5)   (x2,y2)
    float forward_vel_x = robot.posture_ptr->vel_forward;
    // get the slope of the maximum angle input
    bound(forward_vel_x, robot.param_ptr->core.FORWARD_MAX_VEL * max_vel_psc, robot.param_ptr->core.FORWARD_MAX_VEL / 2.f);
    float max_ang_k = robot.param_ptr->core.ROTATE_MAX_VEL - mabs(forward_vel_x) / robot.param_ptr->core.FORWARD_MAX_VEL * 1 - 0.7f;

    bound(rotation_input, max_ang_k);
    cmd.forward.input_rev(forward_input / robot.param_ptr->core.WHEEL_RADIUS, dt); // rad/s
    cmd.yaw.input_rev(rotation_input, dt);                                         // yaw.d_val update in here
}

void Planner_Class::height_cmd(float input, float dt)
{
    cmd.height.input_limit(input, dt,
                           robot.param_ptr->core.MAX_HEIGHT_VEL, -robot.param_ptr->core.MAX_HEIGHT_VEL,
                           robot.param_ptr->core.MAX_HEIGHT, robot.param_ptr->core.MIN_HEIGHT);
}

void Planner_Class::pitch_cmd(float input, float dt)
{
    float max = ctrl.set_leg_len < pitch_limit_height ? pitch_limit_angle + 0.01 : robot.param_ptr->core.MAX_PITCH;
    if (last_flag)
    {
        last_pitch_val = ctrl.set_pitch;
    }
    else
    {
        last_flag = 1;
        cmd.pitch.val = last_pitch_val;
    }
    if (robot.mode == Robot_Class::TRANSFORM_DOWN)
    {
        last_flag = 0;
        cmd.pitch.input_limit(input, dt, robot.param_ptr->core.MAX_PITCH_VEL, -robot.param_ptr->core.MAX_PITCH_VEL, 0.1, -0.5);
    }
    else
    {
        cmd.pitch.input_limit(input, dt, robot.param_ptr->core.MAX_PITCH_VEL, -robot.param_ptr->core.MAX_PITCH_VEL, max, -robot.param_ptr->core.MAX_PITCH);
    }
}

void Planner_Class::roll_cmd(const float input, const float dt)
{
    const float LEG_MAX_DIFF_ANGLE = atanf((robot.param_ptr->core.LEG_MAX_LEN - robot.param_ptr->core.LEG_MIN_LEN) / robot.param_ptr->core.WHEEL_DISTANCE);

    if (!robot.flag_ptr->balance || robot.mode == Robot_Class::TRANSFORM_UP)
    {
        ctrl.set_roll = robot.posture_ptr->ground_tilt;
        ctrl.set_d_roll = 0;

        return;
    }

    float cmd_forward = robot.posture_ptr->vel_forward, // cmd.forward.d_val * robot.param_ptr->core.WHEEL_RADIUS,
        cmd_rotate = robot.posture_ptr->vel_yaw;        // cmd.yaw.d_val;
    float cmd_forward2 = cmd.forward.d_val * robot.param_ptr->core.WHEEL_RADIUS,
          cmd_rotate2 = cmd.yaw.d_val;
    float result = -(cmd_forward * cmd_rotate) / GRAVITY;
    float result2 = -(cmd_forward2 * cmd_rotate2) / GRAVITY;
    deadzone(result, 0.04f);
    deadzone(result2, 0.04f);
    // result = 0;
    ctrl.set_d_roll = (result + result2) / 2.f;
    float Cmd = input + ctrl.set_d_roll,
          d_Cmd = 0;

    float max_cmd = tanf(robot.posture_ptr->ground_tilt + LEG_MAX_DIFF_ANGLE),
          min_cmd = tanf(robot.posture_ptr->ground_tilt - LEG_MAX_DIFF_ANGLE);
    if (max_cmd > robot.param_ptr->core.MAX_ROLL)
        max_cmd = robot.param_ptr->core.MAX_ROLL;
    if (min_cmd < -robot.param_ptr->core.MAX_ROLL)
        min_cmd = -robot.param_ptr->core.MAX_ROLL;

    if (Cmd < min_cmd)
    {
        Cmd = min_cmd;
        d_Cmd = 0;
    }
    else if (Cmd > max_cmd)
    {
        Cmd = max_cmd;
        d_Cmd = 0;
    }

    float maxRoll = ctrl.set_roll + robot.param_ptr->core.MAX_ROLL_RATE * dt,
          minRoll = ctrl.set_roll - robot.param_ptr->core.MAX_ROLL_RATE * dt;

    if (Cmd > maxRoll)
    {
        ctrl.set_roll = maxRoll;
        ctrl.set_d_roll = 0;
    }
    else if (Cmd < minRoll)
    {
        ctrl.set_roll = minRoll;
        ctrl.set_d_roll = 0;
    }
    else
    {
        ctrl.set_roll = Cmd;
        ctrl.set_d_roll = d_Cmd;
    }
    // ctrl.set_roll = 0;
}

void Planner_Class::leg_len_cmd(const float cmd, const float d_cmd)
{
    float cmd_len = cmd / cosf(robot.posture_ptr->_pitch);

    float d_length = mabs(robot.legL_ptr->len - robot.legR_ptr->len) / 2.f;
    float max_len = (robot.param_ptr->core.LEG_MAX_LEN - d_length), min_len = 0;
    if (robot.mode == Robot_Class::STAND)
        min_len = (robot.param_ptr->core.LEG_MIN_LEN + d_length);
    else if (robot.mode == Robot_Class::TRANSFORM_UP)
    {
        min_len = (robot.param_ptr->core.LEG_MIN_LEN);
    }
    else
    {
        min_len = (robot.param_ptr->core.LEG_LEN_RETRACT);
    }
    if (cmd_len > max_len)
    {
        ctrl.set_leg_len = max_len;
        ctrl.set_d_leg_len = 0;
    }
    else if (cmd_len < min_len)
    {
        ctrl.set_leg_len = min_len;
        ctrl.set_d_leg_len = 0;
    }
    else
    {
        ctrl.set_leg_len = cmd_len;
        ctrl.set_d_leg_len = d_cmd;
        // ctrl.set_d_leg_len = d_cmd / robot.posture_ptr->cos_tilt +
        //                      cmd * sinf(robot.posture_ptr->tilt) / (robot.posture_ptr->cos_tilt * robot.posture_ptr->cos_tilt) * robot.posture_ptr->d_tilt;
    }
}

void Planner_Class::tail_cmd(const float input, const float dt)
{
    cmd.tail_angle.input_limit(input, dt,
                               robot.param_ptr->core.TAIL_CHANGE_VEL, -robot.param_ptr->core.TAIL_CHANGE_VEL,
                               0, -M_PI);
}

float debug_forward_err = 0;
void Planner_Class::wheel_plan(const float dt, float forward_d_val)
{
    pos = (robot.legL_ptr->wheel_pos + robot.legR_ptr->wheel_pos) / 2.f + (robot.legL_ptr->wheel.rev + robot.legR_ptr->wheel.rev) * M_PI;

    if (ctrl.lqr2.run_status > 1 && ctrl.lqr2.run_status < 4)
    {
        pos_rec = pos + 6 * forward_d_val * dt;
        pos_cmd_stop = pos_rec;
    }
    if (ctrl.lqr2.run_status == 4)
    {
        pos_rec = pos + 6 * robot.posture_ptr->vel_forward * dt / robot.param_ptr->core.WHEEL_RADIUS;
    }

    if (ctrl.lqr2.lqr_start_flag == false)
    {
        pos_rec = pos;
    }
    static float err = 0;
    err = (this->pos_rec /*+ forward_d_val * dt*/ - pos) * robot.param_ptr->core.WHEEL_RADIUS;

    if (err > pos_err_limit)
        err = pos_err_limit;
    else if (err < -pos_err_limit)
        err = -pos_err_limit;
    if (robot.mode == robot.CAR)
    {
        pos_rec = pos;
    }

    ctrl.pos_forward_err = err / robot.param_ptr->core.WHEEL_RADIUS;
    ctrl.lqr2.pos_cmd_err = (pos_cmd_stop - pos);
}

void Planner_Class::plan_update(float dt)
{
    static bool down_finish = false;
    float setPitch = 0;
    float set_d_pitch = 0;
    bound(dt, 0.05f, 0.f);

    transform_handler(cmd.transform);
    this->wheel_cmd(cmd.forward_input, cmd.rotate_input, dt);
    this->roll_cmd(cmd.roll_input + robot.param_ptr->imu_bias.x, dt);
    this->height_cmd(cmd.height_input, dt);
    this->jumpHandle.update(dt);
    switch (robot.mode)
    {
    case Robot_Class::CAR:
        robot.Set_Robot_Flag(Robot_Class::BALANCE, false);
        break;
    case Robot_Class::TRANSFORM_UP: // 在这里安排ramp函数决定设定值何时执行
        this->upHandle.update(dt);
        robot.tail_phase_up = this->upHandle.phase;
        if (this->upHandle.phase == 0)
        {
            this->leg_len_cmd(this->upHandle.height_change(robot.param_ptr->core.MAX_HEIGHT), 0);
            if (this->upHandle.height_ramp.finish())
            {
                this->upHandle.height_ramp.reset(1.0);
                this->upHandle.init_height = robot.param_ptr->core.MAX_HEIGHT;
                this->upHandle.phase = 1;
            }
        }
        else if (this->upHandle.phase == 1)
        {
            this->tail_cmd(this->upHandle.tail_change(robot.param_ptr->core.TAIL_UP_POS), dt);
            if (this->upHandle.tail_ramp.finish())
            {
                this->upHandle.phase = 2;
                this->upHandle.height_ramp.reset(1.0);
            }
        }
        else if (this->upHandle.phase == 2)
        {
            this->leg_len_cmd(this->upHandle.height_change(cmd.height.val), 0);
        }

        ctrl.set_roll = this->upHandle.roll_change(ctrl.set_roll);
        if (this->upHandle.finish() && this->upHandle.phase == 2)
        {
            ctrl.transform(Robot_Class::STAND);
            this->upHandle.phase = 0;
        }
        break;

    case Robot_Class::TRANSFORM_DOWN:
        this->downHandle.update(dt);
        robot.tail_phase_down = this->downHandle.phase;
        if (this->downHandle.phase == 0) // 抬腿
        {
            this->downHandle.dest_height = this->robot.param_ptr->core.MAX_HEIGHT;
            this->leg_len_cmd(this->downHandle.height_change(cmd.height.val), 0);
            if (this->downHandle.height_ramp.finish())
            {
                this->downHandle.height_ramp.reset(2.0);
                this->downHandle.phase = 1;
            }
        }
        else if (this->downHandle.phase == 1) // 转尾巴
        {
            this->tail_cmd(this->downHandle.tail_change(robot.param_ptr->core.TAIL_UP_POS), dt);
            if (this->downHandle.tail_ramp.finish())
            {
                this->downHandle.phase = 2;
            }
        }
        else if (this->downHandle.phase == 2) // 下降高度
        {
            this->downHandle.dest_height = this->robot.param_ptr->core.MIN_HEIGHT;
            this->leg_len_cmd(this->downHandle.height_change(this->robot.param_ptr->core.MAX_HEIGHT), 0);
        }
        // setPitch = this->downHandle.head_angle_change(cmd.pitch.val);
        // ctrl.set_roll = this->downHandle.roll(ctrl.set_roll);
        if (this->downHandle.finish() && this->downHandle.phase == 2)
        {
            // 重新回到最开始的模样
            ctrl.transform(Robot_Class::CAR);
            this->downHandle.phase = 0;
            this->upHandle.init_tail_angle = robot.param_ptr->core.TAIL_DOWN_POS;
        }
        // printf("phase :%d", this->downHandle.phase);
        break;
    case Robot_Class::TRANSFORM_REST:
        this->wheel_cmd_reset();
        this->jumpHandle.tilt_cmd_reset();
        if (!down_finish)
        {
            this->reach_ground = true;
            down_finish = true;
        }
        break;
    case Robot_Class::STAND:
        setPitch = cmd.pitch.val;
        set_d_pitch = 0;

        robot.Set_Robot_Flag(Robot_Class::BALANCE, true);
        this->leg_len_cmd(this->jumpHandle.set_height,
                          this->jumpHandle.set_d_height);
        this->pitch_cmd(cmd.pitch_input, dt);
        down_finish = false;
        break;
    }

    ctrl.set_pitch = setPitch;
    ctrl.set_d_pitch = set_d_pitch;
}
void Planner_Class::plan_start(void)
{
    robot.wdg.start();
    this->wheel_cmd_reset();
    // robot.mode = robot.CAR;
    ctrl.set_tilt = robot.param_ptr->imu_bias.y;
    robot.condition = Robot_Class::INITIALIZE;
    robot.Set_Robot_Flag(Robot_Class::BALANCE, false);
    robot.Set_Robot_Flag(Robot_Class::LEG_ON, false);
}

void Planner_Class::transform_handler(const uint8_t input)
{
    switch (robot.mode)
    {
    case Robot_Class::CAR:
        if (input & Command::TRANSFORM_UP)
        {
            this->upHandle.start();
            ctrl.transform(Robot_Class::TRANSFORM_UP);
        }
        break;
    case Robot_Class::STAND:
    case Robot_Class::TRANSFORM_UP:
        if (input & Command::TRANSFORM_DOWN)
        {
            this->downHandle.start();
            ctrl.transform(Robot_Class::TRANSFORM_DOWN);
        }
        break;
    case Robot_Class::CLASH_MODE:

        break;
    }
}

void Planner_Class::wdg_check_update(float dt)
{
    bound(dt, 0.05f, 0.f);

    if (cmd.kill /*|| cmd.crash_flag || ((mabs(robot.legL_ptr->tilt) > 1.4 || mabs(robot.legR_ptr->tilt)>1.4) && (robot.mode == Robot_Class::STAND))*/)
    {
        this->kill_flag = true;
    }
    else
    {
        wdg.clearFlag(RC_KILL, Watchdog::CRITICAL);
    }
}
