#include "Task.hpp"

void Task_Class::set_robot_vel(float cmd_forward_vel, float cmd_yaw_vel)
{
    bound(cmd_forward_vel, param.core.FORWARD_MAX_VEL);
    bound(cmd_yaw_vel, param.core.ROTATE_MAX_VEL);
    this->cmd.forward_input = cmd_forward_vel;
    this->cmd.rotate_input = cmd_yaw_vel;
}

void Task_Class::set_robot_pitch(float cmd_pitch_input, Command::input_mode_t cmd_mode)
{
    if (cmd_mode == Command::INPUT_VEL)
    {
        bound(cmd_pitch_input, param.core.MAX_PITCH_VEL);
    }
    else
    {
        bound(cmd_pitch_input, param.core.MAX_PITCH);
    }
    this->cmd.pitch_input = cmd_pitch_input;
    this->cmd.pitch.ctrl_mode = cmd_mode;
}

void Task_Class::set_modal(uint8_t modal)
{
    this->modal = modal;
}

void Task_Class::set_stand(bool stand_up)
{
    if (robot.mode == robot.TRANSFORM_REST && stand_up == true)
    {
        if (initHandle.get_init_val())
        {
            cmd.transform = Command::TRANSFORM_UP;
            cmd.enable_transform = true;
        }
        this->cmd.kill = false;
        this->start_reset = true;
    }
    else if (stand_up && robot.mode != robot.STAND && robot.mode != robot.TRANSFORM_UP && robot.mode != robot.TRANSFORM_REST)
    {
        cmd.transform = Command::TRANSFORM_UP;
        cmd.enable_transform = true;
        this->start_reset = false;
    }
    else if (robot.mode != robot.CAR && stand_up == false)
    {
        cmd.transform = Command::TRANSFORM_DOWN;
        cmd.enable_transform = false;
    }
}

void Task_Class::set_operate_mode(uint8_t operate_mode)
{
    this->operate_mode = operate_mode;
}

void Task_Class::set_brake(bool brake)
{
    this->brake = brake;
}

void Task_Class::set_roll_angle(float roll_angle)
{
    bound(roll_angle, param.core.MAX_ROLL_SET);

    this->cmd.roll_input = roll_angle;
}

void Task_Class::set_height(float height, Command::input_mode_t cmd_mode)
{
    // cmd.height_input  =  this->rc.height * (param.core.MAX_HEIGHT - param.core.MIN_HEIGHT) + param.core.MIN_HEIGHT;
    if (cmd_mode == Command::INPUT_POS)
    {
        bound(height, param.core.MAX_HEIGHT - 0.04f, param.core.MIN_HEIGHT);
    }
    else
    {
        bound(height, param.core.MAX_HEIGHT_VEL, param.core.MIN_HEIGHT);
    }
    this->cmd.height_input = height;
    this->cmd.height.ctrl_mode = cmd_mode;
}

void Task_Class::set_sensor_mode(uint8_t sensor_mode)
{
    this->sensor_mode = sensor_mode;
}

void Task_Class::set_imu_bias(float bias_x, float bias_y)
{
    this->param.imu_bias.x = bias_x;
    this->param.imu_bias.y = bias_y;
}
void Task_Class::set_cmd_lateral_vel(float cmd_lateral_vel)
{
    this->cmd_lateral_vel = cmd_lateral_vel;
}

void Task_Class::set_kill(bool kill)
{
    wdg.setFlag(RC_KILL, Watchdog::CRITICAL);
    cmd.kill = kill;
}

void Task_Class::set_cmd_reset(void)
{
    cmd.height.ctrl_mode = Command::INPUT_POS;
    cmd.pitch.ctrl_mode = Command::INPUT_VEL;
    cmd.kill = false;
    cmd.height_input = param.core.MIN_HEIGHT;
    cmd.pitch_input = 0;
    cmd.pitch.val = 0;
    cmd.pitch.d_val = 0;
    cmd.roll_input = 0;
    cmd.forward_input = 0;
    cmd.rotate_input = 0;
}

void Task_Class::get_robot_vel(float &cmd_forward_vel, float &cmd_yaw_vel)
{
    cmd_forward_vel = robot.posture_ptr->vel_forward;
    cmd_yaw_vel = robot.posture_ptr->vel_yaw;
}

void Task_Class::get_modal(uint8_t &modal)
{
    modal = this->fast_status;
}

void Task_Class::get_stand(bool &stand_up)
{
    stand_up = robot.mode;
}

void Task_Class::get_operate_mode(uint8_t &operate_mode)
{
    operate_mode = this->operate_mode;
}

void Task_Class::get_brake(bool &brake)
{
    brake = this->brake;
}

void Task_Class::get_cmd_lateral_vel(float &cmd_lateral_vel)
{
    cmd_lateral_vel = this->cmd_lateral_vel;
}

void Task_Class::get_roll_angle(float &roll_angle)
{
    roll_angle = robot.posture_ptr->roll;
}

void Task_Class::get_height(float &height)
{
    height = robot.posture_ptr->height;
}

void Task_Class::get_sensor_mode(uint8_t &sensor_mode)
{
    sensor_mode = this->sensor_mode;
}

void Task_Class::get_imu_bias(float &bias_x, float &bias_y)
{
    bias_x = this->imu_bias_x;
    bias_y = this->imu_bias_y;
}

void Task_Class::get_robot_mode(uint8_t &mode)
{
    mode = robot.mode;
}

void Task_Class::get_vel(float &forward_vel, float &yaw_vel)
{
    forward_vel = robot.posture_ptr->vel_forward; // this->forward // this->forward_vel;
    yaw_vel = robot.posture_ptr->vel_yaw;         // this->yaw_vel;
}

void Task_Class::get_tilt(float &l_tilt, float &l_d_tilt, float &r_tilt, float &r_d_tilt)
{
    l_tilt = robot.legL_ptr->tilt;
    l_d_tilt = robot.legL_ptr->d_tilt;
    r_tilt = robot.legR_ptr->tilt;
    r_d_tilt = robot.legR_ptr->d_tilt;
}

void Task_Class::get_pitch(float &pitch, float &d_pitch)
{
    pitch = robot.posture_ptr->head_pitch;
    d_pitch = robot.posture_ptr->d_head_pitch;
}

void Task_Class::get_leg_len(float &left_len, float &right_len)
{
    left_len = robot.legL_ptr->len;
    right_len = robot.legR_ptr->len;
}

void Task_Class::get_quat(float *quat)
{
    // quat = this->quat[id];
    quat[0] = robot.posture_ptr->_quaternion.w;
    quat[1] = robot.posture_ptr->_quaternion.x;
    quat[2] = robot.posture_ptr->_quaternion.y;
    quat[3] = robot.posture_ptr->_quaternion.z;
}

void Task_Class::get_omega_body(float *omega_body)
{
    omega_body[0] = robot.posture_ptr->_droll;
    omega_body[1] = robot.posture_ptr->d_head_pitch;
    omega_body[2] = robot.posture_ptr->d_yaw;
}

void Task_Class::get_acc_body(float *acc_body)
{
    acc_body[0] = robot.posture_ptr->accl[0]; // this->acc_body[id];
    acc_body[1] = robot.posture_ptr->accl[1];
    acc_body[2] = robot.posture_ptr->accl[2];
}

void Task_Class::get_error(uint32_t *error)
{
    error = robot.wdg.error;
}

const WB6_Parameter *Task_Class::get_robot_param()
{
    return robot.param_ptr;
}