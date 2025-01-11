#include "keyboard.hpp"

void remote_cmd::keyboard_init(void)
{
    // this->rc    = &robot_task.RC;
}

void remote_cmd::stop_move(float dt)
{
    rc.move_forward = 0.f;
    rc.move_left = 0.f;
    if (rc.tilt_left > 0)
        rc.tilt_left -= 0.1f * dt;
    else if (rc.tilt_left < 0)
        rc.tilt_left += 0.1f * dt;
    else
        rc.tilt_left = 0;

    rc.head_up = 0.f;
}

void remote_cmd::check_key_release(float dt)
{
    this->key_status = wb_keyboard_get_key();
    if (this->key_status == -1 || this->key_status == 52)
    {
        this->release_time += dt;
        if (this->release_time > 0.05f)
        {
            this->release_key_flag = true;
        }
        bound(this->release_time, 2.f, 0.f);
    }
    else
    {
        this->release_time = 0.f;
        this->release_key_flag = false;
    }
    if (this->release_key_flag)
        key_control.stop_move(dt);
}
/**
 * @brief:键盘输入检测
 * @author: Dandelion
 * @Date: 2024-05-28 11:15:59
 * @param {Task_Class} &robot_task
 * @param {float} dt
 * @return {*}
 */
void remote_cmd::keyboard_update(Task_Class &robot_task, float dt)
{
    static bool down_up = false;
    float speed_psc = 1.0f;
    float speed_psc_rotate = 1.0f;
    float head_psc = 1.0f;
    float head_psc_lr = 1.0f;
    float height_psc = 1.0f;

    bound(dt, 0.05f, 0.f);

    key_control.check_key_release(dt);
    // this->key_status = wb_keyboard_get_key();
    // key_control.key_status = 43;
    switch (this->key_status)
    {
    /*Body motion instruction:make the robot go forward,back and go left ,right.*/
    case remote_cmd::robot_key::FOWARD:
    {
        rc.move_forward += speed_psc * dt;

        bound(rc.move_forward, 1.0f, -1.0f);
    }
    break;
    case remote_cmd::robot_key::BACK:
    {
        rc.move_forward -= speed_psc * dt;
        bound(rc.move_forward, 1.0f, -1.0f);
    }
    break;
    case remote_cmd::robot_key::LEFT:
    {
        rc.move_left += speed_psc_rotate * dt;
        bound(rc.move_left, 1.0f, -0.1f);
    }
    break;
    case remote_cmd::robot_key::RIGHT:
    {
        rc.move_left -= speed_psc_rotate * dt;
        bound(rc.move_left, 1.0f, -1.0f);
    }
    break;

    // /*Head motion instruction:make the robot head turn up,dowm and turn left and right.*/
    // case remote_cmd::robot_key::HEAD_UP:
    // {
    //     rc.head_up += head_psc * dt;
    //     bound(rc.head_up, 0.5f, -0.5f);
    //     // printf("HEAD_UP:%d,headangle:%0.3f\n",key_status,rc.head_up);
    // }
    // break;
    // case remote_cmd::robot_key::HEAD_DOWM:
    // {
    //     rc.head_up -= head_psc * dt;
    //     bound(rc.head_up, 0.5f, -.5f);
    //     // printf("HEAD_DOWM:%d,headangle:%0.3f\n",key_status,rc.head_up);
    // }
    // break;
    case remote_cmd::robot_key::HEAD_LEFT:
    {
        rc.tilt_left -= head_psc_lr * dt;

        bound(rc.tilt_left, 1.f, -1.f);
    }
    break;
    case remote_cmd::robot_key::HEAD_RIGHT:
    {
        rc.tilt_left += head_psc_lr * dt;
        bound(rc.tilt_left, 1.f, -1.f);
    }
    break;

        /*set the robot hight and jump status*/
        // case remote_cmd::robot_key::JUMP:
        // {
        //     rc.enable_jump = true;
        // }
        // break;
    case remote_cmd::robot_key::MASS_HIGHT:
    {
        rc.height += height_psc * dt;
        bound(rc.height, 1.0f, 0.f);
    }
    break;
    case remote_cmd::robot_key::MASS_DOWM:
    {
        rc.height -= height_psc * dt;
        bound(rc.height, 1.f, 0.f);
    }
    break;

    /*robot mode instruction*/
    case remote_cmd::robot_key::TRANSFORM_UP:
    {
        down_up = true;
    }
    break;
    case remote_cmd::robot_key::TRANSFORM_DOWM:
    {
        down_up = false;
    }
    break;
    // case remote_cmd::robot_key::JUMP_STEP:
    // {
    //     printf("jumpsteps!!!\n");
    // }
    // break;

    /*other instruction*/
    case remote_cmd::robot_key::EMERGENCY_STOP:
    {
    }
    break;
    case remote_cmd::robot_key::BRAKE:
    {
    }
    break;
    case remote_cmd::robot_key::RESET:
    {
        robot_task.set_kill(false);
    }
    break;
    case remote_cmd::robot_key::RESURGENCE:
    {
    }
    break;
    case remote_cmd::robot_key::KILL:
    {
        down_up = false;
        robot_task.set_kill(true);
    }
    break;
    }
    robot_task.set_stand(down_up);
    robot_task.set_robot_vel(rc.move_forward * robot_task.get_robot_param()->core.FORWARD_MAX_VEL,
                             rc.move_left * robot_task.get_robot_param()->core.ROTATE_MAX_VEL);
    robot_task.set_robot_pitch(rc.head_up * robot_task.get_robot_param()->core.MAX_PITCH);
    robot_task.set_roll_angle(rc.tilt_left * robot_task.get_robot_param()->core.MAX_ROLL_SET);
    robot_task.set_height(rc.height * (robot_task.get_robot_param()->core.MAX_HEIGHT - robot_task.get_robot_param()->core.MIN_HEIGHT) +
                          robot_task.get_robot_param()->core.MIN_HEIGHT);
}