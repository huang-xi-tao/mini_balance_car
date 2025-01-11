#include "webots_handler.hpp"
#include "DIABLO.hpp"
#include "keyboard.hpp"

#ifdef GENERATE_K_PARAM_LISTS
#include "lqr_k_generator.hpp"
#endif

remote_cmd key_control;
double position_last[MOTOR_NUM] = {0},
       position[MOTOR_NUM] = {0},
       velocity[MOTOR_NUM] = {0},
       torque[MOTOR_NUM] = {0};
double torset[MOTOR_NUM] = {0};
double posset[MOTOR_NUM] = {0, 1.0, 0, 1.0}, posset_tail;

void Get_Robot(Middleware_Class *middle)
{
  getMotorPosition(position);
  // YHJ_TODO:for_test
  for (uint8_t i = 0; i < MOTOR_NUM; i++)
  {
    torque[i] = torset[i];
  }
  for (int id = 0; id < MOTOR_NUM; id++)
  {
    velocity[id] = (position[id] - position_last[id]) / (float)TIME_STEP * 1000.f;
    middle->downbridge.Set_Motor(
        (float)(torque[id]),
        (float)(velocity[id]),
        (float)(position[id]),
        id);
    position_last[id] = position[id];
  }
  middle->downbridge.Set_Battery(28, 1, 100);
  const double *qua = getInertialUnit();
  const double *accl = getAccleration();
  const double *gyro = getGyro();
  middle->downbridge.Set_Gyro(gyro[0], gyro[1], gyro[2]);
  middle->downbridge.Set_Quaternion((float)qua[3], (float)qua[0], (float)qua[1], (float)qua[2]);
  middle->downbridge.Set_Accl(accl[0] / 9.8, accl[1] / 9.8, accl[2] / 9.8);
}

void Drive_Robot(Middleware_Class *middle, unsigned char mode)
{ // here apply motor torque limit
  float torque_receive[8] = {0};
  float torque_set[MOTOR_NUM] = {0};
  float pos_tail_receive;
  // printf("tau: ");
  for (int id = 0; id < 4; id++)
  {
    middle->downbridge.Get_Motor_Torque(torque_receive[id], id); // 8 motors out
    torque_set[id] = torque_receive[id];
  }
  middle->downbridge.Get_Motor_Pos(pos_tail_receive, 4);
  posset_tail = pos_tail_receive;
  // torque_set[0] = torque_receive[2]; // barL, kneeL
  // torque_set[1] = torque_receive[3]; // wheelL
  // torque_set[2] = torque_receive[6]; // barR, kneeR
  // torque_set[3] = torque_receive[7]; // wheelR
  for (int id = 0; id < MOTOR_NUM; id++)
  {
    // setMotorTorqueConstraint(
    //     torque_set[id], velocity[id] * 60 / (2 * M_PI), torque_max[id],
    //     rpm_max[id]);
    // if (fabs(torque_set[id]) > torque_limit[id])
    // {
    //   torque_set[id] = torque_limit[id] * torque_set[id] / fabs(torque_set[id]);
    // }
    bound(torque_set[id], torque_limit[id]);
    torset[id] = torque_set[id];
  }
  setMotorTorque(torset, mode);
  setTailMotorPosition(&posset_tail); // 这里还是得要单独处理一下
}

void lqr_k_calc()
{
  // float len[2] = {0.25, 0.25};
  // robot_task.get_leg_len(len[0], len[1]);
  // if (fabs(len[0]) < 0.1 || fabs(len[1]) < 0.1)
  // {
  //   len[0] = len[1] = 0.1;
  // }
  // leg_wheel_drive_mode mode = robot_task.get_leg_wheel_drive_mode();
  // ksolver.setMode(mode);
  // ksolver.refresh();
  // ksolver.calcKgnd(len);
  // ksolver.calcKair(len);
  // robot_task.set_lqr_k_gnd(ksolver.getKgnd());
  // robot_task.set_lqr_k_air(ksolver.getKair());
}

#ifndef GENERATE_K_PARAM_LISTS
Robot_Class *robot_likeAscento_ptr = NULL;
int main(int argc, char **argv)
{
  WB6_Parameter param(param_core, param_lqr, kin_param);
  Middleware_Class middle;
  Robot_Class robot_likeAscento(param, middle);
  Controller_Class ctrl(robot_likeAscento);
  Planner_Class planner(robot_likeAscento, ctrl);
  Task_Class robot_task(robot_likeAscento, param, ctrl, planner);
  robot_likeAscento_ptr = &robot_likeAscento;

#ifdef USE_GNU_PLOT
  std::ofstream outfile;
  /*初始化gnuplot*/
  outfile.open("data.txt");
  // 启动gnuplot程序并创建管道
  FILE *gnuplotPipe = popen("gnuplot", "w");
  // 设置绘图参数
  fprintf(gnuplotPipe, "set terminal x11 noraise\n");
  fprintf(gnuplotPipe, "set origin  0,0\n");
  fprintf(gnuplotPipe, "set title 'Joint Angle Data'\n");
  fprintf(gnuplotPipe, "set xlabel 'Time (s)'\n");
  fprintf(gnuplotPipe, "set ylabel 'Angle (rad)'\n");
  fprintf(gnuplotPipe, "set grid\n");
  fprintf(gnuplotPipe, "set style line 1 lc rgb '#ff0000' lt 1 lw 2\n");
  fprintf(gnuplotPipe, "set style line 2 lc rgb '#0000ff' lt 1 lw 2\n");
  float draw_time = 0;
#endif
  float dt = TIME_STEP / 1000.f;
  float frequency = 30.0;
  float stamp = 0.0f;
  wb_robot_init();
  initDevices();
  std::cout << "----------Sim Start--------" << std::endl;
  key_control.keyboard_init();
  robot_task.Run_Fast_Init();
  robot_likeAscento.Tail_Down_FirstTime();
  // robot_task.medium_status = robot_task.MEDIUM_INIT_MODAL;
  while (wb_robot_step(TIME_STEP) != -1)
  {
#ifdef USE_GNU_PLOT
    draw_time += dt;
    outfile << draw_time << " " << ax << " " << bx << std::endl;
    fprintf(gnuplotPipe, "plot 'data.txt' using 1:2 with lines,'data.txt' using 1:3 with lines\n");
    // fprintf(gnuplotPipe, "plot 'data.txt' using 2:4 with lines\n");
    fflush(gnuplotPipe);
#endif
    // update the keyboard control
    key_control.keyboard_update(robot_task, dt);
    stamp += dt;
    Get_Robot(&middle);
    robot_task.Run_Fast(dt); // 快线程运行
    robot_task.Run_Medium(dt);
    Drive_Robot(&middle, robot_likeAscento.mode);
    // printf("keyboard:%d, forward_input: %.2f,", key_control.key_status, ctrl.cmd.forward_input);
    // printf("torset_L: %.2f, torset_R: %.2f, robot.cmd.forward.d_val:%.2f\t", torset[0], torset[2], robot_likeAscento.cmd.forward.d_val);
  }
  wb_robot_cleanup();
  return 0;
}

#else

int main(int argc, char **argv)
{
  generate_k_lists();
  wb_robot_cleanup();
  return 0;
}

#endif