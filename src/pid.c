/*
 * PID Controller Implementation in C
 * 
 * Created by Joshua Saxby (aka @saxbophone) on 1 Jan, 2016
 * 
 * My own attempt at implementing the PID algorithm in some (hopefully) clean, understandable C.
 *
 * See LICENSE for licensing details.
 */

#include "pid.h"

//PWM占空比,0时电机停止，255时电机速度最快
//kp = （255/占空比255时，这个电机一秒钟的脉冲数）* （当前电机的占空比/255）
//PID_data.kp = (255.0 / encoderPos_Max) * (sp / 255.0);

PID_State pid_iterate(PID_Calibration calibration, PID_State state) {
    // 计算期望值与实际值之间的差异（误差）
    double error = state.target - state.actual; //误差 = 目标值 - 当前值
    // 计算并更新积分项
    state.integral += (error * state.time_delta); //积分项 += （误差 * 自上次采样/计算以来的时间）
    // 计算导数
    double derivative = (error - state.previous_error) / state.time_delta; //导数项 = （误差 - 上一次的误差）/ 自上次采样/计算以来的时间
    // 根据算法计算输出值
    state.output = (
        (calibration.kp * error) + (calibration.ki * state.integral) + (calibration.kd * derivative) //输出 = (kp * 误差) + (ki * 积分项) + (kd * 微分项)
    );
    // 将 state.previous_error 更新为此迭代中计算的误差值
    state.previous_error = error; //上一次误差 = 误差
    // 返回反映计算结果的 state 结构体
    return state;
}
