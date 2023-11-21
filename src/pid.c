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


PID_State pid_iterate(PID_Calibration calibration, PID_State state) {
    // 计算期望值与实际值之间的差异（误差）
    double error = state.target - state.actual;
    // 计算并更新积分项
    state.integral += (error * state.time_delta);
    // 计算导数
    double derivative = (error - state.previous_error) / state.time_delta;
    // 根据算法计算输出值
    state.output = (
        (calibration.kp * error) + (calibration.ki * state.integral) + (calibration.kd * derivative)
    );
    // 将 state.previous_error 更新为此迭代中计算的误差值
    state.previous_error = error;
    // 返回反映计算结果的 state 结构体
    return state;
}
