/*
 * C语言中的PID控制器实现
 * 
 * 由Joshua Saxby（又名@saxbophone）于2016年1月创建
 * 
 * 这是我尝试以一种（希望是）清晰易懂的C语言实现PID算法。
 *
 * 请查看许可详情，请查看LICENSE。
 */

// 防止多次包含
#ifndef SAXBOPHONE_PID_H
#define SAXBOPHONE_PID_H

#ifdef __cplusplus
extern "C"{
#endif

    typedef struct pid_calibration {
        /*
         * 结构体PID_Calibration
         * 
         * 用于存储PID控制器的校准常数的结构体
         * 这些常数用于调整算法，它们取值取决于应用程序 - 换句话说，这取决于具体应用...
         */
        double kp; // 比例增益
        double ki; // 积分增益
        double kd; // 微分增益
    } PID_Calibration;

    typedef struct pid_state {
        /*
         * 结构体PID_State
         * 
         * 用于存储PID控制器的当前状态的结构体。
         * 这用作输入值传递给PID算法函数，该函数还返回一个反映算法建议的PID_State结构体。
         * 
         * 注意：此结构体中的output字段由PID算法函数设置，并在实际计算中被忽略。
         */
        double actual; // 测量的实际值
        double target; // 期望的值
        double time_delta; // 自上次采样/计算以来的时间 - 在更新状态时应该设置
        // 先前计算的实际与目标之间的误差（初始为零）
        double previous_error;
        double integral; // 随时间的积分误差总和
        double output; // 由算法计算的修改后的输出值，用于补偿误差
    } PID_State;

    /*
     * PID控制器算法实现
     * 
     * 给定P、I和D值的PID校准以及当前PID控制器状态的PID_State，
     * 计算PID控制器的新状态，并设置输出状态以补偿算法定义的任何误差
     */
    PID_State pid_iterate(PID_Calibration calibration, PID_State state);

#ifdef __cplusplus
} // extern "C"
#endif

// 头文件结束
#endif
