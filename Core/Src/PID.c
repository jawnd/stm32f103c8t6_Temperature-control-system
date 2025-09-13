#include "PID.h"

/**
 * @brief 初始化PID控制器
 * @param pid 指向PID控制器结构体的指针
 * 
 * 该函数将PID控制器的所有内部变量初始化为0，
 * 为控制器的首次运行做好准备。
 */
void PIDController_Init(PIDController *pid) {

	/* 清除控制器变量，为新的控制循环做准备 */
	pid->integrator = 0.0f;         // 积分项清零
	pid->prevError  = 0.0f;         // 上一次误差清零

	pid->differentiator  = 0.0f;    // 微分项清零
	pid->prevMeasurement = 0.0f;    // 上一次测量值清零

	pid->out = 0.0f;                // 输出值清零

}

/**
 * @brief 更新PID控制器输出
 * @param pid 指向PID控制器结构体的指针
 * @param setpoint 设定值（目标值）
 * @param measurement 实际测量值
 * @return 控制器输出值
 * 
 * 该函数实现了完整的PID控制算法，包括比例、积分和微分三个部分，
 * 并包含了抗积分饱和和微分项低通滤波等优化措施。
 */
float PIDController_Update(PIDController *pid, float setpoint, float measurement) {

	/*
	* 计算误差信号
	* 误差 = 设定值 - 实际测量值
	*/
    float error = setpoint - measurement;


	/*
	* 比例项（Proportional）
	* 比例项直接与误差成正比，增益为Kp
	* Kp越大，系统响应越快，但过大会导致系统不稳定
	*/
    float proportional = pid->Kp * error;


	/*
	* 积分项（Integral）
	* 积分项用于消除稳态误差，对误差进行累积
	* 使用梯形法则进行数值积分，提高计算精度
	* 积分项 = 原积分项 + 0.5 * Ki * T * (当前误差 + 上次误差)
	*/
    pid->integrator = pid->integrator + 0.5f * pid->Ki * pid->T * (error + pid->prevError);

	/* 
	* 积分饱和抗饱和处理（Anti-wind-up）
	* 当积分项超出预设范围时，将其限制在合理范围内
	* 防止积分项过大导致系统超调或响应迟缓
	*/
    if (pid->integrator > pid->limMaxInt) {
        // 积分项超过最大限制，将其设为最大值
        pid->integrator = pid->limMaxInt;

    } else if (pid->integrator < pid->limMinInt) {
        // 积分项低于最小限制，将其设为最小值
        pid->integrator = pid->limMinInt;
    }


	/*
	* 微分项（Derivative）
	* 微分项用于预测系统变化趋势，提高系统响应速度
	* 对测量值求微分（而非对误差求微分），避免设定值突变对系统造成冲击
	* 使用一阶低通滤波器实现带限微分器，减少噪声影响
	* 微分项 = -(2*Kd*(测量值变化量) + (2*τ-T)*上次微分项) / (2*τ+T)
	*/
    pid->differentiator = -(2.0f * pid->Kd * (measurement - pid->prevMeasurement)	/* 注意：对测量值求微分，因此方程前有负号 */
                        + (2.0f * pid->tau - pid->T) * pid->differentiator)          /* τ是低通滤波器时间常数 */
                        / (2.0f * pid->tau + pid->T);


	/*
	* 计算控制器总输出并应用输出限制
	* 输出 = 比例项 + 积分项 + 微分项
	*/
    pid->out = proportional + pid->integrator + pid->differentiator;

    /* 
	* 输出值限制
	* 确保控制器输出在预设范围内，防止执行器饱和
	*/
    if (pid->out > pid->limMax) {
        // 输出超过最大限制，将其设为最大值
        pid->out = pid->limMax;

    } else if (pid->out < pid->limMin) {
        // 输出低于最小限制，将其设为最小值
        pid->out = pid->limMin;
    }

	/* 
	* 保存当前误差和测量值，供下次计算使用
	*/
    pid->prevError       = error;           // 保存当前误差
    pid->prevMeasurement = measurement;     // 保存当前测量值

	/* 返回控制器输出值 */
    return pid->out;

}
