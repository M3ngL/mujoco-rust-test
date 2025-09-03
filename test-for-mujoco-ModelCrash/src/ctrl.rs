// 引入 PID 控制器结构体和实现
use super::imu;

pub struct PIDController {
    kp: f64,
    ki: f64,
    kd: f64,

    last_err: f64,
    i_err: f64, // 误差积分项
}

impl PIDController {
    pub fn calculate(&mut self, err: f64, dt: f64) -> f64 {
        self.i_err += err * dt;
        let out = err * self.kp + self.i_err * self.ki + (err - self.last_err) / dt * self.kd;
        self.last_err = err;
        out
    }

    pub fn new(kp: f64, ki: f64, kd: f64) -> Self {
        PIDController {
            kp,
            ki,
            kd,
            last_err: 0.0,
            i_err: 0.0,
        }
    }
}


/// 飞行控制器
pub struct FlightController {
    pitch_pid: PIDController,
    target_pitch_rate: f64, // 目标俯仰角速度
}

impl FlightController {
    /// 创建一个新的飞行控制器实例
    /// kp, ki, kd 是俯仰 PID 控制器的增益
    pub fn new(kp: f64, ki: f64, kd: f64) -> Self {
        FlightController {
            pitch_pid: PIDController::new(kp, ki, kd),
            target_pitch_rate: 0.0, // 初始目标俯仰角速度设置为 0 (例如，保持水平)
        }
    }

    /// 设置目标俯仰角速度
    pub fn set_target_pitch_rate(&mut self, target: f64) {
        self.target_pitch_rate = target;
    }

    /// 根据 IMU 数据更新控制器并计算推力输出
    pub fn update(&mut self, imu_data: imu::ImuData, dt: f64) -> f64 {
        // 计算俯仰误差 (目标 - 当前)
        let pitch_error = self.target_pitch_rate - imu_data.pitch_rate;

        // 使用 PID 控制器计算推力输出
        let mut thrust_output = self.pitch_pid.calculate(pitch_error, dt);

        // 将推力输出限制在合理范围内 (例如，0.0 到 1.0，代表 0% 到 100% 油门)
        thrust_output = thrust_output.max(0.0).min(1.0);

        thrust_output
    }
}