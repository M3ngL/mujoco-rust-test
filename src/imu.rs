/// 模拟 IMU 数据，仅包含俯仰角速度
#[derive(Debug, Clone, Copy)]
pub struct ImuData {
    pub pitch_rate: f64, // 俯仰角速度 (例如，围绕Y轴的角速度)
    // 实际项目中会包含更多 IMU 数据，如roll_rate, yaw_rate, acc_x, acc_y, acc_z 等
}