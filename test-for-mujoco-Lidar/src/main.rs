// main.rs
mod camera;
mod ctrl;
mod imu;
mod ui;
mod lidar;

use std::ptr;
use glfw::Context;
use mujoco_rs_sys::render::*;
use mujoco_rust::model::ObjType;
// use mujoco_rust::State;
// use mujoco_rs_sys::mjData;
use std::f64::consts::PI;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    // 模拟 MuJoCo 仿真器的时间步长
    const SIMULATION_DT: f64 = 0.01; // 100 Hz 更新频率

    // 初始化飞行控制器, 调整 PID 增益
    let mut flight_controller = ctrl::FlightController::new(0.5, 0.1, 0.05);

    println!("开始飞行控制器模拟...");
    println!("--------------------------------------------------");

    // 模拟来自 MuJoCo 的 IMU 数据
    let model = mujoco_rust::Model::from_xml("/home/m3/project/myRustPilot/x2/scene.xml".to_string()).unwrap();
    let simulation = mujoco_rust::Simulation::new(model.clone());

    let actuator_num = unsafe { (*simulation.model.ptr()).nu };
    // 初始化控制向量
    let mut ctrl: Vec<f64> = vec![0.0; actuator_num as usize]; 
    simulation.control(&ctrl);
    let mj_model = unsafe { *model.ptr() };


    // println!("{}", sensor_names.iter().map(|(i, name): &(_, _)| format!("{}: {}", i, name)).collect::<Vec<_>>().join(", "));
    println!("{}", simulation.model.geoms().iter().map(|g| g.name.to_string()).collect::<Vec<_>>().join(", "));
    


    let (mut cam, mut opt, mut scn, mut con, mut window) = ui::ui_init(&simulation);

    cam.distance = 10.0;
    
    // get Lidar sensor ids
    let angles = [
        0, 15, 30, 45, 60, 75, 90, 105, 120, 135, 150, 165,
        180, 195, 210, 225, 240, 255, 270, 285, 300, 315, 330, 345,
    ];
    let mut rf_ids: Vec<u16>  = Vec::new();
    for angle in angles.iter() {
        let sensor_name = format!("rf_{}", angle);
        let id = model.name_to_id(ObjType::SITE, &sensor_name).unwrap();
        rf_ids.push(id);
    }


    let lidar_width = 800;
    let lidar_height = 800;
    let mut lidar_window = lidar::init_lidar_window(800, 800)?;
    let mut buffer: Vec<u32> = vec![0; lidar_width * lidar_height];

    // Lidar 坐标线
    let center_x = lidar_width as f64 / 2.0;
    let center_y = lidar_height as f64 / 2.0;
    let max_radius = 10.0; // 最大距离10米
    let pixels_per_meter = (lidar_width as f64 / 2.0 - 20.0) / max_radius;

    // 模拟运行一段时间
    for i in 0..500 { // 模拟运行 500 步，即 5 秒 (500 * 0.01s)
        let current_time = i as f64 * SIMULATION_DT;

        let sensor_index = 0; // 例如 IMU 是第 0 个传感器
        let sensor_dim = unsafe { (*simulation.model.ptr()).sensor_dim.add(sensor_index) };
        let sensor_dim = unsafe { *sensor_dim } as usize;
        
        let imu_data = imu::ImuData{pitch_rate: simulation.sensordata()[0]};
        
        
        // 更新飞行控制器并获取推力输出
        let thrust = flight_controller.update(imu_data, SIMULATION_DT);
        let geoms = simulation.model.geoms();

        // update lidar window
        buffer.fill(0xFFFFFFFF); // 白色背景，确保清除上一次记录点

        for r in 1..=10 {
            let radius = r as f64 * pixels_per_meter;
            lidar::draw_circle_outline(&mut buffer, lidar_width, lidar_height, center_x, center_y, radius, 0x800000);
        }

        for theta in (0..360).step_by(30) {
            let theta_rad = theta as f64 * PI / 180.0;
            let x1 = center_x;
            let y1 = center_y;
            let x2 = center_x + (max_radius * pixels_per_meter) * theta_rad.cos();
            let y2 = center_y + (max_radius * pixels_per_meter) * theta_rad.sin();
            lidar::draw_line(&mut buffer, lidar_width, lidar_height, x1, y1, x2, y2, 0xFF000000); // 黑色角度线
        }

        let mut points = Vec::new();
        for (i, &id) in rf_ids.iter().enumerate() {
            let distance = simulation.sensordata()[id as usize];
            let theta = angles[i] as f64 * PI / 180.0;
            // println!("Angle {}°: {:.2}m", angles[i], distance);
            if distance >= 0.0 && distance <= 10.0 {
                points.push((theta, distance));
        }

        // 绘制激光雷达点（黑色）
        for (theta, r) in &points {
            let x = center_x + (r * pixels_per_meter) * theta.cos();
            let y = center_y + (r * pixels_per_meter) * theta.sin();
            lidar::draw_circle(&mut buffer, lidar_width, lidar_height, x, y, 3.0, 0xFF000000);
        }
        // update Lidar window
        lidar_window.update_with_buffer(&buffer, lidar_width, lidar_height)?;

        unsafe {
            mujoco_rs_sys::no_render::mjv_updateScene(
                simulation.model.ptr(),
                simulation.state.ptr(),
                &mut opt,
                ptr::null(),
                &mut cam,
                0xFFFFFF,
                &mut scn,
            );

            let (width, height) = window.get_framebuffer_size();
            let viewport = mujoco_rs_sys::render::mjrRect_ {
                left: 0,
                bottom: 0,
                width,
                height,
            };

            mujoco_rs_sys::mjr_render(viewport, &mut scn, &mut con);
            window.swap_buffers();
        }

        ctrl[0] = 2.0;
        ctrl[1] = 2.0; 
        ctrl[2] = 2.0;
        ctrl[3] = 2.0;

        simulation.control(&ctrl);

        simulation.step();
    }
}

    unsafe{
        mjv_freeScene(&mut scn);
        mjr_freeContext(&mut con);
    }

    println!("--------------------------------------------------");
    println!("模拟结束。");
    Ok(())
}

