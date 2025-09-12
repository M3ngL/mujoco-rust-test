// main.rs
mod ui;
mod lidar;

use std::ptr;
use glfw::Context;

use mujoco_rs_sys::render::*;
use mujoco_rust::model::ObjType;

// use mujoco_rust::State;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    // 模拟 MuJoCo 仿真器的时间步长
    const SIMULATION_DT: f64 = 0.01; // 100 Hz 更新频率        

    println!("Sim Start...");
    println!("--------------------------------------------------");

    // 模拟来自 MuJoCo 的 IMU 数据
    let model = mujoco_rust::Model::from_xml("/home/m3/project/myRustPilot/x2/simple_scene.xml".to_string()).unwrap();
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
    let mut lidar_window = lidar::init_lidar_window(lidar_width, lidar_height)?;
    let mut buffer: Vec<u32> = vec![0; lidar_width * lidar_height];


    // 模拟运行一段时间
    for i in 0..500 { // 模拟运行 500 步，即 5 秒 (500 * 0.01s)
        let current_time = i as f64 * SIMULATION_DT;

        lidar::update_lidar_buffer(lidar_width, lidar_height, &mut buffer, &rf_ids, &angles, &simulation);
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

        ctrl[0] = 4.0;
        ctrl[1] = 4.0; 
        ctrl[2] = 4.0;
        ctrl[3] = 4.0;

        simulation.control(&ctrl);

        simulation.step();
    }


    unsafe{
        mjv_freeScene(&mut scn);
        mjr_freeContext(&mut con);
    }

    println!("--------------------------------------------------");
    println!("Sim Done.");
    Ok(())
}

