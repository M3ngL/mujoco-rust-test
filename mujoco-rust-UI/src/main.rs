// main.rs
mod ui;
use std::ptr;
use glfw::Context;
// use mujoco_rust::{self, ObjType};
use mujoco_rs_sys::*;
use mujoco_rust::model::ObjType;

const SIMULATION_DT: f64 = 0.01; // 100 Hz 更新频率

fn main() { 

    println!("Sim Start...");
    println!("--------------------------------------------------");

    // 模拟来自 MuJoCo 的 IMU 数据
    let model = mujoco_rust::Model::from_xml("/home/m3/project/myRustPilot/x2/scene.xml".to_string()).unwrap();
    let simulation = mujoco_rust::Simulation::new(model.clone());

    let actuator_num = unsafe { (*simulation.model.ptr()).nu };
    // 初始化控制向量
    let mut ctrl: Vec<f64> = vec![0.0; actuator_num as usize]; 
    simulation.control(&ctrl);
    let mj_model = unsafe { *model.ptr() };

    println!("{}", simulation.model.geoms().iter().map(|g| g.name.to_string()).collect::<Vec<_>>().join(", "));
    
    let cam1_id = simulation.model.name_to_id(ObjType::CAMERA, "camera1").unwrap() as i32;
    let cam2_id = simulation.model.name_to_id(ObjType::CAMERA, "camera2").unwrap() as i32;
    let cam3_id = simulation.model.name_to_id(ObjType::CAMERA, "camera3").unwrap() as i32;
    let cam4_id = simulation.model.name_to_id(ObjType::CAMERA, "camera4").unwrap() as i32;

    let mut ui_state = ui::ui_init(&simulation, [cam1_id].as_ref());

    // 模拟运行一段时间
    for i in 0..500 { 
        let current_time = i as f64 * SIMULATION_DT;

       ui::update_scene(&simulation, &mut ui_state);
     
        ctrl[0] = 5.0;
        ctrl[1] = 5.0;
        ctrl[2] = 5.0;
        ctrl[3] = 5.0;

        simulation.control(&ctrl);
        simulation.step();

    }

    unsafe{
        for i in 0..ui_state.scenes.len() {
            mjv_freeScene(&mut ui_state.scenes[i]);
            mjr_freeContext(&mut ui_state.contexts[i]);
        }

    }

    println!("--------------------------------------------------");
    println!("Sim Done.");
}