// main.rs
mod ui;
use std::ptr;
use glfw::Context;
// use mujoco_rust::{self, ObjType};
use mujoco_rs_sys::*;
use mujoco_rust::model::ObjType;

const SIMULATION_DT: f64 = 0.01; // 100 Hz

fn main() { 

    println!("Sim Start...");
    println!("--------------------------------------------------");

    // init model
    let model = mujoco_rust::Model::from_xml("../x2/scene.xml".to_string()).unwrap();
    let simulation = mujoco_rust::Simulation::new(model.clone());

    // init ctrl vector 
    let actuator_num = unsafe { (*simulation.model.ptr()).nu };
    let mut ctrl: Vec<f64> = vec![0.0; actuator_num as usize]; 

    println!("{:?}", simulation.model.geoms().iter().map(|g| g.name.to_string()).collect::<Vec<_>>().join(", "));
    
    let cam1_id = simulation.model.name_to_id(ObjType::CAMERA, "camera1").unwrap() as i32;
    let cam2_id = simulation.model.name_to_id(ObjType::CAMERA, "camera2").unwrap() as i32;
    let cam3_id = simulation.model.name_to_id(ObjType::CAMERA, "camera3").unwrap() as i32;
    let cam4_id = simulation.model.name_to_id(ObjType::CAMERA, "camera4").unwrap() as i32;

    let mut ui_state_1st = ui::ui_init(&simulation, [cam1_id, cam2_id].as_ref());
    let mut ui_state_3rd = ui::ui_init(&simulation, [0x7FFFFFFF].as_ref());

    // sim start
    for i in 0..500 { 
        let current_time = i as f64 * SIMULATION_DT;

        ui::update_scene(&simulation, &mut ui_state_1st);
        ui::update_scene(&simulation, &mut ui_state_3rd);
        
        // ctrl array fixed settings
        ctrl[..4].fill(4.5);

        simulation.control(&ctrl);
        simulation.step();

    }
    ui::free_glfw(&mut ui_state_1st);
    ui::free_glfw(&mut ui_state_3rd);

    println!("--------------------------------------------------");
    println!("Sim Done.");
}