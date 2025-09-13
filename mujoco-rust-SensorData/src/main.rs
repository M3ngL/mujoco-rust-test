// main.rs
use std::ptr;
use glfw::Context;
use mujoco_rs_sys::render::*;
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
    
    
    let mj_model = unsafe { *model.ptr() };
    let nsensor = unsafe { (*simulation.model.ptr()).nsensor};
    println!("sensor:{}", nsensor);

    // get sensor data by sensor definition sequence
    // 0: gyro
    let gyro_start = unsafe { *mj_model.sensor_adr.add(0) } as usize;
    let gyro_dim = unsafe { *mj_model.sensor_dim.add(0) } as usize;
    let gyro_data = &simulation.sensordata()[gyro_start..gyro_start + gyro_dim];

    // 1: accelerometer
    let acc_start = unsafe { *mj_model.sensor_adr.add(1) } as usize;
    let acc_dim = unsafe { *mj_model.sensor_dim.add(1) } as usize;
    let acc_data = &simulation.sensordata()[acc_start..acc_start + acc_dim];

    // 2: framequat
    let att_start = unsafe { *mj_model.sensor_adr.add(2) } as usize;
    let att_dim = unsafe { *mj_model.sensor_dim.add(2) } as usize;
    let att_data = &simulation.sensordata()[att_start..att_start + att_dim];

    // get sensor data by sensor name
    // 3. 360° Lidar ()
    let angles = [
        0, 15, 30, 45, 60, 75, 90, 105, 120, 135, 150, 165,
        180, 195, 210, 225, 240, 255, 270, 285, 300, 315, 330, 345,
    ];
    // get Rangefinder ids
    let mut rf_ids: Vec<u16>  = Vec::new();
    for angle in angles.iter() {
        let sensor_name = format!("rf_{}", angle);
        let id = model.name_to_id(ObjType::SITE, &sensor_name).unwrap();
        rf_ids.push(id);
    }

    // let mut sensor_names: Vec<(_, _)> = Vec::<(T, T)>::new();
    // println!("{}", sensor_names.iter().map(|(i, name): &(_, _)| format!("{}: {}", i, name)).collect::<Vec<_>>().join(", "));
    // println!("{}", simulation.model.geoms().iter().map(|g| g.name.to_string()).collect::<Vec<_>>().join(", "));
    

    // sim running for a while
    for i in 0..500 {
        // get data from id
        for id in rf_ids.iter() {
            let data = simulation.sensordata()[*id as usize + 1];
            print!("rf_{}: {:?} ", id, data);
        }

        // update sensor data
        let gyro_data = &simulation.sensordata()[gyro_start..gyro_start + gyro_dim];
        let acc_data = &simulation.sensordata()[acc_start..acc_start + acc_dim];
        let att_data = &simulation.sensordata()[att_start..att_start + att_dim];
        
        println!("\ngyro_data:{:?}\n acc_data:{:?}\n att_data:{:?}", gyro_data, acc_data, att_data);
     
        // ctrl array fixed settings
        ctrl[..4].fill(4.5);
        simulation.control(&ctrl);

        // sim forward a step
        simulation.step();

    }

    println!("--------------------------------------------------");
    println!("Sim Done.");
}