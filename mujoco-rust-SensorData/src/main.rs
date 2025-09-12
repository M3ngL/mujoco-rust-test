// main.rs
mod camera;
mod ui;
use std::ptr;
use glfw::Context;
use mujoco_rs_sys::render::*;
use mujoco_rust::model::ObjType;

fn main() {
    // 模拟 MuJoCo 仿真器的时间步长
    const SIMULATION_DT: f64 = 0.01; // 100 Hz 更新频率
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
    let nsensor = unsafe { (*simulation.model.ptr()).nsensor}; // 你可以实现一个nsensor()方法，类似nbody()
    // let mut sensor_names: Vec<(_, _)> = Vec::new();

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

    // 3. rangefinder
    let rf_start = unsafe { *mj_model.sensor_adr.add(3) } as usize;
    let rf_dim = unsafe { *mj_model.sensor_dim.add(3) } as usize;
    let rf_data = &simulation.sensordata()[rf_start..rf_start + rf_dim];
    
    // 4. 360Lidar (get Sensor IDs)
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


    // get sensor id
    // let sensor_id = simulation
    //     .model
    //     .name_to_id(ObjType::SENSOR, "body_rf_front")
    //     .expect(&format!("无法找到传感器: {}", "body_rf_front"));

    // println!("{}", sensor_names.iter().map(|(i, name): &(_, _)| format!("{}: {}", i, name)).collect::<Vec<_>>().join(", "));
    println!("{}", simulation.model.geoms().iter().map(|g| g.name.to_string()).collect::<Vec<_>>().join(", "));
    
    let (mut cam, mut opt, mut scn, mut con, mut window) = ui::ui_init(&simulation);

    cam.distance = 10.0;


    // 模拟运行一段时间
    for i in 0..500 { // 模拟运行 500 步，即 5 秒 (500 * 0.01s)
        let current_time = i as f64 * SIMULATION_DT;

        for i in rf_ids.iter() {
            let d = simulation.sensordata()[*i as usize];
            println!("{}: {:?} ", i, d);
        }

        // 更新飞行控制器并获取推力输出
        let geoms = simulation.model.geoms();

        let gyro_data = &simulation.sensordata()[gyro_start..gyro_start + gyro_dim];
        let acc_data = &simulation.sensordata()[acc_start..acc_start + acc_dim];
        let att_data = &simulation.sensordata()[att_start..att_start + att_dim];
        let rf_data = &simulation.sensordata()[rf_start..rf_start + rf_dim];
    
        // println!("{:?}", rf_data);
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
}