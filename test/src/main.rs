// main.rs
mod camera;
mod ctrl;
mod imu;
mod ui;
use std::ptr;
use glfw::Context;

fn main() {
    // 模拟 MuJoCo 仿真器的时间步长
    const SIMULATION_DT: f64 = 0.01; // 100 Hz 更新频率

    // 初始化飞行控制器，这里需要根据你的无人机和仿真环境调整 PID 增益
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
    let nsensor = unsafe { (*simulation.model.ptr()).nsensor}; // 你可以实现一个nsensor()方法，类似nbody()
    // let mut sensor_names = Vec::new();

    // 0: gyro
    let gyro_start = unsafe { *mj_model.sensor_adr.add(0) } as usize;
    let gyro_dim = unsafe { *mj_model.sensor_dim.add(0) } as usize;
    let gyro_data = &simulation.sensordata()[gyro_start..gyro_start + gyro_dim];

    // 1: accelerometer
    let acc_start = unsafe { *mj_model.sensor_adr.add(1) } as usize;
    let acc_dim = unsafe { *mj_model.sensor_dim.add(1) } as usize;
    let acc_data = &simulation.sensordata()[acc_start..acc_start + acc_dim];

    // 2: framequat (attitude quaternion)
    let att_start = unsafe { *mj_model.sensor_adr.add(2) } as usize;
    let att_dim = unsafe { *mj_model.sensor_dim.add(2) } as usize;
    let att_data = &simulation.sensordata()[att_start..att_start + att_dim];

    // println!("{:?}, {:?}, {:?}", gyro_data, acc_data, att_data);
    // println!("{}", sensor_names.iter().map(|(i, name)| format!("{}: {}", i, name)).collect::<Vec<_>>().join(", "));

    println!("{}", simulation.model.geoms().iter().map(|g| g.name.to_string()).collect::<Vec<_>>().join(", "));
    
    let mut glfw = glfw::init(glfw::FAIL_ON_ERRORS).unwrap();

    glfw.window_hint(glfw::WindowHint::ContextCreationApi(
        glfw::ContextCreationApi::Egl,
    ));
    glfw.window_hint(glfw::WindowHint::OpenGlProfile(
        glfw::OpenGlProfileHint::Compat
    ));

    glfw.window_hint(glfw::WindowHint::ContextVersion(3, 3));
    glfw.window_hint(glfw::WindowHint::OpenGlForwardCompat(true));
        
    // 2. 创建窗口，并设置 OpenGL 上下文
    let (mut window, events) = glfw
        .create_window(1200, 900, "MuJoCo UI", glfw::WindowMode::Windowed)
        .expect("无法创建 GLFW 窗口。");
    
    // let (mut cam, mut opt, mut scn, mut con, mut window) = ui::ui_init(&simulation);

    // 初始化场景和上下文

    // let cam_id = simulation.model.name_to_id(ObjType::CAMERA, "track").unwrap();
    // println!("相机 ID: {}\n", cam_id);

    // 模拟运行一段时间
    for i in 0..500 { // 模拟运行 500 步，即 5 秒 (500 * 0.01s)
        let current_time = i as f64 * SIMULATION_DT;

        let sensor_index = 0; // 例如 IMU 是第 0 个传感器
        let sensor_dim = unsafe { (*simulation.model.ptr()).sensor_dim.add(sensor_index) };
        let sensor_dim = unsafe { *sensor_dim } as usize;
        
        let imu_data = imu::ImuData{pitch_rate: simulation.sensordata()[0]};

        // 设置一个在某个时间点改变的目标俯仰角速度
        if current_time > 2.0 && current_time < 3.0 {
            flight_controller.set_target_pitch_rate(0.2); // 2秒后将目标俯仰角速度设置为 0.2 rad/s
        } else {
            flight_controller.set_target_pitch_rate(0.0); // 其他时间保持为 0
        }

        // 更新飞行控制器并获取推力输出
        let thrust = flight_controller.update(imu_data, SIMULATION_DT);


        let geoms = simulation.model.geoms();
        // let xpos = simulation.xpos();

        let gyro_data = &simulation.sensordata()[gyro_start..gyro_start + gyro_dim];
        let acc_data = &simulation.sensordata()[acc_start..acc_start + acc_dim];
        let att_data = &simulation.sensordata()[att_start..att_start + att_dim];

        // println!("{:?}, {:?}, {:?}", gyro_data, acc_data, att_data);
        
        if i % 100 == 0 {
            // camera::get_camera_jpg(&simulation, 1080, 1920);
        }
        // unsafe {
        //     mujoco_rs_sys::no_render::mjv_updateScene(
        //         simulation.model.ptr(),
        //         simulation.state.ptr(),
        //         &mut opt,
        //         ptr::null(),
        //         &mut cam,
        //         0xFFFFFF,
        //         &mut scn,
        //     );

        //     let (width, height) = window.get_framebuffer_size();
        //     let viewport = mujoco_rs_sys::render::mjrRect_ {
        //         left: 0,
        //         bottom: 0,
        //         width,
        //         height,
        //     };

        //     mujoco_rs_sys::mjr_render(viewport, &mut scn, &mut con);
        //     window.swap_buffers();
        // }

        // 处理事件
        // glfw.poll_events();
     
        ctrl[0] = 5.0; // 假设第一个执行器是推力执行器
        ctrl[1] = 5.0; 
        ctrl[2] = 5.0;
        ctrl[3] = 5.0;

        simulation.control(&ctrl);

        simulation.step();

    }

    // unsafe{
    //     mjv_freeScene(&mut scn);
    //     mjr_freeContext(&mut con);
    // }

    println!("--------------------------------------------------");
    println!("模拟结束。");
}