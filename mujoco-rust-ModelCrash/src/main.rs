// main.rs
mod camera;
mod ui;
use std::ptr;
use glfw::Context;
use mujoco_rs_sys::render::*;
use mujoco_rust::model::ObjType;
// use mujoco_rust::State;
// use mujoco_rs_sys::mjData;

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


    // println!("{}", sensor_names.iter().map(|(i, name): &(_, _)| format!("{}: {}", i, name)).collect::<Vec<_>>().join(", "));
    println!("{}", simulation.model.geoms().iter().map(|g| g.name.to_string()).collect::<Vec<_>>().join(", "));
    
    let (mut cam, mut opt, mut scn, mut con, mut window) = ui::ui_init(&simulation);

    cam.distance = 10.0;
    
    // Crash Test
    let ptr = unsafe { mujoco_rs_sys::no_render::mj_makeData(model.ptr() as *const mjModel_) };
    let mj_data = unsafe { &*ptr };
    
    println!("{:?}",unsafe { mj_data.ncon });

    // 模拟运行一段时间
    for i in 0..500 { // 模拟运行 500 步，即 5 秒 (500 * 0.01s)
        let current_time = i as f64 * SIMULATION_DT;

        let sensor_index = 0; // 例如 IMU 是第 0 个传感器
        let sensor_dim = unsafe { (*simulation.model.ptr()).sensor_dim.add(sensor_index) };
        let sensor_dim = unsafe { *sensor_dim } as usize;
        
    
        let ptr = unsafe { mujoco_rs_sys::no_render::mj_makeData(model.ptr() as *const mjModel_) };
        let mj_data = unsafe { &*ptr };
        println!("{:?}", unsafe { mj_data.ncon });

        let geoms = simulation.model.geoms();
    
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
        ctrl[2] = 3.5;
        ctrl[3] = 3.5;

        simulation.control(&ctrl);

        simulation.step();

    //     // 检查是否有碰撞发生
    //     if simulation.state.ncon > 0 {
    //         println!("检测到 {} 个接触！", simulation.state.ncon);

    //         // 遍历所有接触点
    //         for i in 0..simulation.state.ncon {
    //             // 获取 mjContact 结构体
    //             let contact = &simulation.state.contact[i];

    //             // 获取发生碰撞的两个几何体的名称（通过 ID 查找）
    //             let geom1_name = simulation.model.id_to_name(
    //                 ObjType::GEOM,
    //                 contact.geom1 as usize,
    //             ).unwrap_or("未知几何体");

    //             let geom2_name = simulation.model.id_to_name(
    //                 ObjType::GEOM,
    //                 contact.geom2 as usize,
    //             ).unwrap_or("未知几何体");

    //             println!("--- 碰撞详情 ---");
    //             println!("碰撞几何体：{} 和 {}", geom1_name, geom2_name);
    //             println!("渗透深度 (dist): {:.4}", contact.dist);
                
    //             // 访问碰撞位置
    //             let pos = contact.pos;
    //             println!("世界坐标系中的位置 (pos): [{:.2}, {:.2}, {:.2}]", pos[0], pos[1], pos[2]);
    //         }

    }

    unsafe{
        mjv_freeScene(&mut scn);
        mjr_freeContext(&mut con);
    }

    println!("--------------------------------------------------");
    println!("Sim Done.");
}
