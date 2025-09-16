// main.rs
mod ui;
mod lidar;



fn main() -> Result<(), Box<dyn std::error::Error>> {

    println!("Sim Start...");
    println!("--------------------------------------------------");

    // init model
    let model = mujoco_rust::Model::from_xml("../x2/scene.xml".to_string()).unwrap();
    let simulation = mujoco_rust::Simulation::new(model.clone());

    // init ctrl vector 
    let actuator_num = unsafe { (*simulation.model.ptr()).nu };
    let mut ctrl: Vec<f64> = vec![0.0; actuator_num as usize]; 

    // init glfw
    let mut glfw = glfw::init(glfw::FAIL_ON_ERRORS).unwrap();
    let mut ui_state = ui::ui_init(&mut glfw, &simulation); // 3rd-person perspective 

    
    // let mj_model = unsafe { *model.ptr() };
    println!("{}", simulation.model.geoms().iter().map(|g| g.name.to_string()).collect::<Vec<_>>().join(", "));
    
    // get Lidar sensor ids
    let rf_ids= lidar::get_lidar_id(model);

    // init lidar window
    let lidar_width = 800;
    let lidar_height = 800;
    let mut lidar_window = lidar::init_lidar_window(lidar_width, lidar_height)?;
    let mut lidar_buffer: Vec<u32> = vec![0; lidar_width * lidar_height];


    // sim running until the window closes
    while !ui_state.window.should_close() {

        lidar::update_lidar_buffer(lidar_width, lidar_height, &mut lidar_buffer, &rf_ids, &simulation);
        // update Lidar window
        lidar_window.update_with_buffer(&lidar_buffer, lidar_width, lidar_height)?;
        
        // update UI render scene
        ui::update_scene(&simulation, &mut ui_state);

        // ctrl array fixed settings
        ctrl[..4].fill(4.5);
        simulation.control(&ctrl);

        // sim forward a step
        simulation.step();
        
        // Get events in real time
        glfw.poll_events();
        glfw::flush_messages(&ui_state.events);
    }

    ui::free_resource(&mut ui_state);

    println!("--------------------------------------------------");
    println!("Sim Done.");
    Ok(())
}

