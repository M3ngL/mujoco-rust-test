// main.rs
use mujoco_rust::model::ObjType;
mod ui;

fn main() { 

    println!("Sim Start...");
    println!("--------------------------------------------------");

    // init model
    let model = mujoco_rust::Model::from_xml("../x2/scene.xml".to_string()).unwrap();
    let simulation = mujoco_rust::Simulation::new(model.clone());

    // init ctrl vector 
    let actuator_num = unsafe { (*simulation.model.ptr()).nu };
    let mut ctrl: Vec<f64> = vec![0.0; actuator_num as usize]; 

    // get camera ID by specifying name
    let cam1_id = simulation.model.name_to_id(ObjType::CAMERA, "camera1").unwrap() as i32;
    let cam2_id = simulation.model.name_to_id(ObjType::CAMERA, "camera2").unwrap() as i32;
    let cam3_id = simulation.model.name_to_id(ObjType::CAMERA, "camera3").unwrap() as i32;
    let cam4_id = simulation.model.name_to_id(ObjType::CAMERA, "camera4").unwrap() as i32;

    // init different UI
    let mut glfw = glfw::init(glfw::FAIL_ON_ERRORS).unwrap();

    // 1st-person perspective 
    let mut ui_state_1st = ui::ui_init(&mut glfw, &simulation, [cam1_id, cam2_id, cam3_id, cam4_id].as_ref());
    // 3rd-person perspective 
    let mut ui_state_3rd = ui::ui_init(&mut glfw, &simulation, [0x7FFFFFFF].as_ref());

    // sim running until the window closes
    while !ui_state_3rd.window.should_close() {

        ui::update_scene(&simulation, &mut ui_state_1st);
        ui::update_scene(&simulation, &mut ui_state_3rd);
        
        // ctrl array fixed settings
        ctrl[..4].fill(4.5);
        simulation.control(&ctrl);

        // sim forward a step
        simulation.step();

        // Get events in real time
        glfw.poll_events();
        glfw::flush_messages(&ui_state_1st.events);
        glfw::flush_messages(&ui_state_3rd.events);
    }

    ui::free_glfw(&mut ui_state_1st);
    ui::free_glfw(&mut ui_state_3rd);

    println!("--------------------------------------------------");
    println!("Sim Done.");
}