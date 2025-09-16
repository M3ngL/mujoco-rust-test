// main.rs
mod ui;

fn main() {

    // init model
    let model = mujoco_rust::Model::from_xml("../x2/scene.xml".to_string()).unwrap();
    let simulation = mujoco_rust::Simulation::new(model.clone());

    // init ctrl vector 
    let actuator_num = unsafe { (*simulation.model.ptr()).nu };
    let mut ctrl: Vec<f64> = vec![0.0; actuator_num as usize]; 

    // init glfw
    let mut glfw = glfw::init(glfw::FAIL_ON_ERRORS).unwrap();
    let mut ui_state = ui::ui_init(&mut glfw, &simulation, [0x7FFFFFFF].as_ref()); // 3rd-person perspective 
    
    // sim running until the window closes
    while !ui_state.window.should_close() {

        // update render scene
        ui::update_scene(&simulation, &mut ui_state);

        // fixed ctrl array
        ctrl[..3].fill(3.7);
        ctrl[3] = 4.0;
        simulation.control(&ctrl);

        // sim forward a step
        simulation.step();

        // get contact data in mj_data
        unsafe {
            let mj_data = &*simulation.state.ptr(); // mj_data needs to be updated in real time
            let ncon = mj_data.ncon as usize;
            for i in 0..ncon {
                let contact = &*mj_data.contact.add(i);
                let pos = contact.pos;
                println!(
                    "contact {} at [{}, {}, {}]",
                    i, pos[0], pos[1], pos[2]
                );
            }
        }
        // Get events in real time
        glfw.poll_events();
        glfw::flush_messages(&ui_state.events);
    }
    // free glfw resources
    ui::free_glfw(&mut ui_state);

}

