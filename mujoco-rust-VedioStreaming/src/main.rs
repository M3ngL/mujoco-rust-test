use std::io::Write;
mod stream;

fn main() {
    // init model
    let model = mujoco_rust::Model::from_xml("../x2/scene.xml".to_string()).unwrap();
    let simulation = mujoco_rust::Simulation::new(model.clone());

    // init ctrl vector 
    let actuator_num = unsafe { (*simulation.model.ptr()).nu };
    let mut ctrl: Vec<f64> = vec![0.0; actuator_num as usize]; 

    // init ffmpeg
    let mut ffmpeg = stream::init_ffmpeg();
    let mut stdin = ffmpeg.stdin.take().unwrap();

    // init glfw
    let mut glfw = glfw::init(glfw::FAIL_ON_ERRORS).unwrap();
    let mut ui_state = stream::glfw_init(&mut glfw, &simulation);

    // sim running until the window closes
    while !ui_state.window.should_close() {
        // ctrl array fixed settings
        ctrl[..4].fill(4.5);
        simulation.control(&ctrl);

        // write frame into ffmpeg
        let frame = stream::update_mjscene(&simulation,  &mut ui_state);
        let _ = stdin.write_all(&frame);

        // sim forward a step
        simulation.step();
    }

    // free glfw resource
    stream::free_resource(&mut ui_state);
    // close stdin
    drop(stdin);

     // wait for FFmpeg end
    match ffmpeg.wait_with_output() {
        Ok(output) => {
            if !output.status.success() {
                eprintln!("FFmpeg error: {}", String::from_utf8_lossy(&output.stderr));
                return;
            }
        }
        Err(e) => {
            eprintln!("Error: wait for FFmpeg failed: {}", e);
            return;
        }
    }

}