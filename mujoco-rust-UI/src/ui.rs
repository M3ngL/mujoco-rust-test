// ui.rs
use mujoco_rs_sys::render;
use mujoco_rs_sys::no_render;
use glfw;
use glfw::Context;
use gl;
use mujoco_rust::Simulation;
use std::ptr;
use std::sync::mpsc;

use mujoco_rust::model::ObjType;

pub struct UIState {
    pub cameras: Vec<render::mjvCamera_>,
    pub opt: render::mjvOption_,
    pub scenes: Vec<render::mjvScene_>,
    pub contexts: Vec<render::mjrContext_>,
    pub window: glfw::Window,
    pub events: mpsc::Receiver<(f64, glfw::WindowEvent)>,
}

pub fn ui_init(glfw: &mut glfw::Glfw, simulation: &Simulation, cam_ids: &[i32]) -> UIState {

    // create window
    let (mut window, events) = glfw
        .create_window(1200, 900, "MuJoCo UI", glfw::WindowMode::Windowed)
        .expect("Unable to create GLFW window.");

    // associate GLFW window with an OpenGL state
    window.make_current();
    
    // Enable GLFW window listening for specific user input events.
    window.set_key_polling(true); // keyboard input 
    window.set_cursor_pos_polling(true); // mouse position
    window.set_mouse_button_polling(true); // which key is pressed by the mouse 
    window.set_scroll_polling(true); // mouse wheel infor

    // dynamically loading OpenGL functions
    gl::load_with(|symbol| window.get_proc_address(symbol) as *const _);

    // initialize MuJoCo render structure
    let mut cameras = Vec::new();
    let mut scenes = Vec::new();
    let mut contexts = Vec::new();
    let mut opt = render::mjvOption_::default();

    //Initialize scene and context for each camera
    for &cam_id in cam_ids {
        let mut cam = render::mjvCamera_::default();
        let mut scn = render::mjvScene_::default();
        let mut con = render::mjrContext_::default();
        unsafe {
            no_render::mjv_defaultCamera(&mut cam);
            render::mjv_defaultScene(&mut scn);
            render::mjr_defaultContext(&mut con);

            no_render::mjv_makeScene(simulation.model.ptr(), &mut scn, 1000);
            render::mjr_makeContext(simulation.model.ptr(), &mut con, 200);
        }
        // 3rd-person perspective 
        if cam_id == 0x7FFFFFFF {
            cam.type_ = 1; // free perspective 
            cam.trackbodyid = 1; // Set tracked object ID
            cam.distance = 5.0;
        } else { // 1st-person perspective 
            cam.type_ = 2; // fixed perspective 
            cam.fixedcamid = cam_id;
        }
        // output
        cameras.push(cam);
        scenes.push(scn);
        contexts.push(con);
    }

    unsafe {
        no_render::mjv_defaultOption(&mut opt);
    }

    UIState {
        cameras,
        opt,
        scenes,
        contexts,
        window,
        events
    }
}

pub fn update_scene(simulation: &Simulation, ui_state: &mut UIState) {
    ui_state.window.make_current();
    unsafe {
        // get window size
        let (width, height) = ui_state.window.get_framebuffer_size();
        let num_cameras = ui_state.cameras.len().min(4);
        let cols = if num_cameras < 2 { 1 } else { 2 };
        let rows = if num_cameras <= 2 { 1 } else { 2 };
        let sub_window_width = width / cols as i32;
        let sub_window_height = height / rows as i32;

        // clear buffer
        gl::Clear(gl::COLOR_BUFFER_BIT | gl::DEPTH_BUFFER_BIT);

        // update & render
        for i in 0..num_cameras {
            no_render::mjv_updateScene(
                simulation.model.ptr(),
                simulation.state.ptr(),
                &ui_state.opt,
                ptr::null(),
                &mut ui_state.cameras[i],
                0xFFFFFF,
                &mut ui_state.scenes[i],
            );

            // calc sub window's pos in the main window
            let row = i / cols;
            let col = i % cols;

            // define sub window viewport
            let viewport = render::mjrRect_ {
                left: col as i32 * sub_window_width,
                bottom: (rows - 1 - row) as i32 * sub_window_height,
                width: sub_window_width,
                height: sub_window_height,
            };

            // render scene
            render::mjr_render(viewport, &mut ui_state.scenes[i], &mut ui_state.contexts[i]);
        }

        // swap buffer to display render scene
        ui_state.window.swap_buffers();
    }
}


pub fn free_glfw(ui_state: &mut UIState){
    unsafe{
        for i in 0..ui_state.scenes.len() {
            render::mjv_freeScene(&mut ui_state.scenes[i]);
            render::mjr_freeContext(&mut ui_state.contexts[i]);
        }
    }
}
    