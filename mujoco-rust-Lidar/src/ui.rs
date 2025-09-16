// ui.rs
use mujoco_rs_sys::render;
use mujoco_rs_sys::no_render;
use glfw;
use glfw::Context;
use gl;
use mujoco_rust::Simulation;
use std::ptr;
use std::sync::mpsc;

pub struct UIState {
    pub cam: render::mjvCamera_,
    pub opt: render::mjvOption_,
    pub scn: render::mjvScene_,
    pub con: render::mjrContext_,
    pub window: glfw::Window,
    pub events: mpsc::Receiver<(f64, glfw::WindowEvent)>
}

pub fn ui_init(glfw: &mut glfw::Glfw, simulation: &Simulation) -> UIState {
    // create window
    let (mut window, events) = glfw
        .create_window(1200, 900, "MuJoCo UI", glfw::WindowMode::Windowed)
        .expect("Unable to create GLFW window.");

    // associate GLFW window with an OpenGL state
    window.make_current();

    // window settings init
    window.set_key_polling(true);
    window.set_cursor_pos_polling(true);
    window.set_mouse_button_polling(true);
    window.set_scroll_polling(true);

    gl::load_with(|symbol| window.get_proc_address(symbol) as *const _);

    // init UIstate
    let mut opt = render::mjvOption_::default();
    let mut cam = render::mjvCamera_::default();
    let mut scn = render::mjvScene_::default();
    let mut con = render::mjrContext_::default();

    unsafe {
        // init camera & scene
        no_render::mjv_defaultCamera(&mut cam);
        render::mjv_defaultScene(&mut scn);
        render::mjr_defaultContext(&mut con);
        no_render::mjv_defaultOption(&mut opt);

        no_render::mjv_makeScene(simulation.model.ptr(), &mut scn, 2000);
        render::mjr_makeContext(simulation.model.ptr(), &mut con, 200);
    }
        
    cam.type_ = 1; // free viewport
    cam.trackbodyid = 1; // Set tracked object ID
    cam.distance = 5.0;

    UIState {
        cam,
        opt,
        scn,
        con,
        window,
        events
    }
}

pub fn update_scene(simulation: &Simulation, ui_state: &mut UIState){
    ui_state.window.make_current();
    unsafe {
        // get window size
        let (width, height) = ui_state.window.get_framebuffer_size();

        // clear buffer
        gl::Clear(gl::COLOR_BUFFER_BIT | gl::DEPTH_BUFFER_BIT);

        // update & render
        no_render::mjv_updateScene(
            simulation.model.ptr(),
            simulation.state.ptr(),
            &ui_state.opt,
            ptr::null(),
            &mut ui_state.cam,
            0xFFFFFF,
            &mut ui_state.scn,
        );

        // define window viewport
        let viewport = render::mjrRect_ {
            left: 0,
            bottom: 0,
            width: width,
            height: height,
        };

        // render scene
        render::mjr_render(viewport, &mut ui_state.scn, &mut ui_state.con);
        
        gl::PixelStorei(gl::PACK_ALIGNMENT, 1);

        // init rgb array
        let mut rgb = vec![0u8; (width * height * 3) as usize];
        
        // read the main window's Pixels
        render::mjr_readPixels(rgb.as_mut_ptr(), ptr::null_mut(), viewport, &mut ui_state.con);

        // render scene
        render::mjr_render(viewport, &mut ui_state.scn, &mut ui_state.con);

        // swap buffer to display render scene
        ui_state.window.swap_buffers();
    }
}

pub fn free_resource(ui_state: &mut UIState){
    unsafe {
            render::mjv_freeScene(&mut ui_state.scn);
            render::mjr_freeContext(&mut ui_state.con);
    }
}