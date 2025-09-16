// stream.rs
use std::process::{Command, Stdio};

use mujoco_rs_sys::render;
use mujoco_rs_sys::no_render;
use glfw::Context;
use glfw;
use gl;
use mujoco_rust::Simulation;
use std::ptr;


pub struct UIState {
    pub cam: render::mjvCamera_,
    pub opt: render::mjvOption_,
    pub scn: render::mjvScene_,
    pub con: render::mjrContext_,
    pub window: glfw::Window,
}

pub fn glfw_init(glfw: &mut glfw::Glfw, simulation: &Simulation) -> UIState {
    let (mut window, _events) = {
        glfw.window_hint(glfw::WindowHint::Visible(false));
        glfw.create_window(640, 480, "hidden", glfw::WindowMode::Windowed)
            .expect("Unable to create hidden GLFW window.")
    };

    // window settings init
    window.make_current();
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
    }
}

pub fn init_ffmpeg() -> std::process::Child {
    let rtsp_url = "rtsp://localhost:8554/mystream".to_string();
    let ffmpeg = Command::new("ffmpeg")
        .args([
            "-f", "rawvideo",
            "-pixel_format", "rgb24",
            "-video_size", "640x480",
            "-framerate", "30",
            "-i", "pipe:",
            "-c:v", "libx264",
            "-pix_fmt", "yuv420p",
            "-preset", "ultrafast",
            "-tune", "zerolatency",
            "-f", "rtsp",
            "-rtsp_transport", "tcp",
            &rtsp_url
        ])
        .stdin(Stdio::piped())
        .stderr(Stdio::piped())
        .spawn() // Spawn the process
        .expect("Failed to start FFmpeg");
    
    ffmpeg
}

pub fn update_mjscene(simulation: &Simulation, ui_state: &mut UIState) -> Vec<u8> {
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

        // flips the image
        let mut flipped_rgb = vec![0u8; (width * height * 3) as usize];
        for y in 0..height {
            for x in 0..width {
                let src_idx = ((y * width + x) * 3) as usize;
                let dst_idx = (((height - 1 - y) * width + x) * 3) as usize;
                flipped_rgb[dst_idx] = rgb[src_idx];
                flipped_rgb[dst_idx + 1] = rgb[src_idx + 1];
                flipped_rgb[dst_idx + 2] = rgb[src_idx + 2];
            }
        }
        flipped_rgb
    }
}

pub fn free_resource(ui_state: &mut UIState){
    unsafe {
            render::mjv_freeScene(&mut ui_state.scn);
            render::mjr_freeContext(&mut ui_state.con);
    }
}