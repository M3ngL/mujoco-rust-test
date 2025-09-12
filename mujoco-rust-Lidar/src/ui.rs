// ui.rs
use mujoco_rs_sys::mjr_render;
use mujoco_rs_sys::render::*;
use mujoco_rs_sys::no_render::{mjv_makeScene};
use glfw;
use glfw::Context;
use gl;
use mujoco_rust::Simulation;
use std::ptr;


pub fn ui_init(simulation: &Simulation) -> (mjvCamera_, mjvOption_, mjvScene_, mjrContext_, glfw::Window) {
    let mut glfw = glfw::init(glfw::FAIL_ON_ERRORS).unwrap();

    // 2. 创建窗口，并设置 OpenGL 上下文
    let (mut window, _events) = glfw
        .create_window(1200, 900, "MuJoCo UI", glfw::WindowMode::Windowed)
        .expect("无法创建 GLFW 窗口。");

    // 设置窗口为当前上下文
    window.make_current();
    window.set_key_polling(true);
    window.set_cursor_pos_polling(true);
    window.set_mouse_button_polling(true);
    window.set_scroll_polling(true);

    gl::load_with(|symbol| window.get_proc_address(symbol) as *const _);


    let mut cam = mjvCamera_::default();
    let mut opt = mjvOption_::default();
    let mut scn = mjvScene_::default();
    let mut con = mjrContext_::default();

    unsafe{
        mjv_defaultCamera(&mut cam);
        mjv_defaultOption(&mut opt);
        mjv_defaultScene(&mut scn);
        mjr_defaultContext(&mut con);
        mjv_makeScene(simulation.model.ptr(), &mut scn, 10000);
        mjr_makeContext(simulation.model.ptr(), &mut con, 200);
    }
    (cam, opt, scn, con, window)
}


pub fn update_scene(
    simulation: &Simulation,
    window: &mut glfw::Window, // 需要传入 window
    cam: &mut mujoco_rs_sys::no_render::mjvCamera_,
    opt: &mut mujoco_rs_sys::no_render::mjvOption_,
    scn: &mut mujoco_rs_sys::no_render::mjvScene_,
    con: &mut mujoco_rs_sys::render::mjrContext_, // 需要传入 context
) {
    
    unsafe{
        mjv_updateScene(
            simulation.model.ptr(),
            simulation.state.ptr(),
            opt,
            ptr::null(), // 使用默认的perturb
            cam,
            0xFFFFFF,
            scn,
        );

        // 获取窗口大小以设置视口
        let (width, height) = window.get_framebuffer_size();
        let viewport = mjrRect_ {
            left: 0,
            bottom: 0,
            width,
            height,
        };

        // 渲染场景到窗口
        mjr_render(viewport, scn, con);

        // 刷新缓冲区，显示画面
        window.swap_buffers();

        // 处理事件，如键盘输入
        // glfw.poll_events();
    }
}