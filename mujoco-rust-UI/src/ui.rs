// ui.rs
use mujoco_rs_sys::mjr_render;
use mujoco_rs_sys::render::*;
use mujoco_rs_sys::no_render::{mjv_makeScene};
use glfw;
use glfw::Context;
use gl;
use mujoco_rust::Simulation;
use std::ptr;

use mujoco_rust::model::ObjType;

pub struct UIState {
    pub cameras: Vec<mjvCamera_>,
    pub opt: mjvOption_,
    pub scenes: Vec<mjvScene_>,
    pub contexts: Vec<mjrContext_>, // 改为 Vec 以支持多个上下文
    pub window: glfw::Window,
    // pub events: glfw::Receiver<(f64, glfw::WindowEvent)>,
}

pub fn ui_init(simulation: &Simulation, cam_ids: &[i32]) -> UIState {
    let mut glfw = glfw::init(glfw::FAIL_ON_ERRORS).unwrap();

    // 创建窗口，并设置 OpenGL 上下文
    let (mut window, events) = glfw
        .create_window(1200, 900, "MuJoCo UI", glfw::WindowMode::Windowed)
        .expect("无法创建 GLFW 窗口。");

    // 设置窗口为当前上下文
    window.make_current();
    window.set_key_polling(true);
    window.set_cursor_pos_polling(true);
    window.set_mouse_button_polling(true);
    window.set_scroll_polling(true);

    gl::load_with(|symbol| window.get_proc_address(symbol) as *const _);

    // 初始化 MuJoCo 结构
    let mut cameras = Vec::new();
    let mut scenes = Vec::new();
    let mut contexts = Vec::new();
    let mut opt = mjvOption_::default();

    for &cam_id in cam_ids {
        let mut cam = mjvCamera_::default();
        let mut scn = mjvScene_::default();
        let mut con = mjrContext_::default();

        unsafe {
            mjv_defaultCamera(&mut cam);
            mjv_defaultScene(&mut scn);
            mjr_defaultContext(&mut con);

            // 为每个相机初始化场景和上下文
            mjv_makeScene(simulation.model.ptr(), &mut scn, 2000);
            mjr_makeContext(simulation.model.ptr(), &mut con, 200);
        }

        // 设置相机属性
        cam.fixedcamid = cam_id;
        cam.type_ = 2; // 固定相机

        cameras.push(cam);
        scenes.push(scn);
        contexts.push(con);
    }

    unsafe {
        mjv_defaultOption(&mut opt);
    }

    UIState {
        cameras,
        opt,
        scenes,
        contexts,
        window,
        // events, // 返回 events 以供主循环使用
    }
}

pub fn update_scene(simulation: &Simulation, ui_state: &mut UIState) {
    unsafe {
        // 获取窗口大小
        let (width, height) = ui_state.window.get_framebuffer_size();
        let num_cameras = ui_state.cameras.len().min(4);
        let cols = if num_cameras < 2 { 1 } else { 2 };
        let rows = if num_cameras <= 2 { 1 } else { 2 };
        let sub_window_width = width / cols as i32;
        let sub_window_height = height / rows as i32;

        // 清除帧缓冲区
        gl::Clear(gl::COLOR_BUFFER_BIT | gl::DEPTH_BUFFER_BIT);

        // 更新并渲染每个场景
        for i in 0..num_cameras {
            // 更新当前相机的场景
            mjv_updateScene(
                simulation.model.ptr(),
                simulation.state.ptr(),
                &ui_state.opt,
                ptr::null(),
                &mut ui_state.cameras[i],
                0xFFFFFF,
                &mut ui_state.scenes[i],
            );

            // 计算网格中的行和列
            let row = i / cols; // 行索引
            let col = i % cols; // 列索引

            // 定义当前子窗口的视口
            let viewport = mjrRect_ {
                left: col as i32 * sub_window_width,
                bottom: (rows - 1 - row) as i32 * sub_window_height, // 从底部开始，翻转行顺序
                width: sub_window_width,
                height: sub_window_height,
            };

            // 在视口中渲染场景
            mjr_render(viewport, &mut ui_state.scenes[i], &mut ui_state.contexts[i]);
        }

        // 交换缓冲区以显示
        ui_state.window.swap_buffers();
    }
}