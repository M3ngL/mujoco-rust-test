use mujoco_rust::{Simulation, model::ObjType};
use image::{RgbImage, ImageBuffer};
use glfw::{Context, Window};

use mujoco_rs_sys::render::*;
use mujoco_rs_sys as sys;
use std::ptr;

pub fn init_glfw(sim: &Simulation) -> (Window, mjvCamera_, mjvOption_, mjvScene_, mjrContext_) {
    let mut glfw = glfw::init(glfw::FAIL_ON_ERRORS).unwrap();
    let width = 640;
    let height = 480;

    // 创建隐藏窗口
    glfw.window_hint(glfw::WindowHint::Visible(false));
    let (mut window, _events) = glfw
        .create_window(
            width.try_into().unwrap(),
            height.try_into().unwrap(),
            "hidden",
            glfw::WindowMode::Windowed,
        )
        .expect("无法创建 GLFW 窗口");

    // 激活 OpenGL 上下文
    window.make_current();
    glfw.make_context_current(Some(&window));

    // 加载 OpenGL 函数（使用 GLAD 或其他加载器）
    gl::load_with(|symbol| window.get_proc_address(symbol) as *const _);

    let mut cam = mjvCamera_::default();
    let mut opt = mjvOption_::default();
    let mut scn = mjvScene_::default();
    let mut con = mjrContext_::default();

    unsafe {
        mjv_defaultCamera(&mut cam);
        mjv_defaultOption(&mut opt);
        // mjv_defaultScene(&mut scn); // 不需要，手动初始化
        mjr_defaultContext(&mut con);
        mjv_makeScene(sim.model.ptr(), &mut scn, 10000); // 正确设置 maxgeom
        mjr_makeContext(sim.model.ptr(), &mut con, 200); // mjFONTSCALE_200
    }

    (window, cam, opt, scn, con)
}

pub fn update_mjscene(
    sim: &Simulation,
    cam: &mut mjvCamera_,
    opt: &mut mjvOption_,
    scn: &mut mjvScene_,
    con: &mut mjrContext_,
) -> Vec<u8> {
    let width = 640;
    let height = 480;
    cam.distance = 10.0;
    unsafe {
        sys::mjv_updateScene(
            sim.model.ptr(),
            sim.state.ptr(),
            opt, // 应为 &opt
            ptr::null(),
            cam,
            0xFFFFFF,
            scn,
        );

        let viewport = sys::render::mjrRect_ {
            left: 0,
            bottom: 0,
            width,
            height,
        };

        sys::mjr_render(viewport, scn, con);

        // 分配 RGB 缓冲区
        let mut rgb = vec![0u8; 640 * 480 * 3];
        let mut depth = vec![0f32; 640 * 480];

        // 读取像素
        sys::mjr_readPixels(rgb.as_mut_ptr(), depth.as_mut_ptr(), viewport, con);

        rgb
    }
}