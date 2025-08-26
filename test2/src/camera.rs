use mujoco_rust::{Simulation, model::ObjType};
// use nalgebra::{Vector3, Quaternion};
use image::{RgbImage, ImageBuffer};
use glfw::Context;

use mujoco_rs_sys::render::*;
use mujoco_rs_sys as sys;
use std::ptr;

pub fn init_camera(height: usize, width: usize, simulation: &Simulation) -> (sys::render::mjrRect_, sys::render::mjrContext_, sys::render::mjvScene_, sys::render::mjvOption_, sys::render::mjvCamera_) {
    // 获取相机 ID
    let cam_id = simulation
        .model
        .name_to_id(ObjType::CAMERA, "onboard")
        .expect("无法找到 onboard 相机");

    unsafe {
        // 初始化 GLFW
        let mut glfw = glfw::init(glfw::FAIL_ON_ERRORS).expect("无法初始化 GLFW");

        // 创建隐藏窗口
        glfw.window_hint(glfw::WindowHint::Visible(false));
            .create_window(
                width.try_into().unwrap(),
                height.try_into().unwrap(),
                "hidden",
                glfw::WindowMode::Windowed,
            )
            .expect("无法创建 GLFW 窗口");

        // 激活 OpenGL 上下文

        // 初始化 MuJoCo 渲染选项和场景
        let mut vopt = mjvOption_::default();
        let mut cam = mjvCamera_::default();
        let mut scene = mjvScene_::default();
        let mut context = sys::render::mjrContext_::default();

        mjv_defaultOption(&mut vopt);
        mjv_defaultCamera(&mut cam);

        // 设置最大几何体数量
        let maxgeom = 1000; // 根据模型复杂度调整
        mjv_makeScene(simulation.model.ptr(), &mut scene, maxgeom);

        // 初始化渲染上下文
        sys::mjr_defaultContext(&mut context);
        sys::mjr_makeContext(simulation.model.ptr(), &mut context, 150); // mjFONTSCALE_150

        // 配置相机
        cam.fixedcamid = cam_id as i32;
        // cam.type_ = 2; // 固定相机 (2)

        // 设置视口
        let mut viewport = sys::render::mjrRect_ {
            left: 0,
            bottom: 0,
            width: width as i32,
            height: height as i32,
        };
        (viewport, context, scene, vopt, cam)
    }
}



pub fn get_camera_jpg(simulation: &Simulation,
    cam: &mut mujoco_rs_sys::render::mjvCamera_,
    opt: &mut mujoco_rs_sys::render::mjvOption_,
    scn: &mut mujoco_rs_sys::render::mjvScene_,
    con: &mut mujoco_rs_sys::render::mjrContext_, // 需要传入 context
) -> Vec<u8> {
    // 更新场景
    mjv_updateScene(
        simulation.model.ptr(),
        simulation.state.ptr(),
        opt as *mut _,
        ptr::null(),
        cam,
        0xFFFFF, // 渲染所有对象
        scn,
    );


    let viewport = mjrRect_ {
        left: 0,
        bottom: 0,
        640 as i32,
        480 as i32,
    };

    // 渲染到缓冲区
    sys::mjr_render(viewport,
        con as *const sys::render::mjrContext_,
        scn as *const sys::render::mjvScene_);

    // 分配 RGB 缓冲区
    let mut rgb = vec![0u8; width * height * 3];

    // 读取像素
    sys::mjr_readPixels(rgb.as_mut_ptr(), ptr::null_mut(), viewport, con);

    // MuJoCo 像素可能上下翻转，翻转图像
    // let mut flipped_rgb = vec![0u8; width * height * 3];
    // for y in 0..height {
    //     for x in 0..width {
    //         let src_idx = (height - 1 - y) * width * 3 + x * 3;
    //         let dst_idx = y * width * 3 + x * 3;
    //         flipped_rgb[dst_idx..dst_idx + 3].copy_from_slice(&rgb[src_idx..src_idx + 3]);
    //     }
    // }

    // 创建 RgbImage
    // let img: RgbImage  = ImageBuffer::from_raw(width as u32, height as u32, flipped_rgb)
    //     .expect("无法创建 RgbImage");

    // 释放资源
    mjv_freeScene(scn);
    sys::mjr_freeContext(con);
    // img.save("camera.jpg").unwrap();

    // img.into_raw()
    // flipped_rgb
    rgb
    
}
