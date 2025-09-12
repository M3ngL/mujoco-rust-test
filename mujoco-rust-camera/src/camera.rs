use mujoco_rust::{Simulation, model::ObjType};
// use nalgebra::{Vector3, Quaternion};
use image::{RgbImage, ImageBuffer};
use glfw::Context;

use mujoco_rs_sys::no_render::*;
use mujoco_rs_sys as sys;
use std::ptr;



pub fn get_camera_jpg(simulation: &Simulation, height: usize, width: usize){
    // let cam_id = simulation.model.name_to_id(ObjType::CAMERA, "track").unwrap();
    let cam_id = simulation.model.name_to_id(ObjType::CAMERA, "onboard").unwrap();

    unsafe {
        // 初始化渲染上下文
        let mut vopt = mjvOption_::default();
        let mut cam = mjvCamera_::default();
        let mut scene = mjvScene_::default();
        let mut context = sys::render::mjrContext_::default();
        let mut glfw: glfw::Glfw = glfw::init(glfw::FAIL_ON_ERRORS).unwrap();
        
        glfw.window_hint(glfw::WindowHint::Visible(false));
        let (mut window, _events) = glfw.create_window(width.try_into().unwrap(), height.try_into().unwrap(), "hidden", glfw::WindowMode::Windowed)
            .expect("Failed to create GLFW window.");
        
        window.make_current();


        mjv_defaultOption(&mut vopt);
        mjv_defaultCamera(&mut cam);

        let maxgeom = 500; // 根据模型复杂度调整
        mjv_makeScene(simulation.model.ptr(), &mut scene, maxgeom);

        sys::mjr_defaultContext(&mut context);

        // 初始化 context
        sys::mjr_makeContext(simulation.model.ptr(), &mut context, 150);

        // 设置相机为你要的那个
        cam.fixedcamid = cam_id as i32;

        // 固定相机，而不是第三方视角
        cam.type_ = 2;

        // 更新场景
        mjv_updateScene(
            simulation.model.ptr(),
            simulation.state.ptr(),
            &mut vopt,
            ptr::null(),
            &mut cam,
            0xFFFFFF,
            &mut scene,
        );

        // 分配 buffer
        let mut rgb = vec![0u8; width * height * 3];
        let mut depth = vec![0f32; width * height];

        // 渲染到 buffer
        let viewport = sys::render::mjrRect_ { left: 0, bottom: 0, width: width as i32, height: height as i32 };
        sys::mjr_render(
            viewport,
            &mut scene,
            &mut context,
        );
        sys::mjr_readPixels(
            rgb.as_mut_ptr(),
            depth.as_mut_ptr(),
            viewport,
            &mut context,
        );
        let img: RgbImage = ImageBuffer::from_raw(width as u32, height as u32, rgb).unwrap();
        img.save("camera.jpg").unwrap();

        // 9. 释放 scene/context
        mjv_freeScene(&mut scene);
        sys::mjr_freeContext(&mut context);
    }

}