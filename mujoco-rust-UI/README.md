# README

## Render scene & display

Use glfw to initialize the UI window and bind it to the corresponding OpenGL render.

1. Initialize GLFW

````rust
use glfw;
let mut glfw = glfw::init(glfw::FAIL_ON_ERRORS).unwrap();
````

2. Initialize window & listening functions

````rust
use glfw;
use gl;
use glfw::Context;

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
````

3. Further, initialize mujoco render related variables, camera `cam`, scene `scn`, context `con` and option `opt` respectively

````rust
use mujoco_rs_sys::render;
use mujoco_rs_sys::no_render;

let mut cam = render::mjvCamera_::default();
let mut scn = render::mjvScene_::default();
let mut con = render::mjrContext_::default();
let mut opt = render::mjvOption_::default();

unsafe {
    no_render::mjv_defaultCamera(&mut cam);
    render::mjv_defaultScene(&mut scn);
    render::mjr_defaultContext(&mut con);

    no_render::mjv_makeScene(simulation.model.ptr(), &mut scn, 1000);
    render::mjr_makeContext(simulation.model.ptr(), &mut con, 200);
    no_render::mjv_defaultOption(&mut opt);
}
````

where cam defines the rendering angle, scn defines the scene structure, con encapsulates OpenGL related state (e.g. shaders, textures), opt determines how the scene is rendered (e.g. which geometry is displayed, whether lights are enabled, etc.)

At this point, the rendering structure is initialized.

4. Update scenes at each moment in time within the main loop

````rust
use mujoco_rs_sys::render;
use mujoco_rs_sys::no_render;


// sim running until the window closes
while !window.should_close() {
    
    // associate GLFW window with an OpenGL state
    window.make_current();

    // get window size
    let (width, height) = window.get_framebuffer_size();

    // clear buffer
    gl::Clear(gl::COLOR_BUFFER_BIT | gl::DEPTH_BUFFER_BIT);

    // update render
    no_render::mjv_updateScene(
        simulation.model.ptr(),
        simulation.state.ptr(),
        opt,
        ptr::null(),
        &mut cam,
        0xFFFFFF,
        &mut scn,
    );

    // define viewport
    let viewport = render::mjrRect_ {
        left: 0,
        bottom: 0,
        width: width,
        height: height,
    };

    // start render
    render::mjr_render(viewport, &mut scn, &mut con);

    // swap buffer to display render scene
    window.swap_buffers();
    
   	... // sim operation
    
    // Get events in real time
    glfw.poll_events();

}
````

## 1st\3rd-person perspective

Specify whether the rendering perspective is in the first or third person. Change this in the cam attribute.

````rust
use mujoco_rs_sys::render;

let mut cam = render::mjvCamera_::default();

// No actual camera specified
cam.type_ = 1; // free perspective
cam.trackbodyid = 1; // Set tracked object ID
cam.distance = 5.0; // Set distance between perspective and model 

// actual camera specified
cam.type_ = 2; // fixed perspective
cam.fixedcamid = cam_id; // Set fixed camera id
````

In Mujoco render, when no actual `camera` is specified, the default angle of view is the third-person angle of view, so you only need to set the angle type, the model ID for angle tracking movement, and the distance between the angle of view and the model.

![image-20250914162457000](https://gitee.com/m3nglin/pic/raw/master/image/image-20250914162457000.png)

When you want to simulate a camera actually in a certain position in the model, you need to specify the camera ID, and set the rendered perspective to the perspective of the camera, as shown in the figure, which is the perspective directly below the first person perspective.

![image-20250914163021089](https://gitee.com/m3nglin/pic/raw/master/image/image-20250914163021089.png)

## Multiple perspectives in one window

Render multiple camera images into a glfw window, as shown in the figure:

![image-20250914162904677](https://gitee.com/m3nglin/pic/raw/master/image/image-20250914162904677.png)

Since multiple views need to be managed, a structure is defined here for easy management. `Vec` is used in the structure to store information of each camera.

````rust
pub struct UIState {
    pub cameras: Vec<render::mjvCamera_>,
    pub opt: render::mjvOption_,
    pub scenes: Vec<render::mjvScene_>,
    pub contexts: Vec<render::mjrContext_>,
    pub window: glfw::Window,
    pub events: mpsc::Receiver<(f64, glfw::WindowEvent)>,
}
````

First, initialize GLFW as the global graphical management framework.

````rust
let mut glfw = glfw::init(glfw::FAIL_ON_ERRORS).unwrap();
````

Declare the perspective, scene, and OpenGL state depending on the camera (`opt` does not need to be repeated).

````rust
pub fn ui_init(glfw: &mut glfw::Glfw, simulation: &Simulation, cam_ids: &[i32]) -> UIState {

	... // init window & events

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
		// 1st-person perspective 
        cam.type_ = 2; // fixed perspective 
        cam.fixedcamid = cam_id;
        
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
````

To use this function in the main function, pass the camera ID defined in the model file.

````rust
// get camera ID by specifying name
let cam1_id = simulation.model.name_to_id(ObjType::CAMERA, "camera1").unwrap() as i32;
let cam2_id = simulation.model.name_to_id(ObjType::CAMERA, "camera2").unwrap() as i32;
let cam3_id = simulation.model.name_to_id(ObjType::CAMERA, "camera3").unwrap() as i32;
let cam4_id = simulation.model.name_to_id(ObjType::CAMERA, "camera4").unwrap() as i32;

// 1st-person perspective 
let mut ui_state_1st = ui::ui_init(&mut glfw, &simulation, [cam1_id, cam2_id, cam3_id, cam4_id].as_ref());
````

Similarly, to update the rendering angle of each camera in real time, it is necessary to import the rendering angle of each camera as a part of the main window, so its `viewport` needs to be re-defined.

````rust
// get window size
let (width, height) = ui_state.window.get_framebuffer_size();
let num_cameras = ui_state.cameras.len().min(4);
let cols = if num_cameras < 2 { 1 } else { 2 };
let rows = if num_cameras <= 2 { 1 } else { 2 };
let sub_window_width = width / cols as i32;
let sub_window_height = height / rows as i32;

...

for i in 0..num_cameras {
    ...
    
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
    ...
}
````

Call this function in the main loop, passing in the return value of `init_ui()`

````rust
// 1st-person perspective 
let mut ui_state_1st = ui::ui_init(&mut glfw, &simulation, [cam1_id, cam2_id, cam3_id, cam4_id].as_ref());

// sim running until the window closes
while !ui_state_1st.window.should_close() {

    ui::update_scene(&simulation, &mut ui_state_1st);

	... // sim operation

    // Get events in real time
    glfw.poll_events();
}
````

> note: Rendering multiple views at the same time will cause a lot of CPU load, so on-demand rendering is required

## Free glfw resource

Correspondingly, after the end of the main cycle, relevant resources should be released, as follows:

````rust
use mujoco_rs_sys::render;

... 
unsafe{
    render::mjv_freeScene(&mut scn);
    render::mjr_freeContext(&mut con);
}
````

If you have multiple perspectives, you can release them one by one in the for loop

````rust
unsafe{
    for i in 0..ui_state.scenes.len() {
        render::mjv_freeScene(&mut ui_state.scenes[i]);
        render::mjr_freeContext(&mut ui_state.contexts[i]);
    }
}
````