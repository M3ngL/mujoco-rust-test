# README

1. **Initialize GLFW window with properties set to invisible**

````rust
let mut glfw = glfw::init(glfw::FAIL_ON_ERRORS).unwrap();

let (mut window, _events) = {
    glfw.window_hint(glfw::WindowHint::Visible(false));
    glfw.create_window(640, 480, "hidden", glfw::WindowMode::Windowed)
        .expect("Unable to create hidden GLFW window.")
};
````

Note that the window size `640x480` should be consistent with the subsequent ffmpeg initialization video stream resolution `-video_size` parameter, otherwise it will cause video stream picture distortion.

2. **Initialize ffmpeg**

Use `std::process::Command` to invoke FFmpeg command of local system and pass in its command parameters

````rust
use std::process::{Command, Stdio};

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
````

where the `stdin` of the child process is connected to the `stdout` of the parent process (i.e. the current rust project), i.e. the rendered picture frame, so that the parent process can write picture frame data to it.

3. **Frame by frame rendering and passing into the pipeline**

The idea is to generate video frames in the Rust program and write them to the pipeline via `child.stdin.unwrap().write_all(frame_bytes)`. FFmpeg receives pipeline output, encodes and pushes RTSP stream in real time, and other clients can play it via `rtsp://localhost:8554/mystream`

````bash
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
    ... 
   	// read the main window's Pixels
    render::mjr_readPixels(rgb.as_mut_ptr(), ptr::null_mut(), full_viewport, &mut ui_state.con);
	
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
	
    let _ = stdin.write_all(&flipped_rgb);
}
````

The rendered image is consistent with UI rendering, except that after the image rendering is updated,`render::mjr_render` is no longer called to render the displayed image directly, but the rgb data of the image is read and returned (sometimes the video stream reverses the image upside down, so there is also vertically flipped rgb data), and written into the pipeline output.

The video stream effect is as shown in the figure (displayed using QGC of ground station). FFmpeg push needs RTSP protocol server like `mediamtx` as intermediary before it can be transmitted to other clients for display.

![image-20250916102200116](https://gitee.com/m3nglin/pic/raw/master/image/image-20250916102200116.png)