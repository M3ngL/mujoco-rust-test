use std::io::Write;
use std::process::{Command, Stdio};
use std::thread;
use std::time::Duration;
mod camera;
use mujoco_rs_sys::render::*;

fn main() {
    // 配置 FFmpeg 命令
    let rtsp_url = "rtsp://localhost:8554/mystream".to_string();
    let mut ffmpeg: std::process::Child = match Command::new("ffmpeg")
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
        .spawn()
    {
        Ok(child) => child,
        Err(e) => {
            eprintln!("错误: 无法启动 FFmpeg: {}", e);
            return;
        }
    };


    let model = mujoco_rust::Model::from_xml("/home/m3/project/myRustPilot/x2/scene.xml".to_string()).unwrap();
    let simulation = mujoco_rust::Simulation::new(model.clone());

    // 获取 FFmpeg 的 stdin
    let mut stdin: std::process::ChildStdin = match ffmpeg.stdin.take() {
        Some(stdin) => stdin,
        None => {
            eprintln!("错误: 无法获取 FFmpeg stdin");
            return;
        }
    };

    let (mut window, mut VS_cam, mut VS_vopt, mut VS_scene, mut VS_context) = camera::init_glfw(&simulation);
    // 仿真循环
    for i in 0..1000 {
        simulation.step();

        simulation.control([3.5,3.5,3.5,3.5].as_ref());
        let frame = camera::update_mjscene(&simulation,  &mut VS_cam, &mut VS_vopt, &mut VS_scene, &mut VS_context);
        // 写入帧数据到 FFmpeg
        if let Err(e) = stdin.write_all(&frame) {
            eprintln!("错误: 写入 FFmpeg 失败: {}", e);
            return;
        }


        
        // 控制帧率（30 FPS）
        // thread::sleep(Duration::from_millis(1000 / 30));

    }
    unsafe {
            mjv_freeScene(&mut VS_scene);
            mjr_freeContext(&mut VS_context);
        }
    // 关闭 stdin
    drop(stdin);

    // 等待 FFmpeg 进程结束
    match ffmpeg.wait_with_output() {
        Ok(output) => {
            if !output.status.success() {
                eprintln!("FFmpeg 错误: {}", String::from_utf8_lossy(&output.stderr));
                return;
            }
        }
        Err(e) => {
            eprintln!("错误: 等待 FFmpeg 失败: {}", e);
            return;
        }
    }

    println!("视频流传输完成");
}