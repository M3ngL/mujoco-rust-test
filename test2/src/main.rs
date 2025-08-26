use std::io::Write;
use std::process::{Command, Stdio};
use std::thread;
use std::time::Duration;
mod camera;

fn main() {
    // 配置 FFmpeg 命令
    let rtsp_url = "rtsp://localhost:8554/mystream".to_string();
    let mut ffmpeg = match Command::new("ffmpeg")
        .args([
            "-f", "rawvideo",
            "-pixel_format", "rgb24",
            "-video_size", "640x480",
            "-framerate", "60",
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
    let mut stdin = match ffmpeg.stdin.take() {
        Some(stdin) => stdin,
        None => {
            eprintln!("错误: 无法获取 FFmpeg stdin");
            return;
        }
    };

    // 模拟 MuJoCo 渲染的帧数据（每帧 640*480*3 字节）
    let frame_size = 640 * 480 * 3;
    // let mut frame = vec![0u8; frame_size]; // 示例：全黑帧，替换为 MuJoCo 渲染数据
    let ( viewport, mut context, mut scene, mut vopt, mut cam) = camera::init_camera(480, 640, &simulation);
    // 仿真循环
    for i in 0..1000 {
        let frame = camera::get_camera_jpg(&simulation, 480, 640, (viewport, &mut context, &mut scene, &mut vopt, &mut cam));


        // 写入帧数据到 FFmpeg
        if let Err(e) = stdin.write_all(&frame) {
            eprintln!("错误: 写入 FFmpeg 失败: {}", e);
            return;
        }

        simulation.step();
        // 控制帧率（30 FPS）
        // thread::sleep(Duration::from_millis(1000 / 60));

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