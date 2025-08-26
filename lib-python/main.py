# For test Mujoco Vedio streaming to QGC

import mujoco
import numpy as np
import ffmpeg
import asyncio


# 加载模型
model = mujoco.MjModel.from_xml_path("/home/m3/project/myRustPilot/x2/scene.xml")
data = mujoco.MjData(model)

# 初始化渲染器
renderer = mujoco.Renderer(model, height=480, width=640)

# 设置相机
camera_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_CAMERA, "cam")

renderer.update_scene(data, camera=camera_id)
async def stream_video():
    try:
        # 配置 FFmpeg 流
        stream = ffmpeg.input(
            "pipe:",
            format="rawvideo",
            pix_fmt="rgb24",
            s="640x480",
            r=30
        )
        stream = ffmpeg.output(
            stream,
            # "rtsp://172.20.28.190:8554/mystream",
            "rtsp://localhost:8554/mystream",
            format="rtsp",
            pix_fmt="yuv420p",
            vcodec="libx264",
            **{"rtsp_transport": "tcp"}  # 使用 TCP 传输以提高稳定性
        )

        # 启动 FFmpeg 异步进程
        process = ffmpeg.run_async(stream, pipe_stdin=True)

        # 仿真循环
        for _ in range(1000):
            data.ctrl[:] = np.random.uniform(0, 5, size=model.nu)  # 随机控制输入
            mujoco.mj_step(model, data)
            renderer.update_scene(data, camera=camera_id)
            frame = renderer.render()  # 获取 RGB 图像
            process.stdin.write(frame.tobytes())  # 写入管道

        # 关闭管道
        process.stdin.close()
        process.wait()  # 使用同步 wait 方法

    except ffmpeg.Error as e:
        print(f"FFmpeg 错误: {e.stderr.decode()}")
        raise
    except BrokenPipeError:
        print("管道破裂：FFmpeg 进程意外终止，检查 MediaMTX 是否运行或 RTSP 地址是否正确")
        raise
    except Exception as e:
        print(f"其他错误: {str(e)}")
        raise

# 运行异步函数
asyncio.run(stream_video())
