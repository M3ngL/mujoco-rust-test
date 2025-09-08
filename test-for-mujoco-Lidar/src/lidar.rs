use minifb::{Window, WindowOptions, Key};
use std::f64::consts::PI;

pub fn init_lidar_window(width: usize, height: usize) -> Result<Window, Box<dyn std::error::Error>> {
    let window = Window::new(
        "LiDAR Scan (Press ESC to exit)",
        width,
        height,
        WindowOptions {
            borderless: true, // 禁用窗口装饰
            ..WindowOptions::default()
        }
    )?;
    Ok(window)
}


// 绘制圆形
pub fn draw_circle(buffer: &mut [u32], width: usize, height: usize, cx: f64, cy: f64, radius: f64, color: u32) {
    let cx = cx.round() as i32;
    let cy = cy.round() as i32;
    let radius = radius.round() as i32;
    for y in (cy - radius).max(0)..(cy + radius + 1).min(height as i32) {
        for x in (cx - radius).max(0)..(cx + radius + 1).min(width as i32) {
            let dx = x - cx;
            let dy = y - cy;
            if dx * dx + dy * dy <= radius * radius {
                buffer[(y * width as i32 + x) as usize] = color;
            }
        }
    }
}


// 绘制空心圆环（用于距离网格，细线）
pub fn draw_circle_outline(buffer: &mut [u32], width: usize, height: usize, cx: f64, cy: f64, radius: f64, color: u32) {
    let cx = cx.round() as i32;
    let cy = cy.round() as i32;
    let radius = radius.round() as i32;
    let mut x = radius;
    let mut y = 0;
    let mut err = 0;

    while x >= y {
        let points = [
            (cx + x, cy + y),
            (cx + y, cy + x),
            (cx - y, cy + x),
            (cx - x, cy + y),
            (cx - x, cy - y),
            (cx - y, cy - x),
            (cx + y, cy - x),
            (cx + x, cy - y),
        ];

        for (px, py) in points.iter() {
            if *px >= 0 && *px < width as i32 && *py >= 0 && *py < height as i32 {
                buffer[(*py * width as i32 + *px) as usize] = color;
            }
        }

        y += 1;
        err += 1 + 2 * y;
        if 2 * (err - x) + 1 > 0 {
            x -= 1;
            err += 1 - 2 * x;
        }
    }
}

// 绘制线条（Bresenham 算法）
pub fn draw_line(buffer: &mut [u32], width: usize, height: usize, x1: f64, y1: f64, x2: f64, y2: f64, color: u32) {
    let mut x1 = x1.round() as i32;
    let mut y1 = y1.round() as i32;
    let x2 = x2.round() as i32;
    let y2 = y2.round() as i32;

    let dx = (x2 - x1).abs();
    let dy = (y2 - y1).abs();
    let sx = if x1 < x2 { 1 } else { -1 };
    let sy = if y1 < y2 { 1 } else { -1 };
    let mut err = dx - dy;

    loop {
        if x1 >= 0 && x1 < width as i32 && y1 >= 0 && y1 < height as i32 {
            buffer[(y1 * width as i32 + x1) as usize] = color;
        }
        if x1 == x2 && y1 == y2 {
            break;
        }
        let e2 = 2 * err;
        if e2 > -dy {
            err -= dy;
            x1 += sx;
        }
        if e2 < dx {
            err += dx;
            y1 += sy;
        }
    }
}