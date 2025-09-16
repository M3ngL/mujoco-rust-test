# README

This section mainly explains how to display lidar images through the rangefinder in Mujoco model, and the specific lidar image rendering algorithm will not be described again. In summary, lidar maps are implemented using multiple rangefinders in mujoco sim, and the sensor values of rangefinders are their distances from collidable objects.

1. **Get rangefinder IDs**

First, the `rangefinder` defined in the xml file has the following format

````xml
<site name="rf_0" pos="0.1 0 0.05" zaxis="1 0 0"/>
<site name="rf_15" pos="0.1 0 0.05" zaxis="0.9659 0.2588 0"/>
<site name="rf_30" pos="0.1 0 0.05" zaxis="0.8660 0.5 0"/>
<site name="rf_45" pos="0.1 0 0.05" zaxis="0.7071 0.7071 0"/>
<site name="rf_60" pos="0.1 0 0.05" zaxis="0.5 0.8660 0"/>
...
<site name="rf_300" pos="0.1 0 0.05" zaxis="0.5 -0.8660 0"/>
<site name="rf_315" pos="0.1 0 0.05" zaxis="0.7071 -0.7071 0"/>
<site name="rf_330" pos="0.1 0 0.05" zaxis="0.8660 -0.5 0"/>
<site name="rf_345" pos="0.1 0 0.05" zaxis="0.9659 -0.2588 0"/>
````

Get ID using methods in Sensor Data section

````rust
let mut rf_ids: Vec<u16>  = Vec::new();
let angles = [
    0, 15, 30, 45, 60, 75, 90, 105, 120, 135, 150, 165,
    180, 195, 210, 225, 240, 255, 270, 285, 300, 315, 330, 345,
];
for angle in angles.iter() {
    let sensor_name = format!("rf_{}", angle);
    let id = model.name_to_id(ObjType::SITE, &sensor_name).unwrap();
    rf_ids.push(id);
}
````

2. **Get rangefinders' value**

Convert the sensor name (named in degrees) of each rangefinder from degrees to radians to represent the angle of the sensor in polar coordinates; limit the maximum distance that rangefinder can detect, and filter out negative values (when negative, it indicates invalid measurements, e.g., no obstacles)

````rust
use std::f64::consts::PI;

let mut points = Vec::new();
let angles = [
    0, 15, 30, 45, 60, 75, 90, 105, 120, 135, 150, 165,
    180, 195, 210, 225, 240, 255, 270, 285, 300, 315, 330, 345,
];
for (i, &id) in rf_ids.iter().enumerate() {
    let distance = simulation.sensordata()[(id +1) as usize];
    let theta = angles[i] as f64 * PI / 180.0;
    if distance >= 0.0 && distance <= 10.0 {
        points.push((theta, distance));
    }
}
````

3. Lidar mapping

Draw according to the value `distance` of each `rangefinder` and its position in polar coordinates, use `minifb::window` instead of `glfw::window`, because the former is more convenient and faster when drawing 2d images, the code will not be repeated, the effect is as shown in the figure:

![image-20250916115854337](https://gitee.com/m3nglin/pic/raw/master/image/image-20250916115854337.png)