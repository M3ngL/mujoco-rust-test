# README

## How to get sensor data

1. **Obtained by Sensor Definition Order**

When the order of sensor definition in the model file is known, the corresponding sensor value can be obtained from the sensor data array `sensordata()` on the basis of accessing the storage address `sensor_adr` of the sensor group and the dimension `sensor_dim` of each sensor output data.

The xml file below defines the sensor tag.

````xml
<sensor>
    <gyro name="body_gyro" site="imu"/>
    <accelerometer name="body_linacc" site="imu"/>
    <framequat name="body_quat" objtype="site" objname="imu"/>
</sensor>
````

We can get the value of each sensor by the following way, where `sensordata()` returns an array that stores the values of all sensors at the current time.

````rust
let model = mujoco_rust::Model::from_xml("../x2/scene.xml".to_string()).unwrap();
let mj_model = unsafe { *model.ptr() };
// 0: gyro
let gyro_start = unsafe { *mj_model.sensor_adr.add(0) } as usize;
let gyro_dim = unsafe { *mj_model.sensor_dim.add(0) } as usize;
let gyro_data = &simulation.sensordata()[gyro_start..gyro_start + gyro_dim];

// 1: accelerometer
let acc_start = unsafe { *mj_model.sensor_adr.add(1) } as usize;
let acc_dim = unsafe { *mj_model.sensor_dim.add(1) } as usize;
let acc_data = &simulation.sensordata()[acc_start..acc_start + acc_dim];

// 2: framequat
let att_start = unsafe { *mj_model.sensor_adr.add(2) } as usize;
let att_dim = unsafe { *mj_model.sensor_dim.add(2) } as usize;
let att_data = &simulation.sensordata()[att_start..att_start + att_dim];
````

2. **Obtained by Sensor Name**

Also need to know the definition of the sensor name in the model file, as follows

````xml
<body>
  <site name="rf_0" pos="0.1 0 0.05" zaxis="1 0 0"/>
  <site name="rf_15" pos="0.1 0 0.05" zaxis="0.9659 0.2588 0"/>
  <site name="rf_30" pos="0.1 0 0.05" zaxis="0.8660 0.5 0"/>
  ...
  <site name="rf_330" pos="0.1 0 0.05" zaxis="0.8660 -0.5 0"/>
  <site name="rf_345" pos="0.1 0 0.05" zaxis="0.9659 -0.2588 0"/>
</body>
...
<sensor>
  <rangefinder name="body_rf_0" site="rf_0" cutoff="10" noise="0.01"/>
  <rangefinder name="body_rf_15" site="rf_15" cutoff="10" noise="0.01"/>
  <rangefinder name="body_rf_30" site="rf_30" cutoff="10" noise="0.01"/>
  ...
  <rangefinder name="body_rf_315" site="rf_315" cutoff="10" noise="0.01"/>
  <rangefinder name="body_rf_330" site="rf_330" cutoff="10" noise="0.01"/>
  <rangefinder name="body_rf_345" site="rf_345" cutoff="10" noise="0.01"/> 
</sensor>
````

Given the `site name` under the body tag, return its sensor ID in the sensor group by passing its `site name` to `name_to_id()`. 

Then the corresponding value is further obtained from the sensor value array through the ID.

````rust
let model = mujoco_rust::Model::from_xml("../x2/scene.xml".to_string()).unwrap();
let mj_model = unsafe { *model.ptr() };

// get Rangefinder ids
let mut rf_ids: Vec<u16>  = Vec::new();
for angle in angles.iter() {
    let sensor_name = format!("rf_{}", angle);
    let id = model.name_to_id(ObjType::SITE, &sensor_name).unwrap();
    rf_ids.push(id);
}
// get data from id
for id in rf_ids.iter() {
    let data = simulation.sensordata()[*id as usize + 1];
    print!("rf_{}: {:?} ", id, data);
}
````

## How to get model names

As mentioned above, the names of models and sensors are known by directly examining the xml file, but you can also obtain all the structure names defined in the model file through code, including sensor, geometry name, environment name, etc.

````rust
use std::ffi::CStr;
use std::slice;

// get all model names
let model = mujoco_rust::Model::from_xml("../x2/scene.xml".to_string()).unwrap();
let mj_model = unsafe { *model.ptr() };
let mut model_names: Vec<&str> = Vec::new();
unsafe{
    let data = slice::from_raw_parts(mj_model.names as *const u8,
         mj_model.nnames as usize);
    let mut start = 0;
    for (i, &c) in data.iter().enumerate() {
        if c == 0 {
            let s = CStr::from_bytes_with_nul_unchecked(&data[start..=i]);
            model_names.push(s.to_str().unwrap());
            start = i + 1;
        }
    }
}
println!("{:?}", model_names);
````