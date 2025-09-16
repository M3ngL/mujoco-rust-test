# MuJoCo-Rust Demos

https://github.com/MuJoCo-Rust/MuJoCo-Rust This project is an FFI binding for MuJoCo Rust. 

The top-level encapsulation uses `mujoco-rust-0.0.6`.
The underlying binding demonstrates `mujoco-rs-sys-0.0.4` https://docs.rs/mujoco-sys/latest/mujoco_sys/ 

This project only supports up to MuJoCo 2.3.5, so the local system's MuJoCo needs to be set to the corresponding old version. The new version of MuJoCo 3.x is not compatible. 

The download link for MuJoCo release 2.3.5 is https://github.com/google-deepmind/mujoco/releases/tag/2.3.5. 

---

Commonly used classes in MuJoCo 

* mjData
* mjContact
* mjModel
* ...

Within mujoco-rust, it is mainly divided into two categories: `no-render` and `render`, corresponding to non-rendering and rendering-related classes respectively. 

Use the mujoco-rust project approach. 

````rust
use std::ptr;
let model = mujoco_rust::Model::from_xml("simple.xml".to_string()).unwrap();
let simulation = MuJoCoSimulation::new(model);
let mj_data = &*simulation.state.ptr();
let mj_contact = &*mj_data.contact;
````

---

---

Demos mainly explain the operation of drone models and the acquisition of related data and rendered images. 

> The original file for the drone model used can be downloaded from: https://github.com/google-deepmind/mujoco_menagerie/tree/main/skydio_x2 

* **Sensor Data**: Obtain sensor data from the model loaded in mujoco-rust and get its ID based on the model name.
* **UI**: Render the internal images of MuJoCo and display them, or merge multiple images and then display them.
* **Video Streaming**: Transmit the rendered images as video streams to other clients.
* **Lidar**: Draw a lidar map based on the values of multiple Rangefinders.
* **Model Crash**: Obtain relevant information about model collisions, including the positions of collision points, the number of collisions, etc.