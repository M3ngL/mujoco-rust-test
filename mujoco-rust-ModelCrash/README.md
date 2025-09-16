# README

The data related to model collision include `ncon` in `mjData_` describing how many collision points there are at the current moment, and `contact` in `mjData_` defining collision details.

> `mjData_` defined in mujoco-rust is the same as `mjData`

In mujoco-rust, the method to obtain `mjData_` of the current model is as follows:

````rust
let model = mujoco_rust::Model::from_xml("../x2/scene.xml".to_string()).unwrap();
let simulation = mujoco_rust::Simulation::new(model.clone());
unsafe {
    let mj_data = &*simulation.state.ptr(); // &mjData_
}
````

Further obtain relevant collision data, such as the location of each collision point.

````rust
// get contact data in mj_data
unsafe {
    let mj_data = &*simulation.state.ptr(); // mj_data needs to be updated in real time
    let ncon = mj_data.ncon as usize;
    for i in 0..ncon {
        let contact = &*mj_data.contact.add(i);
        let pos = contact.pos;
        println!(
            "contact {} at [{}, {}, {}]",
            i, pos[0], pos[1], pos[2]
        );
    }
}
````

In addition, note that the categories in the model definition file must be consistent, otherwise they will not be included in the collision array. The relevant field is `shape ="x" conaffinity="x"`.