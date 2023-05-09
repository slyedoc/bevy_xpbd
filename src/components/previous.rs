use bevy::prelude::*;

#[derive(Component, Default, Debug)]
pub struct Previous {
    pub(crate) translation: Vec3,
    pub(crate) rotation: Quat,
    pub(crate) pre_solve_linear_velocity: Vec3,
    // pub(crate) _angular_velocity: Vec3,
}
