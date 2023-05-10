mod sphere_sphere;
mod sphere_box;
mod box_box;

pub use sphere_sphere::*;
pub use sphere_box::*;
pub use box_box::*;

use bevy::prelude::*;

#[derive(Debug, PartialEq)]
pub struct Contact {
    pub normal: Vec3,
    pub penetration_depth: f32,
}
