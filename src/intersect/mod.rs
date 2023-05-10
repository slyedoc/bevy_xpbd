mod sphere_sphere;
mod sphere_box;
mod box_box;

pub use sphere_sphere::*;
pub use sphere_box::*;
pub use box_box::*;

use bevy::prelude::*;

#[derive(Debug, PartialEq)]
pub struct Contact {
    /// Position of the contact point in world space on the first object
    pub point1: Vec3,

    /// Position of the contact point in world space on the first object
    pub point2: Vec3,
    
    pub normal: Vec3,
    pub penetration_depth: f32,
}

impl Contact {
    #[inline]
    pub fn swap(mut self) -> Self {
        std::mem::swap(&mut self.point1, &mut self.point2);
        self.normal = -self.normal;
        self
    }
}