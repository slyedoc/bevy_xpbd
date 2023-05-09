use bevy::prelude::*;

#[derive(Debug, Component)]
pub enum Collider {
    Sphere { radius: f32 },
    Box { size: Vec3 },
}

impl Default for Collider {
    fn default() -> Self {
        Collider::new_sphere(1.0)
    }
}

impl Collider {
    pub fn new_sphere(radius: f32) -> Self {
        Collider::Sphere { radius }
    }

    pub fn new_box(x_length: f32, y_length: f32, z_length: f32) -> Self {
        let size = Vec3::new(x_length, y_length, z_length);
        Collider::Box { size }
    }

    pub fn get_inertia_tensor(&self, mass: f32) -> Mat3 {
        match self {
            Collider::Sphere { radius } => {
                let diagonal = (2.0 / 5.0) * mass * radius * radius;
                return Mat3::from_diagonal(Vec3::splat(diagonal));
            }
            Collider::Box { size } => {
                let dd = *size * *size;
                let diagonal = Vec3::new(dd.y + dd.z, dd.x + dd.z, dd.x + dd.y) * mass / 12.0;
                return Mat3::from_diagonal(diagonal);
            }
        }
    }

    // Returns the radius of the collider in world space
    pub fn bounding_radius(&self) -> f32 {
        match self {
            Collider::Sphere { radius } => *radius,
            Collider::Box { size } => {
                // find the largest dimension of the box
                let a = size.x.max(size.y).max(size.z) * 0.5;                
                (a.powf(2.0) * 3.).sqrt()
            }
        }
    }
}
