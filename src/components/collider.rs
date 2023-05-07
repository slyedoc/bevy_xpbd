use bevy::prelude::*;
use crate::components::*;

#[derive(Debug, Component)]
pub enum Collider {
    Sphere {
        radius: f32,
    },
    Box {
        size: Vec3,
    },

}

impl Default for Collider {
    fn default() -> Self {
        Collider::new_sphere(1.0)
    }
}

impl Collider {
    pub fn new_sphere(radius: f32) -> Self {
        Collider::Sphere {
            radius
        }
    }

    pub fn new_box(x_length: f32, y_length: f32, z_length: f32) -> Self {
        let size = Vec3::new(x_length, y_length, z_length);
        Collider::Box {            
            size,
        }
    }

    pub fn get_inertia_tensor(&self, mass: f32) -> Mat3 {

        match self {
            Collider::Sphere { radius } => {
                let i = (2.0 / 5.0) * mass * radius * radius;
                return Mat3::from_diagonal(Vec3::splat(i));        
            },
            Collider::Box { size } => {
                let dd = *size * *size;
                let diagonal = Vec3::new(dd.y + dd.z, dd.x + dd.z, dd.x + dd.y) * mass / 12.0;
                return Mat3::from_diagonal(diagonal);
            },
            
        }
    }

    pub fn update_aabb(&self, aabb: &mut PhysicsAabb, trans: &Transform, velocity: &Velocity, factor: f32) {

        match self {
            Collider::Sphere { radius } => {
                let margin = factor * velocity.linear.length();
                let half_extends = Vec3::splat(radius + margin);
                
                aabb.mins = trans.translation - half_extends;
                aabb.maxs = trans.translation + half_extends;
            },
            Collider::Box { size } => {
                aabb.clear();
        
                let margin = factor * velocity.linear.length();
                let half_extends = *size * 0.5 + Vec3::splat( margin);
                aabb.mins = trans.translation - half_extends;
                aabb.maxs = trans.translation + half_extends;
            },
        }                        
    }
}

