mod components;

pub use components::*;
use bevy::prelude::*;

#[derive(Bundle, Default)]
pub struct PhysicsBundle {
    //pub mode: PhysicsMode,
    pub mass: Mass,
    pub collider: Collider,
    pub velocity: Velocity,
    pub restitution: Restitution,

    // Should not be set by user
    pub inertia_tensor: InertiaTensor,
    pub aabb: PhysicsAabb,
    
    //pub prev_pos: PrevPos,
    //pub prev_rot: PrevRot,
    //pub pre_solve_velocity: PreSolveVelocity,
}

pub struct XpbdPlugin;


impl Plugin for XpbdPlugin {
    fn build(&self, app: &mut App) {
 
    }
}