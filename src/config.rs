use bevy::prelude::*;

#[derive(Resource)]
pub struct PhysicsConfig {    
    pub sub_steps: u8,
    pub gravity: Vec3,
}

impl Default for PhysicsConfig {
    fn default() -> Self {
        Self {            
            sub_steps: 5,
            gravity: Vec3::new(0., -9.81, 0.),
        }
    }
}
