use bevy::prelude::*;

#[derive(Component)]
pub struct Mass {
    value: f32,
    inverse: f32,
}

impl Mass {
    fn new(mass: f32) -> Self {
        Self {
            value: mass,
            inverse: 1.0 / mass,
        }
    }

    fn is_static() -> Self {
        Self {
            value: f32::MAX,
            inverse: 0.0,
        }
    }
}

impl Default for Mass {
    fn default() -> Self {
        Self::new(1.0)
    }
}
