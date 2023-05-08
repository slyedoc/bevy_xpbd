use bevy::prelude::*;

#[derive(Component, Debug)]
pub enum Mass {
    Value(f32),
    Static,
}

#[derive(Component, Debug, Default)]
pub struct InverseMass(pub f32);

impl Default for Mass {
    fn default() -> Self {
        Self::Value(1.0)
    }
}
