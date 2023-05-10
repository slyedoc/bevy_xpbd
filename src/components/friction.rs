use bevy::prelude::*;

/// 0.0: no friction at all, the body slides infinitely\
/// 1.0: high friction\
#[derive(Reflect, Clone, Copy, Component, Debug)]
#[reflect(Component)]
pub struct Friction {
    pub dynamic_coefficient: f32,
    pub static_coefficient: f32,
}

impl Friction {
    pub const ZERO: Self = Self {
        dynamic_coefficient: 0.0,
        static_coefficient: 0.0,
    };

    /// Creates a new Friction component with the same dynamic and static friction coefficients.
    fn new(friction_coefficient: f32) -> Self {
        Self {
            dynamic_coefficient: friction_coefficient,
            static_coefficient: friction_coefficient,
        }
    }
}

impl Default for Friction {
    fn default() -> Self {
        Self::new(0.3)
    }
}