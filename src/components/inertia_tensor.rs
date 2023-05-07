use bevy::prelude::*;

#[derive(Component, Reflect, Debug, Default)]
#[reflect(Component)]
pub struct InertiaTensor {
    pub value: Mat3,
    pub inverse: Mat3,
}
