use bevy::prelude::*;

#[derive(Component, Reflect, Debug, Default, Deref, DerefMut)]
pub struct InertiaTensor(pub Mat3);
    

#[derive(Component, Reflect, Debug, Default, Deref, DerefMut)]
pub struct InverseInertiaTensor(pub Mat3);
