use bevy::prelude::*;

#[derive(Component, Reflect, Debug, Default, Deref, DerefMut)]
#[reflect(Component)]
pub struct LinearVelocity( pub Vec3);

#[derive(Component, Reflect, Debug, Default, Deref, DerefMut)]
#[reflect(Component)]
pub struct AngularVelocity(pub Vec3);
