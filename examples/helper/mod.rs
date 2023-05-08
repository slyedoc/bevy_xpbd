mod camera_controller;
mod overlay;
mod reset;

use bevy_xpbd::PhysicsState;
use overlay::OverlayPlugin;
use reset::ResetPlugin;
use camera_controller::CameraControllerPlugin;
use bevy::prelude::*;

pub mod prelude {
    pub use super::camera_controller::{CameraController, CameraControls};
    pub use super::reset::{Keep, ResetState};
    pub use super::HelperPlugin;
}

pub struct HelperPlugin;

impl Plugin for HelperPlugin {
    fn build(&self, app: &mut App) {
        app.add_plugin(CameraControllerPlugin)
            .add_plugin(OverlayPlugin)
            .add_plugin(ResetPlugin)
            .add_systems(Update, pause_toggle);
    }
}

pub fn pause_toggle(keys: Res<Input<KeyCode>>, mut physics_state: ResMut<NextState<PhysicsState>>,
    current_physics_state: Res<State<PhysicsState>>) {
    if keys.just_pressed(KeyCode::Space) {
        if current_physics_state.eq(&PhysicsState::Paused) {
            physics_state.set(PhysicsState::Running);
        } else {
            physics_state.set(PhysicsState::Paused);
        }        
    }
}
