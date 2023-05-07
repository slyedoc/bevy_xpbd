mod camera_controller;
mod overlay;
mod reset;


use camera_controller::*;
use overlay::OverlayPlugin;
use reset::ResetPlugin;

use bevy::prelude::*;

pub mod prelude {
    pub use super::camera_controller::{CameraController, CameraControls};    
    pub use super::reset::Keep;
    pub use super::HelperPlugin;
}

pub struct HelperPlugin;

impl Plugin for HelperPlugin {
    fn build(&self, app: &mut App) {
        app // .add_plugin(CameraControllerPlugin)
            .add_plugin(OverlayPlugin)
            .add_plugin(ResetPlugin);
    }
}
