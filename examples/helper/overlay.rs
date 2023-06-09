
use bevy::{
    diagnostic::{Diagnostics, FrameTimeDiagnosticsPlugin},
    prelude::*,
};

use super::reset::Keep;

pub struct OverlayPlugin;

impl Plugin for OverlayPlugin {
    fn build(&self, app: &mut App) {
        app.add_plugin(FrameTimeDiagnosticsPlugin::default())
            .add_systems( Startup, setup_overlay)
            .add_systems( Update, update_fps);
    }
}


#[derive(Component)]
struct FpsText;

#[derive(Component)]
struct PhysicsStateText;

#[derive(Component)]
struct DebugStateText;

pub const UI_SIZE: f32 = 30.0;

fn setup_overlay(mut commands: Commands, asset_server: Res<AssetServer>) {        
    let font = asset_server.load("fonts/FiraSans-Bold.ttf");

    commands
        .spawn(TextBundle {
            style: Style {
                position_type: PositionType::Absolute,
                left: Val::Px(10.),
                bottom: Val::Px(10.),                
                align_self: AlignSelf::FlexEnd,
                ..Default::default()
            },
            text: Text {
                sections: vec![
                    TextSection {
                        value: "FPS: ".to_string(),
                        style: TextStyle {
                            font: font.clone(),
                            font_size: UI_SIZE,
                            color: Color::WHITE,
                        },
                    },
                    TextSection {
                        value: "".to_string(),
                        style: TextStyle {
                            font: font.clone(),
                            font_size: UI_SIZE,
                            color: Color::GOLD,
                        },
                    },
                ],
                ..Default::default()
            },
            ..Default::default()
        })
        .insert(Name::new("ui FPS"))
        .insert(Keep)
        .insert(FpsText);
}

fn update_fps(diagnostics: Res<Diagnostics>, mut query: Query<&mut Text, With<FpsText>>) {
    for mut text in query.iter_mut() {
        if let Some(fps) = diagnostics.get(FrameTimeDiagnosticsPlugin::FPS) {
            if let Some(average) = fps.average() {
                // Update the value of the second section
                text.sections[1].value = format!("{:.0}", average);
                text.sections[1].style.color = match average {
                    x if x >= 50.0 => Color::GREEN,
                    x if x > 40.0 && x < 50.0 => Color::YELLOW,
                    x if x <= 40.0 => Color::RED,
                    _ => Color::WHITE,
                };
            }
        }
    }
}
