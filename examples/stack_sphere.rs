mod helper;
use helper::prelude::*;

use bevy::prelude::*;
use bevy_xpbd::*;

fn main() {
    App::new()
        .add_plugins(DefaultPlugins)
        .add_plugin(HelperPlugin) // Utitlity plugins for quality of life
        .add_plugin(XpbdPlugin::default()) // Our physics plugin
        //.add_plugin(PhysicsDebugPlugin)
        // local setup stuff
        .add_systems(Startup, setup_scene)
        .add_systems(OnEnter(ResetState::Playing), setup)
        .run();
}

const COUNT: usize = 10;
pub fn setup_scene(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    // light
    commands.spawn((
        DirectionalLightBundle {
            transform: Transform::from_xyz(50.0, 50.0, 50.0).looking_at(Vec3::ZERO, Vec3::Y),
            directional_light: DirectionalLight {
                shadows_enabled: true,
                ..default()
            },
            ..default()
        },
        Keep,
        Name::new("Light"),
    ));

    commands.spawn((
        Camera3dBundle {
            transform: Transform::from_xyz(0.0, 2.0, 30.0).looking_at(Vec3::ZERO, Vec3::Y),
            ..default()
        },
        CameraController,
        Keep,
        Name::new("Camera"),
    ));

    let ground_size = 100.;
    commands.spawn((
        PbrBundle {
            mesh: meshes.add(Mesh::from(shape::Box::new( ground_size, 1., ground_size))),
            material: materials.add(StandardMaterial {
                base_color: Color::DARK_GREEN,
                ..default()
            }),
            transform: Transform {
                translation: Vec3::new(0., -0.5, 0.),
                ..default()
            },
            ..default()
        },
        PhysicsBundle {
            collider: Collider::new_box(ground_size, 1., ground_size),
            mass: Mass::Static,
            ..default()
        },
        Keep,
        Name::new("Ground"),
    ));
}

pub fn setup(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
    asset_server: Res<AssetServer>,
) {
    let radius = 0.5;

    let texture = asset_server.load("checker_red.png");
    let mat = materials.add(StandardMaterial {
        base_color: Color::WHITE,
        base_color_texture: Some(texture),
        ..default()
    });

    let mesh = meshes.add(Mesh::from(shape::UVSphere {
        radius,
        ..default()
    }));

    for x in 0..COUNT {
        for y in 0..COUNT {
            for z in 0..COUNT {
                commands.spawn((
                    PbrBundle {
                        mesh: mesh.clone(),
                        material: mat.clone(),
                        transform: Transform {
                            translation: Vec3::new(
                                radius * 2. * x as f32,
                                radius * 2. * y as f32 + 5.,
                                radius * 2. * z as f32,
                            ),
                            ..default()
                        },
                        ..default()
                    },
                    PhysicsBundle {
                        collider: Collider::new_sphere(radius),
                        ..default()
                    },
                    Name::new("Sphere"),
                ));
            }
        }
    }
}
