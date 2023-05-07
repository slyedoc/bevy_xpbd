mod helper;
use helper::prelude::*;

use bevy::prelude::*;
use bevy_xpbd::*;

fn main() {
    App::new()
        .add_plugins(DefaultPlugins)
        // our physics plugin
        .add_plugin(HelperPlugin)
        .add_plugin(XpbdPlugin)
        //.add_plugin(PhysicsDebugPlugin)
        // local setup stuff
        .add_startup_system(setup_camera)
        .add_startup_system(setup)
        .run();
}

pub fn setup_camera(mut commands: Commands) {
    // light
    commands
        .spawn(DirectionalLightBundle {
            transform: Transform::from_xyz(50.0, 50.0, 50.0).looking_at(Vec3::ZERO, Vec3::Y),
            directional_light: DirectionalLight {
                shadows_enabled: true,
                ..default()
            },
            ..default()
        })
        .insert(Keep);

    commands
        .spawn(Camera3dBundle {
            transform: Transform::from_xyz(0.0, 2.0, 30.0).looking_at(Vec3::ZERO, Vec3::Y),
            ..default()
        })
        // Add our controller
        .insert(CameraController)
        .insert(Keep);
}

pub fn setup(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    // Ground
    commands
        .spawn(PbrBundle {
            mesh: meshes.add(Mesh::from(shape::Box::new(100., 1., 100.))),
            material: materials.add(StandardMaterial {
                base_color: Color::DARK_GREEN,
                ..default()
            }),
            transform: Transform {
                translation: Vec3::new(0., -1.0, 0.),
                ..default()
            },
            ..default()
        })
        .insert(PhysicsBundle {
            collider: Collider::new_box(100., 1., 100.),
            ..default()
        })
        .insert(Name::new("Ground"));

    // stack

    let radius = 0.5;
    let count = 10;

    let mat = materials.add(StandardMaterial {
        base_color: Color::WHITE,    
        ..default()
    });

    let mesh = meshes.add(Mesh::from(shape::UVSphere {
        radius,
        ..default()
    }));

    for x in 0..count {
        for y in 0..count {
            for z in 0..count {
                commands
                    .spawn(PbrBundle {
                        mesh: mesh.clone(),
                        material: mat.clone(),
                        transform: Transform {
                            translation: Vec3::new(
                                radius * 2. * x as f32,
                                radius * 2. * y as f32,
                                radius * 2. * z as f32,
                            ),
                            ..default()
                        },
                        ..default()
                    })
                    .insert(PhysicsBundle {
                        collider: Collider::new_sphere(radius),
                        ..default()
                    })
                    .insert(Name::new("Sphere"));
            }
        }
    }
}
