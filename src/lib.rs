mod components;
mod config;
mod spatial_hash;

pub use components::*;
pub use config::*;

use bevy::{ecs::query::WorldQuery, prelude::*};
use spatial_hash::SpatialHash;

#[derive(Bundle, Default)]
pub struct PhysicsBundle {
    //pub mode: PhysicsMode,
    pub mass: Mass,
    pub inverse_mass: InverseMass,
    pub collider: Collider,
    pub linear_velocity: LinearVelocity,
    pub angular_velocity: AngularVelocity,
    pub restitution: Restitution,

    // Should not be set by user
    pub inertia_tensor: InertiaTensor,
    pub inverse_inertia_tensor: InverseInertiaTensor,
    pub previous: Previous,
}

#[derive(WorldQuery)]
#[world_query(mutable, derive(Debug))]
pub struct PhysicsQuery {
    entity: Entity,
    transform: &'static mut Transform,
    linear_velocity: &'static mut LinearVelocity,
    angular_velocity: &'static mut AngularVelocity,
    mass: &'static mut Mass,
    inverse_mass: &'static mut InverseMass,
    inertia_tensor: &'static mut InertiaTensor,
    inverse_inertia_tensor: &'static mut InverseInertiaTensor,
    collider: &'static Collider,
    restitution: &'static mut Restitution,
    previous: &'static mut Previous,
}

pub struct XpbdPlugin {
    pub sub_steps: u8,
    pub gravity: Vec3,
}

impl Default for XpbdPlugin {
    fn default() -> Self {
        Self {
            sub_steps: 10,
            gravity: Vec3::new(0., -9.81, 0.),
        }
    }
}

#[derive(States, PartialEq, Eq, Debug, Clone, Hash, Default)]
pub enum PhysicsState {
    #[default]
    Running,
    Paused,
}

impl Plugin for XpbdPlugin {
    fn build(&self, app: &mut App) {
        app.insert_resource(PhysicsConfig {
            sub_steps: self.sub_steps,
            gravity: self.gravity,
        })
        .insert_resource(SpatialHash::new(20.))
        .add_systems(
            Update,
            spawn
                .run_if(in_state(PhysicsState::Running))
                .before(simulate),
        )
        .add_systems(Update, simulate.run_if(in_state(PhysicsState::Running)))
        .add_state::<PhysicsState>();
    }
}

// This will be based on Algorith 2 (page 5) in https://github.com/matthias-research/pages/blob/master/publications/PBDBodies.pdf
//  while simulating do
//      CollectCollisionPairs();                                    broad phase
//      ℎ ← Δ𝑡/numSubsteps;
//      for numSubsteps do                                          intergrate phase
//          for 𝑛 bodies and particles do
//              xprev ← x;
//              v ← v + ℎ fext /𝑚;
//              x ← x + ℎ v;
//              qprev ← q;
//              𝜔 ← 𝜔 + ℎ I−1 (𝜏ext − (𝜔 × (I𝜔)));
//              q ← q + ℎ 21 [𝜔 𝑥 , 𝜔 𝑦 , 𝜔 𝑧 , 0] q;
//              q ← q/|q|;
//          end
//          for numPosIters do                                      solve positions phase
//              SolvePositions(x1 , . . . x𝑛 , q1 , . . . q𝑛 );
//          end
//          for 𝑛 bodies and particles do                           update velocities phase
//              v ← (x − xprev )/ℎ;
//              Δq ← q qprev-1
//              𝝎 ← 2[Δq 𝑥 , Δq 𝑦 , Δq 𝑧 ]/ℎ;
//              𝝎 ← Δ𝑞 𝑤 ≥ 0 ? 𝝎 : −𝝎;
//          end
//          SolveVelocities(v1 , . . . v𝑛 , 𝜔1 , . . . 𝜔 𝑛 );       solve velocities phase
//      end
//  end

fn spawn(mut query: Query<PhysicsQuery, Added<Collider>>) {
    for mut pb in query.iter_mut() {
        pb.inverse_mass.0 = match *pb.mass {
            Mass::Value(v) => 1. / v,
            Mass::Static => 0.,
        };
        pb.inertia_tensor.0 = match *pb.mass {
            Mass::Value(v) => pb.collider.get_inertia_tensor(v),
            Mass::Static => Mat3::IDENTITY,
        };
        pb.inverse_inertia_tensor.0 = match *pb.mass {
            Mass::Value(_) => pb.inertia_tensor.0.inverse(),
            Mass::Static => Mat3::ZERO,
        };
    }
}

fn simulate(
    mut query: Query<PhysicsQuery>,
    time: Res<Time>,
    config: Res<PhysicsConfig>,
    mut spatial_hash: ResMut<SpatialHash>,
    mut gizmos: Gizmos,
) {
    let sdt = time.delta_seconds() / config.sub_steps as f32;
    if sdt == 0. {
        return;
    }

    // Build spatial hash
    spatial_hash.clear();
    for pb in query.iter() {
        let radius =
            pb.collider.bounding_radius() + (pb.linear_velocity.0 * time.delta_seconds()).length();
        gizmos.sphere(
            pb.transform.translation,
            Quat::IDENTITY,
            radius,
            Color::RED.with_a(0.5),
        );
        spatial_hash.insert(pb.entity, pb.transform.translation, radius);
    }
    let collision_pairs = spatial_hash.get_collision_pairs();

    let mut contacts: Vec<(Entity, Entity, Contact)> = Vec::new();

    for _ in 0..config.sub_steps {
        contacts.clear();
        for mut pb in query.iter_mut() {
            if pb.inverse_mass.0 == 0. {
                continue;
            }

            // update linear velocity and position
            pb.previous.translation = pb.transform.translation;
            pb.linear_velocity.0 += sdt * (config.gravity * pb.inverse_mass.0);
            pb.transform.translation += sdt * pb.linear_velocity.0;

            // pre solve values
            pb.previous.pre_solve_linear_velocity = pb.linear_velocity.0;

            // qprev ← q;
            pb.previous.rotation = pb.transform.rotation;

            // (𝜔 × (I𝜔))
            let cross_product = pb
                .angular_velocity
                .0
                .cross(pb.inertia_tensor.0 * pb.angular_velocity.0);
            let external_torque = Vec3::ZERO;
            // 𝜔 ← 𝜔 + ℎ I−1 (𝜏ext − (𝜔 × (I𝜔)));
            pb.angular_velocity.0 +=
                sdt * pb.inverse_inertia_tensor.0 * (external_torque - cross_product);

            // q ← q + ℎ 21 [𝜔 𝑥 , 𝜔 𝑦 , 𝜔 𝑧 , 0] q;
            let scaled_angular_velocity = sdt * 0.5 * pb.angular_velocity.0;
            if scaled_angular_velocity != Vec3::ZERO {
                let rotation_change = Quat::from_xyzw(
                    scaled_angular_velocity.x,
                    scaled_angular_velocity.y,
                    scaled_angular_velocity.z,
                    0.0,
                ) * pb.transform.rotation;
                pb.transform.rotation *= rotation_change; // TODO: this is not an add
                                                          // q ← q/|q|;
                pb.transform.rotation = pb.transform.rotation.normalize();
            }
        }

        // Solve positions
        for (e1, e2) in collision_pairs.iter() {
            let [mut pb1, mut pb2] = query.get_many_mut([*e1, *e2]).unwrap();

            // Static Static check
            if pb1.inverse_mass.0 == 0. && pb2.inverse_mass.0 == 0. {
                continue;
            }

            match (pb1.collider, pb2.collider) {
                (Collider::Sphere { radius: radius_a }, Collider::Sphere { radius: radius_b }) => {
                    if let Some(contact) = sphere_sphere_intersect(
                        pb1.transform.translation,
                        *radius_a,
                        pb2.transform.translation,
                        *radius_b,
                    ) {
                        constrain_body_positions(
                            &mut pb1.transform,
                            &mut pb2.transform,
                            &pb1.inverse_mass,
                            &pb2.inverse_mass,
                            contact.normal,
                            contact.penetration_depth,
                        );
                        contacts.push((pb1.entity, pb2.entity, contact));
                    }
                }
                (Collider::Sphere { radius }, Collider::Box { size: box_size }) => {
                    if let Some(contact) = sphere_box_intersect(
                        pb1.transform.translation,
                        *radius,
                        pb2.transform.translation,
                        *box_size,
                    ) {
                        constrain_body_positions(
                            &mut pb1.transform,
                            &mut pb2.transform,
                            &pb1.inverse_mass,
                            &pb2.inverse_mass,
                            contact.normal,
                            contact.penetration_depth,
                        );
                        contacts.push((pb1.entity, pb2.entity, contact));
                    }
                }
                (Collider::Box { size }, Collider::Sphere { radius }) => { 
                    if let Some(contact) = sphere_box_intersect(
                        pb1.transform.translation,
                        *radius,
                        pb2.transform.translation,
                        *size,
                    ) {
                        constrain_body_positions(
                            &mut pb1.transform,
                            &mut pb2.transform,
                            &pb1.inverse_mass,
                            &pb2.inverse_mass,
                            contact.normal,
                            contact.penetration_depth,
                        );
                        contacts.push((pb1.entity, pb2.entity, contact));
                    }
                },
                (Collider::Box { size: size_a }, Collider::Box { size: size_b }) => {
                    if let Some(contact) = box_box_intersect(
                        pb1.transform.translation,
                        *size_a,
                        pb2.transform.translation,
                        *size_b,
                    ) {
                        constrain_body_positions(
                            &mut pb1.transform,
                            &mut pb2.transform,
                            &pb1.inverse_mass,
                            &pb2.inverse_mass,
                            contact.normal,
                            contact.penetration_depth,
                        );
                        contacts.push((pb1.entity, pb2.entity, contact));
                    }
                },
            }
        }

        for mut pb in query.iter_mut() {
            if pb.inverse_mass.0 == 0. {
                continue;
            }

            // Update linear velocity
            pb.linear_velocity.0 = (pb.transform.translation - pb.previous.translation) / sdt;

            // Update angular velocity
            let delta_rot = pb.transform.rotation * pb.previous.rotation.inverse();
            let delta_rot_vec = Vec3::new(delta_rot.x, delta_rot.y, delta_rot.z);
            pb.angular_velocity.0 =
                2.0 * (if delta_rot.w >= 0.0 {
                    delta_rot_vec
                } else {
                    -delta_rot_vec
                }) / sdt;
        }
        // Solve velocities
        for (e1, e2, contact) in contacts.iter() {
            let [mut pb1, mut pb2] = query.get_many_mut([*e1, *e2]).unwrap();

            let n = (pb2.transform.translation - pb1.transform.translation).normalize();
            let pre_solve_relative_vel =
                pb1.previous.pre_solve_linear_velocity - pb2.previous.pre_solve_linear_velocity;
            let pre_solve_normal_vel = Vec3::dot(pre_solve_relative_vel, n);

            let relative_vel = pb1.linear_velocity.0 - pb2.linear_velocity.0;
            let normal_vel = Vec3::dot(relative_vel, n);
            let restitution = (pb1.restitution.0 + pb2.restitution.0) / 2.;

            let w_sum = pb1.inverse_mass.0 + pb2.inverse_mass.0;

            pb1.linear_velocity.0 +=
                n * (-normal_vel - restitution * pre_solve_normal_vel) * pb1.inverse_mass.0 / w_sum;
            pb2.linear_velocity.0 -=
                n * (-normal_vel - restitution * pre_solve_normal_vel) * pb2.inverse_mass.0 / w_sum;
        }
    }

    // info!("Physics took: {:?}", t0.elapsed());
}

pub struct Contact {
    pub normal: Vec3,
    pub penetration_depth: f32,
}

fn constrain_body_positions(
    trans_a: &mut Transform,
    trans_b: &mut Transform,
    inv_mass_a: &InverseMass,
    inv_mass_b: &InverseMass,
    n: Vec3,
    penetration_depth: f32,
) {
    let w_sum = inv_mass_a.0 + inv_mass_b.0;
    let pos_impulse = n * (-penetration_depth / w_sum);
    trans_a.translation += pos_impulse * inv_mass_a.0;
    trans_b.translation -= pos_impulse * inv_mass_b.0;
}

pub fn sphere_sphere_intersect(
    pos_a: Vec3,
    radius_a: f32,
    pos_b: Vec3,
    radius_b: f32,
) -> Option<Contact> {
    let ab = pos_b - pos_a;
    let combined_radius = radius_a + radius_b;
    let ab_sqr_len = ab.length_squared();
    if ab_sqr_len < combined_radius * combined_radius {
        let ab_length = ab_sqr_len.sqrt();
        let penetration_depth = combined_radius - ab_length;
        let normal = ab / ab_length;
        Some(Contact {
            normal,
            penetration_depth,
        })
    } else {
        None
    }
}

pub fn sphere_box_intersect(
    pos_a: Vec3,
    radius_a: f32,
    pos_b: Vec3,
    size_b: Vec3,
) -> Option<Contact> {
    let box_to_sphere = pos_a - pos_b;
    let box_to_sphere_abs = box_to_sphere.abs();
    let half_extents = size_b / 2.;
    let corner_to_center = box_to_sphere_abs - half_extents;
    let r = radius_a;

    if corner_to_center.x > r || corner_to_center.y > r || corner_to_center.z > r {
        return None;
    }

    let s = box_to_sphere.signum();

    let (normal, penetration_depth) = if corner_to_center.x > 0.
        && corner_to_center.y > 0.
        && corner_to_center.z > 0.
    {
        // Corner case
        let corner_to_center_sqr = corner_to_center.length_squared();
        if corner_to_center_sqr > r * r {
            return None;
        }
        let corner_dist = corner_to_center_sqr.sqrt();
        let penetration = r - corner_dist;
        let n = corner_to_center / corner_dist * -s;
        (n, penetration)
    } else if corner_to_center.x > corner_to_center.y && corner_to_center.x > corner_to_center.z {
        // Closer to X-axis edge
        (Vec3::X * -s.x, -corner_to_center.x + r)
    } else if corner_to_center.y > corner_to_center.x && corner_to_center.y > corner_to_center.z {
        // Closer to Y-axis edge
        (Vec3::Y * -s.y, -corner_to_center.y + r)
    } else {
        // Closer to Z-axis edge
        (Vec3::Z * -s.z, -corner_to_center.z + r)
    };

    Some(Contact {
        normal,
        penetration_depth,
    })
}

pub fn box_box_intersect(pos_a: Vec3, size_a: Vec3, pos_b: Vec3, size_b: Vec3) -> Option<Contact> {
    let half_a = size_a / 2.;
    let half_b = size_b / 2.;
    let ab = pos_b - pos_a;
    let overlap = (half_a + half_b) - ab.abs(); // exploit symmetry

    if overlap.x < 0. || overlap.y < 0. || overlap.z < 0. {
        None
    } else if overlap.x < overlap.y && overlap.x < overlap.z {
        // closer to X-axis edge
        Some(Contact {
            penetration_depth: overlap.x,
            normal: Vec3::X * ab.x.signum(),
        })
    } else if overlap.y < overlap.x && overlap.y < overlap.z {
        // closer to Y-axis edge
        Some(Contact {
            penetration_depth: overlap.y,
            normal: Vec3::Y * ab.y.signum(),
        })
    } else {
        // closer to Z-axis edge
        Some(Contact {
            penetration_depth: overlap.z,
            normal: Vec3::Z * ab.z.signum(),
        })
    }
}