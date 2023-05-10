mod components;
mod config;
mod intersect;
mod spatial_hash;

pub use components::*;
pub use config::*;
pub use intersect::*;
use spatial_hash::SpatialHash;

use bevy::{ecs::query::WorldQuery, prelude::*};

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
            sub_steps: 5,
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
    mut contacts: Local<Vec<(Entity, Entity, Contact)>>,
) {
    let sdt = time.delta_seconds() / config.sub_steps as f32;
    if sdt == 0. {
        return;
    }

    // Build possable collision pairs
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

    for _ in 0..config.sub_steps {
        contacts.clear();

        // Integrate
        for mut pb in query.iter_mut().filter(|x| x.inverse_mass.0 != 0.) {
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
                }
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
                }
            }
        }

        // Update velocities
        for mut pb in query.iter_mut().filter(|x| x.inverse_mass.0 != 0.) {
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

            let pre_solve_relative_vel =
                pb1.previous.pre_solve_linear_velocity - pb2.previous.pre_solve_linear_velocity;
            let pre_solve_normal_vel = Vec3::dot(pre_solve_relative_vel, contact.normal);

            let relative_vel = pb1.linear_velocity.0 - pb2.linear_velocity.0;
            let normal_vel = Vec3::dot(relative_vel, contact.normal);
            let restitution = (pb1.restitution.0 + pb2.restitution.0) / 2.;

            let w_sum = pb1.inverse_mass.0 + pb2.inverse_mass.0;

            pb1.linear_velocity.0 += contact.normal
                * (-normal_vel - restitution * pre_solve_normal_vel)
                * pb1.inverse_mass.0
                / w_sum;
            pb2.linear_velocity.0 -= contact.normal
                * (-normal_vel - restitution * pre_solve_normal_vel)
                * pb2.inverse_mass.0
                / w_sum;
        }
    }
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
