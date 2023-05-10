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
    pub friction: Friction,
    // Should not be set by user
    pub inertia_tensor: InertiaTensor,
    pub inverse_inertia_tensor: InverseInertiaTensor,
    pub previous: Previous,
}

#[derive(WorldQuery)]
#[world_query(mutable, derive(Debug))]
pub struct PhysicsQueryItem {
    entity: Entity,
    transform: &'static mut Transform,
    linear_velocity: &'static mut LinearVelocity,
    angular_velocity: &'static mut AngularVelocity,
    mass: &'static mut Mass,
    inverse_mass: &'static mut InverseMass,
    inertia_tensor: &'static mut InertiaTensor,
    inverse_inertia_tensor: &'static mut InverseInertiaTensor,
    collider: &'static Collider,
    restitution: &'static Restitution,
    friction: &'static Friction,
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

fn spawn(mut query: Query<PhysicsQueryItem, Added<Collider>>) {
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
    mut query: Query<PhysicsQueryItem>,
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
            pb.previous.pre_solve_angular_velocity = pb.angular_velocity.0;

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

            let contact = match (pb1.collider, pb2.collider) {
                (Collider::Sphere { radius: radius_a }, Collider::Sphere { radius: radius_b }) => sphere_sphere_intersect(
                        pb1.transform.translation,
                        *radius_a,
                        pb2.transform.translation,
                        *radius_b,
                    ),                
                (Collider::Sphere { radius }, Collider::Box { size: box_size }) => sphere_box_intersect(
                        pb1.transform.translation,
                        *radius,
                        pb2.transform.translation,
                        *box_size,
                    ),
                (Collider::Box { size }, Collider::Sphere { radius }) => box_sphere_intersect(
                        pb1.transform.translation,
                        *size,
                        pb2.transform.translation,                        
                        *radius,
                    ),
                (Collider::Box { size: size_a }, Collider::Box { size: size_b }) =>  box_box_intersect(
                        pb1.transform.translation,
                        *size_a,
                        pb2.transform.translation,
                        *size_b,
                    ),                 
            };

            if let Some(contact) = contact {
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

        // Update velocities
        for mut pb in query.iter_mut().filter(|x| x.inverse_mass.0 != 0.) {
            // Update linear velocity
            pb.linear_velocity.0 = (pb.transform.translation - pb.previous.translation) / sdt;

            // Update angular velocity
            let delta_rot = pb.transform.rotation * pb.previous.rotation.inverse();            
            pb.angular_velocity.0 = 2.0 * delta_rot.xyz() / sdt;
            if delta_rot.w < 0.0 {
                pb.angular_velocity.0 = -pb.angular_velocity.0;
            }
        }

        // Solve velocities
        for (e1, e2, contact) in contacts.iter() {
            let [mut pb1, mut pb2] = query.get_many_mut([*e1, *e2]).unwrap();

            // let pre_solve_contact_vel1 = pb1.previous.pre_solve_linear_velocity + pb1.previous.pre_solve_angular_velocity.cross(contact.point1);
            // let pre_solve_contact_vel2 = pb2.previous.pre_solve_linear_velocity + pb2.previous.pre_solve_angular_velocity.cross(contact.point2);
            let pre_solve_relative_vel = pb1.previous.pre_solve_linear_velocity - pb2.previous.pre_solve_linear_velocity;
            let pre_solve_normal_vel = contact.normal.dot(pre_solve_relative_vel);

            // Compute relative normal and tangential velocities at the contact point (equation 29)
            //let contact_vel1 = pb1.linear_velocity.0 + pb1.angular_velocity.0.cross(contact.point1);
            //let contact_vel2 = pb2.linear_velocity.0 + pb2.angular_velocity.0.cross(contact.point2);
            let relative_vel = pb1.linear_velocity.0 - pb2.linear_velocity.0;
            let normal_vel = contact.normal.dot(relative_vel);

            // let tangent_vel = relative_vel - contact.normal * normal_vel;
            
            // let rot_matrix1 = Mat3::from_quat(pb1.transform.rotation);
            // let inv_inertia1 = (rot_matrix1 * pb1.inertia_tensor.0) * rot_matrix1.transpose();
            // let w1 = if pb1.inverse_mass.0 != 0. {
            //     let r_cross_n = contact.point1.cross(contact.normal); // Compute the cross product only once
    
            //     // The line below is equivalent to Eq (2) because the component-wise multiplication of a transposed vector and another vector is equal to the dot product of the two vectors.
            //     // a^T * b = a • b
            //     pb1.inverse_mass.0 + r_cross_n.dot(inv_inertia1 * r_cross_n)
            // } else {
            //     // Static and kinematic bodies are a special case, where 0.0 can be thought of as infinite mass.
            //     0.0
            // };


            // let rot_matrix2 = Mat3::from_quat(pb2.transform.rotation);
            // let inv_inertia2 = (rot_matrix2 * pb2.inertia_tensor.0) * rot_matrix2.transpose();
            // let w2 = if pb1.inverse_mass.0 != 0. {
            //     let r_cross_n = contact.point2.cross(contact.normal); // Compute the cross product only once
    
            //     // The line below is equivalent to Eq (2) because the component-wise multiplication of a transposed vector and another vector is equal to the dot product of the two vectors.
            //     // a^T * b = a • b
            //     pb2.inverse_mass.0 + r_cross_n.dot(inv_inertia2 * r_cross_n)
            // } else {
            //     // Static and kinematic bodies are a special case, where 0.0 can be thought of as infinite mass.
            //     0.0
            // };
          
            // let friction_impulse = get_dynamic_friction(
            //     tangent_vel,
            //     pb1.friction,
            //     pb1.friction,
            //     constraint.normal_lagrange,
            //     sub_dt.0,
            // );

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


#[allow(clippy::too_many_arguments)]
fn apply_pos_constraint(
    body1: &mut PhysicsQueryItem,
    body2: &mut PhysicsQueryItem,
    inv_inertia1: Mat3,
    inv_inertia2: Mat3,
    delta_lagrange: f32,
    dir: Vec3,
    r1: Vec3,
    r2: Vec3,
) -> Vec3 {
    // Positional impulse
    let p = delta_lagrange * dir;

    let rot1 = body1.transform.rotation;
    let rot2 = body2.transform.rotation;

    // Update positions and rotations of the bodies (equations 6-9)
    if body1.inverse_mass.0 != 0.0 {
        body1.transform.translation += p * body1.inverse_mass.0;
        body1.transform.rotation = rot1 + (Quat::from_vec4(0.5 * (inv_inertia1 * r1.cross(p)).extend(0.0))) * rot1;    
    }
    if body2.inverse_mass.0 != 0.0 {
        body2.transform.translation -= p * body2.inverse_mass.0;
        body2.transform.rotation = rot2 + (Quat::from_vec4(0.5 * (inv_inertia1 * r1.cross(p)).extend(0.0))) * rot1;            
    }

    p
}
/// Calculates velocity correction caused by dynamic friction.
pub(crate) fn get_dynamic_friction(
    tangent_vel: Vec3,
    friction1: &Friction,
    friction2: &Friction,
    normal_lagrange: f32,
    sub_dt: f32,
) -> Vec3 {
    let tangent_vel_magnitude = tangent_vel.length();

    // Avoid division by zero when normalizing the vector later.
    // We compare against epsilon to avoid potential floating point precision problems.
    if tangent_vel_magnitude.abs() <= f32::EPSILON {
        return Vec3::ZERO;
    }

    // Average of the bodies' dynamic friction coefficients
    let dynamic_friction_coefficient =
        (friction1.dynamic_coefficient + friction2.dynamic_coefficient) * 0.5;

    let normal_force = normal_lagrange / sub_dt.powi(2);

    // Velocity update caused by dynamic friction, never exceeds the magnitude of the tangential velocity itself
    -tangent_vel / tangent_vel_magnitude
        * (sub_dt * dynamic_friction_coefficient * normal_force.abs()).min(tangent_vel_magnitude)
}