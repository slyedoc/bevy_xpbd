mod components;
mod config;
mod spatial_hash;

pub use components::*;
pub use config::*;

use bevy::{ecs::query::WorldQuery, prelude::*, utils::Instant};
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
        .add_systems( Update, spawn.run_if(in_state(PhysicsState::Running)).before(simulate))
        .add_systems( Update, simulate.run_if(in_state(PhysicsState::Running)))
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

    let t0 = Instant::now();
    // Build spatial hash
    spatial_hash.clear();
    for pb in query.iter() {
        let radius = pb.collider.bounding_radius()  + (pb.linear_velocity.0 * time.delta_seconds()).length();
        gizmos.sphere(pb.transform.translation, Quat::IDENTITY, radius, Color::RED.with_a(0.5));
        spatial_hash.insert(
            pb.entity,
            pb.transform.translation,
            radius,
        );
    }
    let collision_pairs = spatial_hash.get_collision_pairs();
    info!("Spatial hash took: {:?}", t0.elapsed());

    for _ in 0..config.sub_steps {
        for mut pb in query.iter_mut() {       
            if pb.inverse_mass.0 == 0. {
                continue;
            }     
            
            // update linear velocity and position
            pb.previous.translation = pb.transform.translation; 
            pb.linear_velocity.0 += sdt * (config.gravity * pb.inverse_mass.0);
            pb.transform.translation += sdt * pb.linear_velocity.0;

            // qprev ← q;
            pb.previous.rotation = pb.transform.rotation;

            // 𝜔 ← 𝜔 + ℎ I−1 (𝜏ext − (𝜔 × (I𝜔)));
            let angular_velocity = &mut *pb.angular_velocity;
            let torque = pb.inertia_tensor.0 * angular_velocity.0;
            let cross_product = angular_velocity.0.cross(torque);
            let external_torque = Vec3::ZERO;
            angular_velocity.0 += sdt * pb.inverse_inertia_tensor.0 * (external_torque - cross_product);
            info!("angular_velocity: {:?}", angular_velocity.0);

            // q ← q + ℎ 21 [𝜔 𝑥 , 𝜔 𝑦 , 𝜔 𝑧 , 0] q;
            let scaled_angular_velocity = sdt * 0.5 * angular_velocity.0;
            if scaled_angular_velocity != Vec3::ZERO {
                info!("scaled_angular_velocity: {:?}", scaled_angular_velocity);
                let rotation_change = Quat::from_xyzw(scaled_angular_velocity.x, scaled_angular_velocity.y, scaled_angular_velocity.z, 0.0) * pb.transform.rotation;
                info!("rotation_change: {:?}", rotation_change);
                pb.transform.rotation *= rotation_change;
            }
            
            // q ← q/|q|;
            pb.transform.rotation = pb.transform.rotation.normalize();
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
                        // Solve position constraints for sphere-sphere collision
                        let ra = contact.point - pb1.transform.translation;
                        let rb = contact.point - pb2.transform.translation;

                        let k_inv_mass = pb1.inverse_mass.0 + pb2.inverse_mass.0;
                        let ra_cross_n = ra.cross(contact.normal);
                        let rb_cross_n = rb.cross(contact.normal);
                        let k_inv_inertia = ra_cross_n.dot(pb1.inverse_inertia_tensor.0 * ra_cross_n) + rb_cross_n.dot(pb2.inverse_inertia_tensor.0 * rb_cross_n);

                        let impulse_magnitude = contact.penetration / (k_inv_mass + k_inv_inertia);
                        let impulse = impulse_magnitude * contact.normal;
        
                        // Update linear velocities
                        pb1.linear_velocity.0 -= pb1.inverse_mass.0 * impulse;
                        pb2.linear_velocity.0 += pb2.inverse_mass.0 * impulse;


                        // Update angular velocities
                        pb1.angular_velocity.0 -= pb1.inverse_inertia_tensor.0 * ra.cross(impulse);
                        pb2.angular_velocity.0 += pb2.inverse_inertia_tensor.0 * rb.cross(impulse);

                        // Update positions
                        pb1.transform.translation -= pb1.inverse_mass.0 * impulse;
                        pb2.transform.translation += pb2.inverse_mass.0 * impulse;
                    }
                }
                (Collider::Sphere { radius }, Collider::Box { size }) => todo!(),
                (Collider::Box { size }, Collider::Sphere { radius }) => todo!(),
                (Collider::Box { size: size_a }, Collider::Box { size: size_b }) => todo!(),
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
            pb.angular_velocity.0 = 2.0 * (if delta_rot.w >= 0.0 { delta_rot_vec } else { -delta_rot_vec }) / sdt;
        }
    }

    info!("Physics took: {:?}", t0.elapsed());
}

pub struct Contact {
    pub point: Vec3,
    pub normal: Vec3,
    pub penetration: f32,
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
        let penetration = combined_radius - ab_length;
        let normal = ab / ab_length;
        Some(Contact {
            point: pos_a + normal * (radius_a - penetration * 0.5),
            normal,
            penetration,
        })
    } else {
        None
    }
}