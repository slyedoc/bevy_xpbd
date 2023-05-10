use super::Contact;
use bevy::prelude::*;

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
            point1: pos_a + Vec3::X * half_a.x,
            point2: pos_b - Vec3::X * half_b.x,
            penetration_depth: overlap.x,
            normal: Vec3::X * ab.x.signum(),
        })
    } else if overlap.y < overlap.x && overlap.y < overlap.z {
        // closer to Y-axis edge
        Some(Contact {
            point1: pos_a + Vec3::Y * half_a.y,
            point2: pos_b - Vec3::Y * half_b.y,
            penetration_depth: overlap.y,
            normal: Vec3::Y * ab.y.signum(),
        })
    } else {
        // closer to Z-axis edge
        Some(Contact {
            point1: pos_a + Vec3::Z * half_a.z,
            point2: pos_b - Vec3::Z * half_b.z,
            penetration_depth: overlap.z,
            normal: Vec3::Z * ab.z.signum(),
        })
    }
}

#[test]
fn box_box_clear() {
    assert!(box_box_intersect(Vec3::ZERO, Vec3::ONE, Vec3::new(1.1, 0., 0.), Vec3::ONE).is_none());
    assert!(box_box_intersect(Vec3::ZERO, Vec3::ONE, Vec3::new(-1.1, 0., 0.), Vec3::ONE).is_none());
    assert!(box_box_intersect(Vec3::ZERO, Vec3::ONE, Vec3::new(0., 1.1, 0.), Vec3::ONE).is_none());
    assert!(box_box_intersect(Vec3::ZERO, Vec3::ONE, Vec3::new(0., -1.1, 0.), Vec3::ONE).is_none());
}

#[test]
fn box_box_intersection() {
    assert!(box_box_intersect(Vec3::ZERO, Vec3::ONE, Vec3::ZERO, Vec3::ONE).is_some());
    assert!(
        box_box_intersect(Vec3::ZERO, Vec3::ONE, Vec3::new(0.9, 0.9, 0.9), Vec3::ONE).is_some()
    );
    assert!(box_box_intersect(
        Vec3::ZERO,
        Vec3::ONE,
        Vec3::new(-0.9, -0.9, -0.9),
        Vec3::ONE
    )
    .is_some());
}

#[test]
fn box_box_contact() {
    let Contact {
        point1: _,
        point2: _,
        normal,
        penetration_depth,
    } = box_box_intersect(Vec3::ZERO, Vec3::ONE, Vec3::new(0.9, 0., 0.), Vec3::ONE).unwrap();

    assert!(normal.x > 0.);
    assert!(normal.y < 0.001);
    assert!((penetration_depth - 0.1).abs() < 0.001);
}
