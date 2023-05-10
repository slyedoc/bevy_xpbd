use super::Contact;
use bevy::prelude::*;

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

    if penetration_depth < 0. {
        return None;
    }
    Some(Contact {
        normal,
        penetration_depth,
    })
}

#[test]
fn test_sphere_box_no_intersect() {
    let sphere_pos = Vec3::new(10., 10., 10.);
    let sphere_radius = 1.0;
    let box_pos = Vec3::new(0., 0., 0.);
    let box_size = Vec3::new(2., 2., 2.);

    assert_eq!(
        sphere_box_intersect(sphere_pos, sphere_radius, box_pos, box_size),
        None
    );
}

#[test]
fn test_sphere_box_intersect_center() {
    let sphere_pos = Vec3::new(0., 0., 0.);
    let sphere_radius = 1.0;
    let box_pos = Vec3::new(0., 0., 0.);
    let box_size = Vec3::new(2., 2., 2.);

    let result = sphere_box_intersect(sphere_pos, sphere_radius, box_pos, box_size);
    assert!(result.is_some());
    let result = result.unwrap();
    assert_eq!(result.normal, Vec3::new(0., 0., -1.));
    assert_eq!(result.penetration_depth, 2.0);
}

#[test]
fn test_sphere_box_intersect_edge() {
    let sphere_pos = Vec3::new(-0.5, 0., 0.);
    let sphere_radius = 0.5;
    let box_pos = Vec3::new(0.5, 0., 0.);
    let box_size = Vec3::new(1., 1., 1.);

    let result = sphere_box_intersect(sphere_pos, sphere_radius, box_pos, box_size).unwrap();
    assert_eq!(result.normal, Vec3::new(1., 0., 0.));
    assert_eq!(result.penetration_depth, 0.0);
}
