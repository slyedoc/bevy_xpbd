use bevy::prelude::*;
use super::Contact;

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

    #[test]
    fn test_sphere_sphere_intersect_no_intersection() {
        let pos_a = Vec3::new(0.0, 0.0, 0.0);
        let radius_a = 1.0;
        let pos_b = Vec3::new(3.0, 0.0, 0.0);
        let radius_b = 1.0;

        let result = sphere_sphere_intersect(pos_a, radius_a, pos_b, radius_b);
        assert_eq!(result, None);
    }

    #[test]
    fn test_sphere_sphere_intersect_touching() {
        let pos_a = Vec3::new(0.0, 0.0, 0.0);
        let radius_a = 2.0;
        let pos_b = Vec3::new(4.0, 0.0, 0.0);
        let radius_b = 2.0;

        let result = sphere_sphere_intersect(pos_a, radius_a, pos_b, radius_b);
        assert_eq!(result, None);
    }

    #[test]
    fn test_sphere_sphere_intersect_intersecting() {
        let pos_a = Vec3::new(0.0, 0.0, 0.0);
        let radius_a = 3.0;
        let pos_b = Vec3::new(4.0, 0.0, 0.0);
        let radius_b = 2.0;

        let result = sphere_sphere_intersect(pos_a, radius_a, pos_b, radius_b);
        match result {
            Some(contact) => {
                let expected_normal = Vec3::new(1.0, 0.0, 0.0);
                assert_eq!(contact.normal, expected_normal);
                assert_eq!(contact.penetration_depth, 1.0);
            },
            None => panic!("Expected Some(Contact), got None"),
        }
    }
