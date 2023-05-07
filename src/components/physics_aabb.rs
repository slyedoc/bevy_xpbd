use bevy::prelude::*;
use std::ops::{Add, AddAssign};

// Note: Adding Xpbd to name so people don't conflict with bevy's Aabb
#[derive(Component, Debug, Reflect, Copy, Clone)]
#[reflect(Component)]
pub struct PhysicsAabb {
    pub mins: Vec3,
    pub maxs: Vec3,
}

impl Default for PhysicsAabb {
    fn default() -> Self {
        Self {
            mins: Vec3::splat(std::f32::MAX),
            maxs: Vec3::splat(std::f32::MIN),
        }
    }
}

impl PhysicsAabb {
    pub fn new(mins: Vec3, maxs: Vec3) -> PhysicsAabb {
        PhysicsAabb { mins, maxs }
    }

    #[inline]
    pub fn intersection(&self, b: &PhysicsAabb) -> bool {
        // Exit with no intersection if separated along an axis
        if self.maxs[0] < b.mins[0] || self.mins[0] > b.maxs[0] {
            return false;
        }
        if self.maxs[1] < b.mins[1] || self.mins[1] > b.maxs[1] {
            return false;
        }
        if self.maxs[2] < b.mins[2] || self.mins[2] > b.maxs[2] {
            return false;
        }
        // Overlapping on all axes means XpbdAabbs are intersecting
        true
    }

    // TODO: performance test form_points vs grow vs add_assign vs expand_by_point, all doing same thing
    pub fn from_points(pts: &[Vec3]) -> Self {
        pts.iter().fold(PhysicsAabb::default(), |acc, pt| acc + *pt)
    }

    pub fn area(&self) -> f32 {
        let e = self.maxs - self.mins; // box extent
        e.x * e.y + e.y * e.z + e.z * e.x
    }

    pub fn expand_by_point(&mut self, rhs: Vec3) {
        self.mins = Vec3::select(rhs.cmplt(self.mins), rhs, self.mins);
        self.maxs = Vec3::select(rhs.cmpgt(self.maxs), rhs, self.maxs);
    }

    pub fn width(&self) -> Vec3 {
        self.maxs - self.mins
    }

    pub fn clear(&mut self) {
        self.mins = Vec3::splat(std::f32::MAX);
        self.maxs = Vec3::splat(std::f32::MIN);        
    }
}


impl Add<Vec3> for PhysicsAabb {
    type Output = Self;
    fn add(self, pt: Vec3) -> Self::Output {
        PhysicsAabb {
            mins: Vec3::select(pt.cmplt(self.mins), pt, self.mins),
            maxs: Vec3::select(pt.cmpgt(self.maxs), pt, self.maxs),
        }
    }
}

impl AddAssign<Vec3> for PhysicsAabb {
    fn add_assign(&mut self, pt: Vec3) {
        self.mins = Vec3::select(pt.cmplt(self.mins), pt, self.mins);
        self.maxs = Vec3::select(pt.cmpgt(self.maxs), pt, self.maxs);
    }
}

impl Add<PhysicsAabb> for PhysicsAabb {
    type Output = Self;
    fn add(self, b: PhysicsAabb) -> Self::Output {
        PhysicsAabb {
            mins: Vec3::min(self.mins, b.mins),
            maxs: Vec3::max(self.maxs, b.maxs),
        }
    }
}

impl Add<&PhysicsAabb> for &PhysicsAabb {
    type Output = PhysicsAabb;
    fn add(self, b: &PhysicsAabb) -> Self::Output {
        PhysicsAabb {
            mins: Vec3::min(self.mins, b.mins),
            maxs: Vec3::max(self.maxs, b.maxs),
        }
    }
}

impl AddAssign<PhysicsAabb> for PhysicsAabb {
    fn add_assign(&mut self, rhs: PhysicsAabb) {
        self.mins = self.mins.min(rhs.mins);
        self.maxs = self.mins.max(rhs.maxs);
    }
}

#[test]
fn test_XpbdAabb_add() {
    let a = PhysicsAabb {
        mins: Vec3::new(0.0, 0.0, 0.0),
        maxs: Vec3::new(1.0, 1.0, 1.0),
    };
    let b = PhysicsAabb {
        mins: Vec3::new(1.0, 1.0, 1.0),
        maxs: Vec3::new(2.0, 2.0, 2.0),
    };
    let c = a + b;
    assert_eq!(c.mins, Vec3::new(0.0, 0.0, 0.0));
    assert_eq!(c.maxs, Vec3::new(2.0, 2.0, 2.0));
}