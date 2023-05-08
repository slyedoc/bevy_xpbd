use bevy::{prelude::*, utils::HashMap};

#[derive(Resource)]
pub struct SpatialHash {
    buckets: HashMap<(i32, i32, i32), Vec<Entity>>,
    entities: HashMap<Entity, (Vec3, f32)>,
    bucket_size: f32,
}

impl SpatialHash {
    pub fn new(bucket_size: f32) -> Self {
        SpatialHash {
            buckets: HashMap::new(),
            entities: HashMap::new(),
            bucket_size,
        }
    }

    pub fn insert(&mut self, entity: Entity, position: Vec3, radius: f32) {
        let min = (
            ((position.x - radius) / self.bucket_size).floor() as i32,
            ((position.y - radius) / self.bucket_size).floor() as i32,
            ((position.z - radius) / self.bucket_size).floor() as i32
        );
        let max = (
            ((position.x + radius) / self.bucket_size).floor() as i32,
            ((position.y + radius) / self.bucket_size).floor() as i32,
            ((position.z + radius) / self.bucket_size).floor() as i32,
        );

        for x in min.0..=max.0 {
            for y in min.1..=max.1 {
                for z in min.2..=max.2 {
                    self.buckets.entry((x, y, z)).or_default().push(entity);
                }
            }
        }

        self.entities.insert(entity, (position, radius));
    }

    pub fn get_nearby_entities(&self, position: Vec3, radius: f32) -> Vec<Entity> {
        let min = (
            ((position.x - radius) / self.bucket_size).floor() as i32,
            ((position.y - radius) / self.bucket_size).floor() as i32,
            ((position.z - radius) / self.bucket_size).floor() as i32,
        );
        let max = (
            ((position.x + radius) / self.bucket_size).floor() as i32,
            ((position.y + radius) / self.bucket_size).floor() as i32,
            ((position.z + radius) / self.bucket_size).floor() as i32,
        );

        let mut entities = Vec::new();
        for x in min.0..=max.0 {
            for y in min.1..=max.1 {
                for z in min.2..=max.2 {
                    if let Some(bucket) = self.buckets.get(&(x, y, z)) {
                        entities.extend_from_slice(&bucket[..]);
                    }
                }
            }
        }

        entities
    }

    pub fn get_collision_pairs(&self) -> Vec<(Entity, Entity)> {
        let mut pairs = Vec::new();

        for (entity1, (position1, radius1)) in self.entities.iter() {
            let nearby_entities = self.get_nearby_entities(*position1, *radius1);
            for entity2 in nearby_entities {
                if entity1 < &entity2 {
                    if let Some((position2, radius2)) = self.entities.get(&entity2) {
                        if position1.distance_squared(*position2) <= (*radius1 + *radius2).powi(2) {
                            pairs.push((*entity1, entity2));
                        }
                    }
                }
            }
        }

        pairs
    }

    pub fn clear(&mut self) {
        self.buckets.clear();
        self.entities.clear();
    }
}
