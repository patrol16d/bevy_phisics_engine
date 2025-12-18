use bevy::prelude::*;

use crate::physics::components::*;
use crate::physics::query::spherecast;

#[derive(Debug, Component, Clone, Copy)]
pub struct CameraCollisionBoom {
    pub target: Entity,
    pub local_target_offset: Vec3,
    pub local_camera_offset: Vec3,
    pub probe_radius: f32,
    pub skin: f32,
    pub layers: Option<CollisionLayers>,
}

pub fn camera_collision_update(
    mut cameras: Query<(Entity, &mut Transform, &CameraCollisionBoom)>,
    targets: Query<&GlobalTransform>,
    colliders: Query<
        (Entity, &Collider, &Transform, Option<&BroadphaseProxy>),
        Without<CameraCollisionBoom>,
    >,
) {
    for (cam_e, mut cam_tr, boom) in cameras.iter_mut() {
        let Ok(target_gt) = targets.get(boom.target) else {
            continue;
        };

        let target_pos: Vec3 = target_gt.compute_transform().translation + boom.local_target_offset;
        let desired_world: Vec3 = target_pos + boom.local_camera_offset;

        let mut dir: Vec3 = desired_world - target_pos;
        let dist: f32 = dir.length();
        if dist <= 1e-6 {
            cam_tr.translation = desired_world;
            cam_tr.look_at(target_pos, Vec3::Y);
            continue;
        }

        dir /= dist;

        let ignore: [Entity; 2] = [cam_e, boom.target];

        let hit: Option<crate::physics::RayCastHit> = spherecast(
            target_pos,
            dir,
            boom.probe_radius,
            dist,
            boom.layers,
            &ignore,
            &colliders,
        );

        let final_pos: Vec3 = match hit {
            Some(h) => {
                let d: f32 = (h.distance - boom.skin).clamp(0.0, dist);
                target_pos + dir * d
            }
            None => desired_world,
        };

        cam_tr.translation = final_pos;
        cam_tr.look_at(target_pos, Vec3::Y);
    }
}
