use bevy::ecs::query::QueryFilter;
use bevy::prelude::*;

use crate::physics::components::*;

#[allow(unused)]
#[derive(Debug, Clone, Copy)]
pub struct RayCastHit {
    pub entity: Entity,
    pub point: Vec3,
    pub normal: Vec3,
    pub distance: f32,
}

// pub fn raycast<F: QueryFilter>(
//     origin: Vec3,
//     dir: Vec3,
//     max_dist: f32,
//     layers: Option<CollisionLayers>,
//     ignore: &[Entity],
//     query: &Query<(Entity, &Collider, &Transform, Option<&BroadphaseProxy>), F>,
// ) -> Option<RayCastHit> {
//     spherecast(origin, dir, 0.0, max_dist, layers, ignore, query)
// }

pub fn spherecast<F: QueryFilter>(
    origin: Vec3,
    dir: Vec3,
    radius: f32,
    max_dist: f32,
    layers: Option<CollisionLayers>,
    ignore: &[Entity],
    query: &Query<(Entity, &Collider, &Transform, Option<&BroadphaseProxy>), F>,
) -> Option<RayCastHit> {
    let dir_len2: f32 = dir.length_squared();
    if dir_len2 <= 1e-12 || max_dist <= 0.0 {
        return None;
    }

    let d: Vec3 = dir / dir_len2.sqrt();
    let mut best: Option<RayCastHit> = None;

    for (entity, collider, transform, proxy_opt) in query.iter() {
        if ignore.iter().any(|e: &Entity| *e == entity) {
            continue;
        }

        if let Some(l) = layers {
            if !l.can_collide(collider.collision_layers) {
                continue;
            }
        }

        if let Some(proxy) = proxy_opt {
            let fat: Aabb3 = proxy.aabb_world.expanded(radius);
            if !ray_aabb_intersect(origin, d, max_dist, fat) {
                continue;
            }
        }

        let wt: Transform = collider_world_transform(transform, collider);

        let hit: Option<RayCastHit> = match collider.shape {
            ColliderShape::Sphere { radius: r0 } => {
                let s: f32 = wt.scale.x.abs().max(wt.scale.y.abs()).max(wt.scale.z.abs());
                let r: f32 = r0 * s + radius;
                ray_sphere(origin, d, max_dist, wt.translation, r, entity)
            }
            ColliderShape::Cuboid { half_extents } => {
                let he: Vec3 = Vec3::new(
                    half_extents.x * wt.scale.x.abs() + radius,
                    half_extents.y * wt.scale.y.abs() + radius,
                    half_extents.z * wt.scale.z.abs() + radius,
                );
                ray_obb(origin, d, max_dist, wt.translation, wt.rotation, he, entity)
            }
        };

        if let Some(h) = hit {
            match best {
                None => best = Some(h),
                Some(prev) => {
                    if h.distance < prev.distance {
                        best = Some(h);
                    }
                }
            }
        }
    }

    best
}

fn collider_world_transform(parent: &Transform, collider: &Collider) -> Transform {
    let mut t: Transform = Transform::IDENTITY;
    t.translation =
        parent.translation + parent.rotation * (collider.offset.translation * parent.scale);
    t.rotation = parent.rotation * collider.offset.rotation;
    t.scale = parent.scale * collider.offset.scale;
    t
}

fn ray_sphere(
    origin: Vec3,
    dir: Vec3,
    max_dist: f32,
    center: Vec3,
    radius: f32,
    entity: Entity,
) -> Option<RayCastHit> {
    let m: Vec3 = origin - center;
    let b: f32 = m.dot(dir);
    let c: f32 = m.dot(m) - radius * radius;

    if c > 0.0 && b > 0.0 {
        return None;
    }

    let discr: f32 = b * b - c;
    if discr < 0.0 {
        return None;
    }

    let t: f32 = (-b - discr.sqrt()).max(0.0);
    if t > max_dist {
        return None;
    }

    let p: Vec3 = origin + dir * t;
    let n: Vec3 = (p - center).normalize_or_zero();

    Some(RayCastHit {
        entity,
        point: p,
        normal: n,
        distance: t,
    })
}

fn ray_obb(
    origin: Vec3,
    dir: Vec3,
    max_dist: f32,
    center: Vec3,
    rot: Quat,
    half: Vec3,
    entity: Entity,
) -> Option<RayCastHit> {
    let inv: Quat = rot.conjugate();
    let o: Vec3 = inv * (origin - center);
    let d: Vec3 = inv * dir;

    let mut tmin: f32 = 0.0;
    let mut tmax: f32 = max_dist;
    let mut hit_axis: i32 = -1;
    let mut hit_sign: f32 = 1.0;

    if !slab(
        d.x,
        o.x,
        half.x,
        &mut tmin,
        &mut tmax,
        0,
        &mut hit_axis,
        &mut hit_sign,
    ) {
        return None;
    }
    if !slab(
        d.y,
        o.y,
        half.y,
        &mut tmin,
        &mut tmax,
        1,
        &mut hit_axis,
        &mut hit_sign,
    ) {
        return None;
    }
    if !slab(
        d.z,
        o.z,
        half.z,
        &mut tmin,
        &mut tmax,
        2,
        &mut hit_axis,
        &mut hit_sign,
    ) {
        return None;
    }

    let t: f32 = tmin;
    if t < 0.0 || t > max_dist {
        return None;
    }

    let local_hit: Vec3 = o + d * t;

    let local_n: Vec3 = match hit_axis {
        0 => Vec3::new(hit_sign, 0.0, 0.0),
        1 => Vec3::new(0.0, hit_sign, 0.0),
        2 => Vec3::new(0.0, 0.0, hit_sign),
        _ => {
            let ax: f32 = (half.x - local_hit.x.abs()).abs();
            let ay: f32 = (half.y - local_hit.y.abs()).abs();
            let az: f32 = (half.z - local_hit.z.abs()).abs();
            if ax <= ay && ax <= az {
                Vec3::new(local_hit.x.signum(), 0.0, 0.0)
            } else if ay <= ax && ay <= az {
                Vec3::new(0.0, local_hit.y.signum(), 0.0)
            } else {
                Vec3::new(0.0, 0.0, local_hit.z.signum())
            }
        }
    };

    let p_world: Vec3 = center + rot * local_hit;
    let n_world: Vec3 = (rot * local_n).normalize_or_zero();

    Some(RayCastHit {
        entity,
        point: p_world,
        normal: n_world,
        distance: t,
    })
}

// zamień funkcję slab na tę wersję (bez zbędnego parametru axis)
fn slab(
    d: f32,
    o: f32,
    h: f32,
    tmin: &mut f32,
    tmax: &mut f32,
    axis_index: i32,
    hit_axis: &mut i32,
    hit_sign: &mut f32,
) -> bool {
    if d.abs() < 1e-12 {
        return o >= -h && o <= h;
    }

    let inv: f32 = 1.0 / d;
    let mut t1: f32 = (-h - o) * inv;
    let mut t2: f32 = (h - o) * inv;

    let mut sign: f32 = -1.0;
    if t1 > t2 {
        let tmp: f32 = t1;
        t1 = t2;
        t2 = tmp;
        sign = 1.0;
    }

    if t1 > *tmin {
        *tmin = t1;
        *hit_axis = axis_index;
        *hit_sign = sign;
    }

    if t2 < *tmax {
        *tmax = t2;
    }

    *tmin <= *tmax
}

fn ray_aabb_intersect(origin: Vec3, dir: Vec3, max_dist: f32, aabb: Aabb3) -> bool {
    let mut tmin: f32 = 0.0;
    let mut tmax: f32 = max_dist;

    if !aabb_slab(
        origin.x, dir.x, aabb.min.x, aabb.max.x, &mut tmin, &mut tmax,
    ) {
        return false;
    }
    if !aabb_slab(
        origin.y, dir.y, aabb.min.y, aabb.max.y, &mut tmin, &mut tmax,
    ) {
        return false;
    }
    if !aabb_slab(
        origin.z, dir.z, aabb.min.z, aabb.max.z, &mut tmin, &mut tmax,
    ) {
        return false;
    }

    tmin <= tmax
}

fn aabb_slab(o: f32, d: f32, minv: f32, maxv: f32, tmin: &mut f32, tmax: &mut f32) -> bool {
    if d.abs() < 1e-12 {
        return o >= minv && o <= maxv;
    }

    let inv: f32 = 1.0 / d;
    let mut t1: f32 = (minv - o) * inv;
    let mut t2: f32 = (maxv - o) * inv;

    if t1 > t2 {
        let tmp: f32 = t1;
        t1 = t2;
        t2 = tmp;
    }

    if t1 > *tmin {
        *tmin = t1;
    }
    if t2 < *tmax {
        *tmax = t2;
    }

    *tmin <= *tmax
}
