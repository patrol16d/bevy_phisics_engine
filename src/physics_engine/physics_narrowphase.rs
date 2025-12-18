// physics_engine/physics_narrowphase.rs

use std::collections::HashSet;

use crate::physics_engine::ContactManifoldsResource;
use bevy::prelude::*;

use crate::physics_components::*;

#[derive(Resource, Default)]
pub struct BroadphasePairs {
    pub pairs: Vec<(Entity, Entity)>,
}

#[derive(Resource, Default)]
pub struct CollisionCache {
    pub current: HashSet<(Entity, Entity)>,
    pub previous: HashSet<(Entity, Entity)>,
}

pub fn narrowphase_build_manifolds(
    mut manifolds_res: ResMut<ContactManifoldsResource>,
    mut cache: ResMut<CollisionCache>,
    pairs: Res<BroadphasePairs>,
    query: Query<(Entity, &Collider, &Transform)>,
) {
    manifolds_res.manifolds.clear();
    cache.current.clear();

    for (a, b) in pairs.pairs.iter().copied() {
        let ordered: (Entity, Entity) = order_pair(a, b);
        cache.current.insert(ordered);

        let Ok([(a_entity, a_col, a_tr), (b_entity, b_col, b_tr)]) = query.get_many([a, b]) else {
            continue;
        };

        let contact_skin: f32 = a_col.contact_skin.max(b_col.contact_skin);

        let mut manifold_opt: Option<ContactManifold> = match (&a_col.shape, &b_col.shape) {
            (ColliderShape::Sphere { radius: ra }, ColliderShape::Sphere { radius: rb }) => {
                sphere_sphere_manifold(
                    a_entity,
                    b_entity,
                    *ra,
                    *rb,
                    a_tr,
                    b_tr,
                    contact_skin,
                    a_col,
                    b_col,
                )
            }
            (ColliderShape::Sphere { radius }, ColliderShape::Cuboid { half_extents }) => {
                sphere_cuboid_manifold(
                    a_entity,
                    b_entity,
                    *radius,
                    *half_extents,
                    a_tr,
                    b_tr,
                    contact_skin,
                    a_col,
                    b_col,
                )
            }
            (ColliderShape::Cuboid { half_extents }, ColliderShape::Sphere { radius }) => {
                let mut m: Option<ContactManifold> = sphere_cuboid_manifold(
                    b_entity,
                    a_entity,
                    *radius,
                    *half_extents,
                    b_tr,
                    a_tr,
                    contact_skin,
                    b_col,
                    a_col,
                );
                if let Some(ref mut man) = m {
                    man.normal_world = -man.normal_world;
                    let tmp: Entity = man.a;
                    man.a = man.b;
                    man.b = tmp;
                    for p in man.points.iter_mut() {
                        p.normal_world = -p.normal_world;
                        let tmpv: Vec3 = p.r_a;
                        p.r_a = p.r_b;
                        p.r_b = tmpv;
                    }
                }
                m
            }
            (
                ColliderShape::Cuboid { half_extents: a_he },
                ColliderShape::Cuboid { half_extents: b_he },
            ) => cuboid_cuboid_manifold(
                a_entity,
                b_entity,
                *a_he,
                *b_he,
                a_tr,
                b_tr,
                contact_skin,
                a_col,
                b_col,
            ),
        };

        if let Some(ref mut m) = manifold_opt {
            m.friction = (a_col.material.friction * b_col.material.friction).sqrt();
            m.restitution = a_col.material.restitution.max(b_col.material.restitution);
            manifolds_res.manifolds.push(m.clone());
        }
    }
}

fn order_pair(a: Entity, b: Entity) -> (Entity, Entity) {
    if a.index() < b.index() {
        (a, b)
    } else {
        (b, a)
    }
}

fn sphere_sphere_manifold(
    a: Entity,
    b: Entity,
    ra: f32,
    rb: f32,
    a_tr: &Transform,
    b_tr: &Transform,
    contact_skin: f32,
    a_col: &Collider,
    b_col: &Collider,
) -> Option<ContactManifold> {
    let aw: Transform = collider_world_transform(a_tr, a_col);
    let bw: Transform = collider_world_transform(b_tr, b_col);

    let pa: Vec3 = aw.translation;
    let pb: Vec3 = bw.translation;

    let sa: f32 = aw.scale.x.abs().max(aw.scale.y.abs()).max(aw.scale.z.abs());
    let sb: f32 = bw.scale.x.abs().max(bw.scale.y.abs()).max(bw.scale.z.abs());

    let r1: f32 = ra * sa;
    let r2: f32 = rb * sb;

    let d: Vec3 = pb - pa;
    let dist2: f32 = d.length_squared();
    let r: f32 = r1 + r2 + contact_skin;
    if dist2 > r * r {
        return None;
    }

    let dist: f32 = dist2.sqrt();
    let normal: Vec3 = if dist > 1e-6 { d / dist } else { Vec3::X };

    let penetration: f32 = (r1 + r2) - dist;
    let contact_point: Vec3 = pa + normal * (r1 - penetration * 0.5);

    let mut manifold: ContactManifold = ContactManifold {
        a,
        b,
        normal_world: normal,
        points: smallvec::SmallVec::new(),
        friction: 0.0,
        restitution: 0.0,
        warm_starting: true,
    };

    manifold.points.push(ContactPoint {
        id: contact_id(&aw, contact_point),
        position_world: contact_point,
        normal_world: normal,
        penetration: penetration.max(0.0),
        r_a: Vec3::ZERO,
        r_b: Vec3::ZERO,
        // normal_impulse_acc: 0.0,
        // tangent_impulse_acc: 0.0,
    });

    Some(manifold)
}

fn sphere_cuboid_manifold(
    sphere_e: Entity,
    cuboid_e: Entity,
    sphere_radius: f32,
    cuboid_he: Vec3,
    sphere_parent: &Transform,
    cuboid_parent: &Transform,
    contact_skin: f32,
    sphere_col: &Collider,
    cuboid_col: &Collider,
) -> Option<ContactManifold> {
    let sw: Transform = collider_world_transform(sphere_parent, sphere_col);
    let cw: Transform = collider_world_transform(cuboid_parent, cuboid_col);

    let sphere_center: Vec3 = sw.translation;

    let s_scale: f32 = sw.scale.x.abs().max(sw.scale.y.abs()).max(sw.scale.z.abs());
    let r: f32 = sphere_radius * s_scale;

    let he_scaled: Vec3 = Vec3::new(
        cuboid_he.x * cw.scale.x.abs(),
        cuboid_he.y * cw.scale.y.abs(),
        cuboid_he.z * cw.scale.z.abs(),
    );

    let inv_rot: Quat = cw.rotation.conjugate();
    let local_p: Vec3 = inv_rot * (sphere_center - cw.translation);

    let clamped_local: Vec3 = Vec3::new(
        local_p.x.clamp(-he_scaled.x, he_scaled.x),
        local_p.y.clamp(-he_scaled.y, he_scaled.y),
        local_p.z.clamp(-he_scaled.z, he_scaled.z),
    );

    let closest_world: Vec3 = cw.translation + cw.rotation * clamped_local;

    let delta: Vec3 = sphere_center - closest_world;
    let dist2: f32 = delta.length_squared();
    let limit: f32 = (r + contact_skin) * (r + contact_skin);
    if dist2 > limit {
        return None;
    }

    let dist: f32 = dist2.sqrt();
    let mut normal: Vec3 = if dist > 1e-6 {
        -(delta / dist)
    } else {
        (cw.translation - sphere_center).normalize_or_zero()
    };
    if normal.length_squared() < 1e-12 {
        normal = Vec3::X;
    }

    let penetration: f32 = (r) - dist;

    let contact_point: Vec3 = closest_world;

    let mut manifold: ContactManifold = ContactManifold {
        a: sphere_e,
        b: cuboid_e,
        normal_world: normal,
        points: smallvec::SmallVec::new(),
        friction: 0.0,
        restitution: 0.0,
        warm_starting: true,
    };

    manifold.points.push(ContactPoint {
        id: contact_id(&cw, contact_point),
        position_world: contact_point,
        normal_world: normal,
        penetration: penetration.max(0.0),
        r_a: Vec3::ZERO,
        r_b: Vec3::ZERO,
        // normal_impulse_acc: 0.0,
        // tangent_impulse_acc: 0.0,
    });

    Some(manifold)
}

fn cuboid_cuboid_manifold(
    a: Entity,
    b: Entity,
    a_he: Vec3,
    b_he: Vec3,
    a_parent: &Transform,
    b_parent: &Transform,
    _contact_skin: f32,
    a_col: &Collider,
    b_col: &Collider,
) -> Option<ContactManifold> {
    let aw: Transform = collider_world_transform(a_parent, a_col);
    let bw: Transform = collider_world_transform(b_parent, b_col);

    let a_center: Vec3 = aw.translation;
    let b_center: Vec3 = bw.translation;

    let a_half: Vec3 = Vec3::new(
        a_he.x * aw.scale.x.abs(),
        a_he.y * aw.scale.y.abs(),
        a_he.z * aw.scale.z.abs(),
    );
    let b_half: Vec3 = Vec3::new(
        b_he.x * bw.scale.x.abs(),
        b_he.y * bw.scale.y.abs(),
        b_he.z * bw.scale.z.abs(),
    );

    let ra: Mat3 = Mat3::from_quat(aw.rotation);
    let rb: Mat3 = Mat3::from_quat(bw.rotation);

    let a_axes: [Vec3; 3] = [ra.x_axis, ra.y_axis, ra.z_axis];
    let b_axes: [Vec3; 3] = [rb.x_axis, rb.y_axis, rb.z_axis];

    let axes: [Vec3; 15] = [
        a_axes[0],
        a_axes[1],
        a_axes[2],
        b_axes[0],
        b_axes[1],
        b_axes[2],
        a_axes[0].cross(b_axes[0]),
        a_axes[0].cross(b_axes[1]),
        a_axes[0].cross(b_axes[2]),
        a_axes[1].cross(b_axes[0]),
        a_axes[1].cross(b_axes[1]),
        a_axes[1].cross(b_axes[2]),
        a_axes[2].cross(b_axes[0]),
        a_axes[2].cross(b_axes[1]),
        a_axes[2].cross(b_axes[2]),
    ];

    let t: Vec3 = b_center - a_center;

    let mut best_axis: Vec3 = Vec3::X;
    let mut best_pen: f32 = f32::INFINITY;
    let mut best_kind: u8 = 0;
    let mut best_face_index: usize = 0;

    let mut k: usize = 0;
    while k < 15 {
        let axis_raw: Vec3 = axes[k];
        let kind: u8 = if k < 3 {
            1
        } else if k < 6 {
            2
        } else {
            3
        };

        if axis_raw.length_squared() < 1e-10 {
            k += 1;
            continue;
        }

        let axis: Vec3 = axis_raw.normalize();

        let a_proj: f32 = obb_project_radius(axis, ra, a_half);
        let b_proj: f32 = obb_project_radius(axis, rb, b_half);
        let dist: f32 = t.dot(axis).abs();

        let overlap: f32 = (a_proj + b_proj) - dist;
        if overlap <= 0.0 {
            return None;
        }

        if overlap < best_pen {
            best_pen = overlap;
            best_axis = axis;
            best_kind = kind;

            if kind == 1 {
                best_face_index = k;
            } else if kind == 2 {
                best_face_index = k - 3;
            } else {
                best_face_index = 0;
            }
        }

        k += 1;
    }

    let dir: f32 = t.dot(best_axis);
    let normal_world: Vec3 = if dir >= 0.0 { best_axis } else { -best_axis };

    let mut manifold: ContactManifold = ContactManifold {
        a,
        b,
        normal_world,
        points: smallvec::SmallVec::new(),
        friction: 0.0,
        restitution: 0.0,
        warm_starting: true,
    };

    let (ref_center, ref_rot, ref_half, inc_center, inc_rot, inc_half, flip): (
        Vec3,
        Mat3,
        Vec3,
        Vec3,
        Mat3,
        Vec3,
        bool,
    ) = if best_kind == 1 {
        (a_center, ra, a_half, b_center, rb, b_half, false)
    } else {
        (b_center, rb, b_half, a_center, ra, a_half, true)
    };

    let ref_world: Transform = Transform {
        translation: ref_center,
        rotation: Quat::from_mat3(&ref_rot),
        scale: Vec3::ONE,
    };

    if best_kind == 3 {
        let pa: Vec3 = a_center + normal_world * obb_support_distance(normal_world, ra, a_half);
        let pb: Vec3 = b_center - normal_world * obb_support_distance(-normal_world, rb, b_half);
        let contact_point: Vec3 = (pa + pb) * 0.5;

        manifold.points.push(ContactPoint {
            id: contact_id(&ref_world, contact_point),
            position_world: contact_point,
            normal_world,
            penetration: best_pen.max(0.0),
            r_a: Vec3::ZERO,
            r_b: Vec3::ZERO,
            // normal_impulse_acc: 0.0,
            // tangent_impulse_acc: 0.0,
        });

        return Some(manifold);
    }

    let ref_axes: [Vec3; 3] = [ref_rot.x_axis, ref_rot.y_axis, ref_rot.z_axis];
    let inc_axes: [Vec3; 3] = [inc_rot.x_axis, inc_rot.y_axis, inc_rot.z_axis];

    let mut ref_face_normal: Vec3 = ref_axes[best_face_index];
    if ref_face_normal.dot(normal_world) < 0.0 {
        ref_face_normal = -ref_face_normal;
    }

    let ref_extent: f32 = match best_face_index {
        0 => ref_half.x,
        1 => ref_half.y,
        _ => ref_half.z,
    };

    let ref_plane_point: Vec3 = ref_center + ref_face_normal * ref_extent;
    let ref_plane_n: Vec3 = ref_face_normal;

    let inc_face_index: usize = incident_face_index(inc_axes, ref_plane_n);
    let mut inc_poly: Vec<Vec3> =
        incident_face_vertices(inc_center, inc_rot, inc_half, inc_face_index, ref_plane_n);

    let (ref_u, ref_v, ref_u_extent, ref_v_extent): (Vec3, Vec3, f32, f32) =
        reference_face_uv(ref_axes, ref_half, best_face_index);

    let side_planes: [(Vec3, Vec3); 4] = [
        (ref_u, ref_plane_point + ref_u * ref_u_extent),
        (-ref_u, ref_plane_point - ref_u * ref_u_extent),
        (ref_v, ref_plane_point + ref_v * ref_v_extent),
        (-ref_v, ref_plane_point - ref_v * ref_v_extent),
    ];

    let mut s: usize = 0;
    while s < 4 {
        let (n, p0) = side_planes[s];
        inc_poly = clip_polygon_against_plane(&inc_poly, n, p0);
        if inc_poly.is_empty() {
            return None;
        }
        s += 1;
    }

    let mut contacts: Vec<(Vec3, f32)> = Vec::new();
    for v in inc_poly.iter().copied() {
        let dist: f32 = (v - ref_plane_point).dot(ref_plane_n);
        let pen: f32 = (-dist).max(0.0);
        if pen > 0.0 {
            let projected: Vec3 = v - ref_plane_n * dist;
            contacts.push((projected, pen.min(best_pen)));
        }
    }

    contacts.sort_by(|a0, b0| b0.1.total_cmp(&a0.1));

    let mut count: usize = 0;
    for (p, pen) in contacts.into_iter() {
        if count >= 4 {
            break;
        }

        manifold.points.push(ContactPoint {
            id: contact_id(&ref_world, p),
            position_world: p,
            normal_world,
            penetration: pen,
            r_a: Vec3::ZERO,
            r_b: Vec3::ZERO,
            // normal_impulse_acc: 0.0,
            // tangent_impulse_acc: 0.0,
        });

        count += 1;
    }

    if manifold.points.is_empty() {
        return None;
    }

    if flip {
        manifold.normal_world = -manifold.normal_world;
    }

    Some(manifold)
}

fn reference_face_uv(axes: [Vec3; 3], half: Vec3, face_index: usize) -> (Vec3, Vec3, f32, f32) {
    match face_index {
        0 => (axes[1], axes[2], half.y, half.z),
        1 => (axes[0], axes[2], half.x, half.z),
        _ => (axes[0], axes[1], half.x, half.y),
    }
}

fn incident_face_index(inc_axes: [Vec3; 3], ref_n: Vec3) -> usize {
    let d0: f32 = inc_axes[0].dot(ref_n).abs();
    let d1: f32 = inc_axes[1].dot(ref_n).abs();
    let d2: f32 = inc_axes[2].dot(ref_n).abs();

    if d0 >= d1 && d0 >= d2 {
        0
    } else if d1 >= d0 && d1 >= d2 {
        1
    } else {
        2
    }
}

fn incident_face_vertices(
    center: Vec3,
    rot: Mat3,
    half: Vec3,
    face_index: usize,
    ref_n: Vec3,
) -> Vec<Vec3> {
    let axes: [Vec3; 3] = [rot.x_axis, rot.y_axis, rot.z_axis];

    let mut n: Vec3 = axes[face_index];
    if n.dot(ref_n) > 0.0 {
        n = -n;
    }

    let (u, v, ue, ve): (Vec3, Vec3, f32, f32) = match face_index {
        0 => (axes[1], axes[2], half.y, half.z),
        1 => (axes[0], axes[2], half.x, half.z),
        _ => (axes[0], axes[1], half.x, half.y),
    };

    let ne: f32 = match face_index {
        0 => half.x,
        1 => half.y,
        _ => half.z,
    };

    let face_center: Vec3 = center + n * ne;

    let p0: Vec3 = face_center + u * ue + v * ve;
    let p1: Vec3 = face_center - u * ue + v * ve;
    let p2: Vec3 = face_center - u * ue - v * ve;
    let p3: Vec3 = face_center + u * ue - v * ve;

    vec![p0, p1, p2, p3]
}

fn clip_polygon_against_plane(poly: &[Vec3], plane_n: Vec3, plane_p: Vec3) -> Vec<Vec3> {
    if poly.is_empty() {
        return Vec::new();
    }

    let mut out: Vec<Vec3> = Vec::new();

    let mut prev: Vec3 = poly[poly.len() - 1];
    let mut prev_dist: f32 = (prev - plane_p).dot(plane_n);

    for curr in poly.iter().copied() {
        let curr_dist: f32 = (curr - plane_p).dot(plane_n);

        let prev_in: bool = prev_dist <= 0.0;
        let curr_in: bool = curr_dist <= 0.0;

        if curr_in && prev_in {
            out.push(curr);
        } else if prev_in && !curr_in {
            let t: f32 = prev_dist / (prev_dist - curr_dist);
            let hit: Vec3 = prev + (curr - prev) * t;
            out.push(hit);
        } else if !prev_in && curr_in {
            let t: f32 = prev_dist / (prev_dist - curr_dist);
            let hit: Vec3 = prev + (curr - prev) * t;
            out.push(hit);
            out.push(curr);
        }

        prev = curr;
        prev_dist = curr_dist;
    }

    out
}

fn obb_project_radius(axis: Vec3, rot: Mat3, half: Vec3) -> f32 {
    (axis.dot(rot.x_axis)).abs() * half.x
        + (axis.dot(rot.y_axis)).abs() * half.y
        + (axis.dot(rot.z_axis)).abs() * half.z
}

fn obb_support_distance(dir: Vec3, rot: Mat3, half: Vec3) -> f32 {
    (dir.dot(rot.x_axis)).abs() * half.x
        + (dir.dot(rot.y_axis)).abs() * half.y
        + (dir.dot(rot.z_axis)).abs() * half.z
}

fn collider_world_transform(parent: &Transform, collider: &Collider) -> Transform {
    let mut t: Transform = Transform::IDENTITY;
    t.translation =
        parent.translation + parent.rotation * (collider.offset.translation * parent.scale);
    t.rotation = parent.rotation * collider.offset.rotation;
    t.scale = parent.scale * collider.offset.scale;
    t
}

fn contact_id(world_from_collider: &Transform, p_world: Vec3) -> ContactId {
    let inv_rot: Quat = world_from_collider.rotation.conjugate();
    let local: Vec3 = inv_rot * (p_world - world_from_collider.translation);

    let sx: f32 = if world_from_collider.scale.x.abs() > 1e-12 {
        world_from_collider.scale.x
    } else {
        1.0
    };
    let sy: f32 = if world_from_collider.scale.y.abs() > 1e-12 {
        world_from_collider.scale.y
    } else {
        1.0
    };
    let sz: f32 = if world_from_collider.scale.z.abs() > 1e-12 {
        world_from_collider.scale.z
    } else {
        1.0
    };

    let unscaled: Vec3 = Vec3::new(local.x / sx, local.y / sy, local.z / sz);

    let q: f32 = 0.01;
    let xi: i32 = (unscaled.x / q).round() as i32;
    let yi: i32 = (unscaled.y / q).round() as i32;
    let zi: i32 = (unscaled.z / q).round() as i32;

    ContactId(hash_i32_3(xi, yi, zi))
}

fn hash_i32_3(x: i32, y: i32, z: i32) -> u64 {
    let mut h: u64 = 0xcbf29ce484222325;
    h = fnv1a_u64(h, x as u32 as u64);
    h = fnv1a_u64(h, y as u32 as u64);
    h = fnv1a_u64(h, z as u32 as u64);
    h
}

fn fnv1a_u64(mut h: u64, v: u64) -> u64 {
    h ^= v;
    h = h.wrapping_mul(0x00000100000001B3);
    h
}
