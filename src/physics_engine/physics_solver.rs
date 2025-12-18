// physics_engine/physics_solver.rs

use std::collections::HashMap;

use crate::physics_engine::ContactManifoldsResource;
use crate::physics_engine::physics_narrowphase::CollisionCache;
use bevy::prelude::*;

use crate::physics_components::*;

#[derive(Resource, Default)]
pub struct ContactImpulseCache {
    pub map: HashMap<(Entity, Entity, ContactId), CachedPointImpulse>,
}

#[derive(Clone, Copy, Default)]
pub struct CachedPointImpulse {
    pub normal_acc: f32,
    pub tangent_acc: f32,
    pub tangent_dir: Vec3,
}

pub fn solve_contacts(
    config: Res<PhysicsConfig>,
    fixed_time: Res<Time<Fixed>>,
    manifolds: Res<ContactManifoldsResource>,
    mut impulse_cache: ResMut<ContactImpulseCache>,
    mut bodies: Query<(&Transform, &mut RigidBody)>,
) {
    let dt: f32 = fixed_time.delta_secs().min(config.fixed_dt);
    if dt <= 0.0 {
        return;
    }

    for manifold in manifolds.manifolds.iter() {
        let Ok([(a_tr, mut a_rb), (b_tr, mut b_rb)]) =
            bodies.get_many_mut([manifold.a, manifold.b])
        else {
            continue;
        };

        let a_dynamic: bool = a_rb.body_type == BodyType::Dynamic;
        let b_dynamic: bool = b_rb.body_type == BodyType::Dynamic;

        let a_kinematic: bool = a_rb.body_type == BodyType::Kinematic;
        let b_kinematic: bool = b_rb.body_type == BodyType::Kinematic;

        let a_active: bool = a_dynamic || a_kinematic;
        let b_active: bool = b_dynamic || b_kinematic;

        let a_solve: bool = a_active
            && a_rb.flags.contains(BodyFlags::ENABLE_SOLVER)
            && a_rb.flags.contains(BodyFlags::ENABLE_COLLISIONS);
        let b_solve: bool = b_active
            && b_rb.flags.contains(BodyFlags::ENABLE_SOLVER)
            && b_rb.flags.contains(BodyFlags::ENABLE_COLLISIONS);

        if !a_solve && !b_solve {
            continue;
        }

        if config.sleep.enable && a_rb.is_sleeping && b_rb.is_sleeping {
            continue;
        }

        let inv_mass_a: f32 = if a_dynamic { a_rb.inv_mass } else { 0.0 };
        let inv_mass_b: f32 = if b_dynamic { b_rb.inv_mass } else { 0.0 };

        let inv_inertia_a: Mat3 = if a_dynamic {
            inv_inertia_world(a_tr.rotation, a_rb.inv_inertia_local)
        } else {
            Mat3::ZERO
        };
        let inv_inertia_b: Mat3 = if b_dynamic {
            inv_inertia_world(b_tr.rotation, b_rb.inv_inertia_local)
        } else {
            Mat3::ZERO
        };

        let com_a: Vec3 = world_center_of_mass(a_tr, &a_rb);
        let com_b: Vec3 = world_center_of_mass(b_tr, &b_rb);

        let normal: Vec3 = manifold.normal_world;
        if normal.length_squared() <= 1e-12 {
            continue;
        }

        let key_pair: (Entity, Entity) = pair_key(manifold.a, manifold.b);

        let wake_if_needed = |a_rb: &mut RigidBody, b_rb: &mut RigidBody, changed: bool| {
            if !changed || !config.sleep.enable {
                return;
            }
            if a_rb.is_sleeping {
                a_rb.is_sleeping = false;
                a_rb.sleep_frames = 0;
            }
            if b_rb.is_sleeping {
                b_rb.is_sleeping = false;
                b_rb.sleep_frames = 0;
            }
        };

        if config.warm_starting && manifold.warm_starting {
            for point in manifold.points.iter() {
                let Some(cached) = impulse_cache
                    .map
                    .get(&(key_pair.0, key_pair.1, point.id))
                    .copied()
                else {
                    continue;
                };

                if cached.normal_acc == 0.0 && cached.tangent_acc == 0.0 {
                    continue;
                }

                let p: Vec3 = point.position_world;
                let ra: Vec3 = p - com_a;
                let rb: Vec3 = p - com_b;

                let va: Vec3 = a_rb.linear_velocity + a_rb.angular_velocity.cross(ra);
                let vb: Vec3 = b_rb.linear_velocity + b_rb.angular_velocity.cross(rb);
                let rv: Vec3 = vb - va;

                let t_dir: Vec3 =
                    tangent_dir_from_relative_velocity(normal, rv, cached.tangent_dir);
                let impulse_ws: Vec3 = normal * cached.normal_acc + t_dir * cached.tangent_acc;

                if a_dynamic && a_rb.flags.contains(BodyFlags::ENABLE_TRANSLATION) {
                    a_rb.linear_velocity -= impulse_ws * inv_mass_a;
                }
                if b_dynamic && b_rb.flags.contains(BodyFlags::ENABLE_TRANSLATION) {
                    b_rb.linear_velocity += impulse_ws * inv_mass_b;
                }

                if a_dynamic && a_rb.flags.contains(BodyFlags::ENABLE_ROTATION) {
                    a_rb.angular_velocity += inv_inertia_a * ra.cross(-impulse_ws);
                }
                if b_dynamic && b_rb.flags.contains(BodyFlags::ENABLE_ROTATION) {
                    b_rb.angular_velocity += inv_inertia_b * rb.cross(impulse_ws);
                }
            }
        }

        let iterations: u32 = config.solver_iterations.max(1);

        for _ in 0..iterations {
            for point in manifold.points.iter() {
                let p: Vec3 = point.position_world;

                let ra: Vec3 = p - com_a;
                let rb: Vec3 = p - com_b;

                let va: Vec3 = a_rb.linear_velocity + a_rb.angular_velocity.cross(ra);
                let vb: Vec3 = b_rb.linear_velocity + b_rb.angular_velocity.cross(rb);
                let rv: Vec3 = vb - va;

                let vel_along_normal: f32 = rv.dot(normal);

                let ra_cn: Vec3 = ra.cross(normal);
                let rb_cn: Vec3 = rb.cross(normal);

                let angular_term_n: f32 =
                    (inv_inertia_a * ra_cn).dot(ra_cn) + (inv_inertia_b * rb_cn).dot(rb_cn);

                let denom_n: f32 = inv_mass_a + inv_mass_b + angular_term_n;
                if denom_n <= 0.0 {
                    continue;
                }

                let penetration: f32 = point.penetration;

                let mut bias: f32 = 0.0;
                if penetration > config.allowed_penetration {
                    let error: f32 = penetration - config.allowed_penetration;
                    bias = (config.baumgarte / dt) * error;
                }

                let restitution: f32 =
                    if vel_along_normal < -config.position_correction.restitution_threshold {
                        manifold.restitution
                    } else {
                        0.0
                    };

                if vel_along_normal > 0.0 && penetration <= config.allowed_penetration {
                    continue;
                }

                let cache_key: (Entity, Entity, ContactId) = (key_pair.0, key_pair.1, point.id);
                let mut cached: CachedPointImpulse = impulse_cache
                    .map
                    .get(&cache_key)
                    .copied()
                    .unwrap_or_default();

                let jn_raw: f32 = (-(1.0 + restitution) * vel_along_normal + bias) / denom_n;

                let normal_old: f32 = cached.normal_acc;
                cached.normal_acc = (cached.normal_acc + jn_raw).max(0.0);
                let jn_delta: f32 = cached.normal_acc - normal_old;

                if jn_delta != 0.0 {
                    wake_if_needed(&mut a_rb, &mut b_rb, true);

                    let impulse_n: Vec3 = normal * jn_delta;

                    if a_dynamic && a_rb.flags.contains(BodyFlags::ENABLE_TRANSLATION) {
                        a_rb.linear_velocity -= impulse_n * inv_mass_a;
                    }
                    if b_dynamic && b_rb.flags.contains(BodyFlags::ENABLE_TRANSLATION) {
                        b_rb.linear_velocity += impulse_n * inv_mass_b;
                    }

                    if a_dynamic && a_rb.flags.contains(BodyFlags::ENABLE_ROTATION) {
                        a_rb.angular_velocity += inv_inertia_a * ra.cross(-impulse_n);
                    }
                    if b_dynamic && b_rb.flags.contains(BodyFlags::ENABLE_ROTATION) {
                        b_rb.angular_velocity += inv_inertia_b * rb.cross(impulse_n);
                    }
                }

                let va2: Vec3 = a_rb.linear_velocity + a_rb.angular_velocity.cross(ra);
                let vb2: Vec3 = b_rb.linear_velocity + b_rb.angular_velocity.cross(rb);
                let rv2: Vec3 = vb2 - va2;

                let t_dir: Vec3 =
                    tangent_dir_from_relative_velocity(normal, rv2, cached.tangent_dir);
                cached.tangent_dir = t_dir;

                let vt: f32 = rv2.dot(t_dir);

                let ra_ct: Vec3 = ra.cross(t_dir);
                let rb_ct: Vec3 = rb.cross(t_dir);

                let angular_term_t: f32 =
                    (inv_inertia_a * ra_ct).dot(ra_ct) + (inv_inertia_b * rb_ct).dot(rb_ct);

                let denom_t: f32 = inv_mass_a + inv_mass_b + angular_term_t;
                if denom_t > 0.0 {
                    let jt_raw: f32 = -vt / denom_t;

                    let max_friction: f32 = manifold.friction * cached.normal_acc;

                    let tangent_old: f32 = cached.tangent_acc;
                    cached.tangent_acc =
                        (cached.tangent_acc + jt_raw).clamp(-max_friction, max_friction);
                    let jt_delta: f32 = cached.tangent_acc - tangent_old;

                    if jt_delta != 0.0 {
                        wake_if_needed(&mut a_rb, &mut b_rb, true);

                        let impulse_t: Vec3 = t_dir * jt_delta;

                        if a_dynamic && a_rb.flags.contains(BodyFlags::ENABLE_TRANSLATION) {
                            a_rb.linear_velocity -= impulse_t * inv_mass_a;
                        }
                        if b_dynamic && b_rb.flags.contains(BodyFlags::ENABLE_TRANSLATION) {
                            b_rb.linear_velocity += impulse_t * inv_mass_b;
                        }

                        if a_dynamic && a_rb.flags.contains(BodyFlags::ENABLE_ROTATION) {
                            a_rb.angular_velocity += inv_inertia_a * ra.cross(-impulse_t);
                        }
                        if b_dynamic && b_rb.flags.contains(BodyFlags::ENABLE_ROTATION) {
                            b_rb.angular_velocity += inv_inertia_b * rb.cross(impulse_t);
                        }
                    }
                }

                impulse_cache.map.insert(cache_key, cached);
            }
        }
    }
}

fn tangent_dir_from_relative_velocity(normal: Vec3, rv: Vec3, fallback: Vec3) -> Vec3 {
    let vt: Vec3 = rv - normal * rv.dot(normal);
    let vt2: f32 = vt.length_squared();
    if vt2 > 1e-12 {
        vt / vt2.sqrt()
    } else if fallback.length_squared() > 1e-12 {
        fallback.normalize()
    } else {
        tangent_from_normal(normal)
    }
}

pub fn solve_positions(
    config: Res<PhysicsConfig>,
    manifolds: Res<ContactManifoldsResource>,
    mut bodies: Query<(&mut Transform, &RigidBody)>,
) {
    let iterations: u32 = config.position_iterations.max(1);

    for _ in 0..iterations {
        for manifold in manifolds.manifolds.iter() {
            let Ok([(mut a_tr, a_rb), (mut b_tr, b_rb)]) =
                bodies.get_many_mut([manifold.a, manifold.b])
            else {
                continue;
            };

            let a_dynamic: bool = a_rb.body_type == BodyType::Dynamic;
            let b_dynamic: bool = b_rb.body_type == BodyType::Dynamic;

            let a_kinematic: bool = a_rb.body_type == BodyType::Kinematic;
            let b_kinematic: bool = b_rb.body_type == BodyType::Kinematic;

            let a_active: bool = a_dynamic || a_kinematic;
            let b_active: bool = b_dynamic || b_kinematic;

            let a_solve: bool = a_active
                && a_rb.flags.contains(BodyFlags::ENABLE_SOLVER)
                && a_rb.flags.contains(BodyFlags::ENABLE_COLLISIONS);
            let b_solve: bool = b_active
                && b_rb.flags.contains(BodyFlags::ENABLE_SOLVER)
                && b_rb.flags.contains(BodyFlags::ENABLE_COLLISIONS);

            if !a_solve && !b_solve {
                continue;
            }

            if config.sleep.enable && a_rb.is_sleeping && b_rb.is_sleeping {
                continue;
            }

            let inv_mass_a: f32 = if a_dynamic { a_rb.inv_mass } else { 0.0 };
            let inv_mass_b: f32 = if b_dynamic { b_rb.inv_mass } else { 0.0 };
            let inv_mass_sum: f32 = inv_mass_a + inv_mass_b;
            if inv_mass_sum <= 0.0 {
                continue;
            }

            let mut max_pen: f32 = 0.0;
            for p in manifold.points.iter() {
                if p.penetration > max_pen {
                    max_pen = p.penetration;
                }
            }

            let penetration: f32 = (max_pen - config.allowed_penetration).max(0.0);
            if penetration <= 0.0 {
                continue;
            }

            let correction_mag: f32 = (penetration * config.position_correction.fraction)
                .min(config.position_correction.max);
            if correction_mag <= 0.0 {
                continue;
            }

            let correction: Vec3 = manifold.normal_world * correction_mag;

            if a_dynamic && a_rb.flags.contains(BodyFlags::ENABLE_TRANSLATION) {
                a_tr.translation -= correction * (inv_mass_a / inv_mass_sum);
            }
            if b_dynamic && b_rb.flags.contains(BodyFlags::ENABLE_TRANSLATION) {
                b_tr.translation += correction * (inv_mass_b / inv_mass_sum);
            }
        }
    }
}

fn tangent_from_normal(n: Vec3) -> Vec3 {
    let a: Vec3 = if n.z.abs() < 0.999 { Vec3::Z } else { Vec3::Y };
    let t: Vec3 = n.cross(a);
    if t.length_squared() > 1e-12 {
        t.normalize()
    } else {
        Vec3::X
    }
}

fn world_center_of_mass(transform: &Transform, body: &RigidBody) -> Vec3 {
    let local: Vec3 = body.center_of_mass_local * transform.scale;
    transform.translation + transform.rotation * local
}

fn inv_inertia_world(rot: Quat, inv_inertia_local: Mat3) -> Mat3 {
    let r: Mat3 = Mat3::from_quat(rot);
    r * inv_inertia_local * r.transpose()
}

fn pair_key(a: Entity, b: Entity) -> (Entity, Entity) {
    if a.index() < b.index() {
        (a, b)
    } else {
        (b, a)
    }
}

pub fn prune_impulse_cache(
    mut impulse_cache: ResMut<ContactImpulseCache>,
    collision_cache: Res<CollisionCache>,
) {
    impulse_cache
        .map
        .retain(|(a, b, _), _| collision_cache.current.contains(&(*a, *b)));
}
