// physics_engine.rs

use bevy::prelude::*;
use bevy::time::{Fixed, Time};
use std::collections::HashSet;

pub use camera_collision::CameraCollisionBoom;
use components::*;
use narrowphase::{BroadphasePairs, CollisionCache};
#[allow(unused)]
pub use query::{RayCastHit, raycast, spherecast};
use solver::ContactImpulseCache;

mod camera_collision;
mod collider_gizmos;
pub mod components;
mod ik;
mod joints;
mod narrowphase;
mod query;
mod solver;

pub struct PhysicsPlugin;

#[derive(SystemSet, Debug, Clone, Copy, PartialEq, Eq, Hash)]
enum PhysicsSet {
    Sync,
    Broadphase,
    Narrowphase,
    Solve,
    Integrate,
    Events,
}

#[derive(Resource, Default)]
pub struct ContactManifoldsResource {
    pub manifolds: Vec<ContactManifold>,
}

impl Plugin for PhysicsPlugin {
    fn build(&self, app: &mut App) {
        app.init_resource::<PhysicsConfig>()
            .insert_resource(Time::<Fixed>::from_seconds(
                PhysicsConfig::default().fixed_dt as f64,
            ))
            .init_resource::<BroadphasePairs>()
            .init_resource::<ContactManifoldsResource>()
            .init_resource::<CollisionCache>()
            .init_resource::<ContactImpulseCache>()
            .add_message::<CollisionEvent>()
            .add_message::<TriggerEvent>()
            .add_message::<JointBreakEvent>()
            .add_systems(Update, ik::ik_fabrik_update)
            .add_systems(Startup, physics_apply_fixed_timestep)
            .configure_sets(
                FixedUpdate,
                (
                    PhysicsSet::Sync,
                    PhysicsSet::Broadphase,
                    PhysicsSet::Narrowphase,
                    PhysicsSet::Solve,
                    PhysicsSet::Integrate,
                    PhysicsSet::Events,
                )
                    .chain(),
            )
            .add_systems(
                FixedUpdate,
                (
                    collider_gizmos::attach_collider_gizmos.in_set(PhysicsSet::Sync),
                    attach_broadphase_proxy.in_set(PhysicsSet::Sync),
                    sync_broadphase_proxies.in_set(PhysicsSet::Sync),
                    wake_on_transform_change.in_set(PhysicsSet::Sync),
                    apply_kinematic_velocities.in_set(PhysicsSet::Sync),
                    broadphase_build_pairs.in_set(PhysicsSet::Broadphase),
                    narrowphase::narrowphase_build_manifolds.in_set(PhysicsSet::Narrowphase),
                    solver::solve_contacts.in_set(PhysicsSet::Solve),
                    solver::solve_positions.in_set(PhysicsSet::Solve),
                    joints::solve_joints_positions.in_set(PhysicsSet::Solve),
                    joints::solve_joints_velocities.in_set(PhysicsSet::Solve),
                    integrate_transforms.in_set(PhysicsSet::Integrate),
                    emit_contact_events.in_set(PhysicsSet::Events),
                    solver::prune_impulse_cache.in_set(PhysicsSet::Events),
                    joints::handle_joint_breaks.in_set(PhysicsSet::Events),
                    update_previous_transforms.in_set(PhysicsSet::Events),
                    collider_gizmos::update_collider_gizmos.in_set(PhysicsSet::Events),
                    camera_collision::camera_collision_update.in_set(PhysicsSet::Events),
                ),
            );
    }
}

fn physics_apply_fixed_timestep(mut fixed_time: ResMut<Time<Fixed>>, config: Res<PhysicsConfig>) {
    *fixed_time = Time::<Fixed>::from_seconds(config.fixed_dt as f64);
}

fn attach_broadphase_proxy(
    mut commands: Commands,
    query: Query<Entity, (Added<Collider>, Without<BroadphaseProxy>)>,
) {
    for entity in query.iter() {
        commands.entity(entity).insert(BroadphaseProxy::default());
    }
}

fn sync_broadphase_proxies(mut query: Query<(&Collider, &Transform, &mut BroadphaseProxy)>) {
    for (collider, transform, mut proxy) in query.iter_mut() {
        let aabb_world: Aabb3 = compute_aabb_world(collider, transform);
        proxy.aabb_world = aabb_world.expanded(proxy.fat_margin);
    }
}

fn broadphase_build_pairs(
    mut pairs: ResMut<BroadphasePairs>,
    query: Query<(Entity, &Collider, &BroadphaseProxy)>,
) {
    pairs.pairs.clear();

    let mut items: Vec<(Entity, Aabb3, CollisionLayers, bool)> = Vec::new();
    for (entity, collider, proxy) in query.iter() {
        items.push((
            entity,
            proxy.aabb_world,
            collider.collision_layers,
            collider.is_sensor,
        ));
    }

    let len: usize = items.len();
    let mut i: usize = 0;
    while i < len {
        let (a_entity, a_aabb, a_layers, _a_sensor) = items[i];
        let mut j: usize = i + 1;
        while j < len {
            let (b_entity, b_aabb, b_layers, _b_sensor) = items[j];
            if a_layers.can_collide(b_layers) && aabb_overlaps(a_aabb, b_aabb) {
                pairs.pairs.push((a_entity, b_entity));
            }
            j += 1;
        }
        i += 1;
    }
}

fn integrate_transforms(
    config: Res<PhysicsConfig>,
    fixed_time: Res<Time<Fixed>>,
    mut query: Query<(&mut Transform, &mut RigidBody)>,
) {
    let dt: f32 = fixed_time.delta_secs().min(config.fixed_dt);

    for (mut transform, mut body) in query.iter_mut() {
        if body.body_type != BodyType::Dynamic {
            continue;
        }

        if config.sleep.enable
            && body.flags.contains(BodyFlags::SLEEPING_ALLOWED)
            && body.is_sleeping
        {
            continue;
        }

        if body.flags.contains(BodyFlags::ENABLE_GRAVITY) {
            let gravity_scale = body.gravity_scale;
            body.linear_velocity += config.gravity * gravity_scale * dt;
        }

        if body.linear_damping > 0.0 {
            let factor: f32 = (-body.linear_damping * dt).exp();
            body.linear_velocity *= factor;
        }

        if body.angular_damping > 0.0 {
            let factor: f32 = (-body.angular_damping * dt).exp();
            body.angular_velocity *= factor;
        }

        if body.flags.contains(BodyFlags::ENABLE_TRANSLATION) {
            transform.translation += body.linear_velocity * dt;
        }

        if body.flags.contains(BodyFlags::ENABLE_ROTATION) {
            let delta_q: Quat = Quat::from_scaled_axis(body.angular_velocity * dt);
            transform.rotation = (delta_q * transform.rotation).normalize();
        }

        if config.sleep.enable && body.flags.contains(BodyFlags::SLEEPING_ALLOWED) {
            let lin2: f32 = body.linear_velocity.length_squared();
            let ang2: f32 = body.angular_velocity.length_squared();

            let lin_th2: f32 = config.sleep.linear_threshold * config.sleep.linear_threshold;
            let ang_th2: f32 = config.sleep.angular_threshold * config.sleep.angular_threshold;

            if lin2 < lin_th2 && ang2 < ang_th2 {
                body.sleep_frames = body.sleep_frames.saturating_add(1);
                if body.sleep_frames >= config.sleep.frames_required {
                    body.is_sleeping = true;
                    body.linear_velocity = Vec3::ZERO;
                    body.angular_velocity = Vec3::ZERO;
                }
            } else {
                body.sleep_frames = 0;
                body.is_sleeping = false;
            }
        }

        body.prev_translation = transform.translation;
        body.prev_rotation = transform.rotation;
        body.has_prev = true;
    }
}

fn emit_contact_events(
    mut cache: ResMut<CollisionCache>,
    mut collision_writer: MessageWriter<CollisionEvent>,
    mut trigger_writer: MessageWriter<TriggerEvent>,
    query: Query<(Entity, &Collider)>,
) {
    let mut started: Vec<(Entity, Entity)> = Vec::new();
    let mut ended: Vec<(Entity, Entity)> = Vec::new();

    for pair in cache.current.iter() {
        if !cache.previous.contains(pair) {
            started.push(*pair);
        }
    }

    for pair in cache.previous.iter() {
        if !cache.current.contains(pair) {
            ended.push(*pair);
        }
    }

    for (a, b) in started.iter().copied() {
        let is_sensor: bool = query.get(a).map(|(_, c)| c.is_sensor).unwrap_or(false)
            || query.get(b).map(|(_, c)| c.is_sensor).unwrap_or(false);

        if is_sensor {
            trigger_writer.write(TriggerEvent {
                sensor: a,
                other: b,
                started: true,
            });
        } else {
            collision_writer.write(CollisionEvent {
                a,
                b,
                started: true,
            });
        }
    }

    for (a, b) in ended.iter().copied() {
        let is_sensor: bool = query.get(a).map(|(_, c)| c.is_sensor).unwrap_or(false)
            || query.get(b).map(|(_, c)| c.is_sensor).unwrap_or(false);

        if is_sensor {
            trigger_writer.write(TriggerEvent {
                sensor: a,
                other: b,
                started: false,
            });
        } else {
            collision_writer.write(CollisionEvent {
                a,
                b,
                started: false,
            });
        }
    }

    let mut next_cache = HashSet::new();
    for p in cache.current.iter() {
        next_cache.insert(*p);
    }
    cache.previous = next_cache;
}

fn compute_aabb_world(collider: &Collider, transform: &Transform) -> Aabb3 {
    let combined_rotation: Quat = transform.rotation * collider.offset.rotation;
    let combined_scale: Vec3 = transform.scale * collider.offset.scale;
    let local_offset_scaled: Vec3 = collider.offset.translation * transform.scale;
    let center_world: Vec3 = transform.translation + transform.rotation * local_offset_scaled;

    match collider.shape {
        ColliderShape::Sphere { radius } => {
            let max_scale: f32 = combined_scale
                .x
                .abs()
                .max(combined_scale.y.abs())
                .max(combined_scale.z.abs());
            let r: f32 = radius * max_scale;
            Aabb3 {
                min: center_world - Vec3::splat(r),
                max: center_world + Vec3::splat(r),
            }
        }
        ColliderShape::Cuboid { half_extents } => {
            let he_scaled: Vec3 = Vec3::new(
                half_extents.x * combined_scale.x.abs(),
                half_extents.y * combined_scale.y.abs(),
                half_extents.z * combined_scale.z.abs(),
            );

            let rot: Mat3 = Mat3::from_quat(combined_rotation);
            let abs_rot: Mat3 =
                Mat3::from_cols(rot.x_axis.abs(), rot.y_axis.abs(), rot.z_axis.abs());
            let world_extents: Vec3 = abs_rot * he_scaled;

            Aabb3 {
                min: center_world - world_extents,
                max: center_world + world_extents,
            }
        }
    }
}

fn aabb_overlaps(a: Aabb3, b: Aabb3) -> bool {
    a.min.x <= b.max.x
        && a.max.x >= b.min.x
        && a.min.y <= b.max.y
        && a.max.y >= b.min.y
        && a.min.z <= b.max.z
        && a.max.z >= b.min.z
}

// fn sync_kinematic_velocities(
//     config: Res<PhysicsConfig>,
//     fixed_time: Res<Time<Fixed>>,
//     mut query: Query<(&Transform, &mut RigidBody)>,
// ) {
//     let dt: f32 = fixed_time.delta_secs().min(config.fixed_dt);
//     if dt <= 0.0 {
//         return;
//     }
//     for (transform, mut body) in query.iter_mut() {
//         if body.body_type != BodyType::Kinematic {
//             continue;
//         }
//         if !body.has_prev {
//             body.prev_translation = transform.translation;
//             body.prev_rotation = transform.rotation;
//             body.has_prev = true;
//             body.linear_velocity = Vec3::ZERO;
//             body.angular_velocity = Vec3::ZERO;
//             continue;
//         }
//         let dp: Vec3 = transform.translation - body.prev_translation;
//         body.linear_velocity = dp / dt;
//         let dq: Quat = transform.rotation * body.prev_rotation.conjugate();
//         body.angular_velocity = dq.to_scaled_axis() / dt;
//         body.prev_translation = transform.translation;
//         body.prev_rotation = transform.rotation;
//     }
// }

fn wake_on_transform_change(
    config: Res<PhysicsConfig>,
    fixed_time: Res<Time<Fixed>>,
    mut query: Query<(&Transform, &mut RigidBody)>,
) {
    let dt: f32 = fixed_time.delta_secs().min(config.fixed_dt);
    if dt <= 0.0 {
        return;
    }

    let pos_eps2: f32 = 1e-10;
    let ang_eps2: f32 = 1e-12;

    for (transform, mut body) in query.iter_mut() {
        if body.body_type == BodyType::Static {
            continue;
        }

        if !body.has_prev {
            body.prev_translation = transform.translation;
            body.prev_rotation = transform.rotation;
            body.has_prev = true;
            continue;
        }

        let dp: Vec3 = transform.translation - body.prev_translation;
        let dq: Quat = transform.rotation * body.prev_rotation.conjugate();

        let moved_pos: bool = dp.length_squared() > pos_eps2;
        let moved_ang: bool = dq.to_scaled_axis().length_squared() > ang_eps2;

        if moved_pos || moved_ang {
            if config.sleep.enable && body.is_sleeping {
                body.is_sleeping = false;
                body.sleep_frames = 0;
            }

            body.linear_velocity = dp / dt;
            body.angular_velocity = dq.to_scaled_axis() / dt;

            body.prev_translation = transform.translation;
            body.prev_rotation = transform.rotation;
        }
    }
}

fn apply_kinematic_velocities(
    config: Res<PhysicsConfig>,
    fixed_time: Res<Time<Fixed>>,
    mut query: Query<(&Transform, &PreviousTransform, &mut RigidBody)>,
) {
    let dt: f32 = fixed_time.delta_secs().min(config.fixed_dt);
    if dt <= 0.0 {
        return;
    }

    for (tr, prev, mut rb) in query.iter_mut() {
        if rb.body_type != BodyType::Kinematic {
            continue;
        }

        if !rb.flags.contains(BodyFlags::ENABLE_COLLISIONS) {
            continue;
        }

        let dp: Vec3 = tr.translation - prev.0.translation;
        rb.linear_velocity = dp / dt;

        let dq: Quat = tr.rotation * prev.0.rotation.conjugate();
        let (axis, angle): (Vec3, f32) = dq.to_axis_angle();
        if axis.length_squared() > 1e-12 && angle.is_finite() {
            rb.angular_velocity = axis.normalize() * (angle / dt);
        } else {
            rb.angular_velocity = Vec3::ZERO;
        }
    }
}

fn update_previous_transforms(
    mut commands: Commands,
    query_added: Query<(Entity, &Transform), Added<RigidBody>>,
    mut query: Query<(&Transform, &mut PreviousTransform), With<RigidBody>>,
) {
    for (e, tr) in query_added.iter() {
        commands.entity(e).insert(PreviousTransform(*tr));
    }

    for (tr, mut prev) in query.iter_mut() {
        prev.0 = *tr;
    }
}
