use bevy::prelude::*;

use crate::physics::components::{BodyType, Collider, ColliderShape, RigidBody};

#[derive(Component, Debug, Clone, Copy)]
pub struct ColliderGizmoDebug {
    pub max_speed: f32,
}

impl Default for ColliderGizmoDebug {
    fn default() -> Self {
        ColliderGizmoDebug { max_speed: 4.0 }
    }
}

pub fn attach_collider_gizmos(
    mut commands: Commands,
    mut gizmo_assets: ResMut<Assets<GizmoAsset>>,
    query: Query<Entity, (Added<Collider>, Without<Gizmo>)>,
) {
    for e in query.iter() {
        let gizmo: GizmoAsset = GizmoAsset::new();
        let handle: Handle<GizmoAsset> = gizmo_assets.add(gizmo);

        info!("attach_collider_gizmos {:?}", e);

        commands.entity(e).insert((
            Gizmo {
                handle,
                line_config: GizmoLineConfig {
                    width: 2.0,
                    ..default()
                },
                ..default()
            },
            ColliderGizmoDebug::default(),
        ));
    }
}

pub fn update_collider_gizmos(
    mut gizmo_assets: ResMut<Assets<GizmoAsset>>,
    mut query: Query<(
        &Transform,
        &Collider,
        Option<&RigidBody>,
        &ColliderGizmoDebug,
        &mut Gizmo,
    )>,
) {
    for (entity_tr, collider, rb_opt, debug, mut gizmo_comp) in query.iter_mut() {
        let (_combined_translation, _combined_rotation, combined_scale): (Vec3, Quat, Vec3) =
            collider_world_transform(entity_tr, collider);

        let (color, line_width) = choose_color(rb_opt, debug.max_speed);

        let mut gizmo: GizmoAsset = GizmoAsset::new();

        match collider.shape {
            ColliderShape::Sphere { radius } => {
                let max_s: f32 = combined_scale
                    .x
                    .abs()
                    .max(combined_scale.y.abs())
                    .max(combined_scale.z.abs());
                let r: f32 = radius * max_s;

                gizmo.sphere(Isometry3d::IDENTITY, r, color).resolution(120);
            }
            ColliderShape::Cuboid { half_extents } => {
                gizmo.primitive_3d(
                    &Cuboid::from_size(half_extents * 2.0),
                    Isometry3d::IDENTITY,
                    color,
                );
            }
        }

        gizmo_comp.handle = gizmo_assets.add(gizmo);
        gizmo_comp.line_config.width = line_width;
    }
}

fn choose_color(rb_opt: Option<&RigidBody>, max_speed: f32) -> (Color, f32) {
    let Some(rb) = rb_opt else {
        return (Color::srgb(0.75, 0.75, 0.75), 2.0);
    };

    match rb.body_type {
        BodyType::Static => (Color::srgb(0.25, 0.25, 0.25), 1.5),
        BodyType::Kinematic => (Color::srgb(1.0, 0.4, 1.0), 3.0),
        BodyType::Dynamic => {
            if rb.is_sleeping {
                return (Color::srgb(0.2, 0.35, 1.0), 2.0);
            }

            let lin: f32 = rb.linear_velocity.length();
            let ang: f32 = rb.angular_velocity.length();
            let s: f32 = lin + ang;

            let denom: f32 = max_speed.max(1e-6);
            let t: f32 = (s / denom).clamp(0.0, 1.0);

            (Color::srgb(t, 1.0 - t, 0.0), 1.0 + s)
        }
    }
}

fn collider_world_transform(entity_tr: &Transform, collider: &Collider) -> (Vec3, Quat, Vec3) {
    let combined_rotation: Quat = entity_tr.rotation * collider.offset.rotation;
    let combined_scale: Vec3 = entity_tr.scale * collider.offset.scale;
    let local_offset_scaled: Vec3 = collider.offset.translation * entity_tr.scale;
    let combined_translation: Vec3 =
        entity_tr.translation + entity_tr.rotation * local_offset_scaled;
    (combined_translation, combined_rotation, combined_scale)
}
