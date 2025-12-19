use bevy::prelude::*;

use crate::physics::components::*;
use crate::physics::{RayCastHit, raycast};

pub struct DebugOverlayPlugin;

impl Plugin for DebugOverlayPlugin {
    fn build(&self, app: &mut App) {
        app.add_systems(Startup, spawn_debug_overlay)
            .add_systems(Update, debug_overlay_update);
    }
}

#[derive(Component)]
struct DebugTextOverlay;

fn spawn_debug_overlay(mut commands: Commands) {
    commands
        .spawn((
            Node {
                position_type: PositionType::Absolute,
                top: Val::Px(10.0),
                right: Val::Px(10.0),
                padding: UiRect::all(Val::Px(10.0)),
                ..default()
            },
            BackgroundColor(Color::srgba(0.0, 0.0, 0.0, 0.55)),
            BorderRadius::all(Val::Px(10.0)),
        ))
        .with_children(|p| {
            p.spawn((
                DebugTextOverlay,
                Text::new("debug"),
                TextFont {
                    font_size: 16.0,
                    ..default()
                },
                TextColor(Color::WHITE),
            ));
        });
}

fn debug_overlay_update(
    q_camera: Query<(Entity, &GlobalTransform), With<Camera3d>>,
    colliders: Query<(Entity, &Collider, &Transform, Option<&BroadphaseProxy>)>,
    q_body: Query<(&Transform, Option<&RigidBody>)>,
    mut q_text: Query<&mut Text, With<DebugTextOverlay>>,
) {
    let (cam_e, cam_gt): (Entity, &GlobalTransform) = match q_camera.single() {
        Ok(v) => v,
        Err(_) => return,
    };

    let cam_tr: Transform = cam_gt.compute_transform();
    let origin: Vec3 = cam_tr.translation;
    let dir: Vec3 = (cam_tr.rotation * -Vec3::Z).normalize_or_zero();

    let ignore: [Entity; 1] = [cam_e];

    let hit: Option<RayCastHit> = raycast(origin, dir, 200.0, None, &ignore, &colliders);

    let mut out: String = String::new();

    match hit {
        Some(h) => {
            let entity: Entity = h.entity;

            if let Ok((tr, rb_opt)) = q_body.get(entity) {
                let (yaw, pitch, roll): (f32, f32, f32) = tr.rotation.to_euler(EulerRot::YXZ);

                out.push_str(&format!("Looking at: {:?}\n", entity));
                out.push_str(&format!(
                    "Pos:  x={:>6.3}  y={:>6.3}  z={:>6.3}\n",
                    tr.translation.x, tr.translation.y, tr.translation.z
                ));
                out.push_str(&format!(
                    "Rot:  yaw={:>6.3}  pitch={:>6.3}  roll={:>6.3}\n",
                    yaw, pitch, roll
                ));

                match rb_opt {
                    Some(rb) => {
                        out.push_str(format_rigid_body(rb).as_str());
                    }
                    None => {
                        out.push_str("RigidBody: <none>\n");
                    }
                }
            } else {
                out.push_str(&format!("Looking at: {:?}\n", entity));
                out.push_str("Transform/RigidBody: <not found>\n");
            }
        }
        None => {
            out.push_str("Looking at: <none>\n");
        }
    }

    let mut text: Mut<Text> = match q_text.single_mut() {
        Ok(v) => v,
        Err(_) => return,
    };

    *text = Text::new(out);
}

fn format_rigid_body(rb: &RigidBody) -> String {
    let mut s: String = String::new();

    s.push_str("RigidBody {\n");

    s.push_str(&format!("  body_type: {:?}\n", rb.body_type));
    // s.push_str(&format!("  flags: {:?}\n", rb.flags));

    s.push_str("  velocities:\n");
    s.push_str(&format!(
        "    linear:  [{:>6.3}, {:>6.3}, {:>6.3}] {:>6.3}\n",
        rb.linear_velocity.x,
        rb.linear_velocity.y,
        rb.linear_velocity.z,
        rb.linear_velocity.length_squared()
    ));
    s.push_str(&format!(
        "    angular: [{:>6.3}, {:>6.3}, {:>6.3}] {:>6.3}\n",
        rb.angular_velocity.x,
        rb.angular_velocity.y,
        rb.angular_velocity.z,
        rb.angular_velocity.length_squared()
    ));

    s.push_str("  damping:\n");
    s.push_str(&format!("    linear:  {:>7.4}\n", rb.linear_damping));
    s.push_str(&format!("    angular: {:>7.4}\n", rb.angular_damping));

    // s.push_str("  mass:\n");
    // s.push_str(&format!("    mass:     {:>7.4}\n", rb.mass));
    // s.push_str(&format!("    inv_mass: {:.6}\n", rb.inv_mass));

    // s.push_str("  inertia_local:\n");
    // s.push_str(&format!(
    //     "    [{:>6.3}, {:>6.3}, {:>6.3}]\n",
    //     rb.inertia_local.x_axis.x, rb.inertia_local.x_axis.y, rb.inertia_local.x_axis.z
    // ));
    // s.push_str(&format!(
    //     "    [{:>6.3}, {:>6.3}, {:>6.3}]\n",
    //     rb.inertia_local.y_axis.x, rb.inertia_local.y_axis.y, rb.inertia_local.y_axis.z
    // ));
    // s.push_str(&format!(
    //     "    [{:>6.3}, {:>6.3}, {:>6.3}]\n",
    //     rb.inertia_local.z_axis.x, rb.inertia_local.z_axis.y, rb.inertia_local.z_axis.z
    // ));

    s.push_str("  center_of_mass_local:\n");
    s.push_str(&format!(
        "    [{:>6.3}, {:>6.3}, {:>6.3}]\n",
        rb.center_of_mass_local.x, rb.center_of_mass_local.y, rb.center_of_mass_local.z
    ));

    s.push_str("  sleep:\n");
    s.push_str(&format!("    is_sleeping: {}\n", rb.is_sleeping));
    s.push_str(&format!("    sleep_frames: {}\n", rb.sleep_frames));

    s.push_str("}\n");

    s
}
