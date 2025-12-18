use bevy::input::mouse::MouseMotion;
use bevy::prelude::*;

use crate::physics::CameraCollisionBoom;
use crate::physics::components::*;
// use crate::physics_engine::{RayCastHit, spherecast};

#[derive(Resource, Debug, Clone, Copy, PartialEq, Eq)]
pub enum CameraMode {
    Follow,
    Free,
}

impl Default for CameraMode {
    fn default() -> Self {
        CameraMode::Follow
    }
}

#[derive(Component, Debug)]
pub struct Player;

#[derive(Component, Debug)]
pub struct PlayerController {
    pub move_speed: f32,
    pub air_control: f32,
}

#[derive(Component, Debug, Default)]
pub struct PlayerInput {
    pub move_axis: Vec2,
}

#[allow(unused)]
#[derive(Component, Debug, Clone)]
pub struct ThirdPersonCamera {
    pub target: Entity,
    pub yaw: f32,
    pub pitch: f32,
    pub distance: f32,
    pub height: f32,
    pub collision_radius: f32,
    pub min_distance: f32,
}

#[derive(Component, Debug)]
pub struct FreeCamera {
    pub move_speed: f32,
}

pub fn toggle_camera_mode(
    mut commands: Commands,
    mut mode: ResMut<CameraMode>,
    keys: Res<ButtonInput<KeyCode>>,
    q_player: Query<Entity, With<Player>>,
    q_cam_follow: Query<Entity, With<ThirdPersonCamera>>,
    q_cam_free: Query<Entity, With<FreeCamera>>,
) {
    if !keys.just_pressed(KeyCode::Tab) {
        return;
    }

    let player_entity: Entity = match q_player.single() {
        Ok(e) => e,
        Err(_) => return,
    };

    match *mode {
        CameraMode::Follow => {
            let cam_entity: Entity = match q_cam_follow.single() {
                Ok(e) => e,
                Err(_) => return,
            };

            commands.entity(cam_entity).remove::<ThirdPersonCamera>();
            commands.entity(cam_entity).remove::<CameraCollisionBoom>();
            commands
                .entity(cam_entity)
                .insert(FreeCamera { move_speed: 10.0 });

            *mode = CameraMode::Free;
        }
        CameraMode::Free => {
            let cam_entity: Entity = match q_cam_free.single() {
                Ok(e) => e,
                Err(_) => return,
            };

            commands.entity(cam_entity).remove::<FreeCamera>();

            let cam_cfg: ThirdPersonCamera = ThirdPersonCamera {
                target: player_entity,
                yaw: 0.0,
                pitch: -0.2,
                distance: 7.5,
                height: 1.8,
                collision_radius: 0.25,
                min_distance: 1.2,
            };

            commands.entity(cam_entity).insert(cam_cfg.clone());

            commands.entity(cam_entity).insert(CameraCollisionBoom {
                target: player_entity,
                local_target_offset: Vec3::new(0.0, cam_cfg.height, 0.0),
                local_camera_offset: Vec3::new(0.0, 0.0, cam_cfg.distance),
                probe_radius: cam_cfg.collision_radius,
                skin: 0.05,
                layers: None,
            });

            *mode = CameraMode::Follow;
        }
    }
}

pub fn third_person_boom_sync(
    mode: Res<CameraMode>,
    q_player: Query<&Transform, With<Player>>,
    mut q_cam: Query<(&ThirdPersonCamera, &mut CameraCollisionBoom)>,
) {
    if *mode != CameraMode::Follow {
        return;
    }

    let player_tr: &Transform = match q_player.single() {
        Ok(v) => v,
        Err(_) => return,
    };

    let (cfg, mut boom): (&ThirdPersonCamera, Mut<CameraCollisionBoom>) = match q_cam.single_mut() {
        Ok(v) => v,
        Err(_) => return,
    };

    // boom.target = boom.target;
    boom.local_target_offset = Vec3::new(0.0, cfg.height, 0.0);

    let yaw_q: Quat = Quat::from_axis_angle(Vec3::Y, cfg.yaw);
    let pitch_q: Quat = Quat::from_axis_angle(Vec3::X, cfg.pitch);
    let aim_q: Quat = yaw_q * pitch_q;

    let dir: Vec3 = (aim_q * Vec3::Z).normalize_or_zero();
    boom.local_camera_offset = dir * cfg.distance;

    let _ = player_tr;
}

pub fn player_input_update(
    mode: Res<CameraMode>,
    keys: Res<ButtonInput<KeyCode>>,
    mut q_input: Query<&mut PlayerInput, With<Player>>,
) {
    if *mode != CameraMode::Follow {
        return;
    }

    let mut input: Mut<PlayerInput> = match q_input.single_mut() {
        Ok(v) => v,
        Err(_) => return,
    };

    let mut x: f32 = 0.0;
    let mut y: f32 = 0.0;

    if keys.pressed(KeyCode::KeyA) {
        x -= 1.0;
    }
    if keys.pressed(KeyCode::KeyD) {
        x += 1.0;
    }
    if keys.pressed(KeyCode::KeyW) {
        y += 1.0;
    }
    if keys.pressed(KeyCode::KeyS) {
        y -= 1.0;
    }

    let v: Vec2 = Vec2::new(x, y);
    input.move_axis = if v.length_squared() > 1e-6 {
        v.normalize()
    } else {
        Vec2::ZERO
    };
}

pub fn player_apply_movement(
    mode: Res<CameraMode>,
    q_cam: Query<(&Transform, &ThirdPersonCamera)>,
    mut q_player: Query<(&PlayerController, &PlayerInput, &mut RigidBody), With<Player>>,
) {
    if *mode != CameraMode::Follow {
        return;
    }

    let (cam_tr, cam_cfg): (&Transform, &ThirdPersonCamera) = match q_cam.single() {
        Ok(v) => v,
        Err(_) => return,
    };

    let (ctrl, input, mut rb): (&PlayerController, &PlayerInput, Mut<RigidBody>) =
        match q_player.single_mut() {
            Ok(v) => v,
            Err(_) => return,
        };

    let yaw_rot: Quat = Quat::from_axis_angle(Vec3::Y, cam_cfg.yaw);

    let forward: Vec3 = yaw_rot * (-Vec3::Z);
    let right: Vec3 = yaw_rot * Vec3::X;

    let desired: Vec3 = right * input.move_axis.x + forward * input.move_axis.y;
    let desired2: f32 = desired.length_squared();

    let target_vel: Vec3 = if desired2 > 1e-6 {
        desired.normalize() * ctrl.move_speed
    } else {
        Vec3::ZERO
    };

    let current: Vec3 = rb.linear_velocity;

    let k: f32 = ctrl.air_control.max(0.0).min(1.0);
    let new_xz: Vec3 = Vec3::new(
        current.x + (target_vel.x - current.x) * k,
        0.0,
        current.z + (target_vel.z - current.z) * k,
    );

    rb.linear_velocity = Vec3::new(new_xz.x, current.y, new_xz.z);

    let _ = cam_tr;
}

pub fn mouse_look_update(
    mode: Res<CameraMode>,
    mut motion: MessageReader<MouseMotion>,
    keys: Res<ButtonInput<KeyCode>>,
    mut q_follow: Query<&mut ThirdPersonCamera>,
    mut q_free: Query<&mut Transform, With<FreeCamera>>,
) {
    let mut delta: Vec2 = Vec2::ZERO;
    for ev in motion.read() {
        delta += ev.delta;
    }

    if delta.length_squared() <= 1e-6 {
        return;
    }

    let sens: f32 = if keys.pressed(KeyCode::ShiftLeft) {
        0.003
    } else {
        0.002
    };

    match *mode {
        CameraMode::Follow => {
            let mut cam: Mut<ThirdPersonCamera> = match q_follow.single_mut() {
                Ok(v) => v,
                Err(_) => return,
            };

            cam.yaw -= delta.x * sens;
            cam.pitch -= delta.y * sens;
            cam.pitch = cam.pitch.clamp(-1.2, 0.2);
        }
        CameraMode::Free => {
            let mut tr: Mut<Transform> = match q_free.single_mut() {
                Ok(v) => v,
                Err(_) => return,
            };

            let (mut yaw, mut pitch, _roll): (f32, f32, f32) = tr.rotation.to_euler(EulerRot::YXZ);
            yaw -= delta.x * sens;
            pitch -= delta.y * sens;
            pitch = pitch.clamp(-1.55, 1.55);

            tr.rotation = Quat::from_euler(EulerRot::YXZ, yaw, pitch, 0.0);
        }
    }
}

pub fn free_camera_update(
    mode: Res<CameraMode>,
    time: Res<Time>,
    keys: Res<ButtonInput<KeyCode>>,
    mut q_cam: Query<(&mut Transform, &FreeCamera)>,
) {
    if *mode != CameraMode::Free {
        return;
    }

    let (mut tr, cfg): (Mut<Transform>, &FreeCamera) = match q_cam.single_mut() {
        Ok(v) => v,
        Err(_) => return,
    };

    let mut axis: Vec3 = Vec3::ZERO;

    if keys.pressed(KeyCode::KeyW) {
        axis.z -= 1.0;
    }
    if keys.pressed(KeyCode::KeyS) {
        axis.z += 1.0;
    }
    if keys.pressed(KeyCode::KeyA) {
        axis.x -= 1.0;
    }
    if keys.pressed(KeyCode::KeyD) {
        axis.x += 1.0;
    }
    if keys.pressed(KeyCode::Space) {
        axis.y += 1.0;
    }
    if keys.pressed(KeyCode::ControlLeft) {
        axis.y -= 1.0;
    }

    let axis2: f32 = axis.length_squared();
    if axis2 <= 1e-6 {
        return;
    }

    let dt: f32 = time.delta_secs();
    let speed: f32 = if keys.pressed(KeyCode::ShiftLeft) {
        cfg.move_speed * 2.5
    } else {
        cfg.move_speed
    };

    let local: Vec3 = axis.normalize() * speed * dt;
    let rotation = tr.rotation;
    tr.translation += rotation * local;
}
