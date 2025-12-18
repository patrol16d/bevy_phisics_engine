// main.rs

use bevy::app::AppExit;
use bevy::prelude::*;

mod fps_overlay;
mod physics_components;
mod physics_engine;
mod player_controller;

use fps_overlay::FpsOverlayPlugin;
use physics_components::*;
use physics_engine::PhysicsPlugin;
use player_controller::*;

use crate::physics_engine::CameraCollisionBoom;

fn main() {
    App::new()
        .add_plugins(DefaultPlugins)
        .add_plugins(PhysicsPlugin)
        .add_plugins(FpsOverlayPlugin::default())
        .init_resource::<CameraMode>()
        .add_systems(Startup, setup)
        .add_systems(
            Update,
            (
                exit_on_esc,
                toggle_camera_mode,
                player_input_update,
                mouse_look_update,
                third_person_boom_sync,
                free_camera_update,
                log_collisions,
                log_triggers,
            ),
        )
        .add_systems(FixedUpdate, player_apply_movement)
        .run();
}

fn setup(
    mut commands: Commands,
    mut config: ResMut<PhysicsConfig>,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    config.fixed_dt = 1.0 / 60.0;
    config.gravity = Vec3::new(0.0, -9.81, 0.0);
    // config.gravity = Vec3::new(0.0, -2.00, 0.0);
    config.solver_iterations = 12;
    config.position_iterations = 4;
    config.allowed_penetration = 0.002;
    config.baumgarte = 0.2;

    spawn_player_and_camera(&mut commands, &mut meshes, &mut materials);

    // light
    commands.spawn((
        DirectionalLight {
            illuminance: 20_000.0,
            shadows_enabled: true,
            ..Default::default()
        },
        Transform::from_xyz(6.0, 10.0, 6.0).looking_at(Vec3::ZERO, Vec3::Y),
    ));

    // floor
    let floor_mesh: Handle<Mesh> = meshes.add(Cuboid::new(40.0, 1.0, 40.0));
    let floor_mat: Handle<StandardMaterial> = materials.add(Color::srgb(0.2, 0.25, 0.22));
    commands.spawn((
        Mesh3d(floor_mesh),
        MeshMaterial3d(floor_mat),
        Transform::from_xyz(0.0, -0.5, 0.0),
        Collider {
            shape: ColliderShape::Cuboid {
                half_extents: Vec3::new(20.0, 0.5, 20.0),
            },
            material: ColliderMaterial {
                friction: 0.9,
                restitution: 0.0,
            },
            is_sensor: false,
            collision_layers: CollisionLayers::default(),
            offset: Transform::IDENTITY,
            contact_skin: 0.002,
        },
        RigidBody {
            body_type: BodyType::Static,
            flags: BodyFlags::ENABLE_COLLISIONS,
            ..Default::default()
        },
    ));

    // static walls
    let size = Vec3::new(10.0, 5.0, 1.0);
    let wall_mesh: Handle<Mesh> = meshes.add(Cuboid::from_size(size));
    let wall_mat: Handle<StandardMaterial> = materials.add(Color::srgb(0.2, 0.25, 0.82));
    commands.spawn((
        Mesh3d(wall_mesh.clone()),
        MeshMaterial3d(wall_mat.clone()),
        Transform::from_xyz(0.0, 2.5, 10.0),
        Collider {
            shape: ColliderShape::Cuboid {
                half_extents: size / 2.0,
            },
            material: ColliderMaterial {
                friction: 0.9,
                restitution: 0.0,
            },
            is_sensor: false,
            collision_layers: CollisionLayers::default(),
            offset: Transform::IDENTITY,
            contact_skin: 0.002,
        },
        RigidBody {
            body_type: BodyType::Static,
            flags: BodyFlags::ENABLE_COLLISIONS,
            ..Default::default()
        },
    ));
    commands.spawn((
        Mesh3d(wall_mesh.clone()),
        MeshMaterial3d(wall_mat.clone()),
        Transform::from_xyz(-4.0, 2.5, 13.0),
        Collider {
            shape: ColliderShape::Cuboid {
                half_extents: size / 2.0,
            },
            material: ColliderMaterial {
                friction: 0.9,
                restitution: 0.0,
            },
            is_sensor: false,
            collision_layers: CollisionLayers::default(),
            offset: Transform::IDENTITY,
            contact_skin: 0.002,
        },
        RigidBody {
            body_type: BodyType::Static,
            flags: BodyFlags::ENABLE_COLLISIONS,
            ..Default::default()
        },
    ));
    commands.spawn((
        Mesh3d(wall_mesh),
        MeshMaterial3d(wall_mat),
        Transform::from_rotation(Quat::from_rotation_y(1.6))
            .with_translation(Vec3::new(-12.0, 2.5, 13.0)),
        Collider {
            shape: ColliderShape::Cuboid {
                half_extents: size / 2.0,
            },
            material: ColliderMaterial {
                friction: 0.9,
                restitution: 0.0,
            },
            is_sensor: false,
            collision_layers: CollisionLayers::default(),
            offset: Transform::IDENTITY,
            contact_skin: 0.002,
        },
        RigidBody {
            body_type: BodyType::Static,
            flags: BodyFlags::ENABLE_COLLISIONS,
            ..Default::default()
        },
    ));

    // wall
    spawn_wall(&mut commands, &mut meshes, &mut materials);

    // ball
    let ball_mesh: Handle<Mesh> = meshes.add(Sphere::new(0.5));
    let ball_mat: Handle<StandardMaterial> = materials.add(Color::srgb(0.9, 0.2, 0.2));
    commands.spawn((
        Mesh3d(ball_mesh),
        MeshMaterial3d(ball_mat),
        Transform::from_xyz(0.0, 6.0, 0.0),
        Collider {
            shape: ColliderShape::Sphere { radius: 0.5 },
            material: ColliderMaterial {
                friction: 0.6,
                restitution: 0.4,
            },
            ..Default::default()
        },
        RigidBody {
            body_type: BodyType::Dynamic,
            flags: BodyFlags::default(),
            // mass: 1.0,
            inv_mass: 1.0,
            linear_velocity: Vec3::new(2.8, 0.0, 0.0),
            angular_velocity: Vec3::new(0.0, 0.0, 4.0),
            linear_damping: 0.02,
            angular_damping: 0.02,
            ..Default::default()
        },
    ));

    // box
    let box_mesh: Handle<Mesh> = meshes.add(Cuboid::new(1.4, 1.4, 1.4));
    let box_mat: Handle<StandardMaterial> = materials.add(Color::srgb(0.2, 0.7, 0.9));
    commands.spawn((
        Mesh3d(box_mesh),
        MeshMaterial3d(box_mat.clone()),
        Transform::from_xyz(0.5, 10.0, -1.0).with_rotation(Quat::from_euler(
            EulerRot::XYZ,
            0.3,
            0.2,
            0.15,
        )),
        Collider {
            shape: ColliderShape::Cuboid {
                half_extents: Vec3::new(0.7, 0.7, 0.7),
            },
            material: ColliderMaterial {
                friction: 0.7,
                restitution: 0.1,
            },
            ..Default::default()
        },
        RigidBody {
            body_type: BodyType::Dynamic,
            flags: BodyFlags::default(),
            // mass: 3.0,
            inv_mass: 1.0 / 3.0,
            linear_velocity: Vec3::new(2.0, -0.5, 0.0),
            angular_velocity: Vec3::new(0.0, 6.0, 0.0),
            linear_damping: 0.02,
            angular_damping: 0.02,
            ..Default::default()
        },
    ));
    let box_mesh: Handle<Mesh> = meshes.add(Cuboid::new(1.2, 0.6, 0.6));
    commands.spawn((
        Mesh3d(box_mesh),
        MeshMaterial3d(box_mat),
        Transform::from_xyz(-2.5, 6.0, 4.0).with_rotation(Quat::from_euler(
            EulerRot::XYZ,
            0.3,
            0.2,
            0.15,
        )),
        Collider {
            shape: ColliderShape::Cuboid {
                half_extents: Vec3::new(0.6, 0.3, 0.3),
            },
            material: ColliderMaterial {
                friction: 0.7,
                restitution: 0.1,
            },
            ..Default::default()
        },
        RigidBody {
            body_type: BodyType::Dynamic,
            flags: BodyFlags::default(),
            // mass: 3.0,
            inv_mass: 1.0 / 3.0,
            linear_velocity: Vec3::new(0.0, 0.0, 0.0),
            angular_velocity: Vec3::new(3.0, -6.0, 0.0),
            linear_damping: 0.02,
            angular_damping: 0.02,
            ..Default::default()
        },
    ));

    spawn_static_bals(&mut commands);
}

fn spawn_static_bals(commands: &mut Commands) {
    commands.spawn((
        Transform::from_xyz(-3.0, 0.6, 1.0),
        Collider {
            shape: ColliderShape::Sphere { radius: 1.0 },
            is_sensor: true,
            collision_layers: CollisionLayers::default(),
            ..Default::default()
        },
        RigidBody {
            body_type: BodyType::Static,
            flags: BodyFlags::ENABLE_COLLISIONS,
            ..Default::default()
        },
    ));

    commands.spawn((
        Transform::from_xyz(-3.0, 0.6, 3.0),
        Collider {
            shape: ColliderShape::Sphere { radius: 0.5 },
            is_sensor: true,
            collision_layers: CollisionLayers::default(),
            ..Default::default()
        },
        RigidBody {
            body_type: BodyType::Static,
            flags: BodyFlags::ENABLE_COLLISIONS,
            ..Default::default()
        },
    ));
    commands.spawn((
        Transform::from_xyz(-2.0, 0.8, 4.0),
        Collider {
            shape: ColliderShape::Sphere { radius: 0.5 },
            is_sensor: true,
            collision_layers: CollisionLayers::default(),
            ..Default::default()
        },
        RigidBody {
            body_type: BodyType::Static,
            flags: BodyFlags::ENABLE_COLLISIONS,
            ..Default::default()
        },
    ));
}

fn spawn_wall(
    commands: &mut Commands,
    meshes: &mut ResMut<Assets<Mesh>>,
    materials: &mut ResMut<Assets<StandardMaterial>>,
) {
    let wall_mesh: Handle<Mesh> = meshes.add(Cuboid::new(1.0, 1.0, 1.0));
    let wall_mat: Handle<StandardMaterial> = materials.add(Color::srgb(0.35, 0.35, 0.45));

    for x in 6..7 {
        for y in 2..4 {
            for z in 1..2 {
                commands.spawn((
                    Mesh3d(wall_mesh.clone()),
                    MeshMaterial3d(wall_mat.clone()),
                    Transform::from_xyz(x as f32, y as f32 - 1.0, z as f32),
                    Collider {
                        shape: ColliderShape::Cuboid {
                            half_extents: Vec3::new(0.5, 0.5, 0.5),
                        },
                        material: ColliderMaterial {
                            friction: 1.8,
                            restitution: 0.0,
                        },
                        ..Default::default()
                    },
                    RigidBody {
                        body_type: BodyType::Dynamic,
                        // flags: !BodyFlags::ENABLE_ROTATION,
                        // mass: 1.0,
                        inv_mass: 1.0 / 1.0,
                        linear_damping: 0.2,
                        angular_damping: 0.2,
                        ..Default::default()
                    },
                ));
            }
        }
    }
}

fn log_collisions(mut reader: MessageReader<CollisionEvent>) {
    for ev in reader.read() {
        if ev.started {
            info!("Collision started: {:?} <-> {:?}", ev.a, ev.b);
        } else {
            info!("Collision ended: {:?} <-> {:?}", ev.a, ev.b);
        }
    }
}

fn log_triggers(mut reader: MessageReader<TriggerEvent>) {
    for ev in reader.read() {
        if ev.started {
            info!(
                "Trigger started: sensor={:?} other={:?}",
                ev.sensor, ev.other
            );
        } else {
            info!("Trigger ended: sensor={:?} other={:?}", ev.sensor, ev.other);
        }
    }
}

fn spawn_player_and_camera(
    commands: &mut Commands,
    meshes: &mut ResMut<Assets<Mesh>>,
    materials: &mut ResMut<Assets<StandardMaterial>>,
) {
    let player_mesh: Handle<Mesh> = meshes.add(Sphere::new(0.5));
    let player_mat: Handle<StandardMaterial> = materials.add(Color::srgb(0.9, 0.85, 0.2));

    let player_entity: Entity = commands
        .spawn((
            Player,
            PlayerController {
                move_speed: 6.0,
                air_control: 0.35,
            },
            PlayerInput::default(),
            Mesh3d(player_mesh),
            MeshMaterial3d(player_mat),
            Transform::from_xyz(0.0, 2.0, 0.0),
            Collider {
                shape: ColliderShape::Sphere { radius: 0.5 },
                material: ColliderMaterial {
                    friction: 1.2,
                    restitution: 0.0,
                },
                ..Default::default()
            },
            RigidBody {
                body_type: BodyType::Dynamic,
                flags: BodyFlags::ENABLE_GRAVITY
                    | BodyFlags::ENABLE_TRANSLATION
                    // | BodyFlags::ENABLE_ROTATION
                    | BodyFlags::ENABLE_COLLISIONS
                    | BodyFlags::ENABLE_SOLVER
                    | BodyFlags::SLEEPING_ALLOWED,
                // mass: 5.0,
                inv_mass: 1.0 / 5.0,
                linear_damping: 0.04,
                angular_damping: 0.08,
                ..Default::default()
            },
        ))
        .id();

    commands.spawn((
        Camera3d::default(),
        Transform::from_xyz(0.0, 4.0, 10.0).looking_at(Vec3::new(0.0, 2.0, 0.0), Vec3::Y),
        ThirdPersonCamera {
            target: player_entity,
            yaw: 0.0,
            pitch: -0.2,
            distance: 7.5,
            height: 1.8,
            collision_radius: 0.25,
            min_distance: 1.2,
        },
        CameraCollisionBoom {
            target: player_entity,
            local_target_offset: Vec3::new(0.0, 1.8, 0.0),
            local_camera_offset: Vec3::new(0.0, 0.0, 7.5),
            probe_radius: 0.25,
            skin: 0.05,
            layers: None,
        },
    ));
}

fn exit_on_esc(keys: Res<ButtonInput<KeyCode>>, mut exit: MessageWriter<AppExit>) {
    if keys.just_pressed(KeyCode::Escape) {
        exit.write(AppExit::Success);
    }
}
