// physics::components.rs

use bevy::prelude::*;

// Tryby ciała i flagi

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum BodyType {
    Static,
    Kinematic,
    Dynamic,
}

bitflags::bitflags! {
    #[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
    pub struct BodyFlags: u32 {
        const ENABLE_GRAVITY      = 1 << 0;
        const ENABLE_TRANSLATION  = 1 << 1;
        const ENABLE_ROTATION     = 1 << 2;
        const ENABLE_COLLISIONS   = 1 << 3;
        const ENABLE_SOLVER       = 1 << 4;
        const CCD                 = 1 << 5;
        const SLEEPING_ALLOWED    = 1 << 6;
    }
}

impl Default for BodyFlags {
    fn default() -> Self {
        BodyFlags::ENABLE_GRAVITY
            | BodyFlags::ENABLE_TRANSLATION
            | BodyFlags::ENABLE_ROTATION
            | BodyFlags::ENABLE_COLLISIONS
            | BodyFlags::ENABLE_SOLVER
            | BodyFlags::SLEEPING_ALLOWED
    }
}

// RigidBody

#[derive(Debug, Component, Clone, Copy)]
pub struct RigidBody {
    pub body_type: BodyType,
    pub flags: BodyFlags,

    pub linear_velocity: Vec3,
    pub angular_velocity: Vec3,

    pub linear_damping: f32,
    pub angular_damping: f32,

    pub gravity_scale: f32,

    // pub mass: f32,
    pub inv_mass: f32,

    // pub inertia_local: Mat3,
    pub inv_inertia_local: Mat3,

    pub center_of_mass_local: Vec3,

    pub is_sleeping: bool,
    pub sleep_frames: u16,

    pub prev_translation: Vec3,
    pub prev_rotation: Quat,
    pub has_prev: bool,
}

impl Default for RigidBody {
    fn default() -> Self {
        RigidBody {
            body_type: BodyType::Dynamic,
            flags: BodyFlags::default(),
            linear_velocity: Vec3::ZERO,
            angular_velocity: Vec3::ZERO,
            linear_damping: 0.0,
            angular_damping: 0.0,
            gravity_scale: 1.0,
            // mass: 1.0,
            inv_mass: 1.0,
            // inertia_local: Mat3::IDENTITY,
            inv_inertia_local: Mat3::IDENTITY,
            center_of_mass_local: Vec3::ZERO,
            is_sleeping: false,
            sleep_frames: 0,
            prev_translation: Vec3::ZERO,
            prev_rotation: Quat::IDENTITY,
            has_prev: false,
        }
    }
}

// Collider i kształty bazowe

#[derive(Debug, Clone, Copy)]
pub struct ColliderMaterial {
    pub friction: f32,
    pub restitution: f32,
}

impl Default for ColliderMaterial {
    fn default() -> Self {
        ColliderMaterial {
            friction: 0.7,
            restitution: 0.0,
        }
    }
}

#[derive(Debug, Clone)]
pub enum ColliderShape {
    Sphere { radius: f32 },
    Cuboid { half_extents: Vec3 },
    // potem:
    // Capsule { radius: f32, half_height: f32 },
    // ConvexHull { vertices: Vec<Vec3> },
}

#[derive(Debug, Component, Clone)]
pub struct Collider {
    pub shape: ColliderShape,
    pub material: ColliderMaterial,
    pub is_sensor: bool,
    pub collision_layers: CollisionLayers,
    pub offset: Transform,
    pub contact_skin: f32,
}

impl Default for Collider {
    fn default() -> Self {
        Collider {
            shape: ColliderShape::Sphere { radius: 0.5 },
            material: ColliderMaterial::default(),
            is_sensor: false,
            collision_layers: CollisionLayers::default(),
            offset: Transform::IDENTITY,
            contact_skin: 0.002,
        }
    }
}

// Warstwy / maski kolizji

#[derive(Debug, Clone, Copy)]
pub struct CollisionLayers {
    pub belongs_to: u32,
    pub collides_with: u32,
}

impl CollisionLayers {
    // pub fn new(belongs_to: u32, collides_with: u32) -> Self {
    //     CollisionLayers {
    //         belongs_to,
    //         collides_with,
    //     }
    // }

    pub fn can_collide(self, other: CollisionLayers) -> bool {
        (self.belongs_to & other.collides_with) != 0 && (other.belongs_to & self.collides_with) != 0
    }
}

impl Default for CollisionLayers {
    fn default() -> Self {
        CollisionLayers {
            belongs_to: 1,
            collides_with: u32::MAX,
        }
    }
}

// Broadphase (na razie dane pod AABB)

#[derive(Debug, Clone, Copy)]
pub struct Aabb3 {
    pub min: Vec3,
    pub max: Vec3,
}

impl Aabb3 {
    pub fn expanded(self, margin: f32) -> Self {
        Aabb3 {
            min: self.min - Vec3::splat(margin),
            max: self.max + Vec3::splat(margin),
        }
    }
}

#[derive(Debug, Component, Clone, Copy)]
pub struct BroadphaseProxy {
    pub aabb_world: Aabb3,
    pub fat_margin: f32,
}

impl Default for BroadphaseProxy {
    fn default() -> Self {
        BroadphaseProxy {
            aabb_world: Aabb3 {
                min: Vec3::ZERO,
                max: Vec3::ZERO,
            },
            fat_margin: 0.1,
        }
    }
}

// Kontakty, manifoldy i impuls solver

// #[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
// pub struct ColliderKey {
//     pub entity: Entity,
//     pub collider_index: u16,
// }

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct ContactId(pub u64);

#[derive(Debug, Clone, Copy)]
pub struct ContactPoint {
    pub id: ContactId,

    pub position_world: Vec3,
    pub normal_world: Vec3,
    pub penetration: f32,

    pub r_a: Vec3,
    pub r_b: Vec3,
    // pub normal_impulse_acc: f32,
    // pub tangent_impulse_acc: f32,
}

#[derive(Debug, Clone)]
pub struct ContactManifold {
    pub a: Entity,
    pub b: Entity,

    pub normal_world: Vec3,

    pub points: smallvec::SmallVec<[ContactPoint; 4]>,

    pub friction: f32,
    pub restitution: f32,

    pub warm_starting: bool,
}

// Konfiguracja symulacji

#[derive(Debug, Resource, Clone, Copy)]
pub struct PhysicsConfig {
    pub gravity: Vec3,
    pub fixed_dt: f32,

    pub solver_iterations: u32,
    pub position_iterations: u32,

    pub allowed_penetration: f32,
    pub baumgarte: f32,

    pub warm_starting: bool,

    pub sleep: SleepConfig,
    pub position_correction: PositionCorrectionConfig,
}

#[derive(Debug, Clone, Copy)]
pub struct SleepConfig {
    pub enable: bool,
    pub linear_threshold: f32,
    pub angular_threshold: f32,
    pub frames_required: u16,
}

#[derive(Debug, Clone, Copy)]
pub struct PositionCorrectionConfig {
    pub max: f32,
    pub fraction: f32,
    pub restitution_threshold: f32,
}

impl Default for PhysicsConfig {
    fn default() -> Self {
        PhysicsConfig {
            gravity: Vec3::new(0.0, -9.81, 0.0),
            fixed_dt: 1.0 / 60.0,
            solver_iterations: 12,
            position_iterations: 4,
            allowed_penetration: 0.002,
            baumgarte: 0.2,
            warm_starting: true,
            sleep: SleepConfig {
                enable: true,
                linear_threshold: 0.05,
                angular_threshold: 0.05,
                frames_required: 30,
            },
            position_correction: PositionCorrectionConfig {
                max: 0.2,
                fraction: 0.25,
                restitution_threshold: 1.0,
            },
        }
    }
}

// Jointy / constrainty (pod IK i mechanikę)

#[allow(unused)]
#[derive(Debug, Clone, Copy)]
pub enum JointSpace {
    Local,
    World,
}

#[derive(Debug, Clone, Copy)]
pub struct JointAnchor {
    pub entity: Entity,
    pub anchor: Vec3,
    pub space: JointSpace,
}

#[allow(unused)]
#[derive(Debug, Clone, Copy)]
pub enum JointType {
    Fixed,
    Distance {
        min: f32,
        max: f32,
    },
    Ball,
    Hinge {
        axis: Vec3,
        limits: Option<HingeLimits>,
        motor: Option<HingeMotor>,
        ref_axis_a: Vec3,
        ref_axis_b: Vec3,
    },
}

#[derive(Debug, Component, Clone)]
pub struct Joint {
    pub a: JointAnchor,
    pub b: JointAnchor,

    pub joint_type: JointType,

    pub compliance: f32,
    pub damping: f32,

    pub break_force: f32,
}

// Events (kontakt / trigger)

#[allow(unused)]
#[derive(Debug, Clone, Message)]
pub struct CollisionEvent {
    pub a: Entity,
    pub b: Entity,
    pub started: bool,
}

#[allow(unused)]
#[derive(Debug, Clone, Message)]
pub struct TriggerEvent {
    pub sensor: Entity,
    pub other: Entity,
    pub started: bool,
}

#[derive(Debug, Clone, Message)]
pub struct JointBreakEvent {
    pub joint: Entity,
    pub a: Entity,
    pub b: Entity,
}

#[derive(Debug, Clone, Copy)]
pub struct HingeLimits {
    pub min_angle: f32,
    pub max_angle: f32,
}

#[derive(Debug, Clone, Copy)]
pub struct HingeMotor {
    pub target_speed: f32,
    pub max_torque: f32,
}

#[derive(Debug, Component, Clone)]
pub struct IkChain {
    pub bones: Vec<Entity>,
    pub iterations: u32,
    pub tolerance: f32,
    pub forward_axis_local: Vec3,
}

impl Default for IkChain {
    fn default() -> Self {
        IkChain {
            bones: Vec::new(),
            iterations: 12,
            tolerance: 0.001,
            forward_axis_local: Vec3::Y,
        }
    }
}

#[derive(Debug, Component, Clone, Copy)]
pub struct IkTarget {
    pub entity: Entity,
    pub offset_world: Vec3,
}

#[derive(Debug, Component, Clone, Copy)]
pub struct IkPole {
    pub entity: Entity,
    pub offset_world: Vec3,
}

#[derive(Debug, Component, Clone, Copy)]
pub struct PreviousTransform(pub Transform);

impl Default for PreviousTransform {
    fn default() -> Self {
        PreviousTransform(Transform::IDENTITY)
    }
}
