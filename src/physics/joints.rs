use bevy::prelude::*;

use crate::physics::components::*;

pub fn solve_joints_positions(
    config: Res<PhysicsConfig>,
    fixed_time: Res<Time<Fixed>>,
    joints: Query<(Entity, &Joint)>,
    mut break_writer: MessageWriter<JointBreakEvent>,
    mut bodies: Query<(&mut Transform, &mut RigidBody)>,
) {
    let dt: f32 = fixed_time.delta_secs().min(config.fixed_dt);
    if dt <= 0.0 {
        return;
    }

    let iters: u32 = config.position_iterations.max(1);

    for _ in 0..iters {
        for (joint_e, joint) in joints.iter() {
            let Ok([(mut a_tr, mut a_rb), (mut b_tr, mut b_rb)]) =
                bodies.get_many_mut([joint.a.entity, joint.b.entity])
            else {
                continue;
            };

            if !a_rb.flags.contains(BodyFlags::ENABLE_SOLVER)
                || !b_rb.flags.contains(BodyFlags::ENABLE_SOLVER)
            {
                continue;
            }

            let a_dyn: bool = a_rb.body_type == BodyType::Dynamic;
            let b_dyn: bool = b_rb.body_type == BodyType::Dynamic;

            if !a_dyn && !b_dyn {
                continue;
            }

            let com_a: Vec3 = world_center_of_mass(&a_tr, &a_rb);
            let com_b: Vec3 = world_center_of_mass(&b_tr, &b_rb);

            let pa: Vec3 = joint_anchor_world(&joint.a, &a_tr);
            let pb: Vec3 = joint_anchor_world(&joint.b, &b_tr);

            let ra: Vec3 = pa - com_a;
            let rb: Vec3 = pb - com_b;

            let inv_mass_a: f32 = if a_dyn { a_rb.inv_mass } else { 0.0 };
            let inv_mass_b: f32 = if b_dyn { b_rb.inv_mass } else { 0.0 };

            let inv_inertia_a: Mat3 = if a_dyn {
                inv_inertia_world(a_tr.rotation, a_rb.inv_inertia_local)
            } else {
                Mat3::ZERO
            };
            let inv_inertia_b: Mat3 = if b_dyn {
                inv_inertia_world(b_tr.rotation, b_rb.inv_inertia_local)
            } else {
                Mat3::ZERO
            };

            let alpha: f32 = if joint.compliance > 0.0 {
                joint.compliance / (dt * dt)
            } else {
                0.0
            };

            match joint.joint_type {
                JointType::Distance { min, max } => {
                    let d: Vec3 = pb - pa;
                    let len: f32 = d.length();
                    if len <= 1e-9 {
                        continue;
                    }

                    let n: Vec3 = d / len;

                    let c: f32 = if len < min {
                        len - min
                    } else if len > max {
                        len - max
                    } else {
                        continue;
                    };

                    let denom: f32 = effective_mass_scalar(
                        n,
                        inv_mass_a,
                        inv_mass_b,
                        inv_inertia_a,
                        inv_inertia_b,
                        ra,
                        rb,
                    ) + alpha;

                    if denom <= 0.0 {
                        continue;
                    }

                    let lambda: f32 = -c / denom;

                    if joint.break_force > 0.0 {
                        let f_est: f32 = lambda.abs() / dt;
                        if f_est > joint.break_force {
                            break_writer.write(JointBreakEvent {
                                joint: joint_e,
                                a: joint.a.entity,
                                b: joint.b.entity,
                            });
                            continue;
                        }
                    }

                    let corr: Vec3 = n * lambda;

                    apply_pos_correction(
                        &config,
                        &mut a_tr,
                        &mut a_rb,
                        &mut b_tr,
                        &mut b_rb,
                        inv_mass_a,
                        inv_mass_b,
                        inv_inertia_a,
                        inv_inertia_b,
                        ra,
                        rb,
                        corr,
                    );
                }
                JointType::Ball | JointType::Fixed => {
                    let err: Vec3 = pb - pa;
                    let err2: f32 = err.length_squared();
                    if err2 <= 1e-12 {
                        continue;
                    }

                    let n: Vec3 = err.normalize();
                    let (t1, t2): (Vec3, Vec3) = tangent_basis(n);

                    let axes: [Vec3; 3] = [n, t1, t2];

                    let mut max_force: f32 = 0.0;

                    let mut k: usize = 0;
                    while k < 3 {
                        let axis: Vec3 = axes[k];
                        let c: f32 = err.dot(axis);

                        let denom: f32 = effective_mass_scalar(
                            axis,
                            inv_mass_a,
                            inv_mass_b,
                            inv_inertia_a,
                            inv_inertia_b,
                            ra,
                            rb,
                        ) + alpha;

                        if denom > 0.0 {
                            let lambda: f32 = -c / denom;

                            let f_est: f32 = lambda.abs() / dt;
                            if f_est > max_force {
                                max_force = f_est;
                            }

                            let corr: Vec3 = axis * lambda;

                            apply_pos_correction(
                                &config,
                                &mut a_tr,
                                &mut a_rb,
                                &mut b_tr,
                                &mut b_rb,
                                inv_mass_a,
                                inv_mass_b,
                                inv_inertia_a,
                                inv_inertia_b,
                                ra,
                                rb,
                                corr,
                            );
                        }

                        k += 1;
                    }

                    if joint.break_force > 0.0 && max_force > joint.break_force {
                        break_writer.write(JointBreakEvent {
                            joint: joint_e,
                            a: joint.a.entity,
                            b: joint.b.entity,
                        });
                    }
                }
                JointType::Hinge {
                    axis,
                    limits,
                    motor: _,
                    ref_axis_a,
                    ref_axis_b,
                } => {
                    let err: Vec3 = pb - pa;
                    let err2: f32 = err.length_squared();
                    if err2 > 1e-12 {
                        let n: Vec3 = err.normalize();
                        let (t1, t2): (Vec3, Vec3) = tangent_basis(n);
                        let axes: [Vec3; 3] = [n, t1, t2];

                        let mut max_force: f32 = 0.0;

                        let mut k: usize = 0;
                        while k < 3 {
                            let ax: Vec3 = axes[k];
                            let c: f32 = err.dot(ax);

                            let denom: f32 = effective_mass_scalar(
                                ax,
                                inv_mass_a,
                                inv_mass_b,
                                inv_inertia_a,
                                inv_inertia_b,
                                ra,
                                rb,
                            ) + alpha;

                            if denom > 0.0 {
                                let lambda: f32 = -c / denom;
                                let f_est: f32 = lambda.abs() / dt;
                                if f_est > max_force {
                                    max_force = f_est;
                                }

                                let corr: Vec3 = ax * lambda;
                                apply_pos_correction(
                                    &config,
                                    &mut a_tr,
                                    &mut a_rb,
                                    &mut b_tr,
                                    &mut b_rb,
                                    inv_mass_a,
                                    inv_mass_b,
                                    inv_inertia_a,
                                    inv_inertia_b,
                                    ra,
                                    rb,
                                    corr,
                                );
                            }

                            k += 1;
                        }

                        if joint.break_force > 0.0 && max_force > joint.break_force {
                            break_writer.write(JointBreakEvent {
                                joint: joint_e,
                                a: joint.a.entity,
                                b: joint.b.entity,
                            });
                            continue;
                        }
                    }

                    let axis_world: Vec3 = hinge_axis_world(axis, &joint.a, &a_tr);
                    let axis_len2: f32 = axis_world.length_squared();
                    let ax: Vec3 = axis_world / axis_len2.sqrt();
                    if axis_len2 <= 1e-12 {
                        if let Some(lim) = limits {
                            let ra_w: Vec3 = hinge_ref_world(ref_axis_a, &joint.a, &a_tr, ax);
                            let rb_w: Vec3 = hinge_ref_world(ref_axis_b, &joint.b, &b_tr, ax);

                            let angle: f32 = signed_angle_around_axis(ra_w, rb_w, ax);

                            let c: f32 = if angle < lim.min_angle {
                                angle - lim.min_angle
                            } else if angle > lim.max_angle {
                                angle - lim.max_angle
                            } else {
                                0.0
                            };

                            if c.abs() > 1e-6 {
                                let denom: f32 =
                                    angular_effective_mass_scalar(ax, inv_inertia_a, inv_inertia_b)
                                        + alpha;
                                if denom > 0.0 {
                                    let lambda: f32 = -c / denom;

                                    if joint.break_force > 0.0 {
                                        let f_est: f32 = lambda.abs() / dt;
                                        if f_est > joint.break_force {
                                            break_writer.write(JointBreakEvent {
                                                joint: joint_e,
                                                a: joint.a.entity,
                                                b: joint.b.entity,
                                            });
                                            continue;
                                        }
                                    }

                                    if inv_mass_a > 0.0
                                        && a_rb.flags.contains(BodyFlags::ENABLE_ROTATION)
                                    {
                                        let dw: Vec3 = inv_inertia_a * (-ax * lambda);
                                        let dq: Quat = Quat::from_scaled_axis(dw);
                                        a_tr.rotation = (dq * a_tr.rotation).normalize();
                                        wake(&mut a_rb, &config);
                                    }

                                    if inv_mass_b > 0.0
                                        && b_rb.flags.contains(BodyFlags::ENABLE_ROTATION)
                                    {
                                        let dw: Vec3 = inv_inertia_b * (ax * lambda);
                                        let dq: Quat = Quat::from_scaled_axis(dw);
                                        b_tr.rotation = (dq * b_tr.rotation).normalize();
                                        wake(&mut b_rb, &config);
                                    }
                                }
                            }
                        }
                    }

                    let w1: f32 = inv_mass_a;
                    let w2: f32 = inv_mass_b;

                    if w1 > 0.0 && a_rb.flags.contains(BodyFlags::ENABLE_ROTATION) {
                        let wa: Vec3 = a_tr.rotation * ax;
                        let _ = wa;
                    }
                    if w2 > 0.0 && b_rb.flags.contains(BodyFlags::ENABLE_ROTATION) {
                        let wb: Vec3 = b_tr.rotation * ax;
                        let _ = wb;
                    }

                    let a_axis_world: Vec3 = (a_tr.rotation * ax).normalize_or_zero();
                    let b_axis_world: Vec3 = (b_tr.rotation * ax).normalize_or_zero();

                    let cross: Vec3 = a_axis_world.cross(b_axis_world);
                    let cross2: f32 = cross.length_squared();
                    if cross2 <= 1e-12 {
                        continue;
                    }

                    let ang_axis: Vec3 = cross / cross2.sqrt();
                    let sin_theta: f32 = cross2.sqrt().clamp(0.0, 1.0);
                    let theta: f32 = sin_theta.asin();

                    let (u, v): (Vec3, Vec3) = tangent_basis(ax);

                    let axes2: [Vec3; 2] = [u, v];

                    let mut i: usize = 0;
                    while i < 2 {
                        let t: Vec3 = axes2[i];

                        let c: f32 = theta * ang_axis.dot(t);

                        let denom: f32 =
                            angular_effective_mass_scalar(t, inv_inertia_a, inv_inertia_b) + alpha;

                        if denom > 0.0 {
                            let lambda: f32 = -c / denom;

                            if joint.break_force > 0.0 {
                                let f_est: f32 = lambda.abs() / dt;
                                if f_est > joint.break_force {
                                    break_writer.write(JointBreakEvent {
                                        joint: joint_e,
                                        a: joint.a.entity,
                                        b: joint.b.entity,
                                    });
                                    break;
                                }
                            }

                            if inv_mass_a > 0.0 && a_rb.flags.contains(BodyFlags::ENABLE_ROTATION) {
                                let dw: Vec3 = inv_inertia_a * (-t * lambda);
                                let dq: Quat = Quat::from_scaled_axis(dw);
                                a_tr.rotation = (dq * a_tr.rotation).normalize();
                                wake(&mut a_rb, &config);
                            }

                            if inv_mass_b > 0.0 && b_rb.flags.contains(BodyFlags::ENABLE_ROTATION) {
                                let dw: Vec3 = inv_inertia_b * (t * lambda);
                                let dq: Quat = Quat::from_scaled_axis(dw);
                                b_tr.rotation = (dq * b_tr.rotation).normalize();
                                wake(&mut b_rb, &config);
                            }
                        }

                        i += 1;
                    }
                }
            }
        }
    }
}

pub fn solve_joints_velocities(
    config: Res<PhysicsConfig>,
    fixed_time: Res<Time<Fixed>>,
    joints: Query<(Entity, &Joint)>,
    mut bodies: Query<(&Transform, &mut RigidBody)>,
) {
    let dt: f32 = fixed_time.delta_secs().min(config.fixed_dt);
    if dt <= 0.0 {
        return;
    }

    let iters: u32 = config.solver_iterations.max(1);

    for _ in 0..iters {
        for (_joint_e, joint) in joints.iter() {
            let Ok([(a_tr, mut a_rb), (b_tr, mut b_rb)]) =
                bodies.get_many_mut([joint.a.entity, joint.b.entity])
            else {
                continue;
            };

            if !a_rb.flags.contains(BodyFlags::ENABLE_SOLVER)
                || !b_rb.flags.contains(BodyFlags::ENABLE_SOLVER)
            {
                continue;
            }

            let a_dyn: bool = a_rb.body_type == BodyType::Dynamic;
            let b_dyn: bool = b_rb.body_type == BodyType::Dynamic;

            if !a_dyn && !b_dyn {
                continue;
            }

            let damping: f32 = joint.damping.max(0.0);
            if damping <= 0.0 {
                continue;
            }

            let com_a: Vec3 = world_center_of_mass(a_tr, &a_rb);
            let com_b: Vec3 = world_center_of_mass(b_tr, &b_rb);

            let pa: Vec3 = joint_anchor_world(&joint.a, a_tr);
            let pb: Vec3 = joint_anchor_world(&joint.b, b_tr);

            let ra: Vec3 = pa - com_a;
            let rb: Vec3 = pb - com_b;

            let inv_mass_a: f32 = if a_dyn { a_rb.inv_mass } else { 0.0 };
            let inv_mass_b: f32 = if b_dyn { b_rb.inv_mass } else { 0.0 };

            let inv_inertia_a: Mat3 = if a_dyn {
                inv_inertia_world(a_tr.rotation, a_rb.inv_inertia_local)
            } else {
                Mat3::ZERO
            };
            let inv_inertia_b: Mat3 = if b_dyn {
                inv_inertia_world(b_tr.rotation, b_rb.inv_inertia_local)
            } else {
                Mat3::ZERO
            };

            match joint.joint_type {
                JointType::Distance { min, max } => {
                    let d: Vec3 = pb - pa;
                    let len: f32 = d.length();
                    if len <= 1e-9 {
                        continue;
                    }

                    if !(len < min || len > max) {
                        continue;
                    }

                    let n: Vec3 = d / len;

                    let va: Vec3 = a_rb.linear_velocity + a_rb.angular_velocity.cross(ra);
                    let vb: Vec3 = b_rb.linear_velocity + b_rb.angular_velocity.cross(rb);
                    let rel: f32 = (vb - va).dot(n);

                    let denom: f32 = effective_mass_scalar(
                        n,
                        inv_mass_a,
                        inv_mass_b,
                        inv_inertia_a,
                        inv_inertia_b,
                        ra,
                        rb,
                    );

                    if denom <= 0.0 {
                        continue;
                    }

                    let j: f32 = (-rel * damping) / denom;
                    apply_vel_impulse(
                        &config,
                        &mut a_rb,
                        &mut b_rb,
                        inv_mass_a,
                        inv_mass_b,
                        inv_inertia_a,
                        inv_inertia_b,
                        ra,
                        rb,
                        n * j,
                    );
                }
                JointType::Ball | JointType::Fixed => {
                    let err: Vec3 = pb - pa;
                    let err2: f32 = err.length_squared();
                    if err2 <= 1e-12 {
                        continue;
                    }

                    let n: Vec3 = err.normalize();
                    let (t1, t2): (Vec3, Vec3) = tangent_basis(n);
                    let axes: [Vec3; 3] = [n, t1, t2];

                    let mut k: usize = 0;
                    while k < 3 {
                        let axis: Vec3 = axes[k];

                        let va: Vec3 = a_rb.linear_velocity + a_rb.angular_velocity.cross(ra);
                        let vb: Vec3 = b_rb.linear_velocity + b_rb.angular_velocity.cross(rb);
                        let rel: f32 = (vb - va).dot(axis);

                        let denom: f32 = effective_mass_scalar(
                            axis,
                            inv_mass_a,
                            inv_mass_b,
                            inv_inertia_a,
                            inv_inertia_b,
                            ra,
                            rb,
                        );

                        if denom > 0.0 {
                            let j: f32 = (-rel * damping) / denom;
                            apply_vel_impulse(
                                &config,
                                &mut a_rb,
                                &mut b_rb,
                                inv_mass_a,
                                inv_mass_b,
                                inv_inertia_a,
                                inv_inertia_b,
                                ra,
                                rb,
                                axis * j,
                            );
                        }

                        k += 1;
                    }
                }
                JointType::Hinge {
                    axis,
                    limits: _,
                    motor,
                    ref_axis_a: _,
                    ref_axis_b: _,
                } => {
                    let axis_w: Vec3 = hinge_axis_world(axis, &joint.a, a_tr);
                    let axis_len2: f32 = axis_w.length_squared();
                    if axis_len2 <= 1e-12 {
                        continue;
                    }

                    let ax: Vec3 = axis_w / axis_len2.sqrt();

                    let denom: f32 =
                        angular_effective_mass_scalar(ax, inv_inertia_a, inv_inertia_b);
                    if denom <= 0.0 {
                        continue;
                    }

                    if let Some(motor) = motor {
                        let rel_w: f32 = (b_rb.angular_velocity - a_rb.angular_velocity).dot(ax);
                        let desired: f32 = motor.target_speed;

                        let mut j: f32 = (desired - rel_w) / denom;

                        let max_j: f32 = motor.max_torque.max(0.0) * dt;
                        j = j.clamp(-max_j, max_j);

                        let impulse_w: Vec3 = ax * j;

                        if inv_mass_a > 0.0 && a_rb.flags.contains(BodyFlags::ENABLE_ROTATION) {
                            a_rb.angular_velocity += inv_inertia_a * (-impulse_w);
                            wake(&mut a_rb, &config);
                        }

                        if inv_mass_b > 0.0 && b_rb.flags.contains(BodyFlags::ENABLE_ROTATION) {
                            b_rb.angular_velocity += inv_inertia_b * (impulse_w);
                            wake(&mut b_rb, &config);
                        }
                    }

                    if joint.damping > 0.0 {
                        let damping: f32 = joint.damping.max(0.0);

                        let rel_w: f32 = (b_rb.angular_velocity - a_rb.angular_velocity).dot(ax);
                        let j: f32 = (-rel_w * damping) / denom;

                        let impulse_w: Vec3 = ax * j;

                        if inv_mass_a > 0.0 && a_rb.flags.contains(BodyFlags::ENABLE_ROTATION) {
                            a_rb.angular_velocity += inv_inertia_a * (-impulse_w);
                            wake(&mut a_rb, &config);
                        }

                        if inv_mass_b > 0.0 && b_rb.flags.contains(BodyFlags::ENABLE_ROTATION) {
                            b_rb.angular_velocity += inv_inertia_b * (impulse_w);
                            wake(&mut b_rb, &config);
                        }
                    }
                }
            }
        }
    }
}

fn joint_anchor_world(anchor: &JointAnchor, tr: &Transform) -> Vec3 {
    match anchor.space {
        JointSpace::World => anchor.anchor,
        JointSpace::Local => tr.translation + tr.rotation * (anchor.anchor * tr.scale),
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

fn effective_mass_scalar(
    axis: Vec3,
    inv_mass_a: f32,
    inv_mass_b: f32,
    inv_inertia_a: Mat3,
    inv_inertia_b: Mat3,
    ra: Vec3,
    rb: Vec3,
) -> f32 {
    let ra_cn: Vec3 = ra.cross(axis);
    let rb_cn: Vec3 = rb.cross(axis);

    let angular: f32 = (inv_inertia_a * ra_cn).dot(ra_cn) + (inv_inertia_b * rb_cn).dot(rb_cn);

    inv_mass_a + inv_mass_b + angular
}

fn apply_pos_correction(
    config: &PhysicsConfig,
    a_tr: &mut Transform,
    a_rb: &mut RigidBody,
    b_tr: &mut Transform,
    b_rb: &mut RigidBody,
    inv_mass_a: f32,
    inv_mass_b: f32,
    inv_inertia_a: Mat3,
    inv_inertia_b: Mat3,
    ra: Vec3,
    rb: Vec3,
    corr_world: Vec3,
) {
    if inv_mass_a > 0.0 && a_rb.flags.contains(BodyFlags::ENABLE_TRANSLATION) {
        a_tr.translation -= corr_world * inv_mass_a;
        wake(a_rb, config);
    }
    if inv_mass_b > 0.0 && b_rb.flags.contains(BodyFlags::ENABLE_TRANSLATION) {
        b_tr.translation += corr_world * inv_mass_b;
        wake(b_rb, config);
    }

    if inv_mass_a > 0.0 && a_rb.flags.contains(BodyFlags::ENABLE_ROTATION) {
        let dw: Vec3 = inv_inertia_a * ra.cross(-corr_world);
        let dq: Quat = Quat::from_scaled_axis(dw);
        a_tr.rotation = (dq * a_tr.rotation).normalize();
        wake(a_rb, config);
    }

    if inv_mass_b > 0.0 && b_rb.flags.contains(BodyFlags::ENABLE_ROTATION) {
        let dw: Vec3 = inv_inertia_b * rb.cross(corr_world);
        let dq: Quat = Quat::from_scaled_axis(dw);
        b_tr.rotation = (dq * b_tr.rotation).normalize();
        wake(b_rb, config);
    }
}

fn apply_vel_impulse(
    config: &PhysicsConfig,
    a_rb: &mut RigidBody,
    b_rb: &mut RigidBody,
    inv_mass_a: f32,
    inv_mass_b: f32,
    inv_inertia_a: Mat3,
    inv_inertia_b: Mat3,
    ra: Vec3,
    rb: Vec3,
    impulse_world: Vec3,
) {
    if inv_mass_a > 0.0 && a_rb.flags.contains(BodyFlags::ENABLE_TRANSLATION) {
        a_rb.linear_velocity -= impulse_world * inv_mass_a;
        wake(a_rb, config);
    }
    if inv_mass_b > 0.0 && b_rb.flags.contains(BodyFlags::ENABLE_TRANSLATION) {
        b_rb.linear_velocity += impulse_world * inv_mass_b;
        wake(b_rb, config);
    }

    if inv_mass_a > 0.0 && a_rb.flags.contains(BodyFlags::ENABLE_ROTATION) {
        a_rb.angular_velocity += inv_inertia_a * ra.cross(-impulse_world);
        wake(a_rb, config);
    }
    if inv_mass_b > 0.0 && b_rb.flags.contains(BodyFlags::ENABLE_ROTATION) {
        b_rb.angular_velocity += inv_inertia_b * rb.cross(impulse_world);
        wake(b_rb, config);
    }
}

fn tangent_basis(n: Vec3) -> (Vec3, Vec3) {
    let a: Vec3 = if n.z.abs() < 0.999 { Vec3::Z } else { Vec3::Y };
    let t1: Vec3 = n.cross(a).normalize();
    let t2: Vec3 = n.cross(t1).normalize();
    (t1, t2)
}

fn wake(body: &mut RigidBody, config: &PhysicsConfig) {
    if config.sleep.enable && body.is_sleeping {
        body.is_sleeping = false;
        body.sleep_frames = 0;
    }
}

pub fn handle_joint_breaks(mut commands: Commands, mut reader: MessageReader<JointBreakEvent>) {
    for ev in reader.read() {
        commands.entity(ev.joint).remove::<Joint>();

        info!(
            "Joint breake: joint={:?} a={:?} b={:?}",
            ev.joint, ev.a, ev.b
        );
    }
}

fn angular_effective_mass_scalar(axis: Vec3, inv_inertia_a: Mat3, inv_inertia_b: Mat3) -> f32 {
    (inv_inertia_a * axis).dot(axis) + (inv_inertia_b * axis).dot(axis)
}

fn hinge_axis_world(axis: Vec3, anchor: &JointAnchor, tr: &Transform) -> Vec3 {
    let ax: Vec3 = if axis.length_squared() > 1e-12 {
        axis.normalize()
    } else {
        Vec3::X
    };
    match anchor.space {
        JointSpace::World => ax,
        JointSpace::Local => tr.rotation * ax,
    }
}

fn hinge_ref_world(
    local_ref: Vec3,
    anchor: &JointAnchor,
    tr: &Transform,
    axis_world: Vec3,
) -> Vec3 {
    let mut v: Vec3 = match anchor.space {
        JointSpace::World => local_ref,
        JointSpace::Local => tr.rotation * local_ref,
    };

    v = v - axis_world * v.dot(axis_world);
    if v.length_squared() <= 1e-12 {
        let (u, _): (Vec3, Vec3) = tangent_basis(axis_world);
        return u;
    }

    v.normalize()
}

fn signed_angle_around_axis(a: Vec3, b: Vec3, axis: Vec3) -> f32 {
    let cross: Vec3 = a.cross(b);
    let sin: f32 = cross.dot(axis);
    let cos: f32 = a.dot(b);
    sin.atan2(cos)
}
