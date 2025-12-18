use bevy::prelude::*;

use crate::physics::components::*;

pub fn ik_fabrik_update(
    chains: Query<(Entity, &IkChain, &IkTarget, Option<&IkPole>)>,
    globals: Query<&GlobalTransform>,
    mut locals: Query<&mut Transform>,
) {
    for (_chain_e, chain, target, pole_opt) in chains.iter() {
        if chain.bones.len() < 2 {
            continue;
        }

        let Ok(target_gt) = globals.get(target.entity) else {
            continue;
        };

        let target_pos: Vec3 = target_gt.compute_transform().translation + target.offset_world;

        let mut positions: Vec<Vec3> = Vec::with_capacity(chain.bones.len());
        let mut lengths: Vec<f32> = Vec::with_capacity(chain.bones.len().saturating_sub(1));

        let mut i: usize = 0;
        while i < chain.bones.len() {
            let bone_e: Entity = chain.bones[i];
            let Ok(tr) = locals.get(bone_e) else {
                positions.clear();
                break;
            };
            positions.push(tr.translation);
            i += 1;
        }
        if positions.len() != chain.bones.len() {
            continue;
        }

        let mut total_len: f32 = 0.0;
        i = 0;
        while i + 1 < positions.len() {
            let l: f32 = (positions[i + 1] - positions[i]).length();
            lengths.push(l);
            total_len += l;
            i += 1;
        }

        let root_pos: Vec3 = positions[0];
        let dist_to_target: f32 = (target_pos - root_pos).length();

        if dist_to_target >= total_len && total_len > 0.0 {
            let dir: Vec3 = (target_pos - root_pos).normalize_or_zero();
            positions[0] = root_pos;

            i = 0;
            while i < lengths.len() {
                positions[i + 1] = positions[i] + dir * lengths[i];
                i += 1;
            }
        } else {
            let mut iter: u32 = 0;
            while iter < chain.iterations {
                let new_index = positions.len() - 1;
                positions[new_index] = target_pos;

                i = positions.len() - 1;
                while i > 0 {
                    let dir: Vec3 = (positions[i - 1] - positions[i]).normalize_or_zero();
                    positions[i - 1] = positions[i] + dir * lengths[i - 1];
                    i -= 1;
                }

                positions[0] = root_pos;

                i = 0;
                while i < lengths.len() {
                    let dir: Vec3 = (positions[i + 1] - positions[i]).normalize_or_zero();
                    positions[i + 1] = positions[i] + dir * lengths[i];
                    i += 1;
                }

                let err: f32 = (positions[positions.len() - 1] - target_pos).length();
                if err <= chain.tolerance {
                    break;
                }

                iter += 1;
            }
        }

        if let Some(pole) = pole_opt {
            let Ok(pole_gt) = globals.get(pole.entity) else {
                continue;
            };
            let pole_pos: Vec3 = pole_gt.compute_transform().translation + pole.offset_world;

            let root_to_target: Vec3 = (target_pos - root_pos).normalize_or_zero();
            if root_to_target.length_squared() > 1e-12 {
                let mut j: usize = 1;
                while j + 1 < positions.len() {
                    let plane_p: Vec3 = root_pos;
                    let plane_n: Vec3 = root_to_target;

                    let proj_joint: Vec3 = project_point_on_plane(positions[j], plane_p, plane_n);
                    let proj_pole: Vec3 = project_point_on_plane(pole_pos, plane_p, plane_n);

                    let center: Vec3 = positions[j - 1];
                    let v1: Vec3 = (proj_joint - center).normalize_or_zero();
                    let v2: Vec3 = (proj_pole - center).normalize_or_zero();

                    if v1.length_squared() > 1e-12 && v2.length_squared() > 1e-12 {
                        let signed: f32 = signed_angle_on_plane(v1, v2, plane_n);
                        let rot: Quat = Quat::from_axis_angle(plane_n, signed);

                        let to_joint: Vec3 = positions[j] - center;
                        let rotated: Vec3 = rot * to_joint;
                        positions[j] = center + rotated;

                        let prev: Vec3 = positions[j - 1];
                        let next_len: f32 = lengths[j - 1];
                        let dir: Vec3 = (positions[j] - prev).normalize_or_zero();
                        positions[j] = prev + dir * next_len;

                        let mut k: usize = j;
                        while k < lengths.len() {
                            let dir2: Vec3 = (positions[k + 1] - positions[k]).normalize_or_zero();
                            positions[k + 1] = positions[k] + dir2 * lengths[k];
                            k += 1;
                        }
                    }

                    j += 1;
                }
            }
        }

        i = 0;
        while i < chain.bones.len() {
            let bone_e: Entity = chain.bones[i];
            let Ok(mut tr) = locals.get_mut(bone_e) else {
                break;
            };
            tr.translation = positions[i];
            i += 1;
        }

        i = 0;
        while i + 1 < chain.bones.len() {
            let a_e: Entity = chain.bones[i];
            let b_pos: Vec3 = positions[i + 1];

            let Ok(mut a_tr) = locals.get_mut(a_e) else {
                break;
            };

            let dir: Vec3 = (b_pos - a_tr.translation).normalize_or_zero();
            if dir.length_squared() > 1e-12 {
                let current_fwd: Vec3 =
                    (a_tr.rotation * chain.forward_axis_local).normalize_or_zero();
                if current_fwd.length_squared() > 1e-12 {
                    let q: Quat = Quat::from_rotation_arc(current_fwd, dir);
                    a_tr.rotation = (q * a_tr.rotation).normalize();
                } else {
                    a_tr.rotation = Quat::from_rotation_arc(Vec3::Y, dir).normalize();
                }
            }

            i += 1;
        }
    }
}

fn project_point_on_plane(p: Vec3, plane_p: Vec3, plane_n: Vec3) -> Vec3 {
    let d: f32 = (p - plane_p).dot(plane_n);
    p - plane_n * d
}

fn signed_angle_on_plane(a: Vec3, b: Vec3, plane_n: Vec3) -> f32 {
    let cross: Vec3 = a.cross(b);
    let sin: f32 = cross.dot(plane_n);
    let cos: f32 = a.dot(b);
    sin.atan2(cos)
}
