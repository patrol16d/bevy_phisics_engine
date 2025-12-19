use bevy::diagnostic::{DiagnosticsStore, FrameTimeDiagnosticsPlugin};
use bevy::prelude::*;
use std::collections::VecDeque;

pub struct FpsOverlayPlugin {
    pub config: FpsOverlayConfig,
}

impl Default for FpsOverlayPlugin {
    fn default() -> Self {
        Self {
            config: Default::default(),
        }
    }
}

#[derive(Clone, Resource)]
pub struct FpsOverlayConfig {
    pub text_config: TextFont,
    pub text_color: OverlayColor,
    pub refresh_interval: std::time::Duration,
    pub enabled: bool,
    pub frame_time_graph: FrameTimeGraph,
}

impl Default for FpsOverlayConfig {
    fn default() -> Self {
        Self {
            text_config: TextFont {
                font_size: 12.0,
                ..default()
            },
            text_color: OverlayColor::GREEN,
            refresh_interval: core::time::Duration::from_millis(100),
            enabled: true,
            frame_time_graph: FrameTimeGraph::Enabled {
                min_fps: 0.0,
                target_fps: 144.0,
            },
        }
    }
}

#[allow(unused)]
#[derive(Clone, Copy)]
pub enum OverlayColor {
    GREEN,
    RED,
    BLUE,
    WHITE,
    YELLOW,
}

impl OverlayColor {
    pub fn to_color(self) -> Color {
        match self {
            OverlayColor::GREEN => Color::srgb(0.2, 1.0, 0.2),
            OverlayColor::RED => Color::srgb(1.0, 0.2, 0.2),
            OverlayColor::BLUE => Color::srgb(0.2, 0.6, 1.0),
            OverlayColor::WHITE => Color::srgb(1.0, 1.0, 1.0),
            OverlayColor::YELLOW => Color::srgb(1.0, 1.0, 0.2),
        }
    }
}

#[allow(unused)]
#[derive(Clone)]
pub enum FrameTimeGraph {
    Disabled,
    Enabled { min_fps: f32, target_fps: f32 },
}

#[derive(Component)]
struct FpsOverlayRoot;

#[derive(Component)]
struct FpsOverlayText;

#[derive(Component)]
struct FpsGraphRoot;

#[derive(Resource)]
struct FpsOverlayState {
    acc: f32,
    graph: VecDeque<f32>,
    graph_capacity: usize,
}

impl Default for FpsOverlayState {
    fn default() -> Self {
        FpsOverlayState {
            acc: 0.0,
            graph: VecDeque::new(),
            graph_capacity: 180,
        }
    }
}

#[derive(Component)]
struct FpsGraphSegments {
    entities: Vec<Entity>,
}

impl Plugin for FpsOverlayPlugin {
    fn build(&self, app: &mut App) {
        app.add_plugins(FrameTimeDiagnosticsPlugin::default())
            .insert_resource(self.config.clone())
            .init_resource::<FpsOverlayState>()
            .add_systems(Startup, fps_overlay_spawn)
            .add_systems(Update, fps_overlay_update);
    }
}

fn fps_overlay_spawn(mut commands: Commands, config: Res<FpsOverlayConfig>) {
    if !config.enabled {
        return;
    }

    commands
        .spawn((
            Node {
                position_type: PositionType::Absolute,
                left: Val::Px(12.0),
                top: Val::Px(12.0),
                padding: UiRect::all(Val::Px(5.0)),
                flex_direction: FlexDirection::Column,
                ..default()
            },
            BackgroundColor(Color::srgba(0.0, 0.0, 0.0, 0.35)),
            FpsOverlayRoot,
        ))
        .with_children(|p| {
            p.spawn((
                Text::new("FPS: --\nFrame: -- ms"),
                config.text_config.clone(),
                TextColor(config.text_color.to_color()),
                FpsOverlayText,
            ));

            match config.frame_time_graph {
                FrameTimeGraph::Disabled => {}
                FrameTimeGraph::Enabled {
                    min_fps: _,
                    target_fps: _,
                } => {
                    p.spawn((
                        Node {
                            width: Val::Px(220.0),
                            height: Val::Px(50.0),
                            overflow: Overflow::clip(),
                            ..default()
                        },
                        BackgroundColor(Color::srgba(1.0, 1.0, 1.0, 0.08)),
                        FpsGraphRoot,
                        FpsGraphSegments {
                            entities: Vec::new(),
                        },
                    ));
                }
            }
        });
}

fn fps_overlay_update(
    time: Res<Time>,
    diagnostics: Res<DiagnosticsStore>,
    config: Res<FpsOverlayConfig>,
    mut state: ResMut<FpsOverlayState>,
    mut q_text: Query<&mut Text, With<FpsOverlayText>>,
    mut q_graph: Query<(Entity, &Node, &mut FpsGraphSegments), With<FpsGraphRoot>>,
    mut commands: Commands,
) {
    if !config.enabled {
        return;
    }

    state.acc += time.delta_secs();
    if state.acc < config.refresh_interval.as_secs_f32() {
        return;
    }
    state.acc = 0.0;

    let mut fps_value: f32 = 0.0;
    let mut frame_ms: f32 = 0.0;

    if let Some(d) = diagnostics.get(&FrameTimeDiagnosticsPlugin::FPS) {
        if let Some(v) = d.smoothed() {
            fps_value = v as f32;
        }
    }

    if let Some(d) = diagnostics.get(&FrameTimeDiagnosticsPlugin::FRAME_TIME) {
        if let Some(v) = d.smoothed() {
            frame_ms = (v as f32) * 1000.0;
        }
    }

    if let Ok(mut text) = q_text.single_mut() {
        *text = Text::new(format!("FPS: {:.1}\nFrame: {:.2} ms", fps_value, frame_ms));
    }

    match config.frame_time_graph {
        FrameTimeGraph::Disabled => {}
        FrameTimeGraph::Enabled {
            min_fps,
            target_fps,
        } => {
            let clamped_fps: f32 = fps_value.max(0.0);
            state.graph.push_back(clamped_fps);

            while state.graph.len() > state.graph_capacity {
                state.graph.pop_front();
            }

            let Ok((graph_entity, node, mut segments)) = q_graph.single_mut() else {
                return;
            };

            for e in segments.entities.drain(..) {
                commands.entity(e).despawn();
            }

            let w: f32 = match node.width {
                Val::Px(v) => v,
                _ => 320.0,
            };

            let h: f32 = match node.height {
                Val::Px(v) => v,
                _ => 80.0,
            };

            let min_fps: f32 = min_fps.max(1.0);
            let target_fps: f32 = target_fps.max(min_fps);

            let n: usize = state.graph.len();
            if n < 2 {
                return;
            }

            let dx: f32 = w / (n as f32 - 1.0);
            let thickness: f32 = 2.0;

            let mut prev_x: f32 = 0.0;
            let mut prev_y: f32 = fps_to_y(state.graph[0], h, min_fps, target_fps);

            let mut i: usize = 1;
            while i < n {
                let x: f32 = dx * (i as f32);
                let y: f32 = fps_to_y(state.graph[i], h, min_fps, target_fps);

                let (sx, sy, len, rot) = segment_transform(prev_x, prev_y, x, y);

                let child: Entity = commands
                    .spawn((
                        Node {
                            position_type: PositionType::Absolute,
                            left: Val::Px(sx),
                            top: Val::Px(sy),
                            width: Val::Px(len),
                            height: Val::Px(thickness),
                            ..default()
                        },
                        Transform {
                            translation: Vec3::new(0.0, 0.0, 0.0),
                            rotation: Quat::from_rotation_z(rot),
                            scale: Vec3::ONE,
                        },
                        GlobalTransform::default(),
                        BackgroundColor(Color::srgba(0.2, 1.0, 0.2, 0.85)),
                    ))
                    .id();

                commands.entity(graph_entity).add_child(child);
                segments.entities.push(child);

                prev_x = x;
                prev_y = y;
                i += 1;
            }
        }
    }
}

fn fps_to_y(fps: f32, h: f32, min_fps: f32, target_fps: f32) -> f32 {
    let t: f32 = ((fps - min_fps) / (target_fps - min_fps)).clamp(0.0, 1.0);
    (1.0 - t) * (h - 2.0)
}

fn segment_transform(x0: f32, y0: f32, x1: f32, y1: f32) -> (f32, f32, f32, f32) {
    let dx: f32 = x1 - x0;
    let dy: f32 = y1 - y0;
    let len: f32 = (dx * dx + dy * dy).sqrt().max(0.001);
    let rot: f32 = dy.atan2(dx);
    (x0, y0, len, rot)
}
