use bevy::prelude::*;

#[derive(Resource, Clone, Debug)]
struct CrosshairConfig {
    pub shape: CrosshairShape,
    pub size: f32,
}

#[allow(unused)]
#[derive(Clone, Debug)]
pub enum CrosshairShape {
    Dot,
    Square,
}

pub struct CrosshairPlugin {
    pub shape: CrosshairShape,
    pub size: f32,
}

#[derive(Component)]
struct CrosshairRoot;

impl Plugin for CrosshairPlugin {
    fn build(&self, app: &mut App) {
        app.insert_resource(CrosshairConfig {
            shape: self.shape.clone(),
            size: self.size.clone(),
        })
        .add_systems(Startup, spawn_crosshair);
    }
}

fn spawn_crosshair(mut commands: Commands, config: Res<CrosshairConfig>) {
    commands.spawn((
        CrosshairRoot,
        Node {
            position_type: PositionType::Absolute,
            left: Val::Percent(50.0),
            top: Val::Percent(50.0),
            width: Val::Px(config.size),
            height: Val::Px(config.size),
            margin: UiRect::new(
                Val::Px(-config.size * 0.5),
                Val::Px(0.0),
                Val::Px(-config.size * 0.5),
                Val::Px(0.0),
            ),
            ..default()
        },
        BackgroundColor(Color::WHITE),
        match config.shape {
            CrosshairShape::Dot => BorderRadius::all(Val::Px(config.size * 0.5)),
            CrosshairShape::Square => BorderRadius::DEFAULT,
        },
    ));
}
