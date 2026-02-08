use nannou::prelude::*;
use rand::Rng;

use crate::obstacle::{Circle, Obstacle, Wall};

const EXTENT_X: f32 = 500.0;
const EXTENT_Y: f32 = 500.0;

const BOID_SIZE: f32 = 20.0;
const BOID_SPEED: f32 = 120.0;
const WALL_TURN_FACTOR: f32 = 100.0;

const SENSE_RADIUS: f32 = BOID_SPEED * 2.0;

const OBSTACLE_AVOIDANCE_RAYCAST_RES: i32 = 10;

const BOID_SPAWN_PADDING: f32 = 100.0;

const BOID_OBSTACLE_VIEW_DIST: f32 = BOID_SPEED * 2.0;

const START_BOIDS: u32 = 10;

const DRAW_DEBUG_RAYS: bool = true;

const CURSOR_CIRCLE_RADIUS: f32 = 50.0;

fn main() {
    nannou::app(model).update(update).simple_window(view).run();
}

mod obstacle;

struct Model {
    boids: Vec<Boid>,
    queued_drawings: DrawingsList,
    obstacles: Vec<Box<dyn Obstacle>>,
}

type DrawingsList = Vec<Box<dyn Fn(&Draw)>>;

#[derive(Clone, Copy)]
struct Boid {
    id: u32,
    pos: Vec2,
    // Around X Axis
    heading: f32,
}

fn update(app: &App, model: &mut Model, update: Update) {
    model.queued_drawings.clear();
    let delta_time = if app.keys.down.contains(&Key::Space) {
        0.0
    } else {
        update.since_last.as_secs_f32()
    };
    let circ = Circle::new(app.mouse.position(), CURSOR_CIRCLE_RADIUS);
    let obstacles: Vec<_> = model
        .obstacles
        .iter()
        .map(|i| -> &dyn Obstacle { &**i })
        .chain(std::iter::once(&circ as _))
        .collect();

    for i in 0..model.boids.len() {
        let turn = apply_flock_rules(&model.boids[i], &model.boids);

        let boid = &mut model.boids[i];
        boid.heading += turn * delta_time;

        let avoidance_dl = avoid_obstacles(boid, &obstacles[..], delta_time);
        model.queued_drawings.extend(avoidance_dl);

        boid.pos += Vec2::X.rotate(boid.heading) * delta_time * BOID_SPEED;
    }
}

fn apply_flock_rules(boid: &Boid, boids: &[Boid]) -> f32 {
    let mut total_turning = 0.0;
    let other_boids: Vec<_> = boids
        .iter()
        .filter(|b| b.id != boid.id && b.pos.distance(boid.pos) <= SENSE_RADIUS)
        .collect();

    // Separation
    total_turning += other_boids
        .iter()
        .fold(Vec2::ZERO, |acc, i| acc - i.pos + boid.pos)
        .angle_between(Vec2::X)
        - boid.heading;

    if !total_turning.is_finite() {
        return 0.0;
    }

    total_turning
}

fn avoid_obstacles(boid: &mut Boid, obstacles: &[&dyn Obstacle], delta_time: f32) -> DrawingsList {
    let (total_turn, dl) = (-OBSTACLE_AVOIDANCE_RAYCAST_RES..=OBSTACLE_AVOIDANCE_RAYCAST_RES)
        .map(|i| {
            let angle_offset = i as f32 / OBSTACLE_AVOIDANCE_RAYCAST_RES as f32 * PI / 2.0;
            let angle = boid.heading + angle_offset;
            let ray_v = Vec2::X.rotate(angle);
            let mut nearest_dist = f32::INFINITY;
            for obstacle in obstacles {
                let dist = obstacle.ray_test(boid.pos, ray_v);
                nearest_dist = nearest_dist.min(dist);
            }
            let deflection_angle = if angle_offset > 0.0 {
                0.5 * PI
            } else {
                -0.5 * PI
            };
            let deflection = deflection_angle * delta_time * nearest_dist.recip()
                / OBSTACLE_AVOIDANCE_RAYCAST_RES as f32;

            (
                if nearest_dist < BOID_OBSTACLE_VIEW_DIST {
                    deflection
                } else {
                    0.0
                },
                nearest_dist,
                angle,
            )
        })
        .fold((0.0, vec![]), |mut acc, i| {
            if i.1.is_finite() && DRAW_DEBUG_RAYS {
                let boid_pos = boid.pos;
                acc.1.push(Box::new(move |draw: &Draw| {
                    draw.line()
                        .start(boid_pos)
                        .end(boid_pos + Vec2::X.rotate(i.2) * i.1.min(BOID_OBSTACLE_VIEW_DIST))
                        .color(RED);
                }) as _);
            }
            (acc.0 + i.0, acc.1)
        });

    boid.heading -= total_turn * WALL_TURN_FACTOR;
    dl
}

fn model(_app: &App) -> Model {
    Model {
        boids: random_boids(),
        queued_drawings: vec![],
        obstacles: walls(),
    }
}

fn walls() -> Vec<Box<dyn Obstacle>> {
    vec![
        Box::new(Wall::hor(-EXTENT_Y)) as _,
        Box::new(Wall::hor(EXTENT_Y)) as _,
        Box::new(Wall::vert(-EXTENT_X)) as _,
        Box::new(Wall::vert(EXTENT_X)) as _,
    ]
}

fn view(app: &App, model: &Model, frame: Frame) {
    let draw = app.draw();
    draw.background().color(LIGHTGRAY);

    // Draw walls
    draw.rect().x(-EXTENT_X).w(10.0).h(EXTENT_Y * 2.0);
    draw.rect().x(EXTENT_X).w(10.0).h(EXTENT_Y * 2.0);
    draw.rect().y(-EXTENT_Y).h(10.0).w(EXTENT_X * 2.0);
    draw.rect().y(EXTENT_X).h(10.0).w(EXTENT_X * 2.0);

    draw.ellipse()
        .radius(CURSOR_CIRCLE_RADIUS)
        .xy(app.mouse.position())
        .color(BLUE);

    // Draw boids
    for boid in &model.boids {
        // TODO: transform this to be centered on the actual boid
        draw.tri()
            .w(BOID_SIZE)
            .h(BOID_SIZE)
            .z_radians(boid.heading)
            .xy(boid.pos)
            .color(DARKGRAY);

        draw.ellipse().color(RED).xy(boid.pos).w(10.0).h(10.0);
    }

    model.queued_drawings.iter().for_each(|i| (*i)(&draw));
    draw.to_frame(app, &frame).unwrap();
}

fn random_boids() -> Vec<Boid> {
    let mut rng = rand::rng();
    (0..START_BOIDS)
        .map(|i| {
            let x =
                rng.random_range(-EXTENT_X + BOID_SPAWN_PADDING..=EXTENT_X - BOID_SPAWN_PADDING);
            let y =
                rng.random_range(-EXTENT_Y + BOID_SPAWN_PADDING..=EXTENT_Y - BOID_SPAWN_PADDING);
            let heading = rng.random_range(0.0..2.0 * PI);

            Boid {
                pos: Vec2::new(x, y),
                heading,
                id: i,
            }
        })
        .collect()
}
