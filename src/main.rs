use std::sync::{Arc, Mutex, OnceLock};

use nannou::prelude::*;
use rand::Rng;

use crate::obstacle::{Circle, Obstacle, Wall};

// Config
const EXTENT_X: f32 = 300.0;
const EXTENT_Y: f32 = 300.0;
const BOID_SIZE: f32 = 5.0;
const BOID_SPEED: f32 = 15.0;
const MAX_TURN_RADIANS_PER_SECOND: f32 = PI * 1.0;

// Flocking parameters
const SENSE_RADIUS: f32 = BOID_SPEED * 7.5;
const SEP_STRENGTH: f32 = 2.0;
const ALIGN_STRENGTH: f32 = 2.0;
const COHESION_STRENGTH: f32 = 1.0;

// Obstacle avoidance
const BOID_OBSTACLE_VIEW_DIST: f32 = BOID_SPEED * 10.0;
const OBSTACLE_AVOIDANCE_RAYCAST_RES: i32 = 10;
const WALL_TURN_FACTOR: f32 = 200.0;

// Spawning parameters
const BOID_SPAWN_PADDING: f32 = 100.0;
const START_BOIDS: u32 = 200;

// Tools for me
const DRAW_DEBUG_RAYS: bool = false;
const TIME_SCALE: f32 = 10.0;
const CREATED_OBSTACLE_RADIUS: f32 = 50.0;
const CURSOR_ATTRACTION_STRENGTH: f32 = 2.0;

fn main() {
    nannou::app(model).update(update).simple_window(view).run();
}

mod obstacle;

struct Model {
    boids: Vec<Boid>,
    obstacles: Vec<Box<dyn Obstacle>>,
    lmb_down_previously: bool,
}

type DrawingsList = Vec<Box<dyn Fn(&Draw) + Send>>;

#[derive(Clone, Copy)]
struct Boid {
    id: u32,
    pos: Vec2,
    // Degrees from x-axis (CCW I think not sure tho tbh it doesn't really matter I hope)
    heading: f32,
}

static QUEUED_DRAWINGS: OnceLock<Arc<Mutex<DrawingsList>>> = OnceLock::new();

fn queue_drawing(func: Box<dyn Fn(&Draw) + Send>) {
    QUEUED_DRAWINGS
        .get_or_init(|| Arc::new(Mutex::new(vec![])))
        .lock()
        .unwrap()
        .push(func);
}

fn clear_drawings() {
    let mut cur = QUEUED_DRAWINGS
        .get_or_init(|| Arc::new(Mutex::new(vec![])))
        .lock()
        .unwrap();
    cur.clear();
}

fn update(app: &App, model: &mut Model, update: Update) {
    clear_drawings();

    if let Some(pos) = app.mouse.buttons.left().if_down()
        && !model.lmb_down_previously
    {
        model
            .obstacles
            .push(Box::new(Circle::new(pos, CREATED_OBSTACLE_RADIUS)));
        model.lmb_down_previously = true;
    } else if model.lmb_down_previously && app.mouse.buttons.left().is_up() {
        model.lmb_down_previously = false;
    }

    let delta_time = if app.keys.down.contains(&Key::Space) {
        0.0
    } else {
        update.since_last.as_secs_f32() * TIME_SCALE
    };
    let obstacles: Vec<_> = model
        .obstacles
        .iter()
        .map(|i| -> &dyn Obstacle { &**i })
        .collect();

    for i in 0..model.boids.len() {
        let raw_turn = apply_flock_rules(&model.boids[i], &model.boids)
            + attract_to_cursor(app, &model.boids[i])
            + avoid_obstacles(&model.boids[i], &obstacles[..]);
        let turn = raw_turn.clamp(-MAX_TURN_RADIANS_PER_SECOND, MAX_TURN_RADIANS_PER_SECOND);

        let boid = &mut model.boids[i];
        boid.heading += turn * delta_time;

        boid.pos += Vec2::X.rotate(boid.heading) * delta_time * BOID_SPEED;
    }
}

fn attract_to_cursor(app: &App, boid: &Boid) -> f32 {
    let cursor_pos = app.mouse.position();
    let dir = cursor_pos - boid.pos;
    let heading_dir = Vec2::X.rotate(boid.heading);
    let attraction = heading_dir.angle_between(dir) * dir.length() / EXTENT_X;
    attraction * CURSOR_ATTRACTION_STRENGTH
}

fn apply_flock_rules(boid: &Boid, boids: &[Boid]) -> f32 {
    let mut total_turning = 0.0;
    let other_boids: Vec<_> = boids
        .iter()
        .filter(|b| b.id != boid.id && b.pos.distance(boid.pos) <= SENSE_RADIUS)
        .collect();

    if other_boids.is_empty() {
        return 0.0;
    }

    //                  SEPARATION
    //  ===========================================
    let avg_avoidance_dir = other_boids.iter().fold(Vec2::ZERO, |acc, i| {
        let diff = i.pos - boid.pos;
        // println!("diff = {diff:?}");
        acc - diff.normalize() * diff.length_recip()
    });

    if avg_avoidance_dir.length() > 0.0001 {
        let heading_vec = Vec2::X.rotate(boid.heading);
        let turn_amt = heading_vec.angle_between(avg_avoidance_dir)
            * SEP_STRENGTH
            * avg_avoidance_dir.length();

        total_turning += turn_amt;
    }

    if !total_turning.is_finite() {
        return 0.0;
    }

    //                  ALIGNMENT
    //  ===========================================
    let avg_heading = other_boids.iter().map(|i| i.heading).sum::<f32>() / other_boids.len() as f32;

    let target_dir = Vec2::X.rotate(avg_heading);
    let cur_dir = Vec2::X.rotate(boid.heading);
    let diff = cur_dir.angle_between(target_dir);
    total_turning += diff * ALIGN_STRENGTH;

    //                  COHESION
    //  ===========================================
    let avg_nearby_boid_loc =
        other_boids.iter().fold(Vec2::ZERO, |acc, i| acc + i.pos) / other_boids.len() as f32;

    let heading_vec = Vec2::X.rotate(boid.heading);
    let cohesion_turn_amt =
        heading_vec.angle_between(avg_nearby_boid_loc - boid.pos) * COHESION_STRENGTH;

    total_turning += cohesion_turn_amt;

    if !total_turning.is_finite() {
        return 0.0;
    }
    total_turning
}

fn avoid_obstacles(boid: &Boid, obstacles: &[&dyn Obstacle]) -> f32 {
    let total_turn: f32 = (-OBSTACLE_AVOIDANCE_RAYCAST_RES..=OBSTACLE_AVOIDANCE_RAYCAST_RES)
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
            let deflection =
                deflection_angle * nearest_dist.recip() / OBSTACLE_AVOIDANCE_RAYCAST_RES as f32;

            if nearest_dist.is_finite() && DRAW_DEBUG_RAYS {
                let boid_pos = boid.pos;
                queue_drawing(Box::new(move |draw: &Draw| {
                    draw.line()
                        .start(boid_pos)
                        .end(
                            boid_pos
                                + Vec2::X.rotate(angle) * nearest_dist.min(BOID_OBSTACLE_VIEW_DIST),
                        )
                        .color(RED);
                }) as _);
            }
            if nearest_dist < BOID_OBSTACLE_VIEW_DIST {
                deflection
            } else {
                0.0
            }
        })
        .sum();

    -total_turn * WALL_TURN_FACTOR
}

fn model(_app: &App) -> Model {
    Model {
        boids: random_boids(),
        obstacles: walls(),
        lmb_down_previously: false,
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

    // Draw obstacles
    for i in &model.obstacles {
        i.draw(&draw);
    }

    // Draw boids
    for boid in &model.boids {
        draw.tri()
            .w(BOID_SIZE)
            .h(BOID_SIZE)
            .z_radians(boid.heading)
            .xy(boid.pos + Vec2::X.rotate(boid.heading) * 15.0)
            .color(DARKGRAY);
    }

    QUEUED_DRAWINGS
        .get_or_init(|| Arc::new(Mutex::new(vec![])))
        .lock()
        .unwrap()
        .iter()
        .for_each(|i| (*i)(&draw));
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
