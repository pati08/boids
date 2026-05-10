use nannou::{Draw, color::RED, glam::Vec2};

pub trait Obstacle {
    fn ray_test(&self, u: Vec2, v: Vec2) -> f32;
    fn desc(&self) -> String {
        String::from("unnamed obstacle")
    }
    fn draw(&self, draw: &Draw) {}
}

pub struct Circle {
    pos: Vec2,
    radius: f32,
}

impl Circle {
    pub fn new(pos: Vec2, radius: f32) -> Self {
        Self { pos, radius }
    }
}

impl Obstacle for Circle {
    fn ray_test(&self, u: Vec2, v: Vec2) -> f32 {
        let vector_to_center = self.pos - u;

        if vector_to_center.dot(v) <= 0.0 {
            return f32::INFINITY;
        }

        let a = -v.y;
        let b = v.x;
        let c = -(a * u.x + b * u.y);

        let l1 = (a * self.pos.x + b * self.pos.y + c).abs() / (a.powi(2) + b.powi(2)).sqrt();
        if l1 > self.radius {
            return f32::INFINITY;
        }
        let l2 = (self.radius.powi(2) - l1.powi(2)).sqrt();
        let sq_dist_to_center = vector_to_center.length_squared();
        let total_length_to_closest_point = (sq_dist_to_center - l1.powi(1)).sqrt();
        total_length_to_closest_point - l2
    }
    fn desc(&self) -> String {
        format!("circle at {:?} with radius {}", self.pos, self.radius)
    }
    fn draw(&self, draw: &Draw) {
        draw.ellipse().xy(self.pos).radius(self.radius).color(RED);
    }
}

pub enum Wall {
    Vertical { x: f32 },
    Horizontal { y: f32 },
}

impl Wall {
    pub fn hor(y: f32) -> Self {
        Self::Horizontal { y }
    }
    pub fn vert(x: f32) -> Self {
        Self::Vertical { x }
    }
}

impl Obstacle for Wall {
    fn ray_test(&self, u: Vec2, v: Vec2) -> f32 {
        match self {
            Wall::Vertical { x } => {
                let dx = *x - u.x;
                if v.x.signum() != dx.signum() {
                    f32::INFINITY
                } else {
                    dx / v.x
                }
            }
            Wall::Horizontal { y } => {
                let dy = *y - u.y;
                if v.y.signum() != dy.signum() {
                    f32::INFINITY
                } else {
                    dy / v.y
                }
            }
        }
    }
    fn desc(&self) -> String {
        match self {
            Self::Vertical { x } => format!("vertical wall at {x}"),
            Self::Horizontal { y } => format!("horizontal wall at {y}"),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn ray_test() {
        let wall = Wall::vert(10.0);
        let ru = Vec2::new(0.0, 0.0);
        let rv = Vec2::new(1.0, 0.0);
        let dist = wall.ray_test(ru, rv);
        assert_eq!(dist, 10.0);
    }
}
