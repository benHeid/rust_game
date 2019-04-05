use libm;
use rand::prelude::*;
use rand::Rng;
use stm32f7_discovery::{
    lcd::{Color},
};

const IMG: [u8; 30 * 30 * 2] = *include_bytes!("dragonResized.data");
pub static COLORS: [(u8, u8, u8); 5] = [
    (255, 0, 0),
    (0, 128, 0),
    (0, 0, 128),
    (192, 192, 192),
    (255, 255, 0),
];

#[derive(Copy, Clone)]
pub struct Box {
    pub size: u16,
    pub pos: Vector2d,
    pub vel: Vector2d,
    pub col: Color,
}
#[derive(Copy, Clone)]
pub struct Dragon(pub Box);

impl core::ops::Deref for Dragon {
    type Target = Box;
    fn deref(&self) -> &Box {
        &self.0
    }
}

impl core::ops::DerefMut for Dragon {
    fn deref_mut(&mut self) -> &mut Box {
        &mut self.0
    }
}

impl Dragon {
    pub fn render(
        &mut self,
        layer: &mut stm32f7_discovery::lcd::Layer<stm32f7_discovery::lcd::FramebufferArgb8888>,
    ) {
        for y in 0..self.size {
            for x in 0..self.size {
                let i = 20 + x as usize + self.pos.x as usize;
                let j = 20 + y as usize + self.pos.y as usize;
                let wert = IMG[2 * (x + y * 30) as usize + 1];
                if wert > 25 {
                    layer.print_point_color_at(i, j, self.col)
                }
            }
        }
    }
    pub fn hit(&mut self, hit: &Vector2d, background_col: &mut Color) -> (bool, bool) {
        if self.pos.x + 20 <= hit.x && hit.x <= self.pos.x + self.size as i16 + 20 {
            //set derender color of object
            self.col = Color::from_hex(0xffffff);
            if self.pos.y + self.size as i16 + 20 >= hit.y && hit.y >= self.pos.y + 20 {
                if self.col.to_rgb() == background_col.to_rgb() {
                    return (true, true);
                } else {
                    return (true, false);
                }
            }
        }
        (false, false)
    }
}

impl Box {
    pub fn new(size: u16, x: i16, y: i16, vel_x: i16, vel_y: i16, vel_col: Color) -> Self {
        Self {
            size,
            pos: Vector2d { x: x, y: y },
            vel: Vector2d { x: vel_x, y: vel_y },
            col: vel_col,
        }
    }
    pub fn random(rand: &mut StdRng) -> Self {
        let tupel = COLORS[rand.gen_range(0, 4)];
        let color = Color::rgb(tupel.0, tupel.1, tupel.2);
        Self::new(
            30,
            rand.gen_range(60, 400),
            rand.gen_range(60, 200),
            rand.gen_range(-10, 10),
            rand.gen_range(-10, 10),
            color,
        )
    }
    pub fn render(
        &mut self,
        layer: &mut stm32f7_discovery::lcd::Layer<stm32f7_discovery::lcd::FramebufferArgb8888>,
    ) {
         for i in (20 + self.pos.x)..=(20 + self.pos.x + self.size as i16) {
             for j in (20 + self.pos.y)..=(20 + self.pos.y + self.size as i16) {
                 layer.print_point_color_at(i as usize, j as usize, Color::rgb(IMG[2*i as usize+j as usize], 0, 0))
             }
         }
    }

    pub fn derender(
        &mut self,
        layer: &mut stm32f7_discovery::lcd::Layer<stm32f7_discovery::lcd::FramebufferArgb8888>,
        bg_color: Color,
    ) {
        for i in (20 + self.pos.x)..(20 + self.pos.x + self.size as i16) {
            for j in (20 + self.pos.y)..(20 + self.pos.y + self.size as i16) {
                layer.print_point_color_at(i as usize, j as usize, bg_color)
            }
        }
    }

    pub fn hit(&mut self, hit: &Vector2d) -> bool {
        if self.pos.x + 20 <= hit.x && hit.x <= self.pos.x + self.size as i16 + 20 {
            //set derender color of object
            self.col = Color::from_hex(0xffffff);
            if self.pos.y + self.size as i16 + 20 >= hit.y && hit.y >= self.pos.y + 20 {
                return true;
            }
        }
        return false;
    }

    pub fn collision(&self, b: &Box) -> Option<Vector2d> {
        if self.intersect(b) {
            let normal_vector = Vector2d{x:self.pos.x - b.pos.x, y:self.pos.y - b.pos.y}; 
            let inner_product_vel = (self.vel.x - b.vel.x) * normal_vector.x + (self.vel.y - b.vel.y) * normal_vector.y;
            let squared_dist = normal_vector.x * normal_vector.x + normal_vector.y * normal_vector.y;


            return Some(Vector2d {
                x: self.vel.x - libm::ceilf(inner_product_vel as f32 /squared_dist as f32 * normal_vector.x as f32) as i16,
                y: self.vel.y - libm::ceilf(inner_product_vel as f32 /squared_dist as f32 * normal_vector.y as f32) as i16,
            });
        } else {
                return None;
        }
    }

    pub fn intersect(&self, b: &Box) -> bool {
        if self.pos.x == b.pos.x
            && self.pos.y == b.pos.y
            && self.vel.x == b.vel.x
            && self.vel.y == b.vel.y
            && self.size == b.size
        {
            return false;
        }
        
        ((self.pos.x - b.pos.x).abs() < (self.size as i16))
            && ((self.pos.y - b.pos.y).abs()
                < (self.size as i16))
    }

    pub fn update_speed(&mut self, new_speed: &Vector2d) {
        self.vel = *new_speed;
        if self.vel.x > 10 {
            self.vel.x = 10;
        }
        if self.vel.y > 10 {
            self.vel.y = 10;
        }
    }


    pub fn next(&mut self) {
        let height = 252;
        let width = 460;
        let offset = 20;
        if offset + self.size as i16 + self.pos.x as i16 + self.vel.x > width {
            self.vel.x *= -1;
        }
        if offset + self.pos.x as i16 + self.vel.x < 20 {
            self.vel.x *= -1;
        }
        if offset + self.size as i16 + self.pos.y as i16 + self.vel.y > height {
            self.vel.y *= -1;
        }
        if offset + self.pos.y as i16 + self.vel.y < 20 {
            self.vel.y *= -1;
        }
        self.pos.x += self.vel.x;
        self.pos.y += self.vel.y;
    }
}

impl PartialEq for Box {
    fn eq(&self, b: &Box) -> bool {
        if self.pos.x == b.pos.x
            && self.pos.y == b.pos.y
            && self.vel.x == b.vel.x
            && self.vel.y == b.vel.y
            && self.size == b.size
        {
            return true;
        }
        false
    }
}

#[derive(Copy, Clone)]
pub struct Vector2d {
    pub x: i16,
    pub y: i16,
}
