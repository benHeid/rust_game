#![no_std]

use core::fmt;
use libm;
use rand::prelude::*;
use rand::Rng;
use stm32f7_discovery::lcd::Color;
use stm32f7_discovery::lcd::{HEIGHT, WIDTH};
const DRAGON: [u8; 30 * 30 * 2] = *include_bytes!("dragon_image.data");

const OFFSET: i16 = 20;
pub static COLORS: [(u8, u8, u8); 5] = [
    (255, 0, 0),
    (0, 128, 0),
    (0, 0, 128),
    (255, 140, 0),
    (255, 255, 0),
];

#[derive(Copy, Clone)]
pub struct Circle {
    pub radius: u16,
    pub pos: Vector2d,
    pub vel: Vector2d,
    pub col: Color,
}

#[derive(Copy, Clone)]
pub struct Box {
    pub size: Vector2d,
    pub pos: Vector2d,
    pub col: Color,
}

#[derive(Copy, Clone)]
pub struct Dragon(pub Circle);

impl core::ops::Deref for Dragon {
    type Target = Circle;
    fn deref(&self) -> &Circle {
        &self.0
    }
}

impl core::ops::DerefMut for Dragon {
    fn deref_mut(&mut self) -> &mut Circle {
        &mut self.0
    }
}

impl Dragon {
    pub fn render(
        &mut self,
        layer: &mut stm32f7_discovery::lcd::Layer<stm32f7_discovery::lcd::FramebufferArgb8888>,
    ) {
        for y in 0..2 * self.radius {
            for x in 0..2 * self.radius {
                let i = OFFSET as usize + x as usize + self.pos.x as usize - self.radius as usize;
                let j = OFFSET as usize + y as usize + self.pos.y as usize - self.radius as usize;
                let value = DRAGON[2 * (x + y * 30) as usize + 1];
                if value > 25 {
                    layer.print_point_color_at(i, j, self.col)
                }
            }
        }
    }
    pub fn derender(
        &mut self,
        layer: &mut stm32f7_discovery::lcd::Layer<stm32f7_discovery::lcd::FramebufferArgb8888>,
        bg_color: Color,
    ) {
        for i in (OFFSET + self.pos.x)..(OFFSET + self.pos.x + 2 * self.radius as i16) {
            for j in (OFFSET + self.pos.y)..(OFFSET + self.pos.y + 2 * self.radius as i16) {
                layer.print_point_color_at(
                    i as usize - self.radius as usize,
                    j as usize - self.radius as usize,
                    bg_color,
                )
            }
        }
    }

    pub fn hit(&mut self, hit: Vector2d, background_col: &mut Color) -> (bool, bool) {
        if i32::from(hit.x - self.pos.x - OFFSET) * i32::from(hit.x - self.pos.x - OFFSET)
            + i32::from(hit.y - self.pos.y - OFFSET) * i32::from(hit.y - self.pos.y - OFFSET)
            < i32::from(self.radius) * i32::from(self.radius)
        {
            if self.col.to_rgb() == background_col.to_rgb() {
                return (true, true);
            } else {
                return (true, false);
            }
        }
        (false, false)
    }
}

impl Circle {
    pub fn new(radius: u16, x: i16, y: i16, vel_x: i16, vel_y: i16, vel_col: Color) -> Self {
        Self {
            radius,
            pos: Vector2d { x, y },
            vel: Vector2d { x: vel_x, y: vel_y },
            col: vel_col,
        }
    }
    // pub fn render(
    //     &mut self,
    //     layer: &mut stm32f7_discovery::lcd::Layer<stm32f7_discovery::lcd::FramebufferArgb8888>,
    // ) {
    //     for i in (0)..= self.radius as i16 {
    //         for j in (0)..= self.radius as i16 {
    //             if i * i + j * j <= (self.radius * self.radius) as i16 {
    //                 layer.print_point_color_at(OFFSET + self.pos.x as usize + i as usize, OFFSET + self.pos.y as usize + j as usize, self.col);
    //                 layer.print_point_color_at(OFFSET + self.pos.x as usize - i as usize, OFFSET + self.pos.y as usize + j as usize, self.col);
    //                 layer.print_point_color_at(OFFSET + self.pos.x as usize + i as usize, OFFSET + self.pos.y as usize - j as usize, self.col);
    //                 layer.print_point_color_at(OFFSET + self.pos.x as usize - i as usize, OFFSET + self.pos.y as usize - j as usize, self.col);
    //             }
    //         }
    //     }
    // }
    //  pub fn derender(
    //     &mut self,
    //     layer: &mut stm32f7_discovery::lcd::Layer<stm32f7_discovery::lcd::FramebufferArgb8888>,
    //     bg_color: Color,
    // ) {
    //     for i in (0)..=(0 + self.radius as i16) {
    //         for j in (0)..=(0 + self.radius as i16) {
    //             if i * i + j * j <= (self.radius * self.radius) as i16 {
    //                 layer.print_point_color_at(OFFSET + self.pos.x as usize + i as usize, OFFSET + self.pos.y as usize + j as usize, bg_color);
    //                 layer.print_point_color_at(OFFSET + self.pos.x as usize - i as usize, OFFSET + self.pos.y as usize + j as usize, bg_color);
    //                 layer.print_point_color_at(OFFSET + self.pos.x as usize + i as usize, OFFSET + self.pos.y as usize - j as usize, bg_color);
    //                 layer.print_point_color_at(OFFSET + self.pos.x as usize - i as usize, OFFSET + self.pos.y as usize - j as usize, bg_color);
    //             }
    //         }
    //     }
    // }

    pub fn collision(&self, c: &Circle) -> Option<Vector2d> {
        if self.intersect(c) {
            let normal_vector = Vector2d {
                x: self.pos.x - c.pos.x,
                y: self.pos.y - c.pos.y,
            };
            let inner_product_vel =
                (self.vel.x - c.vel.x) * normal_vector.x + (self.vel.y - c.vel.y) * normal_vector.y;
            let squared_dist =
                normal_vector.x * normal_vector.x + normal_vector.y * normal_vector.y;

            Some(Vector2d {
                x: self.vel.x
                    - libm::ceilf(
                        f32::from(inner_product_vel) / f32::from(squared_dist)
                            * f32::from(normal_vector.x),
                    ) as i16,
                y: self.vel.y
                    - libm::ceilf(
                        f32::from(inner_product_vel) / f32::from(squared_dist)
                            * f32::from(normal_vector.y),
                    ) as i16,
            })
        } else {
            None
        }
    }

    fn _get_next(&self) -> (Vector2d, Vector2d) {
        let mut pos_x = self.pos.x + self.vel.x;
        let mut pos_y = self.pos.y + self.vel.y;
        let mut vel_x = self.vel.x;
        let mut vel_y = self.vel.y;
        if self.radius as i16 + pos_x as i16 > WIDTH as i16 - 2 * OFFSET {
            pos_x = WIDTH as i16 - 2 * OFFSET - self.radius as i16;
            vel_x *= -1;
        }
        if pos_x - (self.radius as i16) < 0 {
            pos_x = self.radius as i16;
            vel_x *= -1;
        }
        if self.radius as i16 + pos_y > HEIGHT as i16 - 2 * OFFSET {
            pos_y = HEIGHT as i16 - 2 * OFFSET - self.radius as i16;
            vel_y *= -1;
        }
        if pos_y - (self.radius as i16) < 0 {
            pos_y = self.radius as i16;
            vel_y *= -1;
        }
        (
            Vector2d { x: pos_x, y: pos_y },
            Vector2d { x: vel_x, y: vel_y },
        )
    }

    pub fn intersect(&self, b: &Circle) -> bool {
        if self.pos.x == b.pos.x
            && self.pos.y == b.pos.y
            && self.vel.x == b.vel.x
            && self.vel.y == b.vel.y
            && self.radius == b.radius
        {
            return false;
        }
        let pos_self = self._get_next().0;
        let pos_b = b._get_next().0;

        i32::from(pos_b.x - pos_self.x) * i32::from(pos_b.x - pos_self.x)
            + i32::from(pos_b.y - pos_self.y) * i32::from(pos_b.y - pos_self.y)
            < i32::from(b.radius + self.radius) * i32::from(b.radius + self.radius) + 9
    }

    pub fn update_speed(&mut self, new_speed: Vector2d) {
        self.vel = new_speed;
        if self.vel.x > 10 {
            self.vel.x = 10;
        }
        if self.vel.y > 10 {
            self.vel.y = 10;
        }
        if self.vel.x < -10 {
            self.vel.x = -10;
        }
        if self.vel.y < -10 {
            self.vel.y = -10;
        }
    }

    pub fn next(&mut self) {
        let next = self._get_next();
        self.pos = next.0;
        self.vel = next.1;
    }

    pub fn random(rand: &mut StdRng) -> Self {
        let tupel = COLORS[rand.gen_range(0, 4)];
        let color = Color::rgb(tupel.0, tupel.1, tupel.2);
        Self::new(
            15,
            rand.gen_range(60, 400),
            rand.gen_range(60, 200),
            rand.gen_range(-10, 10),
            rand.gen_range(-10, 10),
            color,
        )
    }
}

impl Box {
    pub fn new(size_x: i16, size_y: i16, x: i16, y: i16, col: Color) -> Self {
        Self {
            size: Vector2d {
                x: size_x,
                y: size_y,
            },
            pos: Vector2d { x, y },
            col,
        }
    }

    pub fn text_box(size: (i16, i16), pos: (i16, i16), col: Color, text:&str, text_col:Color, layer: &mut stm32f7_discovery::lcd::Layer<stm32f7_discovery::lcd::FramebufferArgb8888>) -> Self {
        let mut text_box = Box::new(size.0, size.1, pos.0, pos.1, col);
        text_box.render(layer);
        assert_eq!(text_box.write_str(text, layer, text_col).err(), None);
        text_box
    }

    pub fn render(
        &mut self,
        layer: &mut stm32f7_discovery::lcd::Layer<stm32f7_discovery::lcd::FramebufferArgb8888>,
    ) {
        for i in (self.pos.x)..=(self.pos.x + self.size.x as i16) {
            for j in (self.pos.y)..=(self.pos.y + self.size.y as i16) {
                layer.print_point_color_at(i as usize, j as usize, self.col)
            }
        }
    }

    pub fn derender(
        &mut self,
        layer: &mut stm32f7_discovery::lcd::Layer<stm32f7_discovery::lcd::FramebufferArgb8888>,
        bg_color: Color,
    ) {
        for i in (self.pos.x)..(self.pos.x + self.size.x as i16) {
            for j in (self.pos.y)..(self.pos.y + self.size.y as i16) {
                layer.print_point_color_at(i as usize, j as usize, bg_color)
            }
        }
    }

    pub fn hit(&mut self, hit: Vector2d) -> bool {
        if self.pos.x <= hit.x && hit.x <= self.pos.x + self.size.x as i16 {
            //set derender color of object
            self.col = Color::from_hex(0x00ff_ffff);
            if self.pos.y + self.size.y as i16 >= hit.y && hit.y >= self.pos.y {
                return true;
            }
        }
        false
    }

    pub fn write_str(
        &mut self,
        s: &str,
        layer: &mut stm32f7_discovery::lcd::Layer<stm32f7_discovery::lcd::FramebufferArgb8888>,
        col: stm32f7_discovery::lcd::Color,
    ) -> fmt::Result {
        use font8x8::{self, UnicodeFonts};
        let string_len = s.len();
        let num_line =
            libm::ceilf(f32::from(string_len as i16 * 16) / f32::from(self.size.x)) as i16;
        let mut pos_x = self.pos.x + 5;
        let mut pos_y = self.pos.y - 8 * num_line / 2 + self.size.y as i16 / 2 - 4;
        for c in s.chars() {
            match c {
                ' '..='~' => {
                    let rendered = font8x8::BASIC_FONTS
                        .get(c)
                        .expect("character not found in basic font");
                    for (y, byte) in rendered.iter().enumerate() {
                        for (x, bit) in (0..8).enumerate() {
                            let color = if *byte & (1 << bit) == 0 {
                                self.col
                            } else {
                                col
                            };
                            layer.print_point_color_at(
                                pos_x as usize + 2 * x,
                                pos_y as usize + 2 * y,
                                color,
                            );
                            layer.print_point_color_at(
                                pos_x as usize + 2 * x + 1,
                                pos_y as usize + 2 * y,
                                color,
                            );
                            layer.print_point_color_at(
                                pos_x as usize + 2 * x,
                                pos_y as usize + 2 * y + 1,
                                color,
                            );
                            layer.print_point_color_at(
                                pos_x as usize + 2 * x + 1,
                                pos_y as usize + 2 * y + 1,
                                color,
                            );
                        }
                    }
                    pos_x += 16;
                    if pos_x > self.pos.x + self.size.x as i16 - 16 {
                        pos_x = self.pos.x + 5;
                        pos_y += 16;
                    }
                }
                _ => panic!("unprintable character"),
            }
        }
        Ok(())
    }
}

#[derive(Copy, Clone)]
pub struct Vector2d {
    pub x: i16,
    pub y: i16,
}
