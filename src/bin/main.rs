#![no_std]
#![no_main]
#![feature(alloc_error_handler)]
#![feature(alloc)]
#![feature(vec_remove_item)]
#![warn(clippy::all)]

extern crate stm32f7;
#[macro_use]
extern crate stm32f7_discovery;
#[macro_use]
extern crate alloc;

const MAX_SIMULTANEOUS_DRAGONS_ON_SCREEN: usize = 15;
const COVER_SCREEN: [u8; 480 * 272 * 4] = *include_bytes!("cover_screen.data");

use alloc_cortex_m::CortexMHeap;
use core::alloc::Layout as AllocLayout;
use core::panic::PanicInfo;
use cortex_m_rt::{entry, exception}; 
use dragons::Dragon;
use dragons::COLORS;
use rand::Rng;
use rand::SeedableRng;
use stm32f7::stm32f7x6::{CorePeripherals, Peripherals};
use stm32f7_discovery::{
    gpio::{GpioPort, OutputPin},
    init,
    lcd::{self, Color},
    system_clock::{self, Hz},
    touch,
};

mod dragons;
use dragons::Box;
use dragons::Circle;
use dragons::Vector2d;

const OFFSET: usize = 20;
const HEIGHT: usize = 252;
const WIDTH: usize = 460;

#[global_allocator]
static ALLOCATOR: CortexMHeap = CortexMHeap::empty();
const HEAP_SIZE: usize = 50 * 1024; // in bytes

#[alloc_error_handler]
fn rust_oom(_: AllocLayout) -> ! {
    loop {}
}

#[exception]
fn SysTick() {
    system_clock::tick();
}

#[panic_handler]
fn panic(_info: &PanicInfo) -> ! {
    loop {}
}

#[entry]
fn main() -> ! {
    let core_peripherals = CorePeripherals::take().unwrap();
    let mut systick = core_peripherals.SYST;

    let peripherals = Peripherals::take().unwrap();
    let mut rcc = peripherals.RCC;
    let mut pwr = peripherals.PWR;
    let mut flash = peripherals.FLASH;

    let mut fmc = peripherals.FMC;
    let mut ltdc = peripherals.LTDC;
    let mut sai_2 = peripherals.SAI2;

    init::init_system_clock_216mhz(&mut rcc, &mut pwr, &mut flash);
    init::enable_gpio_ports(&mut rcc);
    //init::enable_syscfg(&mut rcc);

    let gpio_a = GpioPort::new(peripherals.GPIOA);
    let gpio_b = GpioPort::new(peripherals.GPIOB);
    let gpio_c = GpioPort::new(peripherals.GPIOC);
    let gpio_d = GpioPort::new(peripherals.GPIOD);
    let gpio_e = GpioPort::new(peripherals.GPIOE);
    let gpio_f = GpioPort::new(peripherals.GPIOF);
    let gpio_g = GpioPort::new(peripherals.GPIOG);
    let gpio_h = GpioPort::new(peripherals.GPIOH);
    let gpio_i = GpioPort::new(peripherals.GPIOI);
    let gpio_j = GpioPort::new(peripherals.GPIOJ);
    let gpio_k = GpioPort::new(peripherals.GPIOK);
    let mut pins = init::pins(
        gpio_a, gpio_b, gpio_c, gpio_d, gpio_e, gpio_f, gpio_g, gpio_h, gpio_i, gpio_j, gpio_k,
    );

    // configures the system timer to trigger a SysTick exception every 10ms
    init::init_systick(Hz(20), &mut systick, &rcc);
    systick.enable_interrupt();

    init::init_sdram(&mut rcc, &mut fmc);
    let mut lcd = init::init_lcd(&mut ltdc, &mut rcc);
    pins.display_enable.set(true);
    pins.backlight.set(true);
    //
    //// Initialize the allocator BEFORE you use it
    unsafe { ALLOCATOR.init(cortex_m_rt::heap_start() as usize, HEAP_SIZE) }

    let mut layer_1 = lcd.layer_1().unwrap();
    let mut layer_2 = lcd.layer_2().unwrap();
    layer_1.clear();
    layer_2.clear();
    //init variables for touch interactions
    let mut i2c_3 = init::init_i2c_3(peripherals.I2C3, &mut rcc);
    i2c_3.test_1();
    i2c_3.test_2();
    init::init_sai_2(&mut sai_2, &mut rcc);
    init::init_wm8994(&mut i2c_3).expect("WM8994 init failed");
    // touch initialization should be done after audio initialization, because the touch
    // controller might not be ready yet
    touch::check_family_id(&mut i2c_3).unwrap();
    draw_cover_screen(&mut layer_1, &mut i2c_3);
    layer_1.clear();
    layer_2.clear();
    lcd::init_stdout(layer_2);
    let mut last_dragon_create = system_clock::ticks();
    let mut dragons: alloc::vec::Vec<Dragon> = vec![];
    let mut _last_second = system_clock::ticks();
    let mut rand = rand::rngs::StdRng::seed_from_u64(318_273_678_346);
    let mut rgb_tupel = COLORS[rand.gen_range(0, 4)];
    let mut edge_color = Color::rgb(rgb_tupel.0, rgb_tupel.1, rgb_tupel.2);
    lcd.set_background_color(edge_color);
    let mut played_time_in_seconds = 0;
    let mut left_time_in_seconds = 0;

    initiate_screen(
        &mut played_time_in_seconds,
        &mut left_time_in_seconds,
        &mut dragons,
        &mut rand,
        &mut layer_1,
    );

    let mut last_render = system_clock::ticks();
    let mut last_second = system_clock::ticks();
    for d in &mut dragons {
        d.render(&mut layer_1);
    }

    loop {
        loop {
            let ticks = system_clock::ticks();
            if ticks - last_second >= 20 {
                left_time_in_seconds -= 1;
                played_time_in_seconds += 1;
                last_second = ticks;
                print!(
                    "\r           {} seconds left                   ",
                    left_time_in_seconds
                );
            }
            //evry half seconds roll for dragon creation
            if ticks - last_dragon_create >= 10
                && dragons.len() < MAX_SIMULTANEOUS_DRAGONS_ON_SCREEN
            {
                //add dragon
                let dragon = Dragon(Circle::random(&mut rand));
                let mut intersect = false;
                for d in &dragons {
                    if d.intersect(&dragon) {
                        intersect = true;
                        break;
                    }
                }
                if !intersect {
                    dragons.push(dragon);
                }

                //reset timer
                last_dragon_create = ticks;
            }

            if ticks - last_render >= 1 {
                let mut updates: alloc::vec::Vec<Vector2d> = vec![];
                for dragon in &dragons {
                    let mut collision = false;
                    for dragon_o in &dragons {
                        let n_speed: Option<Vector2d> = dragon.collision(dragon_o);
                        if !n_speed.is_none() {
                            updates.push(n_speed.unwrap());
                            collision = true;
                            break;
                        }
                    }
                    if !collision {
                        updates.push(dragon.vel);
                    }
                }
                for i in 0..dragons.len() {
                    dragons[i].update_speed(updates[i]);
                }
                for dragon in &mut dragons {
                    dragon.derender(&mut layer_1, Color::from_hex(0x00ff_ffff));
                    dragon.next();
                    dragon.render(&mut layer_1);
                }
                last_render = ticks;
            }
            for touch in &touch::touches(&mut i2c_3).unwrap() {
                //type cast for lcd
                let t = Vector2d {
                    x: touch.x as i16,
                    y: touch.y as i16,
                };
                let mut remove = alloc::vec::Vec::new();
                // let mut remove: alloc::vec::Vec<usize> = alloc::vec::Vec::new();
                for (i, d) in dragons.iter_mut().enumerate() {
                    let (hit, correct) = d.hit(t, &mut edge_color);
                    if hit {
                        remove.push(i);
                        if correct {
                            //matching hit with edge color
                            left_time_in_seconds += 3;
                        } else {
                            //non matching hit
                            left_time_in_seconds -= 1;
                        }
                    }
                }
                for i in remove.iter().rev() {
                    dragons
                        .remove(*i)
                        .derender(&mut layer_1, Color::from_hex(0x00ff_ffff));
                    let mut new_edge_color = edge_color;
                    while new_edge_color.to_rgb() == edge_color.to_rgb() {
                        rgb_tupel = COLORS[rand.gen_range(0, 4)];
                        new_edge_color = Color::rgb(rgb_tupel.0, rgb_tupel.1, rgb_tupel.2);
                    }
                    edge_color = new_edge_color;
                    lcd.set_background_color(edge_color);
                }
            }

            if left_time_in_seconds == 0 {
                break;
            }
        }
        dragons.clear();
        game_over(&mut layer_1, played_time_in_seconds, &mut i2c_3);
        initiate_screen(
            &mut played_time_in_seconds,
            &mut left_time_in_seconds,
            &mut dragons,
            &mut rand,
            &mut layer_1,
        );
    }
}

fn game_over(
    mut layer: &mut stm32f7_discovery::lcd::Layer<stm32f7_discovery::lcd::FramebufferArgb8888>,
    played_time_in_seconds: u8,
    mut i2c_3: &mut stm32f7_discovery::i2c::I2C<stm32f7::stm32f7x6::I2C3>,
) {
    layer.clear();
    for y in 0..=272 {
        for x in 0..=480 {
            let i = x as usize;
            let j = y as usize;
            let r = COVER_SCREEN[4 * (x + y * 480) as usize];
            let g = COVER_SCREEN[4 * (x + y * 480) as usize + 1];
            let b = COVER_SCREEN[4 * (x + y * 480) as usize + 2];
            layer.print_point_color_at(i, j, Color::rgb(255, g, b));
        }
    }

    let mut score = Box::new(280, 60, 100, 60, Color::from_hex(0x00ff_ffff));
    score.render(&mut layer);
    assert_eq!(
        score
            .write_str(&format!("Your Score is {}",played_time_in_seconds), layer, Color::from_hex(0x0000_0000))
            .err(),
        None
    );

    let mut try_again_button = Box::new(280, 60, 100, 160, Color::from_hex(0x00ff_ffff));
    try_again_button.render(&mut layer);
    assert_eq!(
        try_again_button
            .write_str("try again...   :)", layer, Color::from_hex(0x0000_0000))
            .err(),
        None
    );


    let mut new_game = false;
    while !new_game {
        for touch in &touch::touches(&mut i2c_3).unwrap() {
            //type cast for lcd
            let t = Vector2d {
                x: touch.x as i16,
                y: touch.y as i16,
            };
            if try_again_button.hit(t) {
                try_again_button.derender(layer, Color::from_hex(0x00ff_ffff));
                new_game = true;
                break;
            }
        }
    }
}

fn initiate_screen(
    played_time_in_seconds: &mut u8,
    left_time_in_seconds: &mut u8,
    dragons: &mut alloc::vec::Vec<Dragon>,
    rand: &mut rand::rngs::StdRng,
    layer_1: &mut stm32f7_discovery::lcd::Layer<stm32f7_discovery::lcd::FramebufferArgb8888>,
) {
    layer_1.clear();
    for i in OFFSET..=WIDTH {
        for j in OFFSET..=HEIGHT {
            layer_1.print_point_color_at(i, j, Color::from_hex(0x00ff_ffff));
        }
    }
    *played_time_in_seconds = 0;
    *left_time_in_seconds = 20;
    dragons.clear();
    while dragons.len() <= 8 {
        let dragon = Dragon(Circle::random(&mut *rand));
        let intersect = dragons.iter().any(|d| d.intersect(&*dragon));
        if !intersect {
            dragons.push(dragon);
        }
    }
}

fn draw_cover_screen(
    layer: &mut stm32f7_discovery::lcd::Layer<stm32f7_discovery::lcd::FramebufferArgb8888>,
    mut i2c_3: &mut stm32f7_discovery::i2c::I2C<stm32f7::stm32f7x6::I2C3>,
) {

    //wait for input
    for y in 0..=272 {
        for x in 0..=480 {
            let i = x as usize;
            let j = y as usize;
            let r = COVER_SCREEN[4 * (x + y * 480) as usize];
            let g = COVER_SCREEN[4 * (x + y * 480) as usize + 1];
            let b = COVER_SCREEN[4 * (x + y * 480) as usize + 2];
            layer.print_point_color_at(i, j, Color::rgb(r, g, b));
        }
    }
    let mut title = Box::new(240, 60, 120, 100, Color::from_hex(0x00ff_ffff));
    title.render(layer);
    assert_eq!(
        title
            .write_str("Dragon Slayer", layer, Color::from_hex(0x0000_0000))
            .err(),
        None
    );
    loop {
        if touch::touches(&mut i2c_3).unwrap().len() > 0 {
            break;
        }
    }
}
