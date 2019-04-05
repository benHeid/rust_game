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

const MAX_SIMULTANEOUS_DRAGONS_ON_SCREEN: u8 = 15;
const GAME_OVER: [u8; 300 * 75 * 4] = *include_bytes!("game_over.data");

use crate::dragons::Dragon;
use crate::dragons::COLORS;
use alloc_cortex_m::CortexMHeap;
use core::alloc::Layout as AllocLayout;
use core::panic::PanicInfo;
use cortex_m_rt::{entry, exception};
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
    //let mut rng = peripherals.RNG;
    //let mut sdmmc = peripherals.SDMMC1;
    //let syscfg = peripherals.SYSCFG;
    //let ethernet_mac = peripherals.ETHERNET_MAC;
    //let ethernet_dma = peripherals.ETHERNET_DMA;
    //let mut nvic_stir = peripherals.NVIC_STIR;
    //let mut tim6 = peripherals.TIM6;
    //let exti = peripherals.EXTI;

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
    //lcd.set_background_color(Color::from_hex(0x660000));

    let mut layer_1 = lcd.layer_1().unwrap();
    let mut layer_2 = lcd.layer_2().unwrap();
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

    //init variables for touch interactions
    let mut i2c_3 = init::init_i2c_3(peripherals.I2C3, &mut rcc);
    i2c_3.test_1();
    i2c_3.test_2();
    init::init_sai_2(&mut sai_2, &mut rcc);
    init::init_wm8994(&mut i2c_3).expect("WM8994 init failed");
    // touch initialization should be done after audio initialization, because the touch
    // controller might not be ready yet
    touch::check_family_id(&mut i2c_3).unwrap();
    loop {
        loop {
            let ticks = system_clock::ticks();
            if ticks - last_second >= 20 {
                left_time_in_seconds -= 1;
                played_time_in_seconds += 1;
                last_second = ticks;
                print!(
                    "\r           {} seconds left                                          ",
                    left_time_in_seconds
                );
            }
            //evry half seconds roll for dragon creation
            if ticks - last_dragon_create >= 10
                && dragons.len() < MAX_SIMULTANEOUS_DRAGONS_ON_SCREEN as usize
            {
                //add dragon
                let dragon = Dragon(Box::random(&mut rand));
                let mut intersect = false;
                for d in &dragons.clone() {
                    if d.intersect(&dragon) {
                        intersect = true;
                    }

                    if !intersect {
                        dragons.push(dragon);
                    }
                }
                //reset timer
                last_dragon_create = ticks;
            }

            if ticks - last_render >= 1 {
                for b in &mut dragons {
                    let b: &mut Dragon = b;
                    b.derender(&mut layer_1, Color::from_hex(0x00ff_ffff));
                    b.next();
                    b.render(&mut layer_1);
                }
                let mut updates: alloc::vec::Vec<Vector2d> = vec![];
                for b in &dragons {
                    let mut collision = false;
                    for bb in &dragons {
                        let n_speed: Option<Vector2d> = b.collision(bb);
                        if !n_speed.is_none() {
                            updates.push(n_speed.unwrap());
                            collision = true;
                            break;
                        }
                    }
                    if !collision {
                        updates.push(b.vel);
                    }
                }
                for i in 0..dragons.len() {
                    dragons[i].update_speed(&updates[i]);
                }
                last_render = ticks;
            }

            // if ticks - last_led_toggle >= 30 {
            //     if counter % 2 == 0 {
            //         lcd.set_background_color(Color::from_hex(0x006600));
            //     } else {
            //         lcd.set_background_color(Color::from_hex(0x000066));
            //     }
            //     counter += 1;
            //     last_led_toggle = ticks;
            // }
            // poll for new touch data
            for touch in &touch::touches(&mut i2c_3).unwrap() {
                //type cast for lcd
                let t;
                t = Vector2d {
                    x: touch.x as i16,
                    y: touch.y as i16,
                };
                let mut remove = alloc::vec::Vec::new();
                // let mut remove: alloc::vec::Vec<usize> = alloc::vec::Vec::new();
                for (i, d) in dragons.iter_mut().enumerate() {
                    let (a, b) = d.hit(t, &mut edge_color);
                    if a {
                        remove.push(i);
                        if b {
                            //matching hit with edge color
                            left_time_in_seconds += 1;
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
                    lcd.set_background_color(new_edge_color);
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
    let mut b = Box::new(80, 240, 100, 0, 0, Color::from_hex(0x0066_0000));
    print!(
        "\rYour survived for {} seconds, hit button for new turn",
        played_time_in_seconds
    );
    for y in 0..75 {
        for x in 0..300 {
            let i = 90 + x as usize;
            let j = 10 + y as usize;
            let wert = GAME_OVER[4 * (x + y * 300) as usize];
            if wert < 25 {
                layer.print_point_color_at(i, j, Color::from_hex(0x00ff_0000))
            }
        }
    }
    b.render(&mut layer);
    let mut new_game = false;
    while !new_game {
        for touch in &touch::touches(&mut i2c_3).unwrap() {
            //type cast for lcd
            let t;
            t = Vector2d {
                x: touch.x as i16,
                y: touch.y as i16,
            };
            if b.hit(&t) {
                layer.clear();
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
    for i in OFFSET..=WIDTH {
        for j in OFFSET..=HEIGHT {
            layer_1.print_point_color_at(i, j, Color::from_hex(0x00ff_ffff));
        }
    }
    *played_time_in_seconds = 0;
    *left_time_in_seconds = 20;
    dragons.clear();
    while dragons.len() <= 8 {
        let dragon = Dragon(Box::random(&mut *rand));
        let intersect = dragons.iter().any(|d| d.intersect(&*dragon));
        if !intersect {
            dragons.push(dragon);
        }
    }
}
