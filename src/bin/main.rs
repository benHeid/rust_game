#![no_std]
#![no_main]
#![feature(alloc_error_handler)]
#![feature(alloc)]
#![feature(vec_remove_item)]

#[macro_use]
extern crate stm32f7;
#[macro_use]
extern crate stm32f7_discovery;
#[macro_use]
extern crate alloc;

const max_simultaneous_dragons_on_screen: u8 = 15;

use alloc_cortex_m::CortexMHeap;
use core::alloc::Layout as AllocLayout;
use core::panic::PanicInfo;
use cortex_m_rt::{entry, exception};
use math;
use rand::Rng;
use rand::SeedableRng;
use stm32f7::stm32f7x6::{CorePeripherals, Peripherals};
use stm32f7_discovery::{
    gpio::{GpioPort, OutputPin},
    init,
    lcd::{self, Color, TextWriter},
    system_clock::{self, Hz},
    touch,
};

mod dragons;
use dragons::Box;
use dragons::Vector2d;

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
fn panic(info: &PanicInfo) -> ! {
    loop {}
}

#[entry]
fn main() -> ! {
    let core_peripherals = CorePeripherals::take().unwrap();
    let mut systick = core_peripherals.SYST;
    let mut nvic = core_peripherals.NVIC;

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

    let mut number_of_dragons = 0;
    let mut last_dragon_create = system_clock::ticks();
    // let mut rand = rand::rngs::StdRng::seed_from_u64(318273678346);
    let mut boxes: alloc::vec::Vec<Box> = vec![];
    let mut counter = 5;
    let mut last_led_toggle = system_clock::ticks();
    let mut last_render = system_clock::ticks();
    let mut last_second = system_clock::ticks();
            let mut rand = rand::rngs::StdRng::seed_from_u64(318273678346);
    while number_of_dragons <= 8 {
        let dragon = Box::random(&mut rand);
        let mut intersect = false;
        for d in &boxes.clone() {
            if d.intersect(&dragon.clone()) {
                intersect = true;
            }
        }
        if !intersect {
            boxes.push(dragon);
            number_of_dragons = number_of_dragons + 1;
        }
    }
    let OFFSET = 20;
    let HEIGHT = 252;
    let WIDTH = 460;
    for i in OFFSET..=WIDTH {
        for j in OFFSET..=HEIGHT {
            layer_1.print_point_color_at(i, j, Color::from_hex(0xffffff));
        }
    }

    for d in &mut boxes {
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
    let mut seconds = 0;
    print!(
        "\r           {} seconds left                                          ",
        counter
    );
    loop {
        loop {
            let ticks = system_clock::ticks();
            //every half seconds roll for dragon creation
            if ticks - last_dragon_create >= 10 {
                if number_of_dragons < max_simultaneous_dragons_on_screen {
                    //add dragon
                    let dragon = Box::random(&mut rand);
                    let mut intersect = false;
                    for d in &boxes.clone() {
                        if d.intersect(&dragon.clone()) {
                            intersect = true;
                        }
                    }
                    if !intersect {
                        boxes.push(dragon);
                        number_of_dragons = number_of_dragons + 1;
                    }
                    //reset timer
                    last_dragon_create = ticks;
                }
            }
            if ticks - last_second >= 20 {
                counter -= 1;
                seconds += 1;
                last_second = ticks;
                print!(
                    "\r           {} seconds left                                          ",
                    counter
                );
            }
            if ticks - last_render >= 1 {
                for b in &mut boxes {
                    let b: &mut Box = b;
                    b.derender(&mut layer_1, Color::from_hex(0xffffff));
                    b.next();
                    b.render(&mut layer_1);
                }
                let mut updates: alloc::vec::Vec<Vector2d> = vec![];
                for b in &boxes {
                    let mut collision = false;
                    for bb in &boxes {
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
                for i in 0..boxes.len() {
                    boxes[i].update_speed(&updates[i]);
                }
                last_render = ticks;
            }

            if ticks - last_led_toggle >= 30 {
                if counter % 2 == 0 {
                    lcd.set_background_color(Color::from_hex(0x006600));
                } else {
                    lcd.set_background_color(Color::from_hex(0x000066));
                }
                counter += 1;
                last_led_toggle = ticks;
            }
            // poll for new touch data
            for touch in &touch::touches(&mut i2c_3).unwrap() {
                //type cast for lcd
                let t;
                t = Vector2d {
                    x: touch.x as i16,
                    y: touch.y as i16,
                };
                let mut remove: alloc::vec::Vec<Box> = alloc::vec::Vec::new();
                for b in &mut boxes {
                    let b: &mut Box = b;
                    if b.hit(&t) {
                        b.derender(&mut layer_1, Color::from_hex(0xffffff));
                        remove.push(b.clone());
                        lcd.set_background_color(Color::from_hex(0x000066));
                    }
                }
                for b in remove {
                    boxes.remove_item(&b);
                }
            }

            if counter < 0 {
                break;
            }
        }
        layer_1.clear();
        boxes.clear();
        let mut b = Box::new(80, 240, 100, 0, 0, Color::from_hex(0x660000));
        print!(
            "\rYour survived for {} seconds, hit button for new turn",
            seconds
        );
        b.render(&mut layer_1);
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
                    layer_1.clear();
                    new_game = true;
                    break;
                }
            }
        }
        for i in OFFSET..=WIDTH {
            for j in OFFSET..=HEIGHT {
                layer_1.print_point_color_at(i, j, Color::from_hex(0xffffff));
            }
        }
        counter = 5;
        seconds = 0;
    }
}
