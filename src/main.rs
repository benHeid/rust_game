#![no_std]
#![no_main]
#![feature(alloc_error_handler)]

#[macro_use]
extern crate stm32f7;
#[macro_use]
extern crate stm32f7_discovery;

use alloc_cortex_m::CortexMHeap;
use core::alloc::Layout as AllocLayout;
use core::panic::PanicInfo;
use cortex_m_rt::{entry, exception};
use stm32f7::stm32f7x6::{CorePeripherals, Peripherals};
use stm32f7_discovery::{
    gpio::{GpioPort, OutputPin},
    init,
    lcd::Color,
    system_clock::{self, Hz},
};

#[global_allocator]
static ALLOCATOR: CortexMHeap = CortexMHeap::empty();

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
    //let mut sai_2 = peripherals.SAI2;
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
    //unsafe { ALLOCATOR.init(rt::heap_start() as usize, HEAP_SIZE) }
    //lcd.set_background_color(Color::from_hex(0x660000));

    let mut layer_1 = lcd.layer_1().unwrap();
    let mut layer_2 = lcd.layer_2().unwrap();
    layer_1.clear();
    layer_2.clear();
    let mut b = Box::new(20, 30, 150, 2, 2);
    let OFFSET = 20;
    let HEIGHT = 252;
    let WIDTH = 460;
    for i in OFFSET..=WIDTH {
        for j in OFFSET..=HEIGHT {
            layer_1.print_point_color_at(i, j, Color::from_hex(0xffffff));
        }
    }

    b.render(&mut layer_1);

    let mut counter = 0;
    let mut last_led_toggle = system_clock::ticks();
    let mut last_render = system_clock::ticks();
    loop {
        let ticks = system_clock::ticks();
        if ticks - last_render >= 1 {
            b.derender(&mut layer_1, Color::from_hex(0xffffff));
            b.next();
            b.render(&mut layer_1);
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
    }
}

struct Box {
    size: u16,
    x: u16,
    y: u16,
    vel_x: i16,
    vel_y: i16,
}

impl Box {
    fn new(size: u16, x: u16, y: u16, vel_x: i16, vel_y: i16) -> Self {
        Self {
            size,
            x,
            y,
            vel_x,
            vel_y,
        }
    }
    fn render(
        &mut self,
        layer: &mut stm32f7_discovery::lcd::Layer<stm32f7_discovery::lcd::FramebufferArgb8888>,
    ) {
        for i in (20 + self.x)..=(20 + self.x + self.size) {
            for j in (20 + self.y)..=(20 + self.y + self.size) {
                layer.print_point_color_at(i as usize, j as usize, Color::from_hex(0x660000))
            }
        }
    }

    fn derender(
        &mut self,
        layer: &mut stm32f7_discovery::lcd::Layer<stm32f7_discovery::lcd::FramebufferArgb8888>,
        bgColor: Color,
    ) {
        for i in (20 + self.x)..=(20 + self.x + self.size) {
            for j in (20 + self.y)..=(20 + self.y + self.size) {
                layer.print_point_color_at(i as usize, j as usize, bgColor)
            }
        }
    }

    fn next(&mut self) {
        let HEIGHT = 252;
        let WIDTH = 460;
        if self.size as i16 * 2 + self.x as i16 + self.vel_x > WIDTH {
            self.vel_x *= -1;
        }
        if self.size as i16 + self.x as i16 + self.vel_x < 20 {
            self.vel_x *= -1;
        }
        if self.size as i16 * 2 + self.y as i16 + self.vel_y > HEIGHT {
            self.vel_y *= -1;
        }
        if self.size as i16 + self.y as i16 + self.vel_y < 20 {
            self.vel_y *= -1;
        }
        self.x += self.vel_x as u16;
        self.y += self.vel_y as u16;
    }
}
