# 🪖 falling-object-helmet-rs 

# 👥 Team Members
- Jonathan Clifton
- Hyrum Coleman
- Jessica Lopez
- Cameron Monson
- Sami Mower

---

# 📦 Description

The *falling-object-helmet-rs* project provides embedded Rust firmware for a smart safety helmet designed to detect and alert users of incoming falling objects. The firmware synchronizes data from a Doppler radar sensor via UART and controls an alert system composed of an LED strip and haptic vibration motors. When an object is detected moving faster than 3 m/s toward the user, the system raises an alert by activating the visual and tactile feedback components.

This real-time detection and feedback loop is built using the [`embassy`](https://embassy.dev/) async framework, enabling efficient task scheduling without blocking execution on embedded hardware.

---

# 🚀 Getting started 
- [rustup](https://rustup.rs/) - the Rust toolchain is needed to compile Rust code.
- [esp-rs](https://docs.esp-rs.org/book/installation/riscv-and-xtensa.html) - the embedded rust book for esp targets, contains further details for setting up rust on the esp32.
- Run `cargo install espup` to install espup, a tool that simplifies installation of the components required to develop Rust for the esp32.
- Run `espup install` after installing espup, which will install the toolchains required.
- Run `cargo install espflash`. This is the tool that will flash code to your board.
- Run `cargo install cargo-espflash`. This is an optional step that provides some qol improvements.
- Run `cat $Home/export-esp.sh` this will print the required environment variables in your terminal, then copy paste the printed commands and run them in the terminal. This will have to be repeated every time you open a new terminal, follow the instructions in the esp-rs book for a permanent solution.
- Now you can execute `cargo build` to build the executable for your code, and `cargo run` to flash your code to the esp32. `cargo run` will also build your code before uploading.

---

# 🧠 Understanding the Codebase

The entry point of the firmware is located in [`src/bin/async_main/main.rs`](src/bin/async_main/main.rs). This modular bin layout allows for future firmware variants to be added without modifying or deleting the current implementation.

The main modules are:

- **`main.rs`**  
  Initializes hardware peripherals, sets up task spawns, and starts the main control loop.

- **`alert.rs`**  
  Contains logic to control the alert system: LED strip patterns and haptic motor activation.

- **`imu.rs`**  
  Placeholder module for future integration with an Inertial Measurement Unit (IMU) via I2C.

- **`uart.rs`**  
  Handles UART communication with the Doppler radar sensor. Reads incoming velocity data, parses it into floating-point values, and triggers the alert signal if velocity exceeds 3 m/s.

- **`wifi.rs`**  
  Provides optional Wi-Fi support, enabled conditionally using Cargo feature flags. Useful for remote data logging or debugging.

This project is built on the [`embassy`](https://github.com/embassy-rs/embassy) asynchronous runtime, which leverages Rust's `async`/`await` system to simplify concurrency in resource-constrained environments. This allows the firmware to appear synchronous in structure while maintaining full non-blocking behavior at runtime.