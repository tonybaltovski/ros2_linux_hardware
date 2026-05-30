# ros2_linux_hardware

ROS 2 packages for talking to I2C devices and GPIO lines on Linux, built around
the kernel's `/dev/i2c-N` and `/dev/gpiochipN` character-device interfaces. No
sysfs, no `libgpiod`, no WiringPi. Tested on a Raspberry Pi 4 running Ubuntu
Server 24.04 + ROS 2 Jazzy.

## Packages

| Package | Description |
|---------|-------------|
| `linux_gpio_interface` | Thread-safe C++ wrapper around the Linux GPIO character-device (`/dev/gpiochipN`) interface. |
| `linux_i2c_demos` | Example ROS 2 nodes that use the device drivers. |
| `linux_i2c_devices` | Device drivers built on the I2C interface — LCM1602 LCD, SSD1306 OLED, PCA9685 PWM, HMC6343 compass, BMI160 IMU. |
| `linux_i2c_interface` | Thread-safe C++ wrapper around the Linux I2C `/dev` interface. |
| `linux_i2c_ros2_control` | `ros2_control` hardware plugins: BMI160 sensor and a generic Linux GPIO system. |

## Supported Hardware

I2C devices:

- **LCM1602** — HD44780-compatible character LCD via PCF8574 I2C expander (default `0x27`).
- **SSD1306** — 128×64 monochrome OLED display (default `0x3C`).
- **PCA9685** — 16-channel, 12-bit PWM/LED controller (default `0x40`).
- **HMC6343** — 3-axis tilt-compensated compass (default `0x19`).
- **BMI160** — 6-axis IMU, accelerometer + gyroscope (default `0x68`, `0x69` if SDO is high).

GPIO:

- Any line on a `/dev/gpiochipN` device. On a Raspberry Pi the 40-pin header is `gpiochip0`.

## Setup (Raspberry Pi 4, Ubuntu Server 24.04 + ROS 2 Jazzy)

### 1. Flash Ubuntu Server

1. Download [Ubuntu Server 24.04 LTS (64-bit)](https://ubuntu.com/download/raspberry-pi) for Raspberry Pi.
2. Flash the image to a microSD card using the [Raspberry Pi Imager](https://www.raspberrypi.com/software/) or `dd`.
3. Boot the Pi and complete first-login setup (default user `ubuntu`, password `ubuntu`).

### 2. Enable I2C

The Raspberry Pi's I2C bus is disabled by default. Enable it by adding a
device-tree overlay:

```bash
sudo nano /boot/firmware/config.txt
```

Add (or uncomment) under `[all]`:

```
dtparam=i2c_arm=on
```

Reboot, then verify the bus exists:

```bash
sudo reboot
ls /dev/i2c-*       # expect /dev/i2c-1
```

GPIO requires no firmware change — `/dev/gpiochip0` is present out of the box.

### 3. Install host tools

```bash
sudo apt update
sudo apt install -y i2c-tools libi2c-dev
```

These are also pulled in via `rosdep`, but you'll want `i2cdetect` available
before building to verify wiring:

```bash
i2cdetect -y 1
```

You should see the addresses of any connected devices (e.g. `27` for the LCD,
`40` for the PCA9685).

### 4. Bus permissions

Both buses default to root-only access. Add your user to the appropriate group
and install the included udev rules so the device nodes are group-readable.

**I2C:**

```bash
sudo usermod -aG i2c $USER
sudo cp linux_i2c_interface/debian/udev /etc/udev/rules.d/60-linux-i2c-interface.rules
sudo udevadm control --reload-rules && sudo udevadm trigger
```

**GPIO:**

```bash
sudo groupadd -f gpio
sudo usermod -aG gpio $USER
sudo cp linux_gpio_interface/debian/udev /etc/udev/rules.d/60-linux-gpio-interface.rules
sudo udevadm control --reload-rules && sudo udevadm trigger
```

Log out and back in (or reboot) for the group change to take effect.

### 5. Build the workspace

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/tonybaltovski/ros2_linux_hardware.git

rosdep update
rosdep install --from-paths . --ignore-src -y

cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

## Wiring

### LCM1602 LCD (I2C `0x27`)

| LCD Pin | Pi Pin |
|---------|--------|
| GND | GND (Pin 6) |
| VCC | 5V (Pin 2) |
| SDA | GPIO 2 / SDA1 (Pin 3) |
| SCL | GPIO 3 / SCL1 (Pin 5) |

### SSD1306 OLED (I2C `0x3C`)

| OLED Pin | Pi Pin |
|----------|--------|
| GND | GND (Pin 6) |
| VCC | 3.3V (Pin 1) |
| SDA | GPIO 2 / SDA1 (Pin 3) |
| SCL | GPIO 3 / SCL1 (Pin 5) |

### PCA9685 PWM Board (I2C `0x40`)

| PCA9685 Pin | Pi Pin |
|-------------|--------|
| GND | GND (Pin 6) |
| VCC | 3.3V (Pin 1) |
| SDA | GPIO 2 / SDA1 (Pin 3) |
| SCL | GPIO 3 / SCL1 (Pin 5) |
| V+  | External servo power supply (5–6 V) |

> **Note:** The PCA9685 V+ terminal is the servo power rail and must be
> connected to an external supply — do **not** power servos from the Pi's 5V
> pin.

### GPIO header

The default GPIO demo uses BCM 17 as an output (LED) and BCM 27 as an input
(button). Pi 40-pin header reference:

| Signal | BCM | Pi Pin |
|--------|-----|--------|
| LED out | 17 | Pin 11 |
| Button in | 27 | Pin 13 |
| GND | — | Pin 9 / 14 / 20 / 25 / 30 / 34 / 39 |
| 3.3V | — | Pin 1 / 17 |

Wire the button between BCM 27 and GND; the demo enables an internal pull-up.

## Running the Demos

### `linux_i2c_demos`

#### Screen — display string messages

A single node supports both the LCM1602 LCD and SSD1306 OLED, selected via the
`display_type` parameter.

```bash
# LCM1602 LCD (default)
ros2 run linux_i2c_demos screen_from_str_topic

# SSD1306 OLED
ros2 run linux_i2c_demos screen_from_str_topic --ros-args -p display_type:=ssd1306
```

| Parameter | Default | Description |
|-----------|---------|-------------|
| `display_type` | `lcm1602` | `lcm1602` or `ssd1306` |
| `i2c_bus` | `1` | I2C bus number |
| `device_id` | `0x27` (lcm1602) / `0x3C` (ssd1306) | 7-bit I2C address |
| `rows` | `4` | Number of LCD rows (lcm1602 only) |
| `columns` | `20` | Number of LCD columns (lcm1602 only) |

Publish a message:

```bash
ros2 topic pub /screen_from_str_topic/input std_msgs/msg/String "{data: 'Hello, Pi!'}"
```

#### Screen — display ROS log output

```bash
ros2 run linux_i2c_demos screen_rosout_logger
```

Accepts the same `display_type`, `i2c_bus`, `device_id`, `rows`, `columns`
parameters as above, plus:

| Parameter | Default | Description |
|-----------|---------|-------------|
| `min_logger_level` | `20` (INFO) | Minimum `rcl_interfaces/Log` level to display |

All log messages at or above the configured level scroll upward on the screen.

#### PCA9685 — servo control

```bash
ros2 run linux_i2c_demos pca9685_servo_control
```

| Parameter | Default | Description |
|-----------|---------|-------------|
| `i2c_bus` | `1` | I2C bus number |
| `device_id` | `0x40` | PCA9685 I2C address |
| `pwm_frequency` | `50.0` | PWM frequency in Hz |

Send throttle commands (one value per channel, normalised to [-1.0, +1.0]):

```bash
ros2 topic pub /pca9685_servo_control/servo_commands \
  std_msgs/msg/Float64MultiArray "{data: [-1.0, 0.0, 1.0]}"
```

Values are clamped to [-1.0, +1.0] and mapped to the default 1.0–2.0 ms
pulse-width range (standard hobby servo):

- **-1.0** (1.0 ms) → one extreme
- **0.0**  (1.5 ms) → centre / ESC stop
- **+1.0** (2.0 ms) → other extreme

#### HMC6343 — compass publisher

```bash
ros2 run linux_i2c_demos hmc6343_publisher
```

Publishes orientation and heading data from the HMC6343.

| Parameter | Default | Description |
|-----------|---------|-------------|
| `i2c_bus` | `1` | I2C bus number |
| `device_id` | `0x19` | HMC6343 I2C address |
| `publish_rate` | `10.0` Hz | Output rate; chip max is 10 Hz |
| `frame_id` | `hmc6343` | `frame_id` stamped on messages |

#### BMI160 — IMU publisher

```bash
ros2 run linux_i2c_demos bmi160_publisher
```

| Parameter | Default | Description |
|-----------|---------|-------------|
| `i2c_bus` | `1` | I2C bus number |
| `device_id` | `0x68` | `0x68` (SDO low) or `0x69` (SDO high) |
| `publish_rate` | `100.0` Hz | Output rate (chip ODR defaults to 100 Hz) |
| `accel_range_g` | `2` | One of `{2, 4, 8, 16}` |
| `gyro_range_dps` | `2000` | One of `{125, 250, 500, 1000, 2000}` |
| `frame_id` | `bmi160` | `frame_id` stamped on messages |

### `linux_i2c_ros2_control`

#### BMI160 — IMU via `imu_sensor_broadcaster`

```bash
ros2 launch linux_i2c_ros2_control bmi160_imu_broadcaster.launch.py
```

Loads a `ros2_control` `SensorInterface` for the BMI160 and spawns
`imu_sensor_broadcaster`, which publishes `sensor_msgs/msg/Imu` on `/imu`.
Configure the I2C address and full-scale ranges via the URDF
[`bmi160_demo.urdf.xacro`](linux_i2c_ros2_control/urdf/bmi160_demo.urdf.xacro).

#### GPIO — via `gpio_command_controller`

```bash
ros2 launch linux_i2c_ros2_control gpio_ros2_control.launch.py
```

Loads a generic Linux GPIO `SystemInterface` and spawns
`gpio_controllers/GpioCommandController`. Default URDF
[`raspi_gpio.urdf.xacro`](linux_i2c_ros2_control/urdf/raspi_gpio.urdf.xacro)
exposes one output (`led`, BCM 17) and one input (`button`, BCM 27, pull-up).

Launch arguments:

| Argument | Default | Description |
|----------|---------|-------------|
| `chip` | `/dev/gpiochip0` | GPIO chip device path |
| `led_line` | `17` | Line offset for the LED |
| `button_line` | `27` | Line offset for the button |

URDF schema (per `<gpio>`):

- `<param name="line">N</param>` — line offset on the chip (required).
- `<param name="direction">input|output</param>` — required.
- `<param name="initial_value">0|1</param>` — output only, default `0`.
- `<param name="bias">as_is|disabled|pull_up|pull_down</param>` — input only, default `as_is`.
- `<param name="drive">push_pull|open_drain|open_source</param>` — output only, default `push_pull`.

Hardware-level params:

- `<param name="chip">/dev/gpiochip0</param>` (default).

## Docker

A multi-arch Dockerfile (linux/amd64, linux/arm64) is provided at the
repository root. Prebuilt images are published to GitHub Container Registry by
the [`docker`](.github/workflows/docker.yaml) workflow on every push to `main`
and on tags `v*`:

```
ghcr.io/tonybaltovski/ros2_linux_hardware:latest
ghcr.io/tonybaltovski/ros2_linux_hardware:main
```

### Pull and run on a Raspberry Pi

```bash
docker pull ghcr.io/tonybaltovski/ros2_linux_hardware:latest

docker run --rm -it \
  --device=/dev/i2c-1 \
  --device=/dev/gpiochip0 \
  --group-add "$(getent group i2c | cut -d: -f3)" \
  --group-add "$(getent group gpio | cut -d: -f3)" \
  --network host \
  ghcr.io/tonybaltovski/ros2_linux_hardware:latest \
  ros2 run linux_i2c_demos pca9685_servo_control
```

- `--device=` exposes each bus to the container (no `--privileged` needed).
- `--group-add` injects the host's GIDs so the (non-root) container user can
  open the device nodes regardless of how the upstream ROS image was built.
- `--network host` is the simplest way to let DDS discovery reach other ROS 2
  nodes on the LAN.

### Build locally

```bash
docker build -t ros2_linux_hardware .
```

For a cross-build from x86 to arm64 (uses qemu, slow):

```bash
docker buildx create --use --name multi
docker buildx build --platform linux/arm64 -t ros2_linux_hardware:arm64 --load .
```

## Troubleshooting

| Problem | Solution |
|---------|----------|
| `Could not open: /dev/i2c-1` | Ensure I2C is enabled in `/boot/firmware/config.txt` and reboot. |
| `Permission denied` on `/dev/i2c-1` | Add your user to the `i2c` group: `sudo usermod -aG i2c $USER` and re-login. |
| `i2cdetect` shows no devices | Check wiring — SDA/SCL may be swapped, or the device may need a pull-up resistor. |
| SSD1306 screen is blank after init | Ensure `display()` is called (the node does this via `print_msg`). Verify address with `i2cdetect`. |
| `Failed to set device` | Verify the address with `i2cdetect -y 1` and update the `device_id` parameter. |
| PCA9685 servos don't move | Ensure external power is connected to the V+ terminal on the PCA9685 board. |
| `Could not open: /dev/gpiochip0` | The chip device is missing on this kernel/board. List with `ls /dev/gpiochip*`. |
| `Permission denied` on `/dev/gpiochip0` | Add your user to the `gpio` group and install the GPIO udev rule (see [Bus permissions](#4-bus-permissions)). |
| GPIO request fails with `EBUSY` | Another process (e.g. a previous run, `pigpiod`, `gpiomon`) is holding the line. Stop it and retry. |
| GPIO input always reads 0 or 1 | Add `bias=pull_up` / `pull_down` in the URDF, or wire an external pull resistor. |

## License

Apache 2.0 — see [LICENSE](LICENSE) for details.
