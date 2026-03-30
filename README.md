# ros2_linux_hardware

ROS 2 packages for interacting with I2C hardware on Linux (tested on Raspberry Pi 4).

## Packages

| Package | Description |
|---------|-------------|
| `linux_i2c_interface` | Thread-safe C++ wrapper around the Linux I2C `/dev` interface. |
| `linux_i2c_devices` | Device drivers built on the interface — LCM1602 LCD, SSD1306 OLED, and PCA9685 PWM controller. |
| `linux_i2c_demos` | Example ROS 2 nodes that use the device drivers. |

## Supported Devices

- **LCM1602** — HD44780-compatible character LCD via PCF8574 I2C expander (default address `0x27`)
- **SSD1306** — 128×64 monochrome OLED display (default address `0x3C`)
- **PCA9685** — 16-channel, 12-bit PWM/LED controller (default address `0x40`)

## Raspberry Pi 4 Setup (Ubuntu Server + ROS 2 Jazzy)

### 1. Flash Ubuntu Server

1. Download [Ubuntu Server 24.04 LTS (64-bit)](https://ubuntu.com/download/raspberry-pi) for Raspberry Pi.
2. Flash the image to a microSD card using the [Raspberry Pi Imager](https://www.raspberrypi.com/software/) or `dd`.
3. Boot the Pi and complete first-login setup (default user `ubuntu`, password `ubuntu`).

### 2. Enable I2C

The Raspberry Pi's I2C bus is disabled by default. Enable it by adding a device-tree overlay:

```bash
# Edit the firmware config
sudo nano /boot/firmware/config.txt
```

Add (or uncomment) the following line under `[all]`:

```
dtparam=i2c_arm=on
```

Then reboot:

```bash
sudo reboot
```

After reboot, verify the bus exists:

```bash
ls /dev/i2c-*
# Expected: /dev/i2c-1
```

### 3. Install I2C Tools and Libraries

```bash
sudo apt update
sudo apt install -y i2c-tools libi2c-dev
```

These are ROS deps but are needed prior to look for devices on the I2C bus.

Grant your user permission to access the I2C bus without `sudo`:

```bash
sudo usermod -aG i2c $USER
```

Log out and back in (or reboot) for the group change to take effect.

A udev rule is included in [`linux_i2c_interface/debian/udev`](linux_i2c_interface/debian/udev).
When building from source, install it manually:

```bash
sudo cp linux_i2c_interface/debian/udev /etc/udev/rules.d/60-linux-i2c-interface.rules
sudo udevadm control --reload-rules && sudo udevadm trigger
```

Scan for connected devices to verify your wiring:

```bash
i2cdetect -y 1
```

You should see the addresses of any connected devices (e.g. `27` for the LCD, `40` for the PCA9685).

### 4. Create a Workspace and Build

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/tonybaltovski/ros2_linux_hardware.git

# Install any missing rosdep dependencies
sudo rosdep init  # only needed once
rosdep update
rosdep install --from-paths . --ignore-src -y

# Build
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

### 5. Wiring

#### LCM1602 LCD (I2C address 0x27)

| LCD Pin | Pi Pin     |
|---------|------------|
| GND     | GND (Pin 6) |
| VCC     | 5V (Pin 2)  |
| SDA     | GPIO 2 / SDA1 (Pin 3) |
| SCL     | GPIO 3 / SCL1 (Pin 5) |

#### SSD1306 OLED (I2C address 0x3C)

| OLED Pin | Pi Pin     |
|----------|------------|
| GND      | GND (Pin 6) |
| VCC      | 3.3V (Pin 1) |
| SDA      | GPIO 2 / SDA1 (Pin 3) |
| SCL      | GPIO 3 / SCL1 (Pin 5) |

#### PCA9685 PWM Board (I2C address 0x40)

| PCA9685 Pin | Pi Pin     |
|-------------|------------|
| GND         | GND (Pin 6) |
| VCC         | 3.3V (Pin 1) |
| SDA         | GPIO 2 / SDA1 (Pin 3) |
| SCL         | GPIO 3 / SCL1 (Pin 5) |
| V+          | External servo power supply (5–6V) |

> **Note:** The PCA9685 V+ terminal is for the servo power rail and must be connected
> to an external supply — do **not** power servos from the Pi's 5V pin.

## Running the Demos

### Screen — Display String Messages

A single node supports both the LCM1602 LCD and SSD1306 OLED, selected via the
`display_type` parameter.

**LCM1602 LCD (default):**

```bash
ros2 run linux_i2c_demos screen_from_str_topic
```

**SSD1306 OLED:**

```bash
ros2 run linux_i2c_demos screen_from_str_topic --ros-args -p display_type:=ssd1306
```

Parameters:

| Parameter | Default | Description |
|-----------|---------|-------------|
| `display_type` | `lcm1602` | `lcm1602` or `ssd1306` |
| `i2c_bus` | `1` | I2C bus number |
| `device_id` | `0x27` (lcm1602) / `0x3C` (ssd1306) | 7-bit I2C address |
| `rows` | `4` | Number of LCD rows (lcm1602 only) |
| `columns` | `20` | Number of LCD columns (lcm1602 only) |

Publish a message to the display:

```bash
ros2 topic pub /screen_from_str_topic/input std_msgs/msg/String "{data: 'Hello, Pi!'}"
```

### Screen — Display ROS Log Output

```bash
ros2 run linux_i2c_demos screen_rosout_logger
```

Uses the same `display_type`, `i2c_bus`, `device_id`, `rows`, and `columns`
parameters as the string-topic node above.  An additional parameter controls the
minimum severity:

| Parameter | Default | Description |
|-----------|---------|-------------|
| `min_logger_level` | `20` (INFO) | Minimum `rcl_interfaces/Log` level to display |

All log messages at or above the configured level from any node will scroll
upward on the screen.

### PCA9685 — Servo Control

```bash
ros2 run linux_i2c_demos pca9685_servo_control
```

Parameters (set via `--ros-args -p`):

| Parameter | Default | Description |
|-----------|---------|-------------|
| `i2c_bus` | `1` | I2C bus number |
| `device_id` | `0x40` | PCA9685 I2C address |
| `pwm_frequency` | `50.0` | PWM frequency in Hz |

Send servo duty-cycle commands (one value per channel, 0.0–1.0):

```bash
# Centre a servo on channel 0 (1.5 ms pulse at 50 Hz ≈ 0.075 duty cycle)
ros2 topic pub /pca9685_servo_control/servo_commands \
  std_msgs/msg/Float64MultiArray "{data: [0.075]}"

# Drive channels 0, 1, and 2 to different positions
ros2 topic pub /pca9685_servo_control/servo_commands \
  std_msgs/msg/Float64MultiArray "{data: [0.025, 0.075, 0.125]}"
```

For standard hobby servos at 50 Hz:
- **~0.025** (0.5 ms) → full left
- **~0.075** (1.5 ms) → centre
- **~0.125** (2.5 ms) → full right

## Troubleshooting

| Problem | Solution |
|---------|----------|
| `Could not open: /dev/i2c-1` | Ensure I2C is enabled in `/boot/firmware/config.txt` and reboot. |
| `Permission denied` on `/dev/i2c-1` | Add your user to the `i2c` group: `sudo usermod -aG i2c $USER` and re-login. |
| `i2cdetect` shows no devices | Check wiring — SDA/SCL may be swapped, or the device may need a pull-up resistor. |
| SSD1306 screen is blank after init | Ensure you call `display()` (the node does this automatically via `print_msg`). Verify address with `i2cdetect`. |
| `Failed to set device` | Verify the device address with `i2cdetect -y 1` and update the `device_id` parameter. |
| PCA9685 servos don't move | Ensure external power is connected to the V+ terminal on the PCA9685 board. |

## License

Apache 2.0 — see [LICENSE](LICENSE) for details.
