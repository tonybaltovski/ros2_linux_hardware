# linux_iio_devices

ROS 2 nodes that read sensors through the Linux **Industrial I/O (IIO)** kernel subsystem rather than poking the I2C bus directly from userspace.

When a chip has a polished mainline kernel driver, IIO is usually the better path: the kernel handles bus arbitration, hardware triggering, kernel-timestamped buffered reads, and per-chip ABI quirks. Userspace just opens `/sys/bus/iio/devices/iio:deviceN/` (or `/dev/iio:deviceN`) and reads numbers.

This package currently ships:

- `iio_imu_publisher` — publishes `sensor_msgs/Imu` and `sensor_msgs/Temperature` for any IIO IMU exposing the standard `in_accel_*` / `in_anglvel_*` channels (BMI160, BMI270, MPU6050, LSM6DSx, ICM426xx, …).
- A small `linux_iio_devices` C++ helper library (`iio_device.hpp`) with `resolve_by_name()`, sysfs read/write helpers, and a `BufferedReader` wrapping `/dev/iio:deviceN` + `poll()`.

It coexists with [`linux_i2c_devices`](../linux_i2c_devices/) — IIO drives the sensors the kernel handles well, the I2C library keeps owning the displays, PWM controllers, and chips with no mainline driver.

## Setup (BMI160 on a Raspberry Pi)

1. **Enable I2C** (`raspi-config` → Interface Options → I2C).
2. **Install the device-tree overlay** so the kernel binds `bmi160_i2c`:
   ```bash
   cd $(ros2 pkg prefix linux_iio_devices)/share/linux_iio_devices/overlays
   sudo dtc -@ -I dts -O dtb -o /boot/firmware/overlays/bmi160.dtbo bmi160-overlay.dts
   echo 'dtoverlay=bmi160' | sudo tee -a /boot/firmware/config.txt
   sudo reboot
   ```
3. **Verify** the device appears:
   ```bash
   $ cat /sys/bus/iio/devices/iio:device0/name
   bmi160
   $ ls /sys/bus/iio/devices/iio:device0/
   buffer  in_accel_scale  in_anglvel_scale  in_temp_offset  ...
   ```
4. **Install the udev rule** to read without root:
   ```bash
   sudo cp $(ros2 pkg prefix linux_iio_devices)/share/linux_iio_devices/debian/udev /etc/udev/rules.d/99-iio.rules
   sudo groupadd -f iio && sudo usermod -aG iio $USER
   sudo udevadm control --reload && sudo udevadm trigger
   # log out / back in
   ```

## Run

Polled sysfs reads (works without INT1 wired):
```bash
ros2 launch linux_iio_devices iio_imu_publisher.launch.py
ros2 topic echo /iio_imu_publisher/imu
```

Hardware-triggered buffered reads (requires INT1 wired and a trigger configured):
```bash
# one-time per boot: pick the chip's data-ready trigger.
echo bmi160-dev0 | sudo tee /sys/bus/iio/devices/iio:device0/trigger/current_trigger

# enable all six scan channels + timestamp, set buffer length, enable buffer.
for ch in in_accel_x in_accel_y in_accel_z \
          in_anglvel_x in_anglvel_y in_anglvel_z in_timestamp; do
    echo 1 | sudo tee /sys/bus/iio/devices/iio:device0/scan_elements/${ch}_en
done
echo 16 | sudo tee /sys/bus/iio/devices/iio:device0/buffer/length
echo 1  | sudo tee /sys/bus/iio/devices/iio:device0/buffer/enable

ros2 launch linux_iio_devices iio_imu_publisher.launch.py use_buffered_reads:=true
```

## Parameters

See [`src/iio_imu_publisher_parameters.yaml`](src/iio_imu_publisher_parameters.yaml). Notable ones:

| Param | Default | Purpose |
|---|---|---|
| `iio_name` | `bmi160` | Locate device by `iio:deviceN/name`. Works with any IIO IMU. |
| `device_path` | `""` | Override `iio_name` lookup; useful with multiple identical IMUs. |
| `use_buffered_reads` | `false` | True → `/dev/iio:deviceN` + `poll()`. Requires a trigger. |
| `publish_rate` | `100.0` | Polled-mode timer rate (ignored in buffered mode). |
| `sampling_frequency_hz` | `0.0` | Optionally program the chip ODR on startup. |
| `publish_temperature` | `true` | Auto-disabled if the chip lacks `in_temp_*`. |

## Buffered-mode scan layout

The buffered code path assumes the common 6-axis layout used by BMI160, BMI270, MPU6050, LSM6DSx and ICM426xx:

```
bytes  0..1   anglvel_x  (le:s16)
bytes  2..3   anglvel_y
bytes  4..5   anglvel_z
bytes  6..7   accel_x
bytes  8..9   accel_y
bytes 10..11  accel_z
bytes 12..15  padding to 8-byte alignment
bytes 16..23  timestamp (s64 ns)
```

If you're targeting a device with a different layout (you can check `scan_elements/in_*_type` and `_index`), use polled mode or extend `iio_imu_publisher.cpp` to demux per device.

## When NOT to use this package

- Your chip has no mainline IIO driver (e.g. HMC6343) → use `linux_i2c_devices` instead.
- You need device-specific features the IIO driver doesn't expose (BMI160 step counter, tap engine) → use `linux_i2c_devices` instead.
- You only need <100 Hz polled reads and don't want to manage device-tree overlays → either is fine; the userspace path is more self-contained.

## License

Apache-2.0.
