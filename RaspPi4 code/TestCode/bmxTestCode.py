import os
import fcntl
import time

# I2C device file
I2C_DEV = "/dev/i2c-1"

# I2C address of the BMX160 IMU sensor
BMX160_ADDR = 0x68

# Register addresses
CHIP_ID_ADDR = 0x00
ACC_CONF_ADDR = 0x40
ACC_RANGE_ADDR = 0x41
GYR_CONF_ADDR = 0x42
GYR_RANGE_ADDR = 0x43
MAG_CONF_ADDR = 0x44

ACC_X_LSB_ADDR = 0x12
GYR_X_LSB_ADDR = 0x0C
MAG_X_LSB_ADDR = 0x04

def read_i2c_byte_data(i2c_fd, register_addr):
    os.write(i2c_fd, bytes([register_addr]))
    return os.read(i2c_fd, 1)

def read_i2c_word_data(i2c_fd, register_addr):
    os.write(i2c_fd, bytes([register_addr]))
    data = os.read(i2c_fd, 2)
    value = (data[1] << 8) | data[0]
    if value >= 0x8000:
        value = -((65535 - value) + 1)
    return value

def write_i2c_byte_data(i2c_fd, register_addr, data):
    os.write(i2c_fd, bytes([register_addr, data]))

def initialize_bmx160(i2c_fd):
    chip_id = read_i2c_byte_data(i2c_fd, CHIP_ID_ADDR)
    if chip_id == b'\xD8':
        print("BMX160 detected successfully.")
        write_i2c_byte_data(i2c_fd, ACC_CONF_ADDR, 0x28)  # 100 Hz, normal mode
        write_i2c_byte_data(i2c_fd, ACC_RANGE_ADDR, 0x03)  # ±2g range
        write_i2c_byte_data(i2c_fd, GYR_CONF_ADDR, 0x28)  # 100 Hz, normal mode
        write_i2c_byte_data(i2c_fd, GYR_RANGE_ADDR, 0x00)  # ±2000 dps range
        write_i2c_byte_data(i2c_fd, MAG_CONF_ADDR, 0x03)  # 25 Hz, normal mode
    else:
        print("Failed to detect BMX160.")
        exit(1)

def read_accelerometer(i2c_fd):
    acc_x = read_i2c_word_data(i2c_fd, ACC_X_LSB_ADDR)
    acc_y = read_i2c_word_data(i2c_fd, ACC_X_LSB_ADDR + 2)
    acc_z = read_i2c_word_data(i2c_fd, ACC_X_LSB_ADDR + 4)
    return acc_x, acc_y, acc_z

def read_gyroscope(i2c_fd):
    gyr_x = read_i2c_word_data(i2c_fd, GYR_X_LSB_ADDR)
    gyr_y = read_i2c_word_data(i2c_fd, GYR_X_LSB_ADDR + 2)
    gyr_z = read_i2c_word_data(i2c_fd, GYR_X_LSB_ADDR + 4)
    return gyr_x, gyr_y, gyr_z

def read_magnetometer(i2c_fd):
    mag_x = read_i2c_word_data(i2c_fd, MAG_X_LSB_ADDR)
    mag_y = read_i2c_word_data(i2c_fd, MAG_X_LSB_ADDR + 2)
    mag_z = read_i2c_word_data(i2c_fd, MAG_X_LSB_ADDR + 4)
    return mag_x, mag_y, mag_z

# Open the I2C bus
i2c_fd = os.open(I2C_DEV, os.O_RDWR)

# Set the I2C slave address
fcntl.ioctl(i2c_fd, 0x0703, BMX160_ADDR)

# Initialize the BMX160
initialize_bmx160(i2c_fd)

# Read and print sensor data
try:
    while True:
        acc = read_accelerometer(i2c_fd)
        gyr = read_gyroscope(i2c_fd)
        mag = read_magnetometer(i2c_fd)
        print(f"Accelerometer: {acc}")
        print(f"Gyroscope: {gyr}")
        print(f"Magnetometer: {mag}")
        time.sleep(1)
finally:
    # Close the I2C bus
    os.close(i2c_fd)
