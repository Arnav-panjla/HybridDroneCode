import os
import fcntl

# I2C device file
I2C_DEV = "/dev/i2c-1"  # Use '/dev/i2c-0' for older Raspberry Pi boards

# I2C address of the BMX160 IMU sensor
BMX160_ADDR = 0x68

# Register addresses for chip ID
CHIP_ID_ADDR = 0x00

def read_i2c_byte_data(i2c_fd, register_addr):
    os.write(i2c_fd, bytes([register_addr]))
    return os.read(i2c_fd, 1)

# Open the I2C bus
i2c_fd = os.open(I2C_DEV, os.O_RDWR)

# Set the I2C slave address
fcntl.ioctl(i2c_fd, 0x0703, BMX160_ADDR)

# Read chip ID
chip_id_data = read_i2c_byte_data(i2c_fd, CHIP_ID_ADDR)

# Close the I2C bus
os.close(i2c_fd)

# Extract chip ID and version
chip_id = int.from_bytes(chip_id_data, "big")

# Print the chip ID
print(f"Chip ID: 0x{chip_id:02X}")
