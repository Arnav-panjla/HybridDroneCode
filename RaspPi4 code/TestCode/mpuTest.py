import smbus
import time
import math

# MPU6050 Registers
MPU6050_ADDR = 0x68
PWR_MGMT_1 = 0x6B
SMPLRT_DIV = 0x19
CONFIG = 0x1A
GYRO_CONFIG = 0x1B
ACCEL_CONFIG = 0x1C
INT_ENABLE = 0x38
ACCEL_XOUT_H = 0x3B
ACCEL_YOUT_H = 0x3D
ACCEL_ZOUT_H = 0x3F
GYRO_XOUT_H = 0x43
GYRO_YOUT_H = 0x45
GYRO_ZOUT_H = 0x47

# Initialize I2C (SMBus)
bus = smbus.SMBus(1)

# MPU6050 Initialization
def MPU_Init():
    # Write to sample rate register
    bus.write_byte_data(MPU6050_ADDR, SMPLRT_DIV, 7)
    
    # Write to power management register
    bus.write_byte_data(MPU6050_ADDR, PWR_MGMT_1, 1)
    
    # Write to configuration register
    bus.write_byte_data(MPU6050_ADDR, CONFIG, 0)
    
    # Write to gyroscope configuration register
    bus.write_byte_data(MPU6050_ADDR, GYRO_CONFIG, 24)
    
    # Write to interrupt enable register
    bus.write_byte_data(MPU6050_ADDR, INT_ENABLE, 1)

# Read raw data from MPU6050
def read_raw_data(addr):
    # Accelero and Gyro values are 16-bit
    try:
        high = bus.read_byte_data(MPU6050_ADDR, addr)
        low = bus.read_byte_data(MPU6050_ADDR, addr+1)
    
    # Concatenate higher and lower value
        value = ((high << 8) | low)
    
    # To get signed value from mpu6050
        if(value > 32768):
            value = value - 65536
        return value
    except Exception as e:
        print('@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@', e)
        return 1

# MPU6050 Initialization
MPU_Init()

print("Reading Data from MPU6050")

while True:
    # Read Accelerometer raw value
    acc_x = read_raw_data(ACCEL_XOUT_H)
    acc_y = read_raw_data(ACCEL_YOUT_H)
    acc_z = read_raw_data(ACCEL_ZOUT_H)
    
    # Read Gyroscope raw value
    gyro_x = read_raw_data(GYRO_XOUT_H)
    gyro_y = read_raw_data(GYRO_YOUT_H)
    gyro_z = read_raw_data(GYRO_ZOUT_H)
    
    # Full scale range +/- 250 degree/C as per sensitivity scale factor
    Ax = acc_x / 16384.0
    Ay = acc_y / 16384.0
    Az = acc_z / 16384.0
    
    Gx = gyro_x / 131.0
    Gy = gyro_y / 131.0
    Gz = gyro_z / 131.0
    
    # Calculate angle values
    angle_x = math.atan(Ay / math.sqrt(Ax**2 + Az**2)) * 57.2958
    angle_y = math.atan(-Ax / math.sqrt(Ay**2 + Az**2)) * 57.2958
    angle_z = math.atan(Az / math.sqrt(Ax**2 + Ay**2)) * 57.2958
    
    print("Ax=%.2f g Ay=%.2f g Az=%.2f g Gx=%.2f °/s Gy=%.2f °/s Gz=%.2f °/s" % (Ax, Ay, Az, Gx, Gy, Gz))
    print("Angle X=%.2f° Angle Y=%.2f° Angle Z=%.2f°" % (angle_x, angle_y, angle_z))
    time.sleep(0.05)
