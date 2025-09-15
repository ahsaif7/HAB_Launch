from time import sleep
from machine import Pin, ADC, I2C
from ulora import LoRa, ModemConfig, SPIConfig
from bmp280 import BMP280
from mpu6050 import MPU6050

# ----------------------
# LoRa Parameters
# ----------------------
RFM95_RST = 27
RFM95_SPIBUS = SPIConfig.rp2_0
RFM95_CS = 5
RFM95_INT = 28
RF95_FREQ = 433
RF95_POW = 20
CLIENT_ADDRESS = 1
SERVER_ADDRESS = 2

lora = LoRa(
    RFM95_SPIBUS,
    RFM95_INT,
    CLIENT_ADDRESS,
    RFM95_CS,
    reset_pin=RFM95_RST,
    freq=RF95_FREQ,
    tx_power=RF95_POW,
    acks=True
)

# ----------------------
# Sensor Setup
# ----------------------

# MQ2 (gas sensor) on ADC1 (GP27)
mq2 = ADC(Pin(27))

# I2C bus for BMP280 + MPU6050
i2c = I2C(0, scl=Pin(21), sda=Pin(20))

# BMP280 at 0x76 (try 0x77 if needed)
bmp = BMP280(i2c, addr=0x76)

# MPU6050 at 0x68
mpu = MPU6050(i2c, addr=0x68)

# ----------------------
# Main Loop
# ----------------------
while True:
    # MQ2 analog value
    mq2_val = mq2.read_u16()

    # BMP280 readings
    try:
        temp = float(bmp.temperature)  # °C
        press = float(bmp.pressure)

        # Fix scaling if in Pa
        if press > 2000:
            press = press / 100

        # Always positive
        press = abs(press)

        # Clamp to realistic range
        if press < 300 or press > 1100:
            press = 1013.25  # default to standard sea-level pressure

    except:
        temp, press = -999, -999

    # MPU6050 readings
    accel = mpu.get_accel_data()
    gyro = mpu.get_gyro_data()
    mpu_temp = mpu.get_temp()

    # Format data into one message
    msg = (
        f"GAS={mq2_val}, "
        f"TEMPERATURE={temp:.2f}C, PRESSURE={press:.2f}hPa, "
        f"INT_TEMP={mpu_temp:.2f}C, "
        f"ACCELEROMETER=({accel['x']:.2f},{accel['y']:.2f},{accel['z']:.2f}), "
        f"GYROSCOPE=({gyro['x']:.2f},{gyro['y']:.2f},{gyro['z']:.2f})"
    )

    # Send via LoRa
    lora.send_to_wait(msg, SERVER_ADDRESS)
    print("Sent:", msg)

    sleep(5)

