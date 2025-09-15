from time import sleep
from machine import Pin, ADC, I2C
from ulora import LoRa, ModemConfig, SPIConfig
from bmp280 import BMP280
from mpu6050 import MPU6050
import math

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
mq2 = ADC(Pin(27))  # MQ2 gas sensor (analog input)

i2c = I2C(0, scl=Pin(21), sda=Pin(20))  # I2C bus

bmp = BMP280(i2c, addr=0x76)  # BMP280 sensor
mpu = MPU6050(i2c, addr=0x68)  # MPU6050 sensor

# Sea-level reference pressure (for altitude calculation)
SEA_LEVEL_PRESSURE = 1013.25

def pressure_to_altitude(pressure_hpa, sea_level=SEA_LEVEL_PRESSURE):
    """Convert pressure (hPa) to altitude (m)."""
    if pressure_hpa <= 0:
        return -999
    return 44330 * (1 - (pressure_hpa / sea_level) ** (1/5.255))

# ----------------------
# Main Loop
# ----------------------
while True:
    mq2_val = mq2.read_u16()

    # BMP280 readings
    try:
        temp = float(bmp.temperature)  # °C
        press = float(bmp.pressure)    # hPa

        if press > 2000:  # sometimes returns Pa
            press = press / 100

        press = abs(press)
        if press < 300 or press > 1100:  # clamp to valid range
            press = SEA_LEVEL_PRESSURE

    except:
        temp, press = -999, -999

    # Calculate altitude
    altitude = pressure_to_altitude(press)

    # MPU6050 readings
    accel = mpu.get_accel_data()
    gyro = mpu.get_gyro_data()

    # Format LoRa message
    msg = (
        f"GAS={mq2_val}, "
        f"TEMP={temp:.2f}C, PRESSURE={press:.2f}hPa, ALT={altitude:.2f}m, "
        f"ACCEL=({accel['x']:.2f},{accel['y']:.2f},{accel['z']:.2f}), "
        f"GYRO=({gyro['x']:.2f},{gyro['y']:.2f},{gyro['z']:.2f})"
    )

    lora.send_to_wait(msg, SERVER_ADDRESS)
    print("Sent:", msg)

    sleep(5)

