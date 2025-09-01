# Code from PROJET-S8

from dynamixel_sdk import PortHandler, PacketHandler

PROTOCOL_VERSION = 1
BAUDRATE = 1000000
DEVICENAME = "COM3"

ADDR_GOAL_POSITION = 30
ADDR_PRESENT_POSITION = 36
ADDR_PRESENT_SPEED = 38
ADDR_PRESENT_TEMPERATURE = 43
ADDR_TORQUE_ENABLE = 24
ADDR_GOAL_SPEED = 32 
ADDR_GOAL_ACCELERATION = 73 # MX-28 only

TORQUE_ENABLE = 1
TORQUE_DISABLE = 0

DEGREE_TO_POSITION = 1023 / 300  # Conversion de degré en unité Dynamixel

MODEL_TYPES = {
    12: "AX-12A",
    29: "MX-28"
}

MODEL_RESOLUTION = {
    "AX-12A": 1023 / 300,
    "MX-28": 4095 / 360,
}

RPM_UNIT = {
    "MX-28": 0.114,
    "AX-12A": 0.111
}

ACCEL_UNIT = 8.583 # °/s²

"""Initialise le contrôleur de moteurs"""
portHandler = PortHandler(DEVICENAME)
packetHandler = PacketHandler(PROTOCOL_VERSION)
server_rate = 0.005