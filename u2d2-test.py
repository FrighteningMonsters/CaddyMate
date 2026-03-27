import os
import time
from dynamixel_sdk import *

ADDR_TORQUE_ENABLE  = 64
ADDR_GOAL_VELOCITY  = 104
ADDR_PROFILE_ACCEL  = 108
ADDR_OPERATING_MODE = 11
DXL_ID              = 1
BAUDRATE            = 57600
PROTOCOL_VERSION    = 2.0
DEVICENAME          = '/dev/ttyUSB0' # Change to USB1 if USB0 doesn't work


portHandler = PortHandler(DEVICENAME)
packetHandler = PacketHandler(PROTOCOL_VERSION)

def set_velocity(velocity):
    dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, DXL_ID, ADDR_GOAL_VELOCITY, velocity)
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))

try:
    if not portHandler.openPort():
        print("Failed to open port. Try: sudo chmod 666 /dev/ttyUSB0")
        exit()
    if not portHandler.setBaudRate(BAUDRATE):
        print("Failed to set baudrate.")
        exit()

    # Initialization: Mode 1 is Velocity Control
    packetHandler.write1ByteTxRx(portHandler, DXL_ID, ADDR_OPERATING_MODE, 1)
    dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, DXL_ID, ADDR_PROFILE_ACCEL, 30)
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))
    packetHandler.write1ByteTxRx(portHandler, DXL_ID, ADDR_TORQUE_ENABLE, 1)

    print("\n--- Dynamixel Interactive Control ---")
    print("1: Spin Clockwise (CW)")
    print("2: Spin Counter-Clockwise (CCW)")
    print("3: Stop")
    print("q: Quit and Close Port")
    print("-------------------------------------")

    while True:
        user_input = input("Enter Command: ").lower()

        if user_input == '1':
            print("Action: CW")
            set_velocity(-200) 
        elif user_input == '2':
            print("Action: CCW")
            set_velocity(200)
        elif user_input == '3':
            print("Action: Stopping")
            set_velocity(0)
        elif user_input == 'q':
            break
        else:
            print("Invalid input. Use 1, 2, 3, or q.")

except KeyboardInterrupt:
    print("\nInterrupted by user.")

finally:
    print("Cleaning up...")
    set_velocity(0)
    packetHandler.write1ByteTxRx(portHandler, DXL_ID, ADDR_TORQUE_ENABLE, 0)
    portHandler.closePort()
    print("Port closed. Safety first!")
