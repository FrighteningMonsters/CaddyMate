from smbus2 import SMBus, i2c_msg
import time

bus = SMBus(1)
ADDR = 0x08

def send_command(cmd):
    msg = i2c_msg.write(ADDR, bytes(cmd, 'utf-8'))
    bus.i2c_rdwr(msg)

def send_command_for_duration(cmd, duration):
    end_time = time.time() + duration
    while time.time() < end_time:
        send_command(cmd)
        time.sleep(0.1)
    send_command("STOP")

while True:
    user_input = input("\nEnter command (or 'q' to exit): ").strip().upper()
    if user_input == 'Q':
        break
    if user_input:
        print(f"Sending '{user_input}' for 2 seconds...")
        send_command_for_duration(user_input, 2)
        print("Stopped.")