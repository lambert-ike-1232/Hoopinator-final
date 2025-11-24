#!/usr/bin/env python3
import serial
import time
from sendStringScript import sendString

leftMotor = int(100)
rightMotor = int(100)

currentState  = "FOLLOW_LINE"
previousState = None

# For intersection tracking
lines_hit = 0
last_cross_time = 0
CROSS_COOLDOWN = 1.0  # seconds before next intersection counts

# For non-blocking "pause at intersection"
stop_until = 0   # time until which motors stay at 0

if __name__ == '__main__':
    ser = serial.Serial('/dev/ttyACM0',115200)
    ser.reset_input_buffer()

    while True:
        # --- Non-blocking stop timer ---
        now = time.time()
        if now < stop_until:
            leftMotor, rightMotor = 0, 0
        else:
            # Normal loop continues below
            pass

        # Send motor command every loop
        sendString('/dev/ttyACM0',115200,'<'+str(leftMotor)+','+str(rightMotor)+'>',0.0001)
       
        if ser.in_waiting > 0:  
            line = ser.readline().decode('utf-8')
            line = line.split(',')

            try:
                x = int(line[0])
                y = int(line[1])
                z = int(line[2])  
                print(line)
            except:
                print("packet dropped")
                continue

            # -------------------------
            # STATE MACHINE
            # -------------------------
            if z < 7000:
                currentState = "INTERSECTION"
            else:
                currentState = "FOLLOW_LINE"

            if currentState == "FOLLOW_LINE" and now >= stop_until:
                # Control law for line following
                leftMotor  = 100 + 0.02*z
                rightMotor = 250 - 0.02*z

            elif currentState == "INTERSECTION":
                print("At intersection")

                if (now - last_cross_time) > CROSS_COOLDOWN:
                    lines_hit += 1
                    last_cross_time = now
                    print(f"Lines hit so far: {lines_hit}")

                    # Example: stop robot for 1s without blocking loop
                    stop_until = now + 1.0
                    leftMotor, rightMotor = 0, 0

            previousState = currentState