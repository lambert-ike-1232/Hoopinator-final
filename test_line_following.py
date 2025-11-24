#!/usr/bin/env python3
"""
Standalone Line Following Monitor

This script monitors the Arduino running LineFollowingTest.ino
It simply reads and displays the sensor data - no control commands sent.

Usage:
    python3 test_line_following.py
"""

import serial
import time

def main():
    print("=== Line Following Test Monitor ===")
    print("Connecting to Arduino...")
    
    try:
        ser = serial.Serial('/dev/ttyACM0', 115200, timeout=1)
        ser.reset_input_buffer()
        print("Connected!")
        print("\nMonitoring line following behavior...\n")
        print("-" * 80)
        
        while True:
            if ser.in_waiting > 0:
                try:
                    line = ser.readline().decode('utf-8').strip()
                    
                    # Print all output from Arduino
                    if line:
                        # Highlight important messages
                        if "CROSS DETECTED" in line or "COMPLETE" in line:
                            print(f"\n{'='*80}")
                            print(f">>> {line}")
                            print(f"{'='*80}\n")
                        else:
                            print(line)
                            
                except UnicodeDecodeError:
                    print("(packet decode error)")
                except Exception as e:
                    print(f"Error: {e}")
                    
            time.sleep(0.01)  # Small delay to prevent CPU overload
            
    except serial.SerialException as e:
        print(f"Error: Could not open serial port - {e}")
        print("\nTroubleshooting:")
        print("1. Make sure Arduino is connected via USB")
        print("2. Check that /dev/ttyACM0 is the correct port")
        print("3. Try: ls /dev/tty* to see available ports")
        print("4. You may need to change permissions: sudo chmod 666 /dev/ttyACM0")
    except KeyboardInterrupt:
        print("\n\nTest monitor stopped by user.")
        if 'ser' in locals():
            ser.close()
        print("Serial connection closed.")

if __name__ == '__main__':
    main()

