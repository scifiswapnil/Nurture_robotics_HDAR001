import serial
import time
import argparse

def main():
    parser = argparse.ArgumentParser(description='Read ESP32 Encoder Data')
    parser.add_argument('--port', type=str, default='/dev/ttyUSB0', help='Serial Port')
    parser.add_argument('--baud', type=int, default=115200, help='Baud Rate')
    args = parser.parse_args()

    try:
        ser = serial.Serial(args.port, args.baud, timeout=1)
        time.sleep(2) # Wait for reset
        print(f"Connected to {args.port}")
        
        while True:
            if ser.in_waiting > 0:
                line = ser.readline().decode('utf-8').strip()
                try:
                    parts = line.split(',')
                    if len(parts) == 4:
                        e1, e2, e3, e4 = map(float, parts)
                        print(f"E1: {e1:0.2f} | E2: {e2:0.2f} | E3: {e3:0.2f} | E4: {e4:0.2f}")
                    else:
                        print(f"Raw: {line}")
                except ValueError:
                    print(f"Error parsing: {line}")
                    
    except serial.SerialException as e:
        print(f"Serial Error: {e}")
    except KeyboardInterrupt:
        print("\nExiting...")
    finally:
        if 'ser' in locals() and ser.is_open:
            ser.close()

if __name__ == '__main__':
    main()
