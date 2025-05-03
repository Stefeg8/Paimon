import serial
import time

def read_tfmini_data(ser):
    while True:
        if ser.read() == b'\x59':  # First header byte
            if ser.read() == b'\x59':  # Second header byte
                data = ser.read(7)
                if len(data) == 7:
                    distance = data[0] + (data[1] << 8)
                    strength = data[2] + (data[3] << 8)
                    checksum = (0x59 + 0x59 + sum(data[:6])) & 0xFF
                    if checksum == data[6]:
                        print(f"Distance: {distance} cm, Strength: {strength}")
                    else:
                        print("Checksum error")
                else:
                    print("Incomplete frame")

if __name__ == "__main__":
    try:
        # Open serial port to TFMini Plus
        ser = serial.Serial(
            port='/dev/serial0',
            baudrate=115200,
            timeout=1
        )
        print("Starting TFMini Plus serial reader...")
        read_tfmini_data(ser)
    except KeyboardInterrupt:
        print("Stopping...")
    except Exception as e:
        print("Error:", e)
    finally:
        if 'ser' in locals():
            ser.close()
