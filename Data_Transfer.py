import serial
import time

ser = serial.Serial("COM5", 115200)

msg = ""
i = 0
while msg != "quit":
    msg = "ab"#input("Msg: ")

    ser.write(msg.encode("ascii"))

    result = ser.read()

    if result == b"1":
        continue
        #print(f"Nominal Transfer Confirmed")
    elif result == b"0":
        print(f"Failed Transfer Confirmed after {i} Iterations")
        break
    else:
        print(f"COM BLACKOUT After {i} Iterations")
        print(result)
        break

ser.close()
