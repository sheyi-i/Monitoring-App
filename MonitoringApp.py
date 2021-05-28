import serial
import keyboardimport threading
import time

# This program enables running of the Monitor App

com_port = "COM3"
baud_rate = 115200 # This baud rate worked for me.

# read data from serial port
def read_data():
    print("Monitoring Mode - Shows the current state value from the port: ")
    while 1:
        if serial_port.in_waiting > 0:
            print("\nState: " + str(serial_port.read()))


# main function
if __name__ == "__main__":
    print("This app allows a user to change the state of their board either forwards or backwards")
    
	
	# serial port configuration
    serial_port = serial.Serial(com_port, baud_rate)

    # runs the read function constantly in the background
    read_thread = threading.Thread(target=read, daemon=True)
    read_thread.start()
    if not serial_port.is_open:
        serial_port.open()

    print("")
    print("Changing the States: ")
    while 1:
        write_system_state = input("Change State. F->(Next State) | B->(Previous State)]: ")
        serial_port.write(bytes(write_system_state, 'utf-8'))
        time.sleep(1)
