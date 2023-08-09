import threading
import time
import os

import nSerial

#-- Thread
thread = None
thread_lock = threading.Lock()

#-- End of thread

def decodeSerialData(data):
    uni_serial_data = data.strip(b'\r\n')
    asc_serial_data = uni_serial_data.decode('utf-8')

    return asc_serial_data.split(',')

def serialHandle(serialName:str):
    ser = nSerial.serialDo(serialName)
    if ser.open(9600, 8, timeout=None):
        while True:
            decode_serial_data = decodeSerialData(ser.read_until(b'\n'))
            p0, p1 = float(decode_serial_data[0]), float(decode_serial_data[1])
            print(f'pressure0:{p0}, pressure1:{p1}')
            
        time.sleep(0.2)

if __name__ == '__main__':
    serialName = 'COM12'
    thread = threading.Thread(target=serialHandle, args=(serialName, ), daemon=True)
    thread.start()
    
    os.system('pause')
    