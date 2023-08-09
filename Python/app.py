import threading
import time
import timeit
import os

import nSerial
from Algorithm import torchPID

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
            setpoint0, setpoint1 = 0.3, 0.2
            t = time.time()
            MLPID0 = torchPID.Net()
            output0 = round(MLPID0.train(setpoint0, p0),2)
            print(f'time in generating output0: {time.time() - t}')
            #todo
            output1 = round(MLPID0.train(setpoint1, p1),2)
            print('output0:{:.2f}, output1:{:.2f}'.format(output0, output1))
            
            time.sleep(0.2)
            

if __name__ == '__main__':
    serialName = 'COM12'
    thread = threading.Thread(target=serialHandle, args=(serialName, ), daemon=True)
    thread.start()

    os.system('pause')
    