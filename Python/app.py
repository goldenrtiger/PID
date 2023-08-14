import threading
import time
import timeit
import os
from datetime import datetime

import nSerial
from Algorithm import torchPIDSystem

#-- Thread
thread = None
thread_lock = threading.Lock()

#-- End of thread

def decodeSerialData(data):
    uni_serial_data = data.strip(b'\r\n')
    asc_serial_data = uni_serial_data.decode('utf-8')

    return asc_serial_data

def encodeMessage(message:str)->bytes:
    return (message + '\r\n').encode('utf-8')    

def calibration(ser):
    ser.write(encodeMessage('Calibration'))
    time.sleep(1)
    print(ser.read_until(b'\n'))

def serialHandle(serialName:str):
    ser = nSerial.serialDo(serialName)
    if ser.open(115200, 8, timeout=None):
        time.sleep(1)
        ser.write(encodeMessage("Calibration"))
        MLPID0 = torchPIDSystem.Net()

        while True:
            asc_serial_data = decodeSerialData(ser.read_until(b'\n'))
            if asc_serial_data == 'Calibration done!':
                print(asc_serial_data)
            else:
                serial_data = asc_serial_data.split(',')
                p0, p1 = float(serial_data[0]), float(serial_data[1])
                print(f'***pressure0:{p0}, pressure1:{p1}')
                setpoint0, setpoint1 = 0.21, 0.2
                t = time.time()
                output0 = round(MLPID0.train(setpoint0, p0),2)
                # print(f'time in generating output0: {time.time() - t} **: {datetime.fromtimestamp(t)}.')
                #todo
                # output1 = round(MLPID0.train(setpoint1, p1),2)
                output1 = 0
                print('***output0:{:.2f}, output1:{:.2f}'.format(output0, output1))
                message = "PIDML:" + str(output0) + ":" + str(setpoint0) + ":" + str(output1) + ":" + str(setpoint1) 
                ser.write(encodeMessage(message))
                
            # time.sleep(0.2)
            

if __name__ == '__main__':
    serialName = 'COM4'
    thread = threading.Thread(target=serialHandle, args=(serialName, ), daemon=True)
    thread.start()

    os.system('pause')
    