import serial.tools.list_ports
import serial
import json

def get_ports():
    print('get_ports in nSerial')
    l_ports = []
    ports = serial.tools.list_ports.comports()
    for port in ports:
        l_ports.append( str(port) )
    return l_ports

class serialDo:    
    def __init__(self, p):
        self.port = p
        self.ser = None

    def checkStatus(self):
        if self.ser :
            return self.ser.is_open
        else :
            return False
    
    def flush(self):
        self.ser.flush()

    def open(self, baud, bytesize, parity=serial.PARITY_NONE, timeout=1):
        # with serial.Serial(self.port, baudrate=baud, bytesize=bytesize, parity=parity, stopbits=1, timeout=timeout) as self.ser:
        try: 
            self.ser = serial.Serial(self.port)
            self.ser.baudrate = baud
            self.ser.bytesize = bytesize
            self.ser.parity = parity
            self.ser.stopbits = 1
            self.ser.timeout = timeout
            print('open in nSerial', self.ser)
            if self.checkStatus():
                self.ser.close()
            self.ser.open()
            return self.ser.is_open
        except ValueError:
            print('command: serial.open. Exception: ValueError')            
        # else: 
        #     print('command: serial.open. Exception: Unknown')

    def close(self):
        try:
            if self.checkStatus():
                self.ser.close()
                print('command:serial.close. ser.is_open', self.ser.is_open)

        except ValueError:
            print('command: serial.close. Exception: ValueError')            
        # except Exception:
        #     print('command: serial.open. Exception: Exception')
        # else: 
        #     print('command: serial.open. Exception: Unknown')        
        
    def write(self, data):
        try:
            data_length = self.ser.write(data)
            # print('serial.write has written', data_length, ' bytes of data')
        except ValueError:
            print('command: serial.write. Exception: ValueError')            
        # except Exception:
        #     print('command: serial.open. Exception: Exception')
        # else: 
        #     print('command: serial.open. Exception: Unknown')        

    def read(self, num):        
        try:
            read_data = self.ser.read(num)
            # print('serial.read has read data:', read_data)
            return read_data
        except ValueError:
            print('command: serial.read. Exception: ValueError')            
        # except Exception:
        #     print('command: serial.open. Exception: Exception')
        # else: 
        #     print('command: serial.open. Exception: Unknown')      
        # 
    def readline(self):        
        try:
            read_data = self.ser.readline()
            # print('serial.readline has read data:', read_data)
            return read_data
        except ValueError:
            print('command: serial.read. Exception: ValueError')            
        # except Exception:
        #     print('command: serial.open. Exception: Exception')
        # else: 
        #     print('command: serial.open. Exception: Unknown') 
        #      

    def read_until(self, expected=b'\n', size=None):        
        try:
            # self.ser.flushInput()
            read_data = self.ser.read_until(expected, size)
            # print('serial.read_until has read data:', read_data)
            return read_data
        except ValueError:
            print('command: serial.read_until. Exception: ValueError')            
        # except Exception:
        #     print('command: serial.open. Exception: Exception')
        # else: 
        #     print('command: serial.open. Exception: Unknown') 
        #   
