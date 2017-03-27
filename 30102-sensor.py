#!/usr/bin/python
# =============================================================================
#        File : 30102-sensor.py
# Description : Read data from Maxim 30102 Sensor 
#      Author : Victor Chinn
#        Date : 02/27/2017
# =============================================================================

import smbus    # System Management Bus
import time     # time utilities
from gpio import PiGpio

I2C_WRITE_ADDR = 0xAE
I2C_READ_ADDR = 0xAF

# register addresses
REG_INTR_STATUS_1 = 0x00
REG_INTR_STATUS_2 = 0x01
REG_INTR_ENABLE_1 = 0x02
REG_INTR_ENABLE_2 = 0x03
REG_FIFO_WR_PTR = 0x04
REG_OVF_COUNTER = 0x05
REG_FIFO_RD_PTR = 0x06
REG_FIFO_DATA = 0x07
REG_FIFO_CONFIG = 0x08
REG_MODE_CONFIG = 0x09
REG_SPO2_CONFIG = 0x0A
REG_LED1_PA = 0x0C
REG_LED2_PA = 0x0D
REG_PILOT_PA = 0x10
REG_MULTI_LED_CTRL1 = 0x11
REG_MULTI_LED_CTRL2 = 0x12
REG_TEMP_INTR = 0x1F
REG_TEMP_FRAC = 0x20
REG_TEMP_CONFIG = 0x21
REG_PROX_INT_THRESH = 0x30
REG_REV_ID = 0xFE
REG_PART_ID = 0xFF


# =============================================================================
# ctypes is an advanced ffi (Foreign Function Interface) package for Python
# that provides  call functions in dlls/shared libraries and has extensive
# facilities to create, access and manipulate simple and complicated
# C data types in Python
# =============================================================================
from ctypes import c_short
from ctypes import c_byte
from ctypes import c_ubyte

def getShort(data, index):
  # return two bytes from data as a signed 16-bit value
  return c_short((data[index+1] << 8) + data[index]).value

def getUShort(data, index):
  # return two bytes from data as an unsigned 16-bit value
  return (data[index+1] << 8) + data[index]

def getChar(data,index):
  # return one byte from data as a signed char
  result = data[index]
  if result > 127:
    result -= 256
  return result

def getUChar(data,index):
  # return one byte from data as an unsigned char
  result =  data[index] & 0xFF
  return result

# =============================================================================
# create a sensor object
# -----------------------------------------------------------------------------

DEVICE = 0x57
Data_IR = []
Data_RED = []

class MAX30102(object):
    """Raspberry Pi 'Maxim 30102 Sensor'."""

    # addr = I2C Chip Address
    def __init__(self,addr=DEVICE):
        self.bus = smbus.SMBus(1)   # Rev 2 Pi, Pi 2 & Pi 3 uses bus 1
                                    # Rev 1 Pi uses bus 0
        self.addr = addr            # register the address with the object

    def readBMP280All(self, addr=DEVICE):
        # Register Addresses
        REG_DATA = 0xF7
        REG_CONTROL = 0xF4
        REG_CONFIG  = 0xF5

        # Oversample setting - page 27
        OVERSAMPLE_TEMP = 2
        OVERSAMPLE_PRES = 2
        MODE = 1

        # write oversample configuration and mode
        control = OVERSAMPLE_TEMP<<5 | OVERSAMPLE_PRES<<2 | MODE
        self.bus.write_byte_data(self.addr, REG_CONTROL, control)

        # Read blocks of calibration data from EEPROM
        # See Page 22 data sheet
        cal1 = self.bus.read_i2c_block_data(self.addr, 0x88, 24)
        cal2 = self.bus.read_i2c_block_data(self.addr, 0xA1, 1)
        cal3 = self.bus.read_i2c_block_data(self.addr, 0xE1, 7)

        # Convert byte data to word values
        dig_T1 = getUShort(cal1, 0)
        dig_T2 = getShort(cal1, 2)
        dig_T3 = getShort(cal1, 4)

        dig_P1 = getUShort(cal1, 6)
        dig_P2 = getShort(cal1, 8)
        dig_P3 = getShort(cal1, 10)
        dig_P4 = getShort(cal1, 12)
        dig_P5 = getShort(cal1, 14)
        dig_P6 = getShort(cal1, 16)
        dig_P7 = getShort(cal1, 18)
        dig_P8 = getShort(cal1, 20)
        dig_P9 = getShort(cal1, 22)

        #   dig_H1 = getUChar(cal2, 0)
        #   dig_H2 = getShort(cal3, 0)
        #   dig_H3 = getUChar(cal3, 2)
        #
        #   dig_H4 = getChar(cal3, 3)
        #   dig_H4 = (dig_H4 << 24) >> 20
        #   dig_H4 = dig_H4 | (getChar(cal3, 4) & 0x0F)
        #
        #   dig_H5 = getChar(cal3, 5)
        #   dig_H5 = (dig_H5 << 24) >> 20
        #   dig_H5 = dig_H5 | (getUChar(cal3, 4) >> 4 & 0x0F)

        #   dig_H6 = getChar(cal3, 6)

        data = self.bus.read_i2c_block_data(self.addr, REG_DATA, 6)
        pres_raw = (data[0] << 12) | (data[1] << 4) | (data[2] >> 4)
        temp_raw = (data[3] << 12) | (data[4] << 4) | (data[5] >> 4)

        # Refine temperature based on calibration per spec
        var1 = ((((temp_raw>>3)-(dig_T1<<1)))*(dig_T2)) >> 11
        var2 = (((((temp_raw>>4) - (dig_T1)) * ((temp_raw>>4) - (dig_T1))) >> 12) * (dig_T3)) >> 14
        t_fine = var1+var2
        temperature = float(((t_fine * 5) + 128) >> 8);

        # Refine pressure and adjust for temperature
        var1 = t_fine / 2.0 - 64000.0
        var2 = var1 * var1 * dig_P6 / 32768.0
        var2 = var2 + var1 * dig_P5 * 2.0
        var2 = var2 / 4.0 + dig_P4 * 65536.0
        var1 = (dig_P3 * var1 * var1 / 524288.0 + dig_P2 * var1) / 524288.0
        var1 = (1.0 + var1 / 32768.0) * dig_P1
        if var1 == 0:
            pressure=0
        else:
            pressure = 1048576.0 - pres_raw
            pressure = ((pressure - var2 / 4096.0) * 6250.0) / var1
            var1 = dig_P9 * pressure * pressure / 2147483648.0
            var2 = pressure * dig_P8 / 32768.0
            pressure = pressure + (var1 + var2 + dig_P7) / 16.0

        return temperature/100.0,pressure/100.0

    def readMAX30102_ID(self):
        # read two registers using block data read of two bytes
        REG_ID     = 0xFE
        (chip_id, chip_version) = self.bus.read_i2c_block_data(self.addr, REG_ID, 2)
        return (chip_id, chip_version)

    def readMAX30102_INT_Status(self):
        int_status1 = self.bus.read_byte_data(self.addr, REG_INTR_STATUS_1)
        int_status2 = self.bus.read_byte_data(self.addr, REG_INTR_STATUS_2)
        return (int_status1, int_status2)
        
    def MAX30102_INIT(self):
        # Initialize the MAX30102
        self.bus.write_byte_data(self.addr,REG_INTR_ENABLE_1,0xD0)  # ORG INTR setting

        self.bus.write_byte_data(self.addr,REG_FIFO_WR_PTR,0x00)    # FIFO_WR_PTR[4:0]
        self.bus.write_byte_data(self.addr,REG_OVF_COUNTER,0x00)    # OVF_COUNTER[4:0]
        self.bus.write_byte_data(self.addr,REG_FIFO_RD_PTR,0x00)    # FIFO_RD_PTR[4:0]

        self.bus.write_byte_data(self.addr,REG_FIFO_CONFIG,0x1F)    # sample avg = 1, fifo rollover=false, fifo almost full = 17 MBED STYLE
        
        self.bus.write_byte_data(self.addr,REG_MODE_CONFIG,0x03)    # 03.24.17 0x02 for Red only, 0x03 for SpO2 mode 0x07 multimode LED
 
        self.bus.write_byte_data(self.addr,REG_SPO2_CONFIG,0x27)    # SPO2_ADC range = 4096nA, SPO2 sample rate (self.addr,100 Hz), LED pulseWidth (self.addr,400uS)
        self.bus.write_byte_data(self.addr,REG_LED1_PA,0x24)        # value for ~ 7mA for LED1
        self.bus.write_byte_data(self.addr,REG_LED2_PA,0x24)        # value for ~ 7mA for LED2
        self.bus.write_byte_data(self.addr,REG_PILOT_PA,0x7F)       # value for ~ 25mA for Pilot LED
        return

    def MAX30102_RESET(self):
        # reset the MAX 30102
        self.bus.write_byte_data(self.addr,REG_MODE_CONFIG,0x40)
        return

    def MAX30102_READ_FIFO(self):
        # read the fifo

        #this is moved to inside the main while loop
        #int_status1 = self.bus.read_byte_data(self.addr, REG_INTR_STATUS_1)
        #int_status2 = self.bus.read_byte_data(self.addr, REG_INTR_STATUS_2)
        # (int_status1, int_status2) = self.bus.read_i2c_block_data(self.addr, REG_ID, 2)
        #print   "I_stat:",int_status1,":",int_status2

        
        #write_ptr = self.bus.read_byte_data(self.addr, REG_FIFO_WR_PTR)
        #read_ptr  = self.bus.read_byte_data(self.addr, REG_FIFO_RD_PTR)
        #num_samples = abs (32 + write_ptr - read_ptr) % 32 
        #print "BEFORE W_PTR:",write_ptr,"R_PTR:",read_ptr,"#_S:",num_samples

        # from the MODE of 30102, determine the number of active channels for measurement data 
        # 3 bytes for RED only channel or 6 bytes for RED and IR data channels
        if (self.bus.read_byte_data(self.addr, REG_MODE_CONFIG) == 2):
            # MODE 0x02 == Heart Rate RED only measurements
            data = self.bus.read_i2c_block_data(self.addr, REG_FIFO_DATA, 3)
            #print "RED:",data[0],data[1],data[2]
            Data_RED.append((data[0]<<16)|(data[1]<<8)|(data[2]))
        else:
            # MODE 0x03 == Heart Rate RED and SpO2 IR measurements
            data = self.bus.read_i2c_block_data(self.addr, REG_FIFO_DATA, 6)
            #print "RED:",data[0],data[1],data[2]
            #print "IR:",data[3],data[4],data[5]
            Data_RED.append((data[0]<<16)|(data[1]<<8)|(data[2]))
            Data_IR.append((data[3]<<16)|(data[4]<<8)|(data[5]))

        # check the write and read pointers to see if they were updated
        #write_ptr = self.bus.read_byte_data(self.addr, REG_FIFO_WR_PTR)
        #read_ptr  = self.bus.read_byte_data(self.addr, REG_FIFO_RD_PTR)
        #num_samples = abs (32 + write_ptr - read_ptr) % 32 
        #print "AFTER read 1 sample W_PTR:",write_ptr,"R_PTR:",read_ptr,"#_S:",num_samples
        ##import pdb; pdb.set_trace()
 
        return
    


# =============================================================================
# main to test from CLI
def main():
    
    # create an instance of my RPi Maxim 30102 Sensor Object
    RPiSensor = MAX30102()
    RPiGPIO = PiGpio()
     
    RPiSensor.MAX30102_RESET()

    # Press key to start
 
    RPiSensor.MAX30102_INIT()

    # Read the Sensor ID.
    #(chip_id, chip_version) = RPiSensor.readMAX30102_ID()
    #print "    Chip ID :", chip_id
    #print "    Version :", chip_version

    EnterKey = raw_input("Place finger then press ENTER to begin...")
    
    print "Starting 500 measurements..."
    
    # Make 500 measurements    
    
    KeepRunning = True    
    while (KeepRunning == True):
        # now setup to make measurements

        for num in range(500):
            while (RPiGPIO.read_int()==1):
                pass
            
            # get the interrupt status and then reset it
            (int_status1,int_status2) = RPiSensor.readMAX30102_INT_Status()
            #print   "I_stat:",int_status1,":",int_status2

            RPiSensor.MAX30102_READ_FIFO() 
            #Data_Samples.append((D1,D2,D3,D4))
            #import pdb; pdb.set_trace()

        # # Read the Sensor ID.
        # (chip_id, chip_version) = RPiSensor.readMAX30102_ID()
        # print "    Chip ID :", chip_id
        # print "    Version :", chip_version
        
        #count = 0
        #for item in Data_RED:
        #    print "Data_RED ",count,":",item
        #    count = count + 1
        #
        #count = 0
        #for item in Data_IR:
        #    print "Data_IR  ",count,":",item
        #    count = count + 1
        
        KeepRunning = False  # stop here...


    # at this point the first 500 samples are recorded...

    #import pdb; pdb.set_trace()
     
    # Data_Red has 500 elements and Data_IR has 500 elements 

    
    while (1):
        # shift last 400 samples to first 400 samples by deleting first 100 samples
        if (len(Data_RED) == 500):
            del Data_RED[0:100]
    
        if (len(Data_IR) == 500):
            del Data_IR[0:100]
    
        # read another 100 samples and append 
        #import pdb; pdb.set_trace()
        
        KeepRunning = True
        while (KeepRunning == True) :
            # now setup to make measurements

            print "Starting 100 measurements..."
            for num in range(100):
                while (RPiGPIO.read_int()==1):
                    pass
            
                # get the interrupt status and then reset it
                (int_status1,int_status2) = RPiSensor.readMAX30102_INT_Status()
                #print   "I_stat:",int_status1,":",int_status2

                RPiSensor.MAX30102_READ_FIFO() # data is stored in Data_RED and Data_IR
                #Data_Samples.append((D1,D2,D3,D4))
                #import pdb; pdb.set_trace()
                
            # 100 more measurements added ... making 500 valid total samples have been measured         
            KeepRunning = False  # stop here...
    
        # then PROCESS to determine heart rate and SpO2 measurements

        # PROCESS DATA_RED AND DATA_IR to determine heartrate and SpO2 level

        # print the results
        print "Results are: ", len(Data_RED)," Data_RED ",len(Data_IR)," Data_IR samples."

        # then loop around to do this every second





if __name__=="__main__":
   main()