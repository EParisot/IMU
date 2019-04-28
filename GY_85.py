# coding: utf-8

from gyro.ITG3200 import ITG3200
from accel.ADXL345 import ADXL345
from magneto.QMC5883L import QMC5883L

from time import sleep

class GY85(object):
    def __init__(self):
        self.gyro = ITG3200()
        self.accel = ADXL345()
        self.magneto = QMC5883L()
        self.magneto.set_declination(0.58)                           #Paris declination
    
    def read(self):
        return ((self.accel.read_data()), (self.gyro.read_data()))
    
    def magneto_read(self):
        return((self.magneto.get_magnet()), self.magneto.get_temp(), self.magneto.get_bearing())
