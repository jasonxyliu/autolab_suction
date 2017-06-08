"""
Python wrapper for the Arduino vacuum suction control
Author: Jason Liu
"""
from SerialCommInterface import VacuumSerialInterface
from time import sleep

import logging
import time
import IPython

# DEFAULT_CONFIG = "cfg/vacuum.yaml"


class Vacuum:
    """
    Abstraction for Arduino vacuum suction. 
    Low level, SerialCommInterface, uses pySerial to communicate with Arduino.
    """

    def __init__(self):
        self._vacuum = VacuumSerialInterface(vacuum_comm="/dev/ttyUSB0", baudrate=115200)
        print 'start robot'
        self._vacuum.start()
    
    def vacuum_on(self):
        self._vacuum.vacuum_on()
    
    def vacuum_off(self):
        self._vacuum.vacuum_off()

    def vacuum_stop(self):
        self._vacuum.stop()
        
def test_vacuum(sleep=5):
    vac = Vacuum()

    print 'suction on'
    vac.vacuum_on()
    time.sleep(sleep)
    print 'suction off'
    vac.vacuum_off()
    time.sleep(sleep-2)
    print 'stop robot'
    vac.vacuum_stop()

if __name__ == '__main__':
    test_vacuum()
