import RPi.GPIO as GPIO

SENSOR_INT = 16    

class PiGpio(object):
    """Raspberry Pi Internet 'IoT GPIO'."""

    def __init__(self):
        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(SENSOR_INT, GPIO.IN)   # 30102 INT PIN as input
#        GPIO.setup(LED2_PIN, GPIO.OUT)      # GRN LED as output
#        GPIO.setup(LED3_PIN, GPIO.OUT)      # BLU LED as output
#        GPIO.setup(SWITCH_PIN, GPIO.IN)     # Switch as input w/pu
#        GPIO.setup(SWITCH_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)

    def read_int(self):
        """Read the INT state."""
        switch = GPIO.input(SENSOR_INT)
        return switch
