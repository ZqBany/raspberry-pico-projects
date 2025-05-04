from machine import Pin, Timer
from time import sleep_ms, ticks_ms

RELAY_OFF = const(1)
RELAY_ON = const(0)
LED_OFF = const(0)
LED_ON = const(1)
SENSOR_DETECTED = const(0)
SENSOR_ABSENT = const(1)

DETECTED_BOUNCETICKS=const(10) #20 ms tick
ABSENT_BOUNCETICKS=const(10)
ABSENT_AFTER_DETECTED_BOUNCETICKS=const(3*50)
TURN_ON_EXTRA_TICKS=const(10*50)
MAX_TURN_ON_TICKS=const(180*50)

DEBUG = const(False)

detected_debounce=0
absent_debounce=0
absent_after_detected_debounce=0
extra_ticks=0

led = Pin(25, Pin.OUT)
proximity_sensor = Pin(14, Pin.IN, Pin.PULL_UP)
water_relay = Pin(16, Pin.OUT)
sensor_state = proximity_sensor.value()
sensor_changed = False
was_turned_on = False
max_turn_on_ticks=0

def detected(sensor) -> bool:
    return not sensor.value()

def proximity_change_callback(pin):
    global sensor_changed
    global sensor_state
    flags = pin.irq().flags()
    if flags & Pin.IRQ_RISING:
        # handle proximity NOT detecting
        if DEBUG:
            print('rising')
        sensor_state = SENSOR_ABSENT
    elif flags & Pin.IRQ_FALLING:
        # handle proximity detecting
        if DEBUG:
            print('falling')
        sensor_state = SENSOR_DETECTED
    sensor_changed = True

proximity_sensor.irq(trigger=Pin.IRQ_FALLING | Pin.IRQ_RISING, handler=proximity_change_callback)
#GPIO.add_event_detect(14, GPIO.RISING, callback=my_callback, bouncetime=200)

def init():
    global sensor_state
    global sensor_changed
    led.value(LED_OFF)
    water_relay.value(RELAY_OFF)
    sensor_state = proximity_sensor.value()
    sensor_changed = False
    detected_debounce=0
    absent_debounce=0
    absent_after_detected_debounce=0
    extra_ticks=0
    max_turn_on_ticks=0
    
def turn_off():
    led.value(LED_OFF)
    water_relay.value(RELAY_OFF)

def turn_on():
    led.value(LED_ON)
    water_relay.value(RELAY_ON)

init()
while True:
    if sensor_changed == True:
        if sensor_state == SENSOR_DETECTED:
            detected_debounce = DETECTED_BOUNCETICKS
        elif sensor_state == SENSOR_ABSENT:
            absent_debounce = ABSENT_BOUNCETICKS
        sensor_changed = False
      
    if detected(proximity_sensor):
        if detected_debounce <=0 and absent_after_detected_debounce<=0 and max_turn_on_ticks > 0:
            turn_on()
            was_turned_on = True
        else:
            if DEBUG and detected_debounce > 0:
                print('debouncing detected')
            if DEBUG and absent_after_detected_debounce > 0:
                print('debouncing absent after detected')
    else:
        if absent_debounce <=0:
            if was_turned_on == True:
                was_turned_on = False
                absent_after_detected_debounce = ABSENT_AFTER_DETECTED_BOUNCETICKS + TURN_ON_EXTRA_TICKS
                extra_ticks = TURN_ON_EXTRA_TICKS
            if extra_ticks <= 0:
                turn_off()
                max_turn_on_ticks = MAX_TURN_ON_TICKS
        elif DEBUG:
            print('debouncing absent')
    if max_turn_on_ticks > 0:
        max_turn_on_ticks = max_turn_on_ticks - 1;
    else:
        turn_off()    
    if absent_after_detected_debounce > 0:
        absent_after_detected_debounce = absent_after_detected_debounce - 1 
    if absent_debounce > 0:
        absent_debounce = absent_debounce - 1
    if detected_debounce > 0:
        detected_debounce = detected_debounce - 1
    if extra_ticks > 0:
        extra_ticks = extra_ticks - 1
    sleep_ms(20)
    