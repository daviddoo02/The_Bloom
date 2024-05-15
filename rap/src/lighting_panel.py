#!/usr/bin/env python3
# Adapted from NeoPixel library strandtest example by Author: Tony DiCola (tony@tonydicola.com)

__authors__ = "David Ho"

import time
from rpi_ws281x import *
import RPi.GPIO as GPIO
import random

# LED strip configuration:
LED_COUNT      = 27*10     # Number of LED pixels.
LED_PIN        = 18      # GPIO pin connected to the pixels (18 uses PWM!).
#LED_PIN        = 10      # GPIO pin connected to the pixels (10 uses SPI /dev/spidev0.0).
LED_FREQ_HZ    = 800000  # LED signal frequency in hertz (usually 800khz)
LED_DMA        = 10      # DMA channel to use for generating a signal (try 10)
LED_BRIGHTNESS = 10      # Set to 0 for darkest and 255 for brightest
LED_INVERT     = False   # True to invert the signal (when using NPN transistor level shift)
LED_CHANNEL    = 0       # set to '1' for GPIOs 13, 19, 41, 45 or 53

class UltraSonic:
    def __init__(self, trigger, echo):
        # GPIO Mode (BOARD / BCM)
        GPIO.setmode(GPIO.BCM)

        # set GPIO Pins
        self.GPIO_TRIGGER = trigger
        self.GPIO_ECHO = echo

        # set GPIO direction (IN / OUT)
        GPIO.setup(self.GPIO_TRIGGER, GPIO.OUT)
        GPIO.setup(self.GPIO_ECHO, GPIO.IN)

    def get_distance(self):
        # set Trigger to HIGH
        GPIO.output(self.GPIO_TRIGGER, True)

        # set Trigger after 0.01ms to LOW
        time.sleep(0.00001)
        GPIO.output(self.GPIO_TRIGGER, False)

        starttime = time.time()
        stoptime = time.time()

        # save StartTime
        while GPIO.input(self.GPIO_ECHO) == 0:
            starttime = time.time()

        # save time of arrival
        while GPIO.input(self.GPIO_ECHO) == 1:
            stoptime = time.time()

        # time difference between start and arrival
        timeelapsed = stoptime - starttime
        # multiply with the sonic speed (34300 cm/s)
        # and divide by 2, because there and back
        distance = (timeelapsed * 34300) / 2

        return distance
    
    def shut_down(self):
        GPIO.cleanup()
        return

# Define functions which animate LEDs in various ways.
def colorWipe(strip, color, wait_ms=50):
    """Wipe color across display a pixel at a time."""
    for i in range(strip.numPixels()):
        strip.setPixelColor(i, color)
        strip.show()
        time.sleep(wait_ms/1000.0)

def theaterChase(strip, color, wait_ms=50, iterations=10):
    """Movie theater light style chaser animation."""
    for j in range(iterations):
        for q in range(3):
            for i in range(0, strip.numPixels(), 3):
                strip.setPixelColor(i+q, color)
            strip.show()
            time.sleep(wait_ms/1000.0)
            for i in range(0, strip.numPixels(), 3):
                strip.setPixelColor(i+q, 0)

def wheel(pos):
    """Generate rainbow colors across 0-255 positions."""
    if pos < 85:
        return Color(pos * 3, 255 - pos * 3, 0)
    elif pos < 170:
        pos -= 85
        return Color(255 - pos * 3, 0, pos * 3)
    else:
        pos -= 170
        return Color(0, pos * 3, 255 - pos * 3)

def rainbow(strip, wait_ms=20, iterations=1):
    """Draw rainbow that fades across all pixels at once."""
    for j in range(256*iterations):
        for i in range(strip.numPixels()):
            strip.setPixelColor(i, wheel((i+j) & 255))
        strip.show()
        time.sleep(wait_ms/1000.0)

def rainbowCycle(strip, wait_ms=20, iterations=5):
    """Draw rainbow that uniformly distributes itself across all pixels."""
    for j in range(256*iterations):
        for i in range(strip.numPixels()):
            strip.setPixelColor(i, wheel((int(i * 256 / strip.numPixels()) + j) & 255))
        strip.show()
        time.sleep(wait_ms/1000.0)

def theaterChaseRainbow(strip, wait_ms=50):
    """Rainbow movie theater light style chaser animation."""
    for j in range(256):
        for q in range(3):
            for i in range(0, strip.numPixels(), 3):
                strip.setPixelColor(i+q, wheel((i+j) % 255))
            strip.show()
            time.sleep(wait_ms/1000.0)
            for i in range(0, strip.numPixels(), 3):
                strip.setPixelColor(i+q, 0)

def startupAnimation(strip, wait_ms=50):
    """Start up all LEDs with dim white and gradually increase brightness."""
    target_brightness = 50  # Brightness level to reach
    current_brightness = 0

    # Gradually increase brightness
    while current_brightness < target_brightness:
        for i in range(strip.numPixels()):
            strip.setPixelColor(i, Color(255,255,255))  # Start with the dimmest color
        strip.setBrightness(current_brightness)
        strip.show()
        current_brightness += 1
        time.sleep(wait_ms / 1000)

def transitionToColors(strip, wait_ms=0.75):
    """Make LEDs glow bright and slowly transition each 27-pixel unit to different colors."""
    for j in range(256):
        for unit in range(LED_COUNT // 27):
            color = wheel((j + unit * 5) % 255)  # Adjust the value to change the color transition speed
            for i in range(unit * 27, (unit + 1) * 27):
                strip.setPixelColor(i, color)
        strip.show()
        time.sleep(wait_ms / 50)

def default_animation(strip, duration=0.75, steps=50):
    """Default animation: fades white LEDs in and out."""
    fade_steps = steps // 2  # Divide the steps into fading in and fading out

    # Fading in
    for i in range(fade_steps):
        brightness = int((i / fade_steps) * 255)
        for j in range(LED_COUNT):
            strip.setPixelColor(j, Color(brightness, brightness, brightness))
        strip.show()
        time.sleep(2 * duration / fade_steps)

    # Fading out
    for i in range(fade_steps, 0, -1):
        brightness = int((i / fade_steps) * 255)
        for j in range(LED_COUNT):
            strip.setPixelColor(j, Color(brightness, brightness, brightness))
        strip.show()
        time.sleep(duration / fade_steps)

def interpolate_color(color1, color2, ratio):
    """Interpolate between two colors."""
    r = int((1 - ratio) * ((color1 >> 16) & 0xFF) + ratio * ((color2 >> 16) & 0xFF))
    g = int((1 - ratio) * ((color1 >> 8) & 0xFF) + ratio * ((color2 >> 8) & 0xFF))
    b = int((1 - ratio) * (color1 & 0xFF) + ratio * (color2 & 0xFF))
    return Color(r, g, b)

def warm_color(strip, duration=2):
    """Transition the whole strip to warm colors for a specified duration."""
    # Define warm colors (red, orange, yellow)
    warm_colors = [
        Color(255, 0, 0),    # Red
        Color(255, 127, 0),  # Orange
        Color(255, 255, 0),   # Yellow
        Color(255, 127, 0),  # Orange
        Color(255, 0, 0)    # Red
    ]
    
    total_time = duration * 1000  # Convert duration to milliseconds
    start_time = time.time() * 1000  # Get current time in milliseconds

    elapsed_time = 0
    color_index = 0

    while elapsed_time < total_time:
        color1 = warm_colors[color_index]
        color2 = warm_colors[(color_index + 1) % len(warm_colors)]

        # Calculate interpolation ratio based on elapsed time and duration
        ratio = elapsed_time / total_time

        # Interpolate between the two colors
        interpolated_color = interpolate_color(color1, color2, ratio)

        for i in range(LED_COUNT):
            strip.setPixelColor(i, interpolated_color)
        strip.show()

        # time.sleep(0.0000001)

        elapsed_time = time.time() * 1000 - start_time

def cool_color(strip, duration=2):
    """Transition the whole strip to cool colors for a specified duration."""
    # Define cool colors (blue, cyan, purple)
    cool_colors = [
        Color(0, 0, 255),    # Blue
        Color(0, 255, 255),  # Cyan
        Color(128, 0, 128),   # Purple
        Color(0, 255, 255),  # Cyan
        Color(0, 0, 255)    # Blue
    ]
    
    total_time = duration * 1000  # Convert duration to milliseconds
    start_time = time.time() * 1000  # Get current time in milliseconds

    elapsed_time = 0
    color_index = 0

    while elapsed_time < total_time:
        color1 = cool_colors[color_index]
        color2 = cool_colors[(color_index + 1) % len(cool_colors)]

        # Calculate interpolation ratio based on elapsed time and duration
        ratio = elapsed_time / total_time

        # Interpolate between the two colors
        interpolated_color = interpolate_color(color1, color2, ratio)

        for i in range(LED_COUNT):
            strip.setPixelColor(i, interpolated_color)
        strip.show()

        elapsed_time = time.time() * 1000 - start_time

# Main program logic follows:
if __name__ == '__main__':
    # Create NeoPixel object with appropriate configuration.
    strip = Adafruit_NeoPixel(LED_COUNT, LED_PIN, LED_FREQ_HZ, LED_DMA, LED_INVERT, LED_BRIGHTNESS, LED_CHANNEL)
    # Intialize the library (must be called once before other functions).
    strip.begin()

    # left_sonar = UltraSonic(17, 27)        # 17, 27
    # right_sonar = UltraSonic(23, 24)       # 23, 24

    try:
        colorWipe(strip, Color(255, 255, 255), 20)  # White wipe
        time.sleep(1)
        # colorWipe(strip, Color(10, 10, 10))  # White wipe
        while True:
            # colorWipe(strip, Color(255, 0, 0))
            # colorWipe(strip, Color(0, 255, 0))
            # default_animation(strip)
            # transitionToColors(strip)
            # warm_color(strip)
            # cool_color(strip)
            rainbow(strip)
            # rainbowCycle(strip)

            # distance_left = left_sonar.get_distance()
            # distance_right = right_sonar.get_distance()

            # print('left: ', distance_left, 'right: ', distance_right)

            # if distance_left < 400 and distance_right < 400:
            #     print('0')
            #     if random.choice([True, False]):
            #         rainbow(strip)
            #     else:
            #         rainbowCycle(strip)
            # elif distance_left < 400:
            #     print('1')
            #     warm_color(strip)
            # elif distance_right < 400:
            #     print('2')
            #     cool_color(strip)
            # else:
            #     print('3')
            #     default_animation(strip)

    except KeyboardInterrupt:
        colorWipe(strip, Color(0,0,0), 10)
        GPIO.cleanup()