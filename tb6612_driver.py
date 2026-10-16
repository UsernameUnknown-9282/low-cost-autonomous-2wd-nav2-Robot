#!/usr/bin/env python3
"""
TB6612FNG Dual H-Bridge Motor Driver Module for Raspberry Pi / Mock Environments.

Pin Assignments (Default for 2WD Robot):
  - STBY (Standby Enable):  GPIO 22 (Active HIGH)
  - Left Motor (Channel A):
      PWMA: GPIO 12 (PWM speed control)
      AIN1: GPIO 5  (Direction control)
      AIN2: GPIO 6  (Direction control)
  - Right Motor (Channel B):
      PWMB: GPIO 13 (PWM speed control)
      BIN1: GPIO 23 (Direction control)
      BIN2: GPIO 24 (Direction control)

Direction Logic:
  - Forward: IN1 = HIGH, IN2 = LOW
  - Reverse: IN1 = LOW,  IN2 = HIGH
  - Stop:    IN1 = LOW,  IN2 = LOW (and PWM = 0.0)
"""

import sys
import logging
from dataclasses import dataclass
from typing import Optional, Dict, Any

logger = logging.getLogger('tb6612_driver')


class Direction:
    STOP = 0
    FORWARD = 1
    REVERSE = -1

    @classmethod
    def to_string(cls, direction: int) -> str:
        if direction == cls.FORWARD:
            return "FORWARD"
        elif direction == cls.REVERSE:
            return "REVERSE"
        return "STOP"


@dataclass
class TB6612PinConfig:
    stby: int = 22
    pwma: int = 12
    ain1: int = 5
    ain2: int = 6
    pwmb: int = 13
    bin1: int = 23
    bin2: int = 24


class BaseTB6612Driver:
    """Base interface for TB6612FNG motor driver."""

    def __init__(self, pins: TB6612PinConfig, frequency: int = 1000):
        self.pins = pins
        self.frequency = frequency
        self.is_enabled = False
        self.is_cleaned_up = False
        self.left_duty = 0.0
        self.left_dir = Direction.STOP
        self.right_duty = 0.0
        self.right_dir = Direction.STOP

    def enable(self) -> None:
        """Set STBY HIGH to enable motor outputs."""
        raise NotImplementedError

    def disable(self) -> None:
        """Set STBY LOW to place driver in standby mode."""
        raise NotImplementedError

    def stop(self) -> None:
        """Stop both motors immediately and set duty cycle to 0."""
        self.set_motors(0.0, Direction.STOP, 0.0, Direction.STOP)

    def set_motor_left(self, duty: float, direction: int) -> None:
        """Set Left Motor (Channel A) duty [0.0, 1.0] and direction."""
        raise NotImplementedError

    def set_motor_right(self, duty: float, direction: int) -> None:
        """Set Right Motor (Channel B) duty [0.0, 1.0] and direction."""
        raise NotImplementedError

    def set_motors(self, left_duty: float, left_dir: int, right_duty: float, right_dir: int) -> None:
        """Set both motors simultaneously."""
        self.set_motor_left(left_duty, left_dir)
        self.set_motor_right(right_duty, right_dir)

    def cleanup(self) -> None:
        """Release GPIO pins."""
        if self.is_cleaned_up:
            return
        self.stop()
        self.disable()
        self.is_cleaned_up = True

    def get_state(self) -> Dict[str, Any]:
        """Return snapshot of driver state."""
        return {
            'enabled': self.is_enabled,
            'left_duty': self.left_duty,
            'left_dir': Direction.to_string(self.left_dir),
            'right_duty': self.right_duty,
            'right_dir': Direction.to_string(self.right_dir),
            'cleaned_up': self.is_cleaned_up,
        }


class MockTB6612Driver(BaseTB6612Driver):
    """
    Mock driver for development, testing, and non-Pi host systems (WSL2 / Linux PC).
    Simulates hardware state changes and logs motor commands cleanly.
    """

    def __init__(self, pins: Optional[TB6612PinConfig] = None, frequency: int = 1000, log_callback=None):
        pins = pins or TB6612PinConfig()
        super().__init__(pins, frequency)
        self.log_callback = log_callback
        self.stby_val = 0
        self.ain1_val = 0
        self.ain2_val = 0
        self.bin1_val = 0
        self.bin2_val = 0
        self._log("[MockTB6612] Initialized Mock Motor Driver (Pins: STBY=%d, PWMA=%d, PWMB=%d)" % 
                  (self.pins.stby, self.pins.pwma, self.pins.pwmb))

    def _log(self, msg: str) -> None:
        if self.log_callback:
            self.log_callback(msg)
        else:
            logger.info(msg)

    def enable(self) -> None:
        self.stby_val = 1
        self.is_enabled = True
        self._log(f"[MockTB6612] STBY ENABLED (GPIO {self.pins.stby} = HIGH)")

    def disable(self) -> None:
        self.stby_val = 0
        self.is_enabled = False
        self._log(f"[MockTB6612] STBY DISABLED (GPIO {self.pins.stby} = LOW)")

    def set_motor_left(self, duty: float, direction: int) -> None:
        duty = max(0.0, min(1.0, float(duty)))
        self.left_duty = duty
        self.left_dir = direction

        if direction == Direction.FORWARD:
            self.ain1_val, self.ain2_val = 1, 0
        elif direction == Direction.REVERSE:
            self.ain1_val, self.ain2_val = 0, 1
        else:
            self.ain1_val, self.ain2_val = 0, 0
            self.left_duty = 0.0

    def set_motor_right(self, duty: float, direction: int) -> None:
        duty = max(0.0, min(1.0, float(duty)))
        self.right_duty = duty
        self.right_dir = direction

        if direction == Direction.FORWARD:
            self.bin1_val, self.bin2_val = 1, 0
        elif direction == Direction.REVERSE:
            self.bin1_val, self.bin2_val = 0, 1
        else:
            self.bin1_val, self.bin2_val = 0, 0
            self.right_duty = 0.0

    def set_motors(self, left_duty: float, left_dir: int, right_duty: float, right_dir: int) -> None:
        self.set_motor_left(left_duty, left_dir)
        self.set_motor_right(right_duty, right_dir)
        self._log(
            f"[MockTB6612] Motor Command -> "
            f"LEFT: {Direction.to_string(self.left_dir):7s} PWM={self.left_duty * 100.0:5.1f}% (AIN1={self.ain1_val}, AIN2={self.ain2_val}) | "
            f"RIGHT: {Direction.to_string(self.right_dir):7s} PWM={self.right_duty * 100.0:5.1f}% (BIN1={self.bin1_val}, BIN2={self.bin2_val})"
        )

    def cleanup(self) -> None:
        if self.is_cleaned_up:
            return
        self.stop()
        self.disable()
        self.is_cleaned_up = True
        self._log("[MockTB6612] Cleaned up mock GPIO pins.")

    def get_state(self) -> Dict[str, Any]:
        base_state = super().get_state()
        base_state.update({
            'stby_pin_val': self.stby_val,
            'ain1_val': self.ain1_val,
            'ain2_val': self.ain2_val,
            'bin1_val': self.bin1_val,
            'bin2_val': self.bin2_val,
            'is_mock': True
        })
        return base_state


class GpiozeroTB6612Driver(BaseTB6612Driver):
    """Hardware driver implementation using gpiozero (Recommended on modern Pi OS)."""

    def __init__(self, pins: Optional[TB6612PinConfig] = None, frequency: int = 1000):
        pins = pins or TB6612PinConfig()
        super().__init__(pins, frequency)
        from gpiozero import PWMOutputDevice, DigitalOutputDevice

        self.stby = DigitalOutputDevice(self.pins.stby, active_high=True, initial_value=False)
        self.pwma = PWMOutputDevice(self.pins.pwma, frequency=self.frequency, initial_value=0.0)
        self.ain1 = DigitalOutputDevice(self.pins.ain1, active_high=True, initial_value=False)
        self.ain2 = DigitalOutputDevice(self.pins.ain2, active_high=True, initial_value=False)

        self.pwmb = PWMOutputDevice(self.pins.pwmb, frequency=self.frequency, initial_value=0.0)
        self.bin1 = DigitalOutputDevice(self.pins.bin1, active_high=True, initial_value=False)
        self.bin2 = DigitalOutputDevice(self.pins.bin2, active_high=True, initial_value=False)

        self.enable()

    def enable(self) -> None:
        self.stby.on()
        self.is_enabled = True

    def disable(self) -> None:
        self.stby.off()
        self.is_enabled = False

    def set_motor_left(self, duty: float, direction: int) -> None:
        duty = max(0.0, min(1.0, float(duty)))
        self.left_duty = duty
        self.left_dir = direction

        if direction == Direction.FORWARD:
            # Cut PWM first if switching direction to prevent H-bridge shoot-through / inrush
            if self.left_dir != Direction.FORWARD and self.pwma.value > 0.0:
                self.pwma.value = 0.0
            self.ain1.on()
            self.ain2.off()
            self.pwma.value = duty
        elif direction == Direction.REVERSE:
            if self.left_dir != Direction.REVERSE and self.pwma.value > 0.0:
                self.pwma.value = 0.0
            self.ain1.off()
            self.ain2.on()
            self.pwma.value = duty
        else:
            self.pwma.value = 0.0
            self.ain1.off()
            self.ain2.off()
            self.left_duty = 0.0

    def set_motor_right(self, duty: float, direction: int) -> None:
        duty = max(0.0, min(1.0, float(duty)))
        self.right_duty = duty
        self.right_dir = direction

        if direction == Direction.FORWARD:
            if self.right_dir != Direction.FORWARD and self.pwmb.value > 0.0:
                self.pwmb.value = 0.0
            self.bin1.on()
            self.bin2.off()
            self.pwmb.value = duty
        elif direction == Direction.REVERSE:
            if self.right_dir != Direction.REVERSE and self.pwmb.value > 0.0:
                self.pwmb.value = 0.0
            self.bin1.off()
            self.bin2.on()
            self.pwmb.value = duty
        else:
            self.pwmb.value = 0.0
            self.bin1.off()
            self.bin2.off()
            self.right_duty = 0.0

    def cleanup(self) -> None:
        if self.is_cleaned_up:
            return
        try:
            self.stop()
            self.disable()
        except Exception as e:
            logger.warning(f"Error stopping in gpiozero cleanup: {e}")

        # Safely close each device individually
        devices = [
            getattr(self, 'pwma', None),
            getattr(self, 'pwmb', None),
            getattr(self, 'ain1', None),
            getattr(self, 'ain2', None),
            getattr(self, 'bin1', None),
            getattr(self, 'bin2', None),
            getattr(self, 'stby', None)
        ]
        for dev in devices:
            if dev is not None:
                try:
                    dev.close()
                except Exception as e:
                    logger.warning(f"Error closing gpiozero device {dev}: {e}")
        self.is_cleaned_up = True


class RpiGpioTB6612Driver(BaseTB6612Driver):
    """Hardware driver implementation using RPi.GPIO."""

    def __init__(self, pins: Optional[TB6612PinConfig] = None, frequency: int = 1000):
        pins = pins or TB6612PinConfig()
        super().__init__(pins, frequency)
        import RPi.GPIO as GPIO
        self.GPIO = GPIO

        self.GPIO.setmode(self.GPIO.BCM)
        self.GPIO.setwarnings(False)

        self.GPIO.setup(self.pins.stby, self.GPIO.OUT, initial=self.GPIO.LOW)
        self.GPIO.setup(self.pins.ain1, self.GPIO.OUT, initial=self.GPIO.LOW)
        self.GPIO.setup(self.pins.ain2, self.GPIO.OUT, initial=self.GPIO.LOW)
        self.GPIO.setup(self.pins.pwma, self.GPIO.OUT)

        self.GPIO.setup(self.pins.bin1, self.GPIO.OUT, initial=self.GPIO.LOW)
        self.GPIO.setup(self.pins.bin2, self.GPIO.OUT, initial=self.GPIO.LOW)
        self.GPIO.setup(self.pins.pwmb, self.GPIO.OUT)

        self.pwm_a = self.GPIO.PWM(self.pins.pwma, self.frequency)
        self.pwm_a.start(0.0)

        self.pwm_b = self.GPIO.PWM(self.pins.pwmb, self.frequency)
        self.pwm_b.start(0.0)

        self.enable()

    def enable(self) -> None:
        self.GPIO.output(self.pins.stby, self.GPIO.HIGH)
        self.is_enabled = True

    def disable(self) -> None:
        self.GPIO.output(self.pins.stby, self.GPIO.LOW)
        self.is_enabled = False

    def set_motor_left(self, duty: float, direction: int) -> None:
        duty = max(0.0, min(1.0, float(duty)))
        self.left_duty = duty
        self.left_dir = direction

        if direction == Direction.FORWARD:
            if self.left_dir != Direction.FORWARD and self.left_duty > 0.0:
                self.pwm_a.ChangeDutyCycle(0.0)
            self.GPIO.output(self.pins.ain1, self.GPIO.HIGH)
            self.GPIO.output(self.pins.ain2, self.GPIO.LOW)
            self.pwm_a.ChangeDutyCycle(duty * 100.0)
        elif direction == Direction.REVERSE:
            if self.left_dir != Direction.REVERSE and self.left_duty > 0.0:
                self.pwm_a.ChangeDutyCycle(0.0)
            self.GPIO.output(self.pins.ain1, self.GPIO.LOW)
            self.GPIO.output(self.pins.ain2, self.GPIO.HIGH)
            self.pwm_a.ChangeDutyCycle(duty * 100.0)
        else:
            self.pwm_a.ChangeDutyCycle(0.0)
            self.GPIO.output(self.pins.ain1, self.GPIO.LOW)
            self.GPIO.output(self.pins.ain2, self.GPIO.LOW)
            self.left_duty = 0.0

    def set_motor_right(self, duty: float, direction: int) -> None:
        duty = max(0.0, min(1.0, float(duty)))
        self.right_duty = duty
        self.right_dir = direction

        if direction == Direction.FORWARD:
            if self.right_dir != Direction.FORWARD and self.right_duty > 0.0:
                self.pwm_b.ChangeDutyCycle(0.0)
            self.GPIO.output(self.pins.bin1, self.GPIO.HIGH)
            self.GPIO.output(self.pins.bin2, self.GPIO.LOW)
            self.pwm_b.ChangeDutyCycle(duty * 100.0)
        elif direction == Direction.REVERSE:
            if self.right_dir != Direction.REVERSE and self.right_duty > 0.0:
                self.pwm_b.ChangeDutyCycle(0.0)
            self.GPIO.output(self.pins.bin1, self.GPIO.LOW)
            self.GPIO.output(self.pins.bin2, self.GPIO.HIGH)
            self.pwm_b.ChangeDutyCycle(duty * 100.0)
        else:
            self.pwm_b.ChangeDutyCycle(0.0)
            self.GPIO.output(self.pins.bin1, self.GPIO.LOW)
            self.GPIO.output(self.pins.bin2, self.GPIO.LOW)
            self.right_duty = 0.0

    def cleanup(self) -> None:
        if self.is_cleaned_up:
            return
        try:
            self.stop()
            self.disable()
        except Exception as e:
            logger.warning(f"Error stopping in RPi.GPIO cleanup: {e}")

        for pwm in [getattr(self, 'pwm_a', None), getattr(self, 'pwm_b', None)]:
            if pwm is not None:
                try:
                    pwm.stop()
                except Exception as e:
                    logger.warning(f"Error stopping PWM: {e}")

        if hasattr(self, 'GPIO'):
            try:
                self.GPIO.cleanup([
                    self.pins.stby,
                    self.pins.pwma, self.pins.ain1, self.pins.ain2,
                    self.pins.pwmb, self.pins.bin1, self.pins.bin2
                ])
            except Exception as e:
                logger.warning(f"Error during RPi.GPIO cleanup: {e}")
        self.is_cleaned_up = True


def create_tb6612_driver(
    pins: Optional[TB6612PinConfig] = None,
    frequency: int = 1000,
    force_mock: bool = False,
    log_callback=None
) -> BaseTB6612Driver:
    """
    Factory function to create the appropriate TB6612 driver:
      1. If force_mock is True -> MockTB6612Driver
      2. Try gpiozero -> GpiozeroTB6612Driver
      3. Try RPi.GPIO -> RpiGpioTB6612Driver
      4. Fallback -> MockTB6612Driver
    """
    pins = pins or TB6612PinConfig()

    if force_mock:
        return MockTB6612Driver(pins=pins, frequency=frequency, log_callback=log_callback)

    # Attempt gpiozero
    try:
        driver = GpiozeroTB6612Driver(pins=pins, frequency=frequency)
        if log_callback:
            log_callback("[TB6612Driver] Successfully initialized gpiozero hardware backend.")
        return driver
    except Exception as e_gz:
        gz_err = str(e_gz)

    # Attempt RPi.GPIO
    try:
        driver = RpiGpioTB6612Driver(pins=pins, frequency=frequency)
        if log_callback:
            log_callback("[TB6612Driver] Successfully initialized RPi.GPIO hardware backend.")
        return driver
    except Exception as e_rpi:
        rpi_err = str(e_rpi)

    # Fallback to Mock
    fallback_msg = (
        f"[TB6612Driver] GPIO hardware unavailable (gpiozero: {gz_err}, RPi.GPIO: {rpi_err}). "
        "Falling back to MockTB6612Driver mode."
    )
    if log_callback:
        log_callback(fallback_msg)
    else:
        logger.warning(fallback_msg)

    return MockTB6612Driver(pins=pins, frequency=frequency, log_callback=log_callback)
