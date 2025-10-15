#!/usr/bin/env python3
"""
HX-1 Hybrid Rocket Engine Data Acquisition System
Raspberry Pi 3B - Simplified DAQ Script with Dummy Data

This script reads sensor data and transmits it to a laptop via TCP/IP over Ethernet.
Hardware interfaces: I2C (pressure sensors), SPI (thermocouple), GPIO (valves)

References:
- Raspberry Pi GPIO: https://sourceforge.net/projects/raspberry-gpio-python/
- Adafruit ADS1x15: https://github.com/adafruit/Adafruit_CircuitPython_ADS1x15
- Adafruit MAX31855: https://github.com/adafruit/Adafruit_CircuitPython_MAX31855
- Python socket programming: https://docs.python.org/3/library/socket.html
- smbus2 I2C library: https://pypi.org/project/smbus2/
"""

import time
import json
import csv
import socket
import threading
from datetime import datetime
from pathlib import Path

# =============================================================================
# HARDWARE LIBRARY IMPORTS
# =============================================================================
# Try to import hardware libraries. If they fail, use dummy data mode.
# Reference: https://learn.adafruit.com/circuitpython-on-raspberrypi-linux
try:
    import board  # CircuitPython board definitions
    import busio  # CircuitPython I2C/SPI bus interface
    import digitalio  # CircuitPython digital I/O
    # ADS1115 16-bit ADC for reading 4-20mA pressure sensors
    # https://github.com/adafruit/Adafruit_CircuitPython_ADS1x15
    from adafruit_ads1x15.ads1115 import ADS1115
    from adafruit_ads1x15.analog_in import AnalogIn
    # MAX31855 thermocouple amplifier
    # https://github.com/adafruit/Adafruit_CircuitPython_MAX31855
    import adafruit_max31855
    # smbus2 for direct I2C communication with load cell amplifier
    # https://pypi.org/project/smbus2/
    from smbus2 import SMBus
    # RPi.GPIO for controlling solenoid valves
    # https://sourceforge.net/projects/raspberry-gpio-python/
    import RPi.GPIO as GPIO
    HARDWARE_AVAILABLE = True
except ImportError as e:
    print(f"Hardware libraries not found: {e}")
    print("Running with DUMMY DATA mode")
    HARDWARE_AVAILABLE = False

# =============================================================================
# CONFIGURATION SECTION
# =============================================================================

# Network Configuration
# The Raspberry Pi will CONNECT to the laptop as a TCP client
# Reference: https://docs.python.org/3/library/socket.html
LAPTOP_IP = "192.168.2.1"  # Change this to your laptop's IP address
DATA_PORT = 4444  # Port number for data transmission

# I2C Device Addresses
# Reference: I2C addressing - https://www.nxp.com/docs/en/user-guide/UM10204.pdf
I2C_BUS_NUMBER = 1  # Raspberry Pi uses I2C bus 1 (pins 3 and 5)
ADS1115_ADDRESS = 0x48  # ADS1115 ADC default address (can be 0x48-0x4B)
LOAD_CELL_AMP_ADDRESS = 0x2A  # HX711 I2C adapter or similar

# SPI Configuration for Thermocouple
# Reference: https://www.raspberrypi.com/documentation/computers/raspberry-pi.html
SPI_CS_PIN = board.D8 if HARDWARE_AVAILABLE else None  # Chip Select pin (CE0)

# GPIO Pin Assignments for Solenoid Valves (BCM numbering)
# Reference: https://pinout.xyz/
VALVE_PINS = {
    'main_valve': 17,      # Main oxidizer valve
    'vent_valve': 27,      # Tank vent valve
    'cooldown_valve': 22,  # Nitrogen cooldown valve
    'igniter': 23          # Igniter relay
}

# Sensor Calibration Constants
# WIKA A-10 Pressure Transducer: 4-20mA output for 0-1000 PSI
# Reference: WIKA A-10 datasheet
PRESSURE_SHUNT_RESISTOR = 250.0  # Ohms (converts 4-20mA to 1-5V)
PRESSURE_MIN_CURRENT = 4.0  # mA (corresponds to 0 PSI)
PRESSURE_MAX_CURRENT = 20.0  # mA (corresponds to 1000 PSI)
PRESSURE_MAX_PSI = 1000.0

# Load Cell Calibration (will be determined during testing)
LOAD_CELL_CALIBRATION = 0.01  # Newtons per raw unit (placeholder)

# Data Sampling Rates (Hz)
SAMPLE_RATE = 100  # 100 samples per second (CDR requirement from page 64)

# File Paths
LOG_DIR = Path("/home/pi/hx1_data")  # Where to save CSV files on Pi
LOG_DIR.mkdir(parents=True, exist_ok=True)  # Create if doesn't exist

# =============================================================================
# DUMMY DATA GENERATOR (used when hardware not available)
# =============================================================================

class DummyDataGenerator:
    """
    Generates realistic dummy sensor data for testing without hardware.
    Simulates a 2-second burn profile based on CDR specifications (page 55).
    """
    def __init__(self):
        self.start_time = time.time()
        
    def get_dummy_data(self):
        """
        Generate dummy data that mimics a real burn.
        
        Burn profile (from CDR page 55):
        - Target chamber pressure: 440 PSI
        - Tank pressure: 800 PSI nominal (blowdown to ~560 PSI)
        - Average thrust: 153 N
        - Chamber temperature: ~3000 K (2727°C)
        
        Returns:
            dict: Sensor readings with realistic noise and dynamics
        """
        import random
        
        # Time since test start
        elapsed = time.time() - self.start_time
        
        # Chamber pressure profile: ramps up, holds steady, drops at end
        if elapsed < 0.2:  # Ignition transient
            chamber_pressure = 440 * (elapsed / 0.2) + random.uniform(-20, 20)
        elif elapsed < 1.8:  # Steady burn
            chamber_pressure = 440 + random.uniform(-15, 15)
        else:  # Tail-off
            chamber_pressure = 440 * (2.0 - elapsed) / 0.2 + random.uniform(-10, 10)
        
        # Tank pressure: decreases as oxidizer is consumed (blowdown)
        # Linear approximation: 800 PSI to 560 PSI over 2 seconds
        tank_pressure = 800 - (240 * elapsed / 2.0) + random.uniform(-5, 5)
        
        # Thrust profile: follows chamber pressure
        thrust = (chamber_pressure / 440.0) * 153 + random.uniform(-5, 5)
        
        # Chamber temperature: follows burn profile
        if elapsed < 0.3:  # Heat-up
            chamber_temp = 2700 * (elapsed / 0.3) + random.uniform(-50, 50)
        elif elapsed < 1.8:  # Steady state
            chamber_temp = 2700 + random.uniform(-100, 100)
        else:  # Cool-down starts
            chamber_temp = 2700 - (500 * (elapsed - 1.8) / 0.2)
        
        return {
            'chamber_pressure_psi': max(0, chamber_pressure),
            'tank_pressure_psi': max(0, tank_pressure),
            'thrust_n': max(0, thrust),
            'chamber_temp_c': max(0, chamber_temp)
        }

# =============================================================================
# SENSOR INTERFACE FUNCTIONS
# =============================================================================

def setup_pressure_sensors(i2c_bus):
    """
    Initialize ADS1115 ADC for reading pressure transducers.
    
    The WIKA A-10 outputs 4-20mA which is converted to 1-5V via a 250Ω resistor.
    We use a 16-bit ADC (ADS1115) to digitize this voltage.
    
    Reference: 
    - ADS1115 datasheet: https://www.ti.com/lit/ds/symlink/ads1115.pdf
    - Adafruit ADS1x15 library: https://github.com/adafruit/Adafruit_CircuitPython_ADS1x15
    
    Args:
        i2c_bus: CircuitPython I2C bus object
        
    Returns:
        dict: Dictionary containing AnalogIn objects for each sensor
    """
    try:
        # Create ADS1115 ADC object
        # The ADS1115 has 4 single-ended inputs (A0-A3) or 2 differential pairs
        ads = ADS1115(i2c_bus, address=ADS1115_ADDRESS)
        
        # Channel 0: Chamber pressure sensor
        # Channel 1: Tank pressure sensor
        # Reference: https://learn.adafruit.com/adafruit-4-channel-adc-breakouts/python-circuitpython
        return {
            'chamber': AnalogIn(ads, 0),  # A0 pin
            'tank': AnalogIn(ads, 1)      # A1 pin
        }
    except Exception as e:
        print(f"Error setting up pressure sensors: {e}")
        return None

def read_pressure(analog_input):
    """
    Convert voltage from pressure transducer to PSI.
    
    Conversion formula:
    1. Read voltage from ADC (1-5V range)
    2. Convert to current: I = V / R (where R = 250Ω)
    3. Map current (4-20mA) to pressure (0-1000 PSI) using linear interpolation
    
    Reference: WIKA A-10 datasheet (4-20mA output specification)
    
    Args:
        analog_input: AnalogIn object from ADS1115
        
    Returns:
        float: Pressure in PSI
    """
    if analog_input is None:
        return 0.0
    
    try:
        # Read voltage from ADC
        voltage = analog_input.voltage
        
        # Convert voltage to current (Ohm's law: I = V/R)
        current_ma = (voltage / PRESSURE_SHUNT_RESISTOR) * 1000.0
        
        # Map 4-20mA to 0-1000 PSI using linear interpolation
        # Formula: PSI = (current - 4) * (1000 / 16)
        pressure_psi = ((current_ma - PRESSURE_MIN_CURRENT) / 
                       (PRESSURE_MAX_CURRENT - PRESSURE_MIN_CURRENT)) * PRESSURE_MAX_PSI
        
        return max(0.0, pressure_psi)  # Clamp to non-negative
    except Exception as e:
        print(f"Error reading pressure: {e}")
        return 0.0

def setup_thermocouple(spi_bus, cs_pin):
    """
    Initialize MAX31855 thermocouple amplifier.
    
    Type K thermocouple for measuring chamber temperature.
    MAX31855 handles cold junction compensation and outputs digital temperature.
    
    Reference:
    - MAX31855 datasheet: https://datasheets.maximintegrated.com/en/ds/MAX31855.pdf
    - Adafruit library: https://github.com/adafruit/Adafruit_CircuitPython_MAX31855
    
    Args:
        spi_bus: CircuitPython SPI bus object
        cs_pin: Chip select pin (digitalio object)
        
    Returns:
        MAX31855 sensor object or None
    """
    try:
        # Create chip select pin
        cs = digitalio.DigitalInOut(cs_pin)
        
        # Initialize MAX31855
        # Reference: https://learn.adafruit.com/thermocouple/python-circuitpython
        sensor = adafruit_max31855.MAX31855(spi_bus, cs)
        return sensor
    except Exception as e:
        print(f"Error setting up thermocouple: {e}")
        return None

def read_temperature(sensor):
    """
    Read temperature from MAX31855 thermocouple.
    
    Reference: https://learn.adafruit.com/thermocouple/python-circuitpython
    
    Args:
        sensor: MAX31855 sensor object
        
    Returns:
        float: Temperature in Celsius or 0.0 on error
    """
    if sensor is None:
        return 0.0
    
    try:
        # MAX31855 returns temperature directly in Celsius
        return sensor.temperature
    except RuntimeError as e:
        # RuntimeError occurs if thermocouple is disconnected or shorted
        print(f"Thermocouple error: {e}")
        return 0.0

def setup_load_cell(i2c_bus_number, address):
    """
    Initialize I2C connection to load cell amplifier.
    
    Using smbus2 for direct I2C communication with HX711 I2C adapter.
    
    Reference:
    - smbus2 documentation: https://pypi.org/project/smbus2/
    - I2C protocol: https://www.nxp.com/docs/en/user-guide/UM10204.pdf
    
    Args:
        i2c_bus_number: I2C bus number (1 for Raspberry Pi)
        address: I2C address of load cell amplifier
        
    Returns:
        SMBus object or None
    """
    try:
        # Open I2C bus
        # Reference: https://smbus2.readthedocs.io/en/latest/
        bus = SMBus(i2c_bus_number)
        return bus
    except Exception as e:
        print(f"Error setting up load cell: {e}")
        return None

def read_thrust(i2c_bus, address):
    """
    Read thrust force from load cell.
    
    This reads raw 24-bit data from the load cell amplifier and converts to Newtons.
    The exact protocol depends on your specific load cell amplifier.
    
    Reference: HX711 datasheet or your specific amplifier documentation
    
    Args:
        i2c_bus: SMBus object
        address: I2C address of amplifier
        
    Returns:
        float: Force in Newtons
    """
    if i2c_bus is None:
        return 0.0
    
    try:
        # Read 4 bytes from I2C device
        # This is a generic example - adjust for your specific amplifier
        # Reference: https://smbus2.readthedocs.io/en/latest/
        data = i2c_bus.read_i2c_block_data(address, 0, 4)
        
        # Convert bytes to 24-bit signed integer
        # Most load cell amplifiers use big-endian format
        raw_value = int.from_bytes(data[:3], byteorder='big', signed=True)
        
        # Convert to Newtons using calibration factor
        force_n = raw_value * LOAD_CELL_CALIBRATION
        
        return force_n
    except Exception as e:
        print(f"Error reading thrust: {e}")
        return 0.0

def setup_valves():
    """
    Initialize GPIO pins for controlling solenoid valves.
    
    All valves are controlled via relays connected to GPIO pins.
    Relays are active HIGH (GPIO HIGH = valve OPEN).
    
    Reference:
    - RPi.GPIO documentation: https://sourceforge.net/projects/raspberry-gpio-python/
    - GPIO pinout: https://pinout.xyz/
    
    Returns:
        bool: True if setup successful
    """
    try:
        # Set GPIO mode to BCM (Broadcom pin numbering)
        # Reference: https://sourceforge.net/p/raspberry-gpio-python/wiki/BasicUsage/
        GPIO.setmode(GPIO.BCM)
        
        # Disable warnings if pins are already configured
        GPIO.setwarnings(False)
        
        # Setup each valve pin as output, initially LOW (closed)
        for valve_name, pin in VALVE_PINS.items():
            GPIO.setup(pin, GPIO.OUT)
            GPIO.output(pin, GPIO.LOW)
            print(f"Initialized {valve_name} on GPIO pin {pin}")
        
        return True
    except Exception as e:
        print(f"Error setting up valves: {e}")
        return False

def control_valve(valve_name, state):
    """
    Open or close a solenoid valve.
    
    Reference: https://sourceforge.net/p/raspberry-gpio-python/wiki/Outputs/
    
    Args:
        valve_name: Name of valve (key from VALVE_PINS)
        state: 'open' or 'close'
    """
    if valve_name not in VALVE_PINS:
        print(f"Unknown valve: {valve_name}")
        return
    
    pin = VALVE_PINS[valve_name]
    
    if state.lower() == 'open':
        GPIO.output(pin, GPIO.HIGH)
        print(f"{valve_name} OPENED")
    elif state.lower() == 'close':
        GPIO.output(pin, GPIO.LOW)
        print(f"{valve_name} CLOSED")

def close_all_valves():
    """
    Emergency shutdown: close all valves.
    
    Reference: https://sourceforge.net/p/raspberry-gpio-python/wiki/Outputs/
    """
    for pin in VALVE_PINS.values():
        GPIO.output(pin, GPIO.LOW)
    print("ALL VALVES CLOSED")

# =============================================================================
# DATA ACQUISITION AND TRANSMISSION
# =============================================================================

def connect_to_laptop(laptop_ip, port, max_retries=5):
    """
    Establish TCP connection to laptop.
    
    The Raspberry Pi acts as a TCP CLIENT that connects to the laptop SERVER.
    This is different from the laptop being the client - the laptop must be
    running the receiver script FIRST and listening for connections.
    
    Socket basics:
    - CLIENT calls socket.connect() to initiate connection
    - SERVER calls socket.bind() and socket.accept() to wait for connections
    - Once connected, both can send/receive data
    
    Reference: https://docs.python.org/3/library/socket.html
    Tutorial: https://realpython.com/python-sockets/
    
    Args:
        laptop_ip: IP address of laptop
        port: Port number (must match receiver)
        max_retries: Number of connection attempts
        
    Returns:
        socket object or None
    """
    for attempt in range(max_retries):
        try:
            # Create TCP socket
            # AF_INET = IPv4, SOCK_STREAM = TCP
            # Reference: https://docs.python.org/3/library/socket.html#socket.socket
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            
            # Set timeout to avoid hanging indefinitely
            sock.settimeout(5.0)
            
            # Connect to laptop (this is the key difference - Pi is CLIENT)
            # Reference: https://docs.python.org/3/library/socket.html#socket.socket.connect
            print(f"Attempting to connect to {laptop_ip}:{port} (attempt {attempt+1}/{max_retries})...")
            sock.connect((laptop_ip, port))
            
            print(f"✓ Connected to laptop at {laptop_ip}:{port}")
            return sock
            
        except socket.timeout:
            print(f"Connection timeout (is receiver running on laptop?)")
        except ConnectionRefusedError:
            print(f"Connection refused (is receiver script running?)")
        except Exception as e:
            print(f"Connection error: {e}")
        
        time.sleep(2)  # Wait before retry
    
    print("Failed to connect to laptop after all retries")
    return None

def send_data(sock, data_dict):
    """
    Send sensor data to laptop via TCP socket.
    
    Data is sent as JSON string with newline delimiter.
    Format: {"key": value, ...}\n
    
    Why JSON?
    - Human readable for debugging
    - Easy to parse on both ends
    - Handles nested data structures
    
    Reference: https://docs.python.org/3/library/json.html
    
    Args:
        sock: Connected socket object
        data_dict: Dictionary of sensor data
        
    Returns:
        bool: True if send successful
    """
    if sock is None:
        return False
    
    try:
        # Convert dictionary to JSON string and add newline
        # Reference: https://docs.python.org/3/library/json.html#json.dumps
        json_string = json.dumps(data_dict) + '\n'
        
        # Encode to bytes and send
        # Reference: https://docs.python.org/3/library/socket.html#socket.socket.sendall
        sock.sendall(json_string.encode('utf-8'))
        
        return True
    except BrokenPipeError:
        print("Connection lost (broken pipe)")
        return False
    except Exception as e:
        print(f"Send error: {e}")
        return False

def save_to_csv(csv_writer, data_dict):
    """
    Save sensor data to CSV file.
    
    CSV format provides backup in case network fails.
    
    Reference: https://docs.python.org/3/library/csv.html
    
    Args:
        csv_writer: csv.writer object
        data_dict: Dictionary of sensor data
    """
    if csv_writer is None:
        return
    
    try:
        # Extract values in consistent order
        row = [
            data_dict['timestamp'],
            data_dict['time_ms'],
            data_dict['chamber_pressure_psi'],
            data_dict['tank_pressure_psi'],
            data_dict['thrust_n'],
            data_dict['chamber_temp_c']
        ]
        csv_writer.writerow(row)
    except Exception as e:
        print(f"CSV write error: {e}")

# =============================================================================
# MAIN TEST EXECUTION
# =============================================================================

def run_test(test_id, duration_seconds=2.0):
    """
    Run a complete test burn.
    
    Test sequence (from CDR page 55):
    1. Initialize sensors
    2. Connect to laptop
    3. Acquire data at 100 Hz
    4. Transmit data in real-time
    5. Save backup CSV
    
    Args:
        test_id: Unique identifier for this test
        duration_seconds: Test duration (default 2.0 seconds per CDR)
    """
    print(f"\n{'='*60}")
    print(f"HX-1 TEST: {test_id}")
    print(f"Duration: {duration_seconds} seconds")
    print(f"{'='*60}\n")
    
    # Initialize hardware or dummy data
    dummy_gen = None
    pressure_sensors = None
    thermocouple = None
    load_cell_bus = None
    
    if HARDWARE_AVAILABLE:
        print("Initializing hardware...")
        
        # Setup I2C bus for pressure sensors
        # Reference: https://learn.adafruit.com/circuitpython-on-raspberrypi-linux/i2c-clock-stretching
        i2c = busio.I2C(board.SCL, board.SDA)
        pressure_sensors = setup_pressure_sensors(i2c)
        
        # Setup SPI bus for thermocouple
        # Reference: https://learn.adafruit.com/circuitpython-on-raspberrypi-linux/spi
        spi = busio.SPI(board.SCK, MOSI=board.MOSI, MISO=board.MISO)
        thermocouple = setup_thermocouple(spi, SPI_CS_PIN)
        
        # Setup load cell
        load_cell_bus = setup_load_cell(I2C_BUS_NUMBER, LOAD_CELL_AMP_ADDRESS)
        
        # Setup valve control
        setup_valves()
        close_all_valves()  # Ensure all closed at start
        
        print("Hardware initialized\n")
    else:
        print("Using DUMMY DATA mode\n")
        dummy_gen = DummyDataGenerator()
    
    # Setup CSV logging
    timestamp_str = datetime.now().strftime("%Y%m%d_%H%M%S")
    csv_filename = LOG_DIR / f"{test_id}_{timestamp_str}.csv"
    csv_file = open(csv_filename, 'w', newline='')
    csv_writer = csv.writer(csv_file)
    
    # Write CSV header
    header = ['timestamp', 'time_ms', 'chamber_pressure_psi', 
              'tank_pressure_psi', 'thrust_n', 'chamber_temp_c']
    csv_writer.writerow(header)
    print(f"Logging to: {csv_filename}\n")
    
    # Connect to laptop
    sock = connect_to_laptop(LAPTOP_IP, DATA_PORT)
    if sock is None:
        print("WARNING: No network connection. Data will only be saved locally.")
    
    # Data acquisition loop
    print("Starting data acquisition...\n")
    start_time = time.time()
    sample_period = 1.0 / SAMPLE_RATE  # Time between samples (0.01 sec for 100 Hz)
    sample_count = 0
    
    try:
        while True:
            loop_start = time.time()
            elapsed = loop_start - start_time
            
            # Check if test duration reached
            if elapsed >= duration_seconds:
                break
            
            # Read all sensors
            if HARDWARE_AVAILABLE:
                # Read from real hardware
                chamber_p = read_pressure(pressure_sensors['chamber']) if pressure_sensors else 0.0
                tank_p = read_pressure(pressure_sensors['tank']) if pressure_sensors else 0.0
                thrust = read_thrust(load_cell_bus, LOAD_CELL_AMP_ADDRESS)
                temp = read_temperature(thermocouple)
            else:
                # Use dummy data
                dummy_data = dummy_gen.get_dummy_data()
                chamber_p = dummy_data['chamber_pressure_psi']
                tank_p = dummy_data['tank_pressure_psi']
                thrust = dummy_data['thrust_n']
                temp = dummy_data['chamber_temp_c']
            
            # Create data packet
            data_packet = {
                'timestamp': datetime.now().isoformat(),
                'time_ms': int(elapsed * 1000),  # Milliseconds since start
                'chamber_pressure_psi': round(chamber_p, 2),
                'tank_pressure_psi': round(tank_p, 2),
                'thrust_n': round(thrust, 2),
                'chamber_temp_c': round(temp, 2)
            }
            
            # Send to laptop
            send_data(sock, data_packet)
            
            # Save to CSV
            save_to_csv(csv_writer, data_packet)
            
            sample_count += 1
            
            # Print progress every 0.5 seconds
            if sample_count % 50 == 0:
                print(f"t={elapsed:.2f}s | P_chamber={chamber_p:.1f} PSI | "
                      f"P_tank={tank_p:.1f} PSI | Thrust={thrust:.1f} N | "
                      f"Temp={temp:.0f}°C")
            
            # Maintain sample rate by sleeping for remaining time
            loop_time = time.time() - loop_start
            sleep_time = sample_period - loop_time
            if sleep_time > 0:
                time.sleep(sleep_time)
            else:
                print(f"WARNING: Sample rate not maintained (loop took {loop_time*1000:.1f}ms)")
    
    except KeyboardInterrupt:
        print("\n\nTest interrupted by user!")
    
    finally:
        # Cleanup
        print(f"\nTest complete. Collected {sample_count} samples")
        
        if HARDWARE_AVAILABLE:
            close_all_valves()
            GPIO.cleanup()
        
        csv_file.close()
        if sock:
            sock.close()
        
        print(f"Data saved to: {csv_filename}")

# =============================================================================
# MAIN PROGRAM
# =============================================================================

def main():
    """
    Main entry point.
    
    Runs test sequence as specified in CDR page 55:
    - Three 2-second burns
    - 30 seconds between burns for cooldown
    """
    print("\n" + "="*60)
    print("HX-1 HYBRID ROCKET ENGINE DATA ACQUISITION SYSTEM")
    print("University of Texas at Arlington")
    print("="*60)
    
    if not HARDWARE_AVAILABLE:
        print("\n⚠ WARNING: Running in DUMMY DATA mode")
        print("Hardware libraries not available")
    
    print(f"\nConfiguration:")
    print(f"  Laptop IP: {LAPTOP_IP}")
    print(f"  Data Port: {DATA_PORT}")
    print(f"  Sample Rate: {SAMPLE_RATE} Hz")
    print(f"  Log Directory: {LOG_DIR}")
    
    try:
        # Run three 2-second burns as per CDR requirements
        for burn_number in range(1, 4):
            test_id = f"HX1_BURN{burn_number}"
            
            input(f"\nPress ENTER to start {test_id}...")
            run_test(test_id, duration_seconds=2.0)
            
            if burn_number < 3:
                print(f"\n⏳ Waiting 30 seconds before next burn (cooldown period)...")
                time.sleep(30)
        
        print("\n" + "="*60)
        print("ALL TESTS COMPLETE")
        print("="*60)
    
    except KeyboardInterrupt:
        print("\n\nShutdown requested by user")
    
    except Exception as e:
        print(f"\n\nERROR: {e}")
        import traceback
        traceback.print_exc()
    
    finally:
        # Final cleanup
        if HARDWARE_AVAILABLE:
            close_all_valves()
            GPIO.cleanup()
        print("\nSystem shutdown complete")

if __name__ == "__main__":
    main()
