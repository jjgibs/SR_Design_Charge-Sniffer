"""
This FW is the driver for the Charge Sniffer developed by JC Rabe at NCSU, under
the supervision of Dr John Muth. All rights reserved.
For inquiries contact: jcrabe@ncsu.edu or muth@ncsu.edu

ToDo:
- MCP23017 conflict with I2C channel 0 for LIS3DH. Get library working with bitbang i2c library. Currently MCP23017 works on I2C0, and LIS3DH is decomissioned.
- Set up rest of hardware control functions (including buttons, MCP23017, temp sense, etc.
- Set up commands to test each hardware component
- Set up commands to map ADC value to LEDs on MCP
- Set up commands to begin streaming ADC value to LEDs
- Add in PID loop to control temp with peltier plate
- Set up two input buttons on interface module to be resets for the device
- Create driver functions for mini screen
- Display data to screen.
"""
import machine 
import utime
import time
import struct
# import ustruct
import sys
# import math
import uos
import _thread
import rp2
import os
import micropython
from LIS3DH import LIS3DH
from MCP23017_new import MCP23017
import select
import random
import math
import array
import gc # For memory handling
#User settings:
DEBUG = True  # Set this to True for debug prints, False otherwise. Can also be enabled through command during run time
DEBUG_Detailed = False # For even more prints
user_mode_serial = True # System starts up in serial mode, button switches to run mode
machine.freq(140000000) #overclock the CPU
#WARNING: Don't go over 240MHz here, since that requires wiping the flash with a special UF2 after it stops working




###############################################################################
# Begin Constants and Setups:

# MCP3561 ADC Variables and Constants
myADC_Addr = 0b01 #check this for each device (kind of annoying) :/
StartConversion = 0b101000 #Device common commands
StaticReadADC   = 0b000001
IncrementalReadADC = 0b000011
CS_ADC=5 #USING SPI0
CLK_ADC=2
MISO_ADC=4
MOSI_ADC=3
miso_adc = machine.Pin(MISO_ADC) # Create a Pin object for MISO
miso_adc.init(machine.Pin.IN, machine.Pin.PULL_UP) # Enable the internal pull-up resistor for MISO
cs_ADC = machine.Pin(CS_ADC, machine.Pin.OUT) #Setup SPI CS pullup resistor for ADC
spi_adc = machine.SPI(0, # Initialize SPI communications for the ADC on channel 0
        baudrate=30000000,
        polarity=1,
        phase=1,
        bits=8,
        firstbit=machine.SPI.MSB,
        sck=machine.Pin(CLK_ADC),
        mosi=machine.Pin(MOSI_ADC),
        miso=machine.Pin(MISO_ADC))


# Accelerometer (LIS3DH) Gibson - Bitbang acc
SDA_acc = 17
SCL_acc =  18
# SDA_acc = 16
# SCL_acc = 17
# sda_acc = machine.Pin(SDA_acc)
# scl_acc = machine.Pin(SCL_acc)
# i2c_acc = machine.I2C(0, scl=machine.Pin(SCL_acc), sda=machine.Pin(SDA_acc), freq=400000)
# lis3dh = LIS3DH(i2c_acc)


# Hall effect sensor (TMAG5170)
CS_MAG=8  # GP8 as the CS pin for the magnetic sensor
MISO_MAG = 12  # GP12 as MISO
MOSI_MAG = 11  # GP11 as MOSI
CLK_MAG = 10   # GP10 as CLK
cs_MAG = machine.Pin(CS_MAG, machine.Pin.OUT) # Initialize the chip select pin
spi_mag = machine.SPI(1,  # Initialize SPI communications for the Magnetic sensor on channel 1
        baudrate=1000000,
        polarity=0,
        phase=0,
        bits=8,
        firstbit=machine.SPI.MSB,
        sck=machine.Pin(CLK_MAG),
        mosi=machine.Pin(MOSI_MAG),
        miso=machine.Pin(MISO_MAG)
)


# MCP23017 IO Expander
i2c_ioexp = machine.I2C(0, sda=machine.Pin(0), scl=machine.Pin(1), freq=1000000)
mcp23017 = MCP23017(i2c_ioexp)


# On board LEDs
led1_pin = 22
led2_pin = 25
# Create blinky LED objects
blink_led1 = machine.Pin(led1_pin, machine.Pin.OUT)
blink_led2 = machine.Pin(led2_pin, machine.Pin.OUT)

DoneCollecting = 0
BUZZER_PIN = 21
buzzer = machine.PWM(machine.Pin(BUZZER_PIN))
lock = _thread.allocate_lock()
#RelayRST to control reset of charge sniffer probe
RelayRST = machine.Pin(19, machine.Pin.OUT)
RelayRST.value(0)
#RelayREF to control reference input of frontend
RelayREF = machine.Pin(20, machine.Pin.OUT)
RelayREF.value(0)


# Register defaults and other values
# MCP3564 REGISTER DEFS
_ADCDATA_ = 0x00
_CONFIG0_ = 0x01
_CONFIG1_ = 0x02
_CONFIG2_ = 0x03
_CONFIG3_ = 0x04
_IRQ_ = 0x05
_MUX_ = 0x06
_SCAN_ = 0x07
_TIMER_ = 0x08
_OFFSETCAL_ = 0x09
_GAINCAL_ = 0x0A
_RESERVED_B_ = 0x0B
_RESERVED_C_ = 0x0C
_LOCK_ = 0x0D
_RESERVED_E_ = 0x0E
_CRCCFG_ = 0x0F
_WRT_CTRL_ = 0b01000010
_RD_CTRL_ = 0b01000001
# TMAG5170 memory register addresses
_MAG_DEVICE_CONFIG = 0x00
_MAG_SENSOR_CONFIG = 0x01
_MAG_SYSTEM_CONFIG = 0x02
_MAG_ALERT_CONFIG = 0x03
_MAG_X_THRX_CONFIG = 0x04
_MAG_Y_THRX_CONFIG = 0x05
_MAG_Z_THRX_CONFIG = 0x06
_MAG_T_THRX_CONFIG = 0x07
_MAG_CONV_STATUS = 0x08
_MAG_X_CH_RESULT = 0x09
_MAG_Y_CH_RESULT = 0x0A
_MAG_Z_CH_RESULT = 0x0B
_MAG_TEMP_RESULT = 0x0C
_MAG_AFE_STATUS = 0x0D
_MAG_SYS_STATUS = 0x0E
_MAG_TEST_CONFIG = 0x0F
_MAG_OSC_MONITOR = 0x10
_MAG_GAIN_CONFIG = 0x11
_MAG_OFFSET_CONFIG = 0x12
_MAG_ANGLE_RESULT = 0x13
_MAG_MAGNITUDE_RESULT = 0x14
DisableCRC = 0x0F000407
ActiveMeasureMode = 0x0020

# Constants for global settings
User_LED_blinking = False # Initial state of the User_LED_blinking (True for blinking, False for off)
# For storing the value right after a reference reset:
ADC_Reference_Val = 0
# Default frequency in Hz
default_frequency_blink_LEDs = 2.0  # 2 Hz
blink_sleep_on = 1 / (2 * default_frequency_blink_LEDs)  # Default
blink_sleep_off = 1 / (2 * default_frequency_blink_LEDs)
# Initialize frequency and duty cycle variables
blink_frequency = default_frequency_blink_LEDs
duty_cycle_percentage_blink_LEDs = 50  # Default duty cycle percentage
# List to store the last 20 commands
command_history = []


# End Constants and Setups
###############################################################################
# Hardware Functions

# Functions for indicator blinky LEDs
def toggle_User_LED_blinking():
    global User_LED_blinking
    User_LED_blinking = not User_LED_blinking
    return f"Blinking {'ON' if User_LED_blinking else 'OFF'}"

def set_Blink_LED_frequency(frequency):
    global blink_frequency, blink_sleep_on, blink_sleep_off, duty_cycle_percentage_blink_LEDs
    try:
        new_frequency = float(frequency)
        if (new_frequency >= 0.5):
            # Update frequency and duty cycle
            blink_frequency = new_frequency
            blink_sleep_off = 1 / (blink_frequency) * (1 - duty_cycle_percentage_blink_LEDs / 100.0)
            blink_sleep_on = 1 / (blink_frequency) * (duty_cycle_percentage_blink_LEDs / 100.0)
            return f"Frequency set to {blink_frequency} Hz, with on sleep: {blink_sleep_on}s, off sleep: {blink_sleep_off}s, Duty cycle: {duty_cycle_percentage_blink_LEDs}%"
        else:
            return "Frequency must be greater than 0.5 Hz"
    except ValueError as e:
        print(f"Error: {e}")
        return "Invalid frequency value"

def set_blink_LED_duty_cycle(duty_cycle_percentage_input):
    global blink_sleep_on, blink_sleep_off, duty_cycle_percentage_blink_LEDs
    try:
        duty_cycle_value = int(duty_cycle_percentage_input)
        if 0 <= duty_cycle_value <= 100:
            # Update duty cycle
            duty_cycle_percentage_blink_LEDs = duty_cycle_value
            blink_sleep_on = 1 / (blink_frequency) * (duty_cycle_percentage_blink_LEDs / 100.0)
            blink_sleep_off = 1 / (blink_frequency) * ((100 - duty_cycle_percentage_blink_LEDs) / 100.0)
            return f"Duty cycle set to {duty_cycle_value}%, on sleep: {blink_sleep_on}s, off sleep: {blink_sleep_off}s"
        else:
            return "Duty cycle must be between 0 and 100"
    except ValueError as e:
        print(f"Error: {e}")
        return "Invalid duty cycle value"

# Function to make a sound with the buzzer
def play_tone(frequency):
#     buzzer.freq(frequency)  # Set the frequency of the PWM signal
#     buzzer.duty_u16(32768)  # Set a 50% duty cycle to produce a square wave tone
    pass

def buzzer_off():
    buzzer.duty_u16(0)  # Turn off the buzzer by setting duty cycle to 0

# Functions for ADC SPI comms and setup

def INIT_MCP3564_CONFIG():
    #configure ADC
    msg1 = bytearray([0b01110110]) #slaveaddr + lockregister + wr
    msg2 = bytearray([0b10100101]) #wr 0xa5 to unlock writing to registers
    cs_ADC.value(0)
    spi_adc.write(msg1)
    utime.sleep_us(1)
    spi_adc.write(msg2)
    utime.sleep_us(1)
    cs_ADC.value(1)
    ADC_SPI_WRT((_TIMER_ << 2) | _WRT_CTRL_, 0x000000)      # TIMER --> Disabled.
    ADC_SPI_WRT((_SCAN_ << 2) | _WRT_CTRL_, 0x000000)       # SCAN --> Disabled.
    #Vref- = AGND, SingleEnded, Vref=Vref+ - Vref-, Vin- = AGND 
    ADC_SPI_WRT((_MUX_ << 2) | _WRT_CTRL_, 0b00001000)      # MUX --> VIN+ = CH0, VIN- = AGND --> (0b00001000).
    ADC_SPI_WRT((_IRQ_ << 2) | _WRT_CTRL_, 0b00000000)            # IRQ --> IRQ Mode = Hi-Z IRQ Output  --> (0b00000000).
    ADC_SPI_WRT((_CONFIG3_ << 2) | _WRT_CTRL_, 0b11000000)  # CONFIG3 --> Conv. Mod = Continuous Conv. Mode, FORMAT = 24b, 
                                                        # CRC_FORMAT = 16b, CRC-COM = Disabled, 
                                                        # OFFSETCAL = Disable, GAINCAL = Disable --> (0b10000000).
    ADC_SPI_WRT((_CONFIG2_ << 2) | _WRT_CTRL_, 0b11010011)  # CONFIG2 --> BOOST_xx = 1x, GAIN_xxx = 1x, AZ_MUX = 0 --> (0b10001011).
    ADC_SPI_WRT((_CONFIG1_ << 2) | _WRT_CTRL_, 0b00010000)  # CONFIG1 --> AMCLK = MCLK, OSR = 98k --> (0b00111100). Highest OSR = best SNR
    ADC_SPI_WRT((_CONFIG0_ << 2) | _WRT_CTRL_, 0b11100011)  # CONFIG0 --> CLK_SEL = INTOSC w/o CLKOUT, CS_SEL = No Bias, ADC_MODE = Conv Mode --> (0b11100011).
    #if sample values are repeated, reduce OSR

# Function to read ADC data
def read_adc_data():
    # SPI Read and Data Processing for ADC
    data = bytearray(3)  # Pre-allocate a buffer
    msg3 = bytearray([0b01000001])
    cs_ADC.value(0)
    spi_adc.write(msg3)
    utime.sleep_us(1)
    spi_adc.readinto(data)  # Read data directly into the buffer
    utime.sleep_us(1)
    cs_ADC.value(1)
    # Combine the bytes read and transform to a decimal value
    combined_value = (data[0] << 16) | (data[1] << 8) | data[2]
    return int(combined_value)

def read_adc_data_filtered():
    duration_ms = 67  # Run loop for 67ms (4 cycles of a 60Hz noise)
    start_time = time.ticks_ms()
    readings = []
    while time.ticks_diff(time.ticks_ms(), start_time) < duration_ms:
        readings.append(read_adc_data())
    return sum(readings) / len(readings) if readings else 0
    
    

def ADC_SPI_WRT(WRT_CMD, WRT_DATA):
    WRT_DATA_LOW = WRT_DATA & 0x0000FF  # Extract Low-Byte from 24-bit Write-Data.
    WRT_DATA_HIGH = (WRT_DATA & 0x00FF00) >> 8  # Extract High-Byte from 24-bit Write-Data.
    WRT_DATA_UPPER = (WRT_DATA & 0xFF0000) >> 16  # Extract Upper-Byte from 24-bit Write-Data.
    REG_ADDR = (WRT_CMD & 0b00111100) >> 2  # Extract MCP3564 Register-Address for Write-CMD
    if (
        REG_ADDR == _CONFIG0_
        or REG_ADDR == _CONFIG1_
        or REG_ADDR == _CONFIG2_
        or REG_ADDR == _CONFIG3_
        or REG_ADDR == _IRQ_
        or REG_ADDR == _MUX_
        or REG_ADDR == _LOCK_
    ):
        cs_ADC.value(0)  # Assert CS Low to enable MCP3564 SPI Interface.
        spi_adc.write(bytearray([WRT_CMD]))  # Transmit Write-CMD as one 8-bit packet.
        utime.sleep_us(1)
        spi_adc.write(bytearray([WRT_DATA_LOW]))  # Transmit Register-Data Low-Byte as one 8-bit packet.
        utime.sleep_us(10)
        cs_ADC.value(1)  # Raise CS High to reset MCP3564 SPI Interface.
    elif (
        REG_ADDR == _SCAN_
        or REG_ADDR == _TIMER_
        or REG_ADDR == _OFFSETCAL_
        or REG_ADDR == _GAINCAL_
    ):
        cs_ADC.value(0)  # Assert CS Low to enable MCP3564 SPI Interface.
        spi_adc.write(bytearray([WRT_CMD]))  # Transmit Write-CMD as one 8-bit packet.
        utime.sleep_us(1)
        spi_adc.write(bytearray([WRT_DATA_UPPER]))  # Transmit Register-Data Upper-Byte as one 8-bit packet.
        utime.sleep_us(1)
        spi_adc.write(bytearray([WRT_DATA_HIGH]))  # Transmit Register-Data High-Byte as one 8-bit packet.
        utime.sleep_us(1)
        spi_adc.write(bytearray([WRT_DATA_LOW]))  # Transmit Register-Data Low-Byte as one 8-bit packet.
        utime.sleep_us(1)
        cs_ADC.value(1)  # Raise CS High to reset MCP3564 SPI Interface.
        
def check_adc_communication():
    # This function could probably be improved, but it's good enough for now...
    result = ""
    #Confirm unlocked:
    msg3 = bytearray([0b01110101])
    cs_ADC.value(0)
    spi_adc.write(msg3)
    utime.sleep_us(1)
    data2 = spi_adc.read(1)
    utime.sleep_us(1)
    cs_ADC.value(1)
    result += "UnlockRegister: 0x{:02X}\n".format(data2[0])
    #Confirm register: MUX = 0x06
    msg3 = bytearray([0b01011001])
    cs_ADC.value(0)
    spi_adc.write(msg3)
    utime.sleep_us(1)
    data = spi_adc.read(1)
    utime.sleep_us(1)
    cs_ADC.value(1)
    result += "Mux reg: {:08b}\n".format(data[0])
    #Confirm register: _CONFIG3_ADC=0x04
    msg3 = bytearray([0b01010001])
    cs_ADC.value(0)
    spi_adc.write(msg3)
    utime.sleep_us(1)
    data = spi_adc.read(1)
    utime.sleep_us(1)
    cs_ADC.value(1)
    result += "Config3: {:08b}\n".format(data[0])
    #Confirm register: _CONFIG2_ADC=0x03
    msg3 = bytearray([0b01001101])
    cs_ADC.value(0)
    spi_adc.write(msg3)
    utime.sleep_us(1)
    data = spi_adc.read(1)
    utime.sleep_us(1)
    cs_ADC.value(1)
    result += "Config2: {:08b}\n".format(data[0])
    #Confirm register: _CONFIG1_ADC=0x02
    msg3 = bytearray([0b01001001])
    cs_ADC.value(0)
    spi_adc.write(msg3)
    utime.sleep_us(1)
    data = spi_adc.read(1)
    utime.sleep_us(1)
    cs_ADC.value(1)
    result += "Config1: {:08b}\n".format(data[0])
    #Confirm register: _CONFIG0_ADC=0x01
    msg3 = bytearray([0b01000101])
    cs_ADC.value(0)
    spi_adc.write(msg3)
    utime.sleep_us(1)
    data = spi_adc.read(1)
    utime.sleep_us(1)
    cs_ADC.value(1)
    result += "Config0: {:08b}\n".format(data[0])
    #Check register: ADCDATA
    msg3 = bytearray([0b01000011])
    cs_ADC.value(0)
    spi_adc.write(msg3)
    utime.sleep_us(1)
    data = spi_adc.read(3)
    utime.sleep_us(1)
    cs_ADC.value(1)
    result += "ADCDATA: {:24b}\n".format(data[0])
    return result
 
 
#TMAG5170 Hall Effect sensor functions
def INIT_TMAG5170_CONFIG():
    #Configure the Hall effect sensor
    #DisableCRC
    data_tx = bytearray([0x0F, 0x00, 0x04, 0x07])
    send_spi_command_mag(data_tx) #disable the CRC
    MAG_SPI_WRT(_MAG_DEVICE_CONFIG, 0x0000) #Default device config values enters into config mode
    MAG_SPI_WRT(_MAG_SYSTEM_CONFIG, 0b0000001000000000) #Diag together, conv at CS pulse command, 32bit, AFE disabled, limit checks off
    MAG_SPI_WRT(_MAG_SENSOR_CONFIG, 0b0000000111000000) #No angle, 1ms sleep, Enable XYZ,50mT range
    #MAG_SPI_WRT(_MAG_ALERT_CONFIG,  0b0000000000000000) #no latch,interrupt mode, no alert on AFE, no result alert,1 conv for alert, alerts off 
    #MAG_SPI_WRT(_MAG_X_THRX_CONFIG, 0x7D83) #Default vaues
    #MAG_SPI_WRT(_MAG_Y_THRX_CONFIG, 0x7D83) #Default vaues
    #MAG_SPI_WRT(_MAG_Z_THRX_CONFIG, 0x7D83) #Default vaues
    #MAG_SPI_WRT(_MAG_CONV_STATUS,   0x0000)
    MAG_SPI_WRT(_MAG_TEST_CONFIG,   0b0000000001000100) #NO CRC
    MAG_SPI_WRT(_MAG_GAIN_CONFIG,   0b0000001111111111) #Gain of 1x
    MAG_SPI_WRT(_MAG_DEVICE_CONFIG, 0b0101000000100000) #32x OSR, MagtempCo = 0, Continuous Conv, No temp 
    
def MAG_SPI_WRT(address, data_to_write):
    # Build TX and RX byte arrays
    data_tx = bytearray(4)
    # MSB is 0 for WRITE command
    data_tx[0] = address
    data_tx[1] = data_to_write >> 8
    data_tx[2] = data_to_write
    data_tx[3] = 0b00000000 #starts conversion on next cs high
    cs_MAG.value(0)  # Set CS pin to low to select the device
    utime.sleep_us(100) #remove
    #print("".join("{:08b}".format(byte) for byte in data_tx)) #debug prints
    data_rx = bytearray(4)
    spi_mag.write(data_tx)
    utime.sleep_us(50)
    spi_mag.readinto(data_rx)
    #print("".join("{:08b}".format(byte) for byte in data_rx)) #debug prints
    #print("--")
    #spi_mag.write(data_tx)  # Send the command byte
    utime.sleep_us(50) #remove
    cs_MAG.value(1)  # Set CS pin back to high
    
def normal_read_mag(output, address, cmd_bits):
    # Build TX byte array
    data_tx = bytearray(4)
    # MSB is 1 for READ command:
    data_tx[0] = (address| 0x80)
    data_tx[1] = 0x00
    data_tx[2] = 0x00
    data_tx[3] = cmd_bits << 4
    cs_MAG.value(0)  # Set CS pin to low to select the device
    utime.sleep_us(5) #remove
    # Send the command and read the response
    #print(" ".join("{:08b}".format(byte) for byte in data_tx)) #debug prints
    data_rx = bytearray(4)
    spi_mag.write(data_tx)
    utime.sleep_us(5)
    spi_mag.readinto(data_rx)
    #spi_mag.write_readinto(data_tx, data_rx)
    #print("Data RX (Binary):", " ".join("{:08b}".format(byte) for byte in data_rx)) #TODO Remove
    utime.sleep_us(5) #remove
    cs_MAG.value(1)  # Set CS pin back to high
    output[0] = (data_rx[1] << 8) | data_rx[2]
    output[1] = (data_rx[0] << 4) | (data_rx[3] >> 4)
    
def send_spi_command_mag(command):
    cs_MAG.value(0)  # Set CS pin to low to select the device
    utime.sleep_us(500) #remove
    spi_mag.write(command)  # Convert command to bytes and send
    utime.sleep_us(500) #remove
    cs_MAG.value(1)  # Set CS pin back to high

def TMAG_read_magnitudes():
    output_x = [0, 0]
    output_y = [0, 0]
    output_z = [0, 0]
    cmd_bits = 0x01  # Replace this with the appropriate CMD bits
    normal_read_mag(output_x, _MAG_X_CH_RESULT, cmd_bits)
    normal_read_mag(output_y, _MAG_Y_CH_RESULT, cmd_bits)
    normal_read_mag(output_z, _MAG_Z_CH_RESULT, cmd_bits)      
    x_magnitude = struct.unpack('>h', struct.pack('>H', output_x[0]))[0]
    y_magnitude = struct.unpack('>h', struct.pack('>H', output_y[0]))[0]
    z_magnitude = struct.unpack('>h', struct.pack('>H', output_z[0]))[0]
    # Magnetic field range in millitesla (mT)
    min_mT = -50
    max_mT = 50
    # Scale values to magnetic field range
    x_scaled = (x_magnitude / 32767) * max_mT
    y_scaled = (y_magnitude / 32767) * max_mT
    z_scaled = (z_magnitude / 32767) * max_mT
    return x_scaled, y_scaled, z_scaled


# LIS3DH Accelerometer functions:
def INIT_LIS3DH():
    # Check if the sensor is connected
    try:
        lis3dh.setup() # Move to init section in main function
        if not lis3dh.who_am_i() == 0x33:
            print("Failed to detect LIS3DH sensor.")
    except:
        print("LIS3DH not found")
    

# MCP23017 IO Expander functions:
def INIT_MCP23017():
    try:
        # Define pin configuration (GPA0 and GPB0 as inputs, the rest as outputs)
        pin_config = {
            0: 1,  # GPA0 as input
            8: 1,  # GPB0 as input
        }
        # Set up pins with user-defined configuration
        mcp23017.setup_pins(pin_config)
        # Enable pull-ups on GPA0 and GPB0
        mcp23017.enable_pullups(pins=[0, 8])
    except Exception as e:
        print(f"MCP23017 not found: {e}")

# End Hardware Functions
###############################################################################
# Begin User Functions

# Command function to check ADC communication
def command_check_adc_communication():
    if DEBUG: print("Checking ADC communication...")
    success = check_adc_communication()
    return "Communication check: " + ("succeeded" if success else "failed")

# Function to check ADC channel 1
def check_adc_channel_1():
    # Read ADC data for channel 1
    data = read_adc_data()
    return "ADC Channel 1 Value: " + str(data)

# Function to check ADC channel 1 and return voltage value
def check_adc_channel_1_V():
    # Read ADC data for channel 1
    data = read_adc_data()
    # Maximum value for a 24-bit ADC
    max_adc_value = 2**24 - 1
    # Reference voltage (3.3V)
    ref_voltage = 3.3
    # Convert to voltage value between 0V and 3.3V
    voltage = (data / max_adc_value) * ref_voltage
    # Return the voltage value formatted to 3 decimal places
    return f"ADC Channel 1 Voltage: {voltage:.3f}V"

# Function to check MAG sensor
def check_mag_values():
    try:
        #x_mag, y_mag, z_mag = TMAG_read_magnitudes()
        x_sum, y_sum, z_sum = 0.0, 0.0, 0.0
        for _ in range(1000):
            x_mag, y_mag, z_mag = TMAG_read_magnitudes()
            x_sum += x_mag
            y_sum += y_mag
            z_sum += z_mag
            utime.sleep_us(10)
        # Calculate averages
        x_mag = x_sum / 1000
        y_mag = y_sum / 1000
        z_mag = z_sum / 1000
        Vector_Magnitude = math.sqrt(x_mag**2 + y_mag**2 + z_mag**2)
        # Return the values and overall strength as a formatted string
        return f"X_MAG: {x_mag}, Y_MAG: {y_mag}, Z_MAG: {z_mag}, Vector_Magnitude: {Vector_Magnitude}"
    except Exception as e:
        if DEBUG: print("Failed to read MAG:", e)
        # Return the failure message
        return "Failed to measure magnetic field"


# Function to check ACC sensor
def check_acc_values():
    try:
        # Initialize sums for the acceleration values
        x_sum, y_sum, z_sum = 0.0, 0.0, 0.0
        # Collect data for 1000 readings
        for _ in range(1000):
            # Read acceleration values
            x_acc, y_acc, z_acc = lis3dh.read_acceleration()
            # Accumulate the values for averaging
            x_sum += x_acc
            y_sum += y_acc
            z_sum += z_acc
            time.sleep_us(10)  # Sleep for 10 microseconds to avoid overloading the processor
        # Calculate averages
        x_acc = x_sum / 1000
        y_acc = y_sum / 1000
        z_acc = z_sum / 1000
        # Calculate the vector magnitude of the acceleration
        vector_magnitude = math.sqrt(x_acc**2 + y_acc**2 + z_acc**2)
        # Return the values and overall magnitude as a formatted string
        return f"X_ACC: {x_acc:.4f}, Y_ACC: {y_acc:.4f}, Z_ACC: {z_acc:.4f}, Vector_Magnitude: {vector_magnitude:.4f}"
    except Exception as e:
        print("Failed to read ACC:", e)
        # Return the failure message
        return "Failed to measure acceleration"


# Function to control or read MCP23017 IO Expander pins
def get_all_pin_values():
    try:
        # Read the current register values for GPA and GPB
        gpio_a_value = mcp23017.read_reg(0x12)  # GPA (Pins 0-7) at register 0x12
        gpio_b_value = mcp23017.read_reg(0x13)  # GPB (Pins 8-15) at register 0x13
        # Return a formatted string showing the binary values of the registers
        return f"RegA: {gpio_a_value:08b}  RegB: {gpio_b_value:08b}"
    except Exception as e:
        return f"Error getting pin values: {e}"

def set_all_pins_high(): #for GPIO expander
    try:
        for pin in range(16):
            mcp23017.set_pin_value(pin, 1)
        return "All pins set to high (1)."
    except Exception as e:
        return f"Error setting all pins to high: {e}"

def set_all_pins_low():
    try:
        for pin in range(16):
            mcp23017.set_pin_value(pin, 0)
        return "All pins set to low (0)."
    except Exception as e:
        return f"Error setting all pins to low: {e}"

def Run_HW_Test():
    try:
        duration = 30000 # Run HW test for 30s
        start_time = time.ticks_ms()  # Get start time in milliseconds
        while time.ticks_diff(time.ticks_ms(), start_time) < duration:
            # This loop does some user checks on the hardware
            print(check_adc_channel_1_V()) # Check if adc works
            print(check_mag_values()) # Check if mag works
            blink_led1.value(1)
            blink_led2.value(1)
            utime.sleep(1)  # Sleep for the on state
            # Turn off the LEDs
            blink_led1.value(0)
            blink_led2.value(0)
            currentState = get_all_pin_values()
            print("press and hold button now")
            utime.sleep_ms(2000)
            newState = get_all_pin_values()
            if currentState != newState:
                print("Button Press detected")
            else:
                print("No button press detected")
            set_all_pins_high()
            utime.sleep_ms(2000)
            set_all_pins_low()
#             play_tone(400)
            utime.sleep(20)
#             play_tone(300)
            utime.sleep(30)
            buzzer_off()
            # End of hardware test loop
        return "HW Test concluded"
    except Exception as e:
        return f"Hardware testing error: {e}"





def set_LEDs_to_front_end():
    global ADC_Reference_Val
    adc_value = read_adc_data_filtered()  # Get current ADC reading with filtering pre-processing
    # Define LED pin ranges (pins 1-7 for negative, 9-15 for positive)
    negative_led_pins = list(range(1, 8))
    positive_led_pins = list(range(9, 16))
    # Turn off all LED pins first (pins 1-7 and 9-15)
    for pin in positive_led_pins + negative_led_pins:
        mcp23017.set_pin_value(pin, 0)
    if adc_value < ADC_Reference_Val:
        # Negative side: calculate the difference below reference
        diff = ADC_Reference_Val - adc_value
        # Linear scaling: full range is from ADC_Reference_Val (diff=0) to 0 (diff=ADC_Reference_Val)
        neg_binsize = ADC_Reference_Val / 7
        num_leds = int(diff / neg_binsize)
        num_leds = min(num_leds, 7)
        print("Negative side: turning on", num_leds, "LED(s)")
        for i in range(num_leds):
            mcp23017.set_pin_value(negative_led_pins[i], 1)
    elif adc_value > ADC_Reference_Val:
        # Positive side: calculate the difference above reference
        diff = adc_value - ADC_Reference_Val
        # Linear scaling: full range is from ADC_Reference_Val up to max (2**24 - 1)
        pos_range = (2**24 - 1) - ADC_Reference_Val
        pos_binsize = pos_range / 7
        num_leds = int(diff / pos_binsize)
        num_leds = min(num_leds, 7)
        print("Positive side: turning on", num_leds, "LED(s)")
        for i in range(num_leds):
            mcp23017.set_pin_value(positive_led_pins[i], 1)
    else:
        print("No difference: all LEDs off")







# End User Functions
###############################################################################
# Begin button use function definitions

def right_button_pressed():
#     for pin in range(8):
#         mcp23017.set_pin_value(pin, 1)
#     utime.sleep_ms(500)
#     set_all_pins_low()
    global ADC_Reference_Val
    if DEBUG: print("reset reference")
    set_all_pins_low()
    RelayRST.value(1)
#     RelayREF.value(1)
    utime.sleep_ms(500)
#     RelayREF.value(0)
    RelayRST.value(0)
    utime.sleep_ms(1000)
    ADC_Reference_Val = read_adc_data_filtered()
    print(ADC_Reference_Val)
    return 0
    
    
def left_button_pressed():
#     for pin in range(8):
#         mcp23017.set_pin_value(pin+8, 1)
#     utime.sleep_ms(500)
#     set_all_pins_low()
    # Enters device into alternate mode
    global ADC_Reference_Val
    global user_mode_serial
    if DEBUG: print("changing mode")
    RelayREF.value(1)
    RelayRST.value(1)
    utime.sleep_ms(500)
    RelayREF.value(0)
    RelayRST.value(0)
    ADC_Reference_Val = read_adc_data()
    user_mode_serial = not user_mode_serial
    return 0



# End button use function definitions
###############################################################################
# Begin User Interfacing


# List of all commands and how they map
def process_command(command):
    try:
        # Check if the command consists solely of 'w' characters (history recall request)
        if command and all(c == 'w' for c in command):  # Check if every character is 'w'
            # Count how many 'w' characters are in the command
            num_w = len(command)
            if num_w <= len(command_history):  # If there are enough commands in history
                previous_command = command_history[-num_w]
                print(f"Re-executing command: {previous_command}")
                # Execute the previous command directly (no recursion)
                return process_command(previous_command)  # Re-execute that command
            else:
                return "Only the last 20 commands are stored."
        # Store the command in the history (max 20) if it's not just 'w'
        if not (command and all(c == 'w' for c in command)):  # If it's not a 'w' command
            if len(command_history) >= 20:
                command_history.pop(0)  # Remove the oldest command if we have more than 20
            command_history.append(command)
            
        # Blinky LEDs user interfacing
        if command == "LEDblinky":
            return toggle_User_LED_blinking()
        elif command.startswith("LEDfrequency"):
            _, value = command.split(" ", 1)
            return set_Blink_LED_frequency(value)
        elif command.startswith("LEDduty"):
            _, value = command.split(" ", 1)
            return set_blink_LED_duty_cycle(value)
        # Quick commands for multiple actions
        elif command == "OFF" or command == "STOP":
            # Disable everything
            return "ALL OFF"
        elif command == "Ping" or command == "ping" or command == "PING":
            return ping()
        # ADC interfacing and testing commands
        elif command == "CheckADCcomms":
            return check_adc_communication()
        elif command == "ADC":
            return check_adc_channel_1()  # Check ADC channel 1
        elif command == "ADC_V":
            return check_adc_channel_1_V()  # Check ADC channel 1
        # MAG interfacing commands
        elif command == "MAG":
            return check_mag_values()
        # ACC interfacing commands
        elif command == "ACC":
            return check_acc_values()
        # MCP23017 interfacing commands
        elif command == "GET_ALL_PINS":
            return get_all_pin_values()
        elif command == "SET_ALL_PINS_HIGH":
            return set_all_pins_high()
        elif command == "SET_ALL_PINS_LOW":
            return set_all_pins_low()
        # General Commands
        elif command == "HW_TEST":
            return Run_HW_Test()
        elif command == "set_LEDs_to_front_end":
            return set_LEDs_to_front_end()
        
        
        


        else:
            return "Unknown command"
    except Exception as e:
        print(f"Error: {e}")
        return "Command execution failed"

# Function to get user input
def non_blocking_serial_input():
    ready, _, _ = select.select([sys.stdin], [], [], 0.001)  # 0.001 seconds timeout
    if ready:
        return sys.stdin.readline().strip()
    return ""

# Function to ping with millis()
def ping():
    end_time = utime.ticks_ms()  # Record the end time in milliseconds
    return "ping {}ms".format(end_time)

# Function to reboot self
def reboot_self():
    # Warning, this is a hard reset and will break serial comms
    machine.reset()
    

# End User Interfacing
###############################################################################
# Begin Main Function
    


# Main loop
def main():
    print('booting up')
    cs_ADC.value(1)
    spi_adc.read(1) #dummy read
    cs_MAG.value(1)
    INIT_MCP3564_CONFIG() # Init the ADC peripheral
    INIT_TMAG5170_CONFIG() # Init the MAG sensor
#     INIT_LIS3DH()
    INIT_MCP23017()
    buzzer_off()
    while True:
        # Check for a command with non-blocking input
        if user_mode_serial: # Then we work in serial mode
            command = non_blocking_serial_input()
            if command:
                # Process the command and print the confirmation message
                result = process_command(command)
                print(result)
        else: # Then we work in stream mode
            set_LEDs_to_front_end()
            utime.sleep_ms(100)
        # Get User button input
        gpio_a_value = mcp23017.read_reg(0x12)  # Read GPA register
        gpio_b_value = mcp23017.read_reg(0x13)  # Read GPB register
        if gpio_a_value & 0x01 == 0:  # Button A pressed (LSB is 0)
            right_button_pressed()
        if gpio_b_value & 0x01 == 0:  # Button B pressed (LSB is 0)
            left_button_pressed()
        # Perform blinky LED actions based on the boolean variable
        if User_LED_blinking:
            # ToDo: make this non-blocking
            # Turn on the LEDs
            blink_led1.value(1)
            blink_led2.value(1)
            utime.sleep(blink_sleep_on)  # Sleep for the on state
            # Turn off the LEDs
            blink_led1.value(0)
            blink_led2.value(0)
            utime.sleep(blink_sleep_off)  # Sleep for the off state
        else:
            # If User_LED_blinking is False, turn off the LEDs
            blink_led1.value(0)
            blink_led2.value(0)
            

if __name__ == "__main__":
    main()














"""
 
def run_data_collection_Acc_Mag(duration_ms, filenameAcc, filenameMag):
    start_ticks = utime.ticks_us()  # Get the start time in microseconds
    end_ticks = start_ticks + (duration_ms * 1000)  # Convert duration from milliseconds to microseconds
    bufferAcc = bytearray()
    bufferMag = bytearray()
    current_ticks = utime.ticks_us()
    
    while current_ticks < end_ticks:
        current_ticks = utime.ticks_us()  # Get the current time in microseconds
        timestamp = (current_ticks - start_ticks)  # Calculate the time since startup
        
        # I2C Read of data from accelerometer:
        x_acc, y_acc, z_acc = motion.acceleration
        utime.sleep_us(100) #Offer some time for transients to settle
        # SPI Read Mag data:
        x_mag, y_mag, z_mag = TMAG_read_magnitudes()
        
        # CSV prep and data writing to files
        csv_data_acc = "{},{},{},{}\n".format(timestamp, x_acc, y_acc, z_acc)
        bufferAcc.extend(csv_data_acc)  # Append data to the buffer
        csv_data_mag = "{},{},{},{}\n".format(timestamp, x_mag, y_mag, z_mag)
        bufferMag.extend(csv_data_mag)
        utime.sleep_ms(2)
    
    utime.sleep_ms(500) #give a chance for the other file to be written
    file = open(filenameAcc, "w")  # Open the file in write mode
    file.write(bufferAcc)
    file.flush()
    file.close()  # Close the file once the loop finishes
    file = open(filenameMag, "w")  # Open the file in write mode
    file.write(bufferMag)
    file.flush()
    file.close()  # Close the file once the loop finishes
 
# Function to wait for acceleration
def wait_for_acceleration():
    while True:
        # I2C Read of data from accelerometer:
        x, y, z = motion.acceleration
        # Calculate total acceleration
        total_acceleration = (x**2 + y**2 + z**2) ** 0.5
        if total_acceleration > BeginAccelerationThreshold:
            break
        # Wait for a short period before reading again
        #utime.sleep_us(100)

# Function to run on core 1
def core1_loop():
    global DoneCollecting
    #print("Starting Now")
    run_data_collection_ADC(duration, filenameADC, 0)
    #print("Done Now")
    with lock:
        DoneCollecting = 1
    _thread.exit()

def BeginDelay():
    #Waits a preconfigured amount of time while flashing LEDs to show status
    #This is important as it sets up the front end and prepares references before a run
    duration_ms = abs(startDelay-1000)  # Duration in milliseconds (subtract tone at start)
    start_time = utime.ticks_ms()
    while utime.ticks_diff(utime.ticks_ms(), start_time) < duration_ms:
        remaining_time = duration_ms - utime.ticks_diff(utime.ticks_ms(), start_time)
        if remaining_time > 5000:
            BlinkyLED.value(1)
            utime.sleep_ms(500)
            BlinkyLED.value(0)
            utime.sleep_ms(500)
        else:
            BlinkyLED.value(1)
            play_tone(700)
            utime.sleep_ms(50)
            BlinkyLED.value(0)
            buzzer_off()
            utime.sleep_ms(50)
            RelayRST.value(1) #Prepare by arming RelayRST
            #RelayREF.value(1) #Prepare reference input
            #Delete old files as we are about to rewrite them:
            delete_Old_Files(filenameADC)
            delete_Old_Files(filenameAcc)
            delete_Old_Files(filenameMag)
   
def delete_Old_Files(file_path):
    # Delete the given file
    try:
        os.remove(file_path)
        print("File deleted successfully.")
    except OSError as e:
        print("Error deleting file:", e)
        
# Function to make a sound with the buzzer
def play_tone(frequency):
    buzzer.freq(frequency)  # Set the frequency of the PWM signal
    buzzer.duty_u16(32768)  # Set a 50% duty cycle to produce a square wave tone

def buzzer_off():
    buzzer.duty_u16(0)  # Turn off the buzzer by setting duty cycle to 0
    
def play_start_tone():
    play_tone(500) #Audible prepare tone
    utime.sleep_ms(200)
    play_tone(600)
    utime.sleep_ms(200)
    play_tone(700)
    utime.sleep_ms(200)
    play_tone(1000) #Audible prepare tone
    utime.sleep_ms(400)
    buzzer_off() 

###############################################################################
#MAIN CODE
        
#setup
cs_ADC.value(1)
cs_MAG.value(1)
spi_adc.read(1) #dummy read
#Prepare peripherals and variables
INIT_MCP3564_CONFIG()
INIT_TMAG5170_CONFIG()
#SPI_CheckMCP()
#Start a file:
filenameADC = "/ADC.txt"  # Specify the filename and path for the CSV file
filenameAcc = "/Acc.txt"
filenameMag = "/Mag.txt"

play_start_tone()
BeginDelay()    #Give user time to load canon
play_tone(1000) #Audible armed tone
wait_for_acceleration() #Wait for start impulse
buzzer_off() #Silence during firing
#Start the loop on Core 1
_thread.start_new_thread(core1_loop, ())
#run Accelerometer collection on Core 0
run_data_collection_Acc_Mag(duration, filenameAcc, filenameMag)
#Indicate on LEDs if recording is done
while True:
    with lock:
        if DoneCollecting:
            ConstLED.value(1)
            RelayREF.value(0) #No need for reference input anymore
            while True:
                BlinkyLED.value(1)
                utime.sleep_ms(1000)
                BlinkyLED.value(0)
                utime.sleep_ms(1000)
        else:
            BlinkyLED.value(1)
            utime.sleep_ms(5)
            BlinkyLED.value(0)
            utime.sleep_ms(5)
            
            """