import network
import time
from fpioa_manager import fm
from Maix import GPIO
import lcd
import image

# iomap for Maixduino's SPI interface with ESP32
fm.register(25, fm.fpioa.GPIOHS10)  # cs (chip select)
fm.register(8, fm.fpioa.GPIOHS11)   # rst (reset)
fm.register(9, fm.fpioa.GPIOHS12)   # rdy (ready)
fm.register(28, fm.fpioa.GPIOHS13)  # mosi (Master Out Slave In)
fm.register(26, fm.fpioa.GPIOHS14)  # miso (Master In Slave Out)
fm.register(27, fm.fpioa.GPIOHS15)  # sclk (serial clock)

# Register the GPIO pins for the LEDs
io_led_tx = 12  # Example for wifi_tx LED pin (GPIO12)
io_led_rx = 14  # Example for wifi_rx LED pin (GPIO14)

# Register LEDs as GPIO
fm.register(io_led_tx, fm.fpioa.GPIO0, force=True)  # wifi_tx LED (Transmit LED)
fm.register(io_led_rx, fm.fpioa.GPIO1, force=True)  # wifi_rx LED (Receive LED)

# Initialize the LEDs as output
led_tx = GPIO(GPIO.GPIO0, GPIO.OUT)
led_rx = GPIO(GPIO.GPIO1, GPIO.OUT)

# Turn off both LEDs initially
led_tx.value(0)
led_rx.value(0)

# Function to blink LEDs
def blink_led(led, duration=0.5):
    """Blink the LED for the given duration."""
    led.value(1)  # Turn on the LED
    time.sleep(duration)
    led.value(0)  # Turn off the LED
    time.sleep(duration)

# Initialize LCD
lcd.init()
lcd.clear()

# Function to display message on LCD
def display_on_lcd(message):
    lcd.display(image.Image())
    WHITE = 0xFFFFFF  # White color in RGB565
    BLACK = 0x000000  # Black color in RGB565
    lcd.draw_string(10, 10, message, WHITE, BLACK)  # White text on black background

# Initialize ESP32 SPI
nic = network.ESP32_SPI(cs=fm.fpioa.GPIOHS10, rst=fm.fpioa.GPIOHS11, rdy=fm.fpioa.GPIOHS12,
                        mosi=fm.fpioa.GPIOHS13, miso=fm.fpioa.GPIOHS14, sclk=fm.fpioa.GPIOHS15)

# Wi-Fi connection function
def connect_wifi(ssid, password):
    # Scan available Wi-Fi networks
    networks = nic.scan()
    print("Available Wi-Fi networks:")
    display_on_lcd("Scanning Wi-Fi...")
    for net in networks:
        print("SSID: {}, Signal strength: {} dBm".format(net[0], net[2]))

    # Attempt to connect to the Wi-Fi network
    print("Attempting to connect to Wi-Fi network: {}".format(ssid))
    display_on_lcd("Connecting to Wi-Fi...")
    blink_led(led_tx)  # Blink TX LED while trying to connect
    try:
        nic.connect(ssid, password)

        # Wait for the connection to be established
        start_time = time.time()
        while not nic.isconnected():
            if time.time() - start_time > 30:  # 30 seconds timeout
                print("Connection attempt timed out.")
                display_on_lcd("Connection timed out")
                blink_led(led_rx)  # Blink RX LED on failure
                return False
            time.sleep(1)

        # Connection successful
        print("Connected to Wi-Fi!")
        display_on_lcd("Connected to " + ssid)
        print("Network state:", nic.ifconfig())
        blink_led(led_tx)  # Blink TX LED on success
        return True
    except Exception as e:
        print("Error connecting to Wi-Fi:", e)
        display_on_lcd("Error connecting")
        blink_led(led_rx)  # Blink RX LED on failure
        return False

# Replace with your Wi-Fi credentials
SSID = "FRITZFARID"  # Your Wi-Fi SSID
PASSWORD = "Farid1994"  # Your Wi-Fi password

# Connect to Wi-Fi
connect_wifi(SSID, PASSWORD)

