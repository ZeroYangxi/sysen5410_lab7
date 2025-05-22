from machine import UART, Pin
import time

# UART Configuration
UART_ID = 0
BAUD_RATE = 115200
#TX_PIN = 16
RX_PIN = 16

# Initialize UART
uart = UART(UART_ID, baudrate=BAUD_RATE, rx=Pin(RX_PIN)) #tx=Pin(TX_PIN),

# Buffer to store partial messages if needed, though IRQ_RXIDLE often gives complete messages
# For this simple example, we'll process whatever `read()` gives us.

def uart_rx_callback(uart_obj):  # Modified to accept one argument
    """
    Callback function to handle incoming UART data.
    The argument uart_obj is the UART instance that triggered the interrupt.
    """
    if uart_obj.any():  # Check if there's data in the UART buffer
        try:
            received_data = uart_obj.read()  # Read all available bytes
            if received_data:
                # Attempt to decode as UTF-8, common for text messages
                message = received_data.decode('utf-8').strip()
                print("Received:", message)
        except UnicodeError:
            # If decoding fails, print the raw bytes
            print("Received (raw bytes):", received_data)
        except Exception as e:
            print("Error in UART callback:", e)

# Configure UART interrupt
# UART.IRQ_RXIDLE: Trigger interrupt when data has been received and the RX line has been idle for a short period.
# This is often better for message-based communication than IRQ_RXNEMPTY (which triggers for every byte).
uart.irq(trigger=UART.IRQ_RXIDLE, handler=uart_rx_callback)

print(f"UART{UART_ID} initialized at {BAUD_RATE} baud on TX=GP{TX_PIN}, RX=GP{RX_PIN}.")
print("Listening for incoming messages...")
print(f"Send data to the Pico's UART RX pin (GP{RX_PIN}).")

# Keep the main script running to allow interrupts to be processed
try:
    while True:
        # The main loop can do other things or just sleep.
        # Interrupts will be handled in the background.
        time.sleep_ms(100)  # Sleep to reduce CPU usage
except KeyboardInterrupt:
    print("Program stopped by user.")
finally:
    # It's good practice to deinitialize UART if it's no longer needed,
    # though for a script that runs until reset/power-off, it might not be critical.
    if 'uart' in locals() and hasattr(uart, 'deinit'):
        uart.deinit()
    print("UART deinitialized.")
