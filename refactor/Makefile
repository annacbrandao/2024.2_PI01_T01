BOARD = arduino:avr:uno
SKETCH = refactor.ino
BUILD_DIR = build
PORT = /dev/ttyUSB0

# Compile using arduino-cli
compile:
	arduino-cli compile --fqbn $(BOARD) --output-dir $(BUILD_DIR) $(SKETCH)

# Extract .hex file
refactor.ino.hex: compile
	cp $(BUILD_DIR)/$(SKETCH).hex .

# Upload using avrdude
upload: refactor.ino.hex
	avrdude -c arduino -p atmega328p -P $(PORT) -b 115200 -D -U flash:w:$<:i

# Clean build directory
clean:
	rm -rf $(BUILD_DIR) refactor.ino.hex
