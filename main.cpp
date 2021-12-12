#include <Arduino.h>

#if defined(ARDUINO_AVR_MEGA2560) || defined(ARDUINO_AVR_LEONARDO)
// The I/O for the panel must be connected to the Serial1 pins (0 and 1).
const int pin_pump = 2;
const int pin_temp[] = { A0 };
#endif

const int pin_temp_count = sizeof(pin_temp) / sizeof(pin_temp[0]);
int last_temps[pin_temp_count];

// Start out the display bytes with a sane value.
uint8_t display_buffer[] = { 0x02, 0x00, 0x01, 0x00, 0x00, 0x01, 0xFF};
const int32_t display_bytes = sizeof(display_buffer) / sizeof(display_buffer[0]); 
bool display_dirty = true;

// loop no faster than 10Hz
const uint32_t loop_microseconds = 1000000 / 10;

uint32_t buttons = 0;
uint32_t last_buttons = 0;

bool pump_running = false;
int temp_setting = 100;
const int temp_min = 60;
const int temp_max = 104;

enum {
    button_jets = 0x01,
    button_light = 0x02,
    button_up = 0x04,
    button_down = 0x08,
};

void setup() 
{
  // delay(3000);
  Serial.begin(9600);
  // while (!Serial);

  // Communications with the control panel is 2400 baud ttl serial.
  Serial1.begin(2400);

  // Serial.println(F("Serial initialized"));
  // delay(100);

  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(pin_pump, OUTPUT);
  
}

void display_update_checksum()
{
  uint8_t sum = 0;
  sum += display_buffer[1];
  sum += display_buffer[2];
  sum += display_buffer[3];
  sum += display_buffer[4];
  display_buffer[5] = sum;

  display_dirty = true;
}

void display_filter(bool on)
{
    const uint8_t mask = 0x10;
    if (on) {
      display_buffer[1] |= mask;
    } else {
      display_buffer[1] &= ~mask;
    }

    display_update_checksum();
}

void display_heat(bool on)
{
    const uint8_t mask = 0x20;
    if (on) {
      display_buffer[1] |= mask;
    } else {
      display_buffer[1] &= ~mask;
    }

    display_update_checksum();
}

void display_temperature(int temp)
{
  display_buffer[4] = temp % 10;
  temp /= 10;
  display_buffer[3] = temp % 10;
  temp /= 10;
  display_buffer[2] = temp % 10;

  display_update_checksum();
}

void loop() {
  uint32_t loop_start_time = micros();
  
  while (Serial1.available()) {
    int raw = Serial1.read();
    // The 4 button bits are replicated and inverted between the low and high nybbles.
    // Check that they match, and extract just one copy.
    if ((raw & 0x0F) == (((raw >> 4) & 0x0F) ^ 0x0F))
    {
      buttons = raw >> 4;
    }
  }

  if (last_buttons != buttons)
  {
    // One or more buttons changed.
    // Serial.print(F("Buttons changed to "));
    // Serial.println(buttons, BIN);

    if (!(last_buttons & button_jets) && (buttons & button_jets))
    {
        // Jets button was pushed.
        pump_running = !pump_running;
        display_filter(pump_running);
        digitalWrite(LED_BUILTIN, pump_running);
        digitalWrite(pin_pump, pump_running);
    }

    if (!(last_buttons & button_up) && (buttons & button_up))
    {
        // Up button was pushed.
        if (temp_setting < temp_max) {
          temp_setting++;
          display_temperature(temp_setting);
        }
        
    }

    if (!(last_buttons & button_down) && (buttons & button_down))
    {
        // Down button was pushed.
        if (temp_setting > temp_min) {
          temp_setting--;
          display_temperature(temp_setting);
        }
    }
    
    last_buttons = buttons;
  }


  for (int i = 0; i < pin_temp_count; i++)
  {
    int value = analogRead(pin_temp[i]);    
    if (value != last_temps[i])
    {
      last_temps[i] = value;
      // Serial.print(F("Temp for sensor #"));
      // Serial.print(i);
      // Serial.print(F(" changed to "));
      // Serial.println(value);
    }
  }

  Serial1.write(display_buffer, display_bytes);

  uint32_t loop_time = micros() - loop_start_time;
  if (loop_time < loop_microseconds)
  {
    uint32_t msRemaining = loop_microseconds - loop_time;
    // On AVR, delayMicroseconds takes an int (16 bits), so it won't work properly if the delay time is over 65535ms
    if (msRemaining > UINT16_MAX) {
      delay(msRemaining / 1000);
    } else 
    {
      delayMicroseconds(msRemaining);
    }
  }
}

