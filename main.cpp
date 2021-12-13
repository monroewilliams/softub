#include <Arduino.h>

//////////////////////////////////////////////////
// board-specific defines
#if defined(ARDUINO_AVR_MEGA2560) || defined(ARDUINO_AVR_LEONARDO)
// The I/O for the panel must be connected to the Serial1 pins (0 and 1).
const int pin_pump = 2;
const int pin_temp[] = { A0 };
#endif

//////////////////////////////////////////////////
// run states
enum
{
  // Just started up.
  runstate_startup,
  // Not running, displaying setpoint temperature.
  runstate_idle,
  // Pump has just started running, we don't have confidence in the temperature reading yet
  runstate_finding_temp,
  // Temperature is known to be low, we're trying to increase it.
  runstate_heating,
  // The pump was turned on manually. Run it for 10 minutes.
  runstate_manual_pump,

  // Go to this state if we detect a problem with the temperature sensors. 
  // It shuts down the pump and sets the display into a "help me" state.
  // The user can reset to startup mode by holding the "light" and "jets" buttons for 5 seconds.
  runstate_panic
};

int runstate = runstate_startup;

// The time of the last transition. This may be used differently by different states.
uint32_t runstate_last_transition_millis = 0;

// The last valid temperature reading we took
int last_temp = 0;
// The millis timestamp when the last_temp was taken
uint32_t last_known_temp_millis = 0;

// Used to display temperature for a short time after the user adjusts it
bool temp_adjusted = false;
uint32_t temp_adjusted_millis = 0;
const uint32_t temp_adjusted_display_millis = 5 * 1000l;

// The amount of time to wait on startup before doing anything
const uint32_t startup_wait_seconds = 5;
// The amount of time we run the pump before believing the temperature reading
const uint32_t temp_settle_millis = 15 * 1000l;
// The amount of time after stopping the pump when we no longer consider the temp valid.
const uint32_t temp_decay_millis = 60 * 1000l;
// The amount of time in the idle state after which we should run to check the temperature.
const uint32_t idle_seconds = 1 * 60 * 60;
// When the user turns on the pump manually, run it for this long.
const uint32_t manual_pump_seconds = 60 * 10;
// The amount of time the user has to hold buttons to escape panic state
const uint32_t panic_wait_seconds = 5;

// Used to flash things in panic mode
bool panic_flash;

/////////////////////////////////////////////////
// other globals
const int pin_temp_count = sizeof(pin_temp) / sizeof(pin_temp[0]);
int last_temps[pin_temp_count];

// Start out the display bytes with a sane value.
uint8_t display_buffer[] = { 0x02, 0x00, 0x01, 0x00, 0x00, 0x01, 0xFF};
const int32_t display_bytes = sizeof(display_buffer) / sizeof(display_buffer[0]); 
bool display_dirty = true;

// loop no faster than 10Hz
const uint32_t loop_microseconds = 1000000l / 10;

uint32_t buttons = 0;
uint32_t last_buttons = 0;

bool pump_running = false;
uint32_t pump_switch_millis = 0;
bool temp_valid = false;

int temp_setting = 100;
const int temp_min = 60;
const int temp_max = 104;

enum {
  button_jets = 0x01,
  button_light = 0x02,
  button_up = 0x04,
  button_down = 0x08,
};


void runstate_transition()
{
  runstate_last_transition_millis = millis();
}

void enter_state(int state)
{
  runstate = state;
  runstate_transition();
}

void display_update_checksum()
{
  uint8_t sum = 0;
  sum += display_buffer[1];
  sum += display_buffer[2];
  sum += display_buffer[3];
  sum += display_buffer[4];
  display_buffer[5] = sum;
}

void display_set_digit(int digit, uint8_t value)
{
  if ((digit >= 0) && (digit < 3))
  {
    if (display_buffer[2 + digit] != value)
    {
      display_buffer[2 + digit] = value;
      display_dirty = true;
    }
  }
}
void display_set_digits(uint8_t a, uint8_t b, uint8_t c)
{
  display_set_digit(0, a);
  display_set_digit(1, b);
  display_set_digit(2, c);
}

void display_set_bits(int byte, int mask, bool value)
{
  if (value) {
    if ((display_buffer[byte] & mask) == 0)
    {
      display_buffer[byte] |= mask;
      display_dirty = true;
    }
  } else {
    if ((display_buffer[byte] & mask) != 0)
    {
      display_buffer[byte] &= ~mask;
      display_dirty = true;
    }
  }
}
void display_filter(bool on)
{
  display_set_bits(1, 0x10, on);
}

void display_heat(bool on)
{
  display_set_bits(1, 0x20, on);
}

// Values in digit places:
// 0 - 9 - digit
// 0x0a - blank
// 0x0b -- "P"
void display_temperature(int temp)
{
  display_set_digits(temp / 100, (temp / 10) % 10, temp % 10);
}

void display_panic()
{
  display_set_digits(panic_flash?0x0b:0x0a, 0x0a, panic_flash?0x0a:0x0b);
  display_filter(panic_flash);
  display_heat(!panic_flash);
}

void display_panic_countdown(int countdown)
{
  display_set_digits(0x0a, countdown, 0x0a);
  display_filter(true);
  display_heat(true);
}

void display_send()
{
  // For now, always send whether dirty or not.
  // if (display_dirty)
  {
    display_update_checksum();
    Serial1.write(display_buffer, display_bytes);
    display_dirty = false;
  }
}

void set_pump(bool running)
{
  if (pump_running != running)
  {
    display_filter(running);
    digitalWrite(LED_BUILTIN, running);
    digitalWrite(pin_pump, running);
    pump_running = running;
    pump_switch_millis = millis();
  }
}

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
  
  enter_state(runstate_startup);
}

void loop() {
  uint32_t loop_start_time = micros();
  
  // Temperature readings are meaningless if the pump isn't running.
  // We also want to ignore temp readings when the pump was just turned on,
  // and consider the temp valid for a short time after it turns off.
  if (pump_running && !temp_valid)
  {
    if (millis() - pump_switch_millis > temp_settle_millis)
    {
      // The pump has been running long enough for temp to be valid.
      temp_valid = true;
    }
  }
  else if (!pump_running && temp_valid)
  {
    if (millis() - pump_switch_millis > temp_decay_millis)
    {
        // The pump has been off long enough that we should no longer consider the temp valid.
      temp_valid = false;
    }
  }

  if (temp_valid)
  {
    // The readings from the temperature sensors should be valid.
    int avg_temp = 0;
    for (int i = 0; i < pin_temp_count; i++)
    {
      int value = analogRead(pin_temp[i]);    
      avg_temp += value;
      if (value != last_temps[i])
      {
        last_temps[i] = value;
        Serial.print(F("Temp for sensor #"));
        Serial.print(i);
        Serial.print(F(" changed to "));
        Serial.println(value);
      }
    }
    // FIXME: check for disagreement in the temp readings and panic if sensors diverged too far
    last_temp = avg_temp / pin_temp_count;
    last_known_temp_millis = millis();

    // HACK: for now, pretend the temp is always 95.
    last_temp = 95;
  }

  // Read buttons
  while (Serial1.available()) {
    int raw = Serial1.read();
    // The 4 button bits are replicated and inverted between the low and high nybbles.
    // Check that they match, and extract just one copy.
    if ((raw & 0x0F) == (((raw >> 4) & 0x0F) ^ 0x0F))
    {
      buttons = raw >> 4;
    }
  }

  uint32_t millis_since_last_transition = (millis() - runstate_last_transition_millis);
  uint32_t seconds_since_last_transition = millis_since_last_transition / 1000l;
  bool jets_toggled = false;

  if (last_buttons != buttons)
  {
    // One or more buttons changed.
    // Serial.print(F("Buttons changed to "));
    // Serial.println(buttons, BIN);

    if (runstate != runstate_panic)
    {
      if (!(last_buttons & button_jets) && (buttons & button_jets))
      {
        // Jets button was pushed.
        jets_toggled = true;
      }

      if (!(last_buttons & button_up) && (buttons & button_up))
      {
        // Up button was pushed.
        if (temp_setting < temp_max) {
          temp_setting++;
          temp_adjusted = true;
          temp_adjusted_millis = millis();
        }
      }

      if (!(last_buttons & button_down) && (buttons & button_down))
      {
        // Down button was pushed.
        if (temp_setting > temp_min) {
          temp_setting--;
          temp_adjusted = true;
          temp_adjusted_millis = millis();
        }
      }

      if (!(last_buttons & button_light) && (buttons & button_light))
      {
        // Light button was pushed.
        // TESTING: go into panic state
        enter_state(runstate_panic);
      }
    } else {
      // When in panic state, the only thing we do is look for the jets and lights buttons to be held down together.
      // Whenever the buttons change, reset the countdown timer.
      runstate_transition();
    }

    last_buttons = buttons;
  }

  if (temp_adjusted)
  {
    // See if the temp-adjusted display window has expired.
    if (millis() - temp_adjusted_millis > temp_adjusted_display_millis)
    {
      temp_adjusted = false;
    }
  }

  // Default to displaying the current temp if it's valid and not recently adjusted, or the set point otherwise.
  if (temp_valid && !temp_adjusted) {
    display_temperature(last_temp);
  } else {
    display_temperature(temp_setting);
  }

  // In most states, we want the heating light off.
  display_heat(false);

  switch(runstate) {
    case runstate_startup:
        display_set_digits(0x0a, 0x00, 0x0a);
      if (seconds_since_last_transition > startup_wait_seconds)
      {
        // Turn on the pump and enter the "finding temp" state.
        enter_state(runstate_finding_temp);
      }
    break;
    case runstate_finding_temp:
      set_pump(true);
      if (temp_valid)
      {
        // Go into the heating state. It will flip back to idle if no heating is needed.
        enter_state(runstate_heating);
      } else {
        // indicate that we're still finding temp
        display_set_digits(0x0a, 0x0b, 0x0a);
      }
    break;
    case runstate_heating:
      set_pump(true);
      display_heat(true);
      // When we're in this state, the temp should always be valid.
      if (last_temp > temp_setting)
      {
         // We've reached the set point. Turn the pump off and go to idle.
        enter_state(runstate_idle);
      }
    break;
    case runstate_idle:
      set_pump(false);
      if (jets_toggled)
      {
        // From the idle state, allow the user to turn on the pump manually.
        enter_state(runstate_manual_pump);
      }
      else if (seconds_since_last_transition > idle_seconds)
      {
        // We've been idle long enough that we should do a temperature check.
        enter_state(runstate_finding_temp);
      }
      else if (temp_adjusted)
      {
        // If the user adjusted the temperature to above the last reading (even if that reading is no longer valid),
        // we may need to heat. Transition to the finding temperature state.
        if (temp_setting > last_temp)
        {
          enter_state(runstate_finding_temp);
        }

      }
    break;
    case runstate_manual_pump:
      set_pump(true);
      if (jets_toggled)
      {
        // From the manual state, allow the user to turn off the pump.
        enter_state(runstate_idle);
      }
    break;
    case runstate_panic:
      set_pump(false);
      if (buttons == (button_jets | button_light))
      {
        // User is holding down the button combo. 
        if (seconds_since_last_transition >= panic_wait_seconds)
        {
          // They've successfully escaped.
          enter_state(runstate_startup);
        } else {
          display_panic_countdown(panic_wait_seconds -  seconds_since_last_transition);
        }
      }
      else
      {
        // Not holding buttons, just flash
        if (millis_since_last_transition >= 500)
        {
          panic_flash = !panic_flash;
          runstate_transition();
        }
        display_panic();
      }
    break;
  }

  display_send();

  uint32_t loop_time = micros() - loop_start_time;
  if (loop_time < loop_microseconds)
  {
    uint32_t msRemaining = loop_microseconds - loop_time;
    // On AVR, delayMicroseconds takes an int (16 bits), so it won't work properly if the delay time is over 65535ms
    if (msRemaining > UINT16_MAX) {
      delay(msRemaining / 1000l);
    } else 
    {
      delayMicroseconds(msRemaining);
    }
  }
}

