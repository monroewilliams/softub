// Enable strlcat (for esp8266, at least)
#define _DEFAULT_SOURCE 1

#include <Arduino.h>

#if defined(__AVR__)
  // Watchdog timer support
  #include <avr/wdt.h>

  // All AVRs have 10-bit ADCs.
  const int ADC_RESOLUTION = 1024;

  // Figure out which AREF option to use, and what the AREF voltage is.
  // Lower AREF means higher resolution within range, so we want the lowest voltage above what we expect to see from the sensors.
  // Many AVRs are able to select an internal AREF of 2.56 volts, which is a good choice.
  // Internal AREF is also likely more accurate/stable than VCC.
  #if defined(INTERNAL2V56)
    // If we're on an architecture that supports this option, use it.
    const double ADC_AREF_VOLTAGE = 2.56;
    const int ADC_AREF_OPTION = INTERNAL2V56;
  #elif defined(INTERNAL2V5)
    // This is apparently available on megaAVR?
    const double ADC_AREF_VOLTAGE = 2.5;
    const int ADC_AREF_OPTION = INTERNAL2V5;
  #else
    // The singular internal AREF varies by CPU type.
    #if defined(__AVR_ATmega32U4__) || defined(__AVR_ATmega8__)
      // Internal AREF is 2.56. Use it.
      const double ADC_AREF_VOLTAGE = 2.56;
      const int ADC_AREF_OPTION = INTERNAL;
    #else
      // internal AREF is too low (1.1v). Use the default VCC
      const double ADC_AREF_VOLTAGE = 5.0;
      const int ADC_AREF_OPTION = DEFAULT;
    #endif

  #endif

double readVcc() {
  // Save previous ADMUX value
  uint8_t previous = ADMUX;
  // Read 1.1V reference against AVcc
  // set the reference to Vcc and the measurement to the internal 1.1V reference
  #if defined(__AVR_ATmega32U4__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
    ADMUX = _BV(REFS0) | _BV(MUX4) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  #elif defined (__AVR_ATtiny24__) || defined(__AVR_ATtiny44__) || defined(__AVR_ATtiny84__)
     ADMUX = _BV(MUX5) | _BV(MUX0) ;
  #else
    ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  #endif  
 
  delay(2); // Wait for Vref to settle
  ADCSRA |= _BV(ADSC); // Start conversion
  while (bit_is_set(ADCSRA,ADSC)); // measuring
 
  uint8_t low  = ADCL; // must read ADCL first - it then locks ADCH  
  uint8_t high = ADCH; // unlocks both
 
  long adc = (high<<8) | low;
  
  // Restore ADMUX
  ADMUX = previous;
  delay(2); // Wait for Vref to settle

  return 1125.300 / adc; // Calculate Vcc (in V); 1125.300 = 1.1*1023
}

#elif defined(ARDUINO_ARCH_ESP8266)
  // ESP8266 has 10 bit ADCs
  const int ADC_RESOLUTION = 1024;

  // DAC measures from 0 to 1v.
  // The board _should_ have a voltage divider so that it reads from 0 to 3.3v.
  const double ADC_AREF_VOLTAGE = 3.3;
  const int ADC_AREF_OPTION = DEFAULT;

  // no watchdog timer support
  // #define wdt_enable(...)
  // #define wdt_disable(...)
  // #define wdt_reset(...)

  // This architecture defines a "panic" macro that conflicts with our local function. 
  #undef panic
#else
  // no watchdog timer support
  #define wdt_enable(...)
  #define wdt_disable(...)
  #define wdt_reset(...)
#endif

//////////////////////////////////////////////////
// board-specific defines
#if defined(ARDUINO_AVR_MEGA2560) || defined(ARDUINO_AVR_LEONARDO)
  // The I/O for the panel must be connected to the Serial1 pins (0 and 1).
  const int pin_pump = 2;
  const int pin_temp[] = { A0, A1 };
  const int OLED_CS = 5;
  const int OLED_DC = 4;
  const int OLED_RESET = 3;
#elif defined(ARDUINO_AVR_PROMICRO16)
  // The I/O for the panel must be connected to the Serial1 pins (0 and 1).
  const int pin_pump = 2;
  const int pin_temp[] = { A0, A1 };
  const int OLED_CS = 7;
  const int OLED_DC = 6;
  const int OLED_RESET = 5;
#elif defined(ARDUINO_ESP8266_WEMOS_D1MINI)
  // The I/O for the panel must be connected to the Serial1 pins (TX/RX).
  const int pin_pump = 0;
  const int pin_temp[] = { A0 };
  const int OLED_CS = 3;
  const int OLED_DC = 2;
  const int OLED_RESET = 1;
#endif

const double ADC_DIVISOR = ADC_AREF_VOLTAGE / double(ADC_RESOLUTION);

//////////////////////////////////////////////////
// Feature defines
// #define SERIAL_DEBUG 1
#define OLED_DISPLAY 1

#ifdef SERIAL_DEBUG
  void debug(const char *format, ...)
  {
    char string[256];
    va_list arg;
    va_start(arg, format);
    vsnprintf(string, sizeof(string), format, arg);

    Serial.println(string);
  }
  void serial_debug_init()
  {
    // delay(3000);
    Serial.begin(9600);
    // while (!Serial);

    debug("Serial initialized");
    // delay(100);
  }
#else
  #define serial_debug_init(...)
  #define debug(...)
#endif


#ifdef OLED_DISPLAY
#include <U8x8lib.h>

#if defined(ARDUINO_AVR_LEONARDO)
  // SCK and MOSI are only available in inconvenient locations on the Leonardo (on the ISCP header). 
  // Use SW SPI for this case.
  
  // These are the same pin assignments that the Uno used for its hardware SPI pins, for shield compatibility.
  const int OLED_CLOCK = 13;
  const int OLED_MOSI = 11;

  // https://www.amazon.com/gp/product/B01N1LZT8L
  U8X8_SH1106_128X64_NONAME_4W_SW_SPI oled(OLED_CLOCK, OLED_MOSI, OLED_CS, OLED_DC , OLED_RESET);
#else
  // Use hardware SPI

  // https://www.amazon.com/gp/product/B01N1LZT8L
  U8X8_SH1106_128X64_NONAME_4W_HW_SPI oled(OLED_CS, OLED_DC , OLED_RESET);
#endif

// Other possibilities:
// U8X8_SSD1306_128X64_NONAME_4W_HW_SPI oled(OLED_CS, OLED_DC , OLED_RESET);
// U8X8_SSD1306_128X64_NONAME_2ND_4W_HW_SPI oled(OLED_CS, OLED_DC , OLED_RESET);

void start_oled()
{
  oled.begin();
  oled.setFont(u8x8_font_8x13_1x2_f);
}

void print_oled(int line, const char *format, ...)
{
  const int width = 16;
  char string[width + 1];
  va_list arg;
  va_start(arg, format);
  vsnprintf(string, sizeof(string), format, arg);

  // pad the remainder of the string with spaces, so it clears any previous text on this line.
  strlcat(string , "                      ", sizeof(string));

  oled.drawString(0, line * 2, string);
}

#else // !OLED_DISPLAY

// Turn these into no-ops.
#define start_oled(...)
#define print_oled(...)

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

  // for testing
  runstate_test,

  // Go to this state if we detect a problem with the temperature sensors. 
  // It shuts down the pump and sets the display into a "help me" state.
  // The user can reset to startup mode by holding the "light" and "jets" buttons for 5 seconds.
  runstate_panic
};

int runstate = runstate_startup;

// The time of the last transition. This may be used differently by different states.
uint32_t runstate_last_transition_millis = 0;

// The time of the last change to buttons pressed. This may be used differently by different states.
uint32_t buttons_last_transition_millis = 0;

// The last valid temperature reading we took
double last_temp = 0;

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
// If smoothed readings ever disagree by this many degrees f, panic.
const int panic_sensor_difference = 10;
// If the temperature reading ever exceeds this many degrees f, panic.
const int panic_high_temp = 110;

// Used to flash things in panic mode
bool panic_flash;

/////////////////////////////////////////////////
// other globals
const int pin_temp_count = sizeof(pin_temp) / sizeof(pin_temp[0]);

// smooth the temperature sampling over this many samples, to filter out noise.
const int temp_sample_count = 32;
int temp_sample_pointer = 0;
int temp_samples[pin_temp_count][temp_sample_count];

// Start out the display bytes with a sane value.
uint8_t display_buffer[] = { 0x02, 0x00, 0x01, 0x00, 0x00, 0x01, 0xFF};
const int32_t display_bytes = sizeof(display_buffer) / sizeof(display_buffer[0]); 
bool display_dirty = true;

// loop no faster than 30Hz
const uint32_t loop_microseconds = 1000000l / 30;

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

void temp_adjust(int amount)
{
  temp_setting += amount;
  if (temp_setting > temp_max)
    temp_setting = temp_max;
  if (temp_setting < temp_min)
    temp_setting = temp_min;
  temp_adjusted = true;
  temp_adjusted_millis = millis();  
}

double temp_to_farenheit(double reading)
{
  // Scale the reading to voltage
  reading *= ADC_DIVISOR;

  // The temperature sensors seem to be LM34s.
  // (Linear, 750mv at 75 degrees F, slope 10mv/degree F)
  // Basically, temperature in F is voltage * 100.
  return (reading * 100);
}

void runstate_transition()
{
  runstate_last_transition_millis = millis();
}

void enter_state(int state)
{
  runstate = state;
  runstate_transition();

#ifdef OLED_DISPLAY
  const char *name = "UNKNOWN";
  switch(state) {
    case runstate_startup:       name = "startup"; break;
    case runstate_idle:          name = "idle"; break;
    case runstate_finding_temp:  name = "finding temp"; break;
    case runstate_heating:       name = "heating"; break;
    case runstate_manual_pump:   name = "manual"; break;
    case runstate_test:          name = "test"; break;
    case runstate_panic:         name = "PANIC"; break;
  }
  print_oled(0, "%s", name);
#endif
}

void panic()
{
  if (runstate != runstate_panic) {
    enter_state(runstate_panic);
  }
}

double smoothed_sensor_reading(int sensor)
{
    if (sensor >= pin_temp_count) {
      sensor = 0;
    }

    double result = 0;
    for (int i = 0; i < temp_sample_count; i++) {
      result += temp_samples[sensor][i];
    }
    result /= temp_sample_count;

    return result;
}

void read_temp_sensors()
{
  double avg_reading = 0;
  for (int i = 0; i < pin_temp_count; i++)
  {
    int value = analogRead(pin_temp[i]);

    // Save the current sample in the ring buffer
    temp_samples[i][temp_sample_pointer] = value;

    // Smooth the temperature sampling over temp_sample_count samples
    double smoothed_value = smoothed_sensor_reading(i);
    avg_reading += smoothed_value;

#ifdef OLED_DISPLAY
    // char raw_string[32];
    // dtostrf(smoothed_value, 1, 0, raw_string);
    char voltage_string[32];
    dtostrf(smoothed_value * ADC_DIVISOR, 1, 3, voltage_string);
    char farenheit_string[32];
    dtostrf(temp_to_farenheit(smoothed_value), 1, 1, farenheit_string);
    print_oled(i + 1, "%s %s %d", farenheit_string, voltage_string, value);
#endif
  }

  // Calculate the average smoothed temp and save it.
  avg_reading /= pin_temp_count;
  last_temp = temp_to_farenheit(avg_reading);

  // If the smoothed readings from sensors 0 and 1 ever differ by more than panic_sensor_difference degrees, panic.
  if (fabs(temp_to_farenheit(smoothed_sensor_reading(0)) - temp_to_farenheit(smoothed_sensor_reading(1))) > panic_sensor_difference)
  {
    panic();
  }
  
  // If calculated temperature is over our defined limit, panic.
  if (last_temp > panic_high_temp) 
  {
    panic();
  }

  // Advance the sample pointer in the ring buffer.
  temp_sample_pointer++;
  if (temp_sample_pointer >= temp_sample_count) {
    temp_sample_pointer = 0;
  }

  // Reset the watchdog timer.
  wdt_reset();
}

void read_buttons()
{
  while (Serial1.available()) {
    int raw = Serial1.read();
    // The 4 button bits are replicated and inverted between the low and high nybbles.
    // Check that they match, and extract just one copy.
    if ((raw & 0x0F) == (((raw >> 4) & 0x0F) ^ 0x0F))
    {
      buttons = raw >> 4;
    }
  }
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
  display_temperature(temp_to_farenheit(smoothed_sensor_reading(panic_flash?0:1)));
  display_heat(!panic_flash);
  display_filter(panic_flash);
}

void display_panic_countdown(int countdown)
{
  display_set_digits(0x0a, countdown, 0x0a);
  display_filter(true);
  display_heat(true);
}

void display_vcc()
{
#if defined(__AVR__)
  #if defined(OLED_DISPLAY)
    double vcc = readVcc();
    // char raw_string[32];
    // dtostrf(smoothed_value, 1, 0, raw_string);
    char vcc_string[32];
    dtostrf(vcc, 1, 3, vcc_string);
    print_oled(1 + pin_temp_count, "VCC = %s", vcc_string);
  #endif
#endif
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
    digitalWrite(pin_pump, running);
    pump_running = running;
    pump_switch_millis = millis();
  }
}

void setup() 
{
  // Just in case.
  wdt_disable();

  serial_debug_init();

  // Communications with the control panel is 2400 baud ttl serial.
  Serial1.begin(2400);

  // Set the AREF voltage for reading from the sensors.
  analogReference(ADC_AREF_OPTION);

  pinMode(pin_pump, OUTPUT);
  // Make really sure the pump is not running.
  digitalWrite(pin_pump, 0);
  
  start_oled();

  display_vcc();

  enter_state(runstate_startup);
}

void check_temp_validity()
{
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
}


void loop() {
  uint32_t loop_start_micros = micros();
  uint32_t loop_start_millis = millis();
  
  read_temp_sensors();

  read_buttons();

  check_temp_validity();

  bool jets_pushed = false;
  bool lights_pushed = false;

  if (last_buttons != buttons)
  {
    // One or more buttons changed.
    buttons_last_transition_millis = loop_start_millis;
    debug("Buttons changed to 0x%02x, buttons_last_transition_millis = %ld", int(buttons), buttons_last_transition_millis);

    if (runstate != runstate_panic)
    {
      if (!(last_buttons & button_jets) && (buttons & button_jets))
      {
        // Jets button was pushed.
        jets_pushed = true;
      }

      if (!(last_buttons & button_up) && (buttons & button_up))
      {
        // Up button was pushed.
        temp_adjust(1);
      }

      if (!(last_buttons & button_down) && (buttons & button_down))
      {
        // Down button was pushed.
        temp_adjust(-1);
      }

      if (!(last_buttons & button_light) && (buttons & button_light))
      {
        lights_pushed = true;
      }
    } else {
      // When in panic state, the only thing we do is look for the jets and lights buttons to be held down together.
      // Whenever the buttons change, reset the countdown timer.
      runstate_transition();
    }

    last_buttons = buttons;
  }

  // Useful timing shortcuts for the state machine
  uint32_t millis_since_last_transition = loop_start_millis - runstate_last_transition_millis;
  uint32_t seconds_since_last_transition = millis_since_last_transition / 1000l;
  uint32_t millis_since_button_change = loop_start_millis - buttons_last_transition_millis;
  uint32_t seconds_since_button_change = millis_since_button_change / 1000l;

  if (temp_adjusted)
  {
    uint32_t millis_since_temp_adjust = millis() - temp_adjusted_millis;

    // See if the temp-adjusted display window has expired.
    if (millis_since_temp_adjust  > temp_adjusted_display_millis)
    {
      temp_adjusted = false;
    }
    else
    {
      if (((buttons == button_up) || (buttons == button_down)) &&
          (millis_since_temp_adjust <= millis_since_button_change))
      {
        // If the user has been holding down the up or down button since the last temp adjustment,
        // and the last temp adjustment was over the repeat time (1/2s for the first adjustment, 1/4s for the next 1.5 seconds, 1/10s thereafter), 
        // adjust again.
        uint32_t repeat_time = 500;
        if (millis_since_button_change > 2000) {
          repeat_time = 100;
        } else if (millis_since_button_change > 500) {
          repeat_time = 250;
        }

        if (millis_since_temp_adjust >= repeat_time)
        {
          temp_adjust((buttons == button_up)?1:-1);
        }
      }
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

  // Pushing the lights button at any time immediately enters the test state.
  // if (runstate != runstate_test && lights_pushed) {
  //   enter_state(runstate_test);
  //   lights_pushed = false;
  // }
  
  switch(runstate) {
    case runstate_startup:
        display_set_digits(0x0a, 0x00, 0x0a);
      if (seconds_since_last_transition > startup_wait_seconds)
      {
        // Before starting the pump for the first time, enable the watchdog timer.
        wdt_enable(WDTO_4S);

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
      if (jets_pushed)
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
      if (jets_pushed)
      {
        // From the manual state, allow the user to turn off the pump.
        enter_state(runstate_idle);
      }
    break;
    case runstate_test:
    {
      if (lights_pushed)
      {
        // Exit the test state
        enter_state(runstate_finding_temp);
        print_oled(3, "");
      } else {
        print_oled(3, "012345678901234567890");
      }
    }
    break;
    case runstate_panic:
      set_pump(false);
      if (buttons == (button_jets | button_light))
      {
        // User is holding down the button combo. 
        if (seconds_since_button_change >= panic_wait_seconds)
        {
          // They've successfully escaped.
          enter_state(runstate_startup);
        } else {
          display_panic_countdown(panic_wait_seconds -  seconds_since_button_change);
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

  // Always update the display of vcc.
  display_vcc();

  display_send();

  uint32_t loop_time = micros() - loop_start_micros;
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

