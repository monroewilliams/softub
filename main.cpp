#include <Arduino.h>


//////////////////////////////////////////////////
// Feature defines
// These are now expected to be set from platformio.ini, like so:
// build_flags = 
// 	-D WEBSERVER=1

// #define SERIAL_DEBUG 1
// #define OLED_DISPLAY 1
// #define WEBSERVER 1
// #define OTA_UPDATE 1
// #define WEBSERVER_DEBUG 1
// #define WEBSERVER_REMOTE_CONTROL 1

#if !defined(MDNS_NAME)
  #define MDNS_NAME "softub"
#endif

// Networking is only available on ESP32.
#if (defined(WEBSERVER) || defined(OTA_UPDATE)) && defined(ARDUINO_ARCH_ESP32)
  #if !defined(WIFI_SSID) || !defined(WIFI_PASSWORD)
    #error "Network credentials must be defined"
  #endif
  #define NETWORK 1
#endif

void network_start();
void network_service();


//////////////////////////////////////////////////
// board-specific defines
#if defined(ARDUINO_AVR_LEONARDO)
  // The I/O for the panel must be connected to the Serial1 pins (0 and 1).
  const int pin_pump = 2;
  const int pin_temp[] = { A5, A4 };

  // Make sure the pins shared with hardware SPI via the shield are high-impedance.
  #define QUIESCE_PINS \
    pinMode(13, INPUT); \
    pinMode(11, INPUT);

  const int OLED_RESET = 10;
  const int OLED_DC = 9;
  const int OLED_CS = 8;
  // The shield has SCK/MOSI broken out from the ICSP header, so we're using hardware SPI.
#elif defined(ARDUINO_AVR_PROMICRO16)
  // The I/O for the panel must be connected to the Serial1 pins (0 and 1).
  const int pin_pump = 2;
  const int pin_temp[] = { A0, A1 };

  const int OLED_RESET = 5;
  const int OLED_DC = 6;
  const int OLED_CS = 7;
#elif defined(ARDUINO_ARDUCAM_IOTAI) || defined(ARDUINO_ESP32_DEV)
  const int pin_pump = D2;
  const int pin_temp[] = { S5, S4, S3 };

  // Make sure the pins shared with hardware SPI via the shield are high-impedance.
  // The spot for pin 13 is defined as "no connection" on this one
  #define QUIESCE_PINS \
    pinMode(D11, INPUT);

  const int PANEL_TX = D4;
  const int PANEL_RX = D5;

  const int OLED_RESET = D10;
  const int OLED_DC = D9;
  const int OLED_CS = D8;

  #define TSENS_OFFSET (-113)
  #define TSENS_MULTIPLIER (4.0)

#elif defined(ARDUINO_WEMOS_D1_R32)
  // SOME pins files for this board ID define the pins using their actual IO line numbers.
  // The esp32doit-espduino board variant is one of those.
  const int pin_pump = IO26;
  const int pin_temp[] = { IO39, IO36 };

  // The TX/RX pins on this board are shared with the USB serial interface, which is problematic.
  // (Even if we're not using serial debug, the bootloader and other services will send data on these lines sometimes.)
  // Use the pins that would be assigned to Serial2 by default.
  // This means the shield pinout needed for this board will be different from the one for the Leonardo. 
  // I admit defeat.
  const int PANEL_TX = IO17;
  const int PANEL_RX = IO16;
  
  const int OLED_RESET = IO5;
  const int OLED_DC = IO13;
  const int OLED_CS = IO12;

  // From my testing of the WeMos board, it appears that the value in the tsens reg increases by 1 for every ~5 degrees farenheit.
  // It is 122 at 75 degrees ambient.
  #define TSENS_OFFSET (-107)
  #define TSENS_MULTIPLIER (5.0)

#endif

//////////////////////////////////////////////////
// CPU-architecture-specific defines

#if defined(__AVR__)
  // Watchdog timer support
  #include <avr/wdt.h>
  void watchdog_init() {
    // Start with the watchdog timer disabled
    wdt_disable();
  }
  void watchdog_start() {
    wdt_enable(WDTO_2S);
  }
  void watchdog_reset() {
    wdt_reset();
  }

  // All AVRs have 10-bit ADCs.
  const int ADC_RESOLUTION = 1024;

  // Figure out which AREF option to use, and what the AREF voltage is.
  // Lower AREF means higher resolution within range, so we want the lowest voltage above what we expect to see from the sensors.
  // Many AVRs are able to select an internal AREF of 2.56 volts, which is a good choice.
  // Internal AREF is also likely more accurate/stable than VCC.
  #if defined(INTERNAL2V56)
    // If we're on an architecture that supports this option, use it.
    const double ADC_AREF_VOLTAGE = 2.56;
    #define ADC_AREF_OPTION INTERNAL2V56
  #elif defined(INTERNAL2V5)
    // This is apparently available on megaAVR?
    const double ADC_AREF_VOLTAGE = 2.5;
    #define ADC_AREF_OPTION INTERNAL2V5
  #else
    // The singular internal AREF varies by CPU type.
    #if defined(__AVR_ATmega32U4__) || defined(__AVR_ATmega8__)
      // Internal AREF is 2.56. Use it.
      const double ADC_AREF_VOLTAGE = 2.56;
      #define ADC_AREF_OPTION INTERNAL
    #else
      // internal AREF is too low (1.1v). Use the default VCC
      const double ADC_AREF_VOLTAGE = 5.0;
      #define ADC_AREF_OPTION DEFAULT
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

uint64_t micros64() {
    return micros();
}

uint64_t millis64() {
    return millis();
}

#elif defined(ARDUINO_ARCH_ESP32)
  // Watchdog timer support
  #include <esp_task_wdt.h>
  void watchdog_init() {
    // Start with the watchdog timer disabled
    esp_task_wdt_init(15, false);
  }
  void watchdog_start() {
    // Start the watchdog timer watching this task
    esp_task_wdt_init(15, true);
    esp_task_wdt_add(NULL);
  }
  void watchdog_reset() {
    esp_task_wdt_reset();
  }

  // ESP32 has 12 bit ADCs
  const int ADC_RESOLUTION = 4096;

  // Ref: https://esp32.com/viewtopic.php?t=1053
  // The ADC measures from 0 to 1.1v, moderated by the attenuation factor.
  // The voltage attenuation defaults to ADC_11db, which scales it by a factor of 1/3.6.
  // const double ADC_AREF_VOLTAGE = 1.1 * 3.6;

  // Reducing the attenuation factor a bit gives us more usable resolution.
  // ADC_0db provides no attenuation, so we measure 0 - 1.1v
  // ADC_2_5db provides attenuation of 1/1.34, so we measure 0 - 1.47v
  // ADC_6db provides an attenuation of 1/2 , so we measure 0 - 2.2v
  // ADC_11db provides an attenuation of 1/3.6, so we measure 0 - 3.96v

  // From comments in the adc.h header: 
  // Due to ADC characteristics, most accurate results are obtained within the following approximate voltage ranges:
  // - 0dB attenuaton (ADC_ATTEN_DB_0) between 100 and 950mV
  // - 2.5dB attenuation (ADC_ATTEN_DB_2_5) between 100 and 1250mV
  // - 6dB attenuation (ADC_ATTEN_DB_6) between 150 to 1750mV
  // - 11dB attenuation (ADC_ATTEN_DB_11) between 150 to 2450mV

  // I don't want to use ADC_0db, as this is only accurage to 0.95v and maxes out at 1.1 = 110 degrees F, which we could see.
  // It's possible that the temperature sensor in the pod would approach 125 degrees, which is the edge of the accurate range for 2.5dB.
  // 6dB should be accurate up to 1.75v = 175 degrees F, which should give us ample headroom.
  #define ADC_ATTENUATION_FACTOR ADC_6db
  const double ADC_AREF_VOLTAGE = 1.1 * 2.0;

  // Handy constants for interpreting esp_timer_get_time()
  const int64_t millisecond = 1000l;
  const int64_t second = 1000 * millisecond;
  const int64_t minute = 60 * second;
  const int64_t hour = 60 * minute;
  const int64_t day = 24 * hour;
  const int64_t year = 365 * day;

  // 64 bit versions of micros() and millis().
  // The 32 bit versions roll over at ~1.2 seconds and ~50 days, respectively.
  // By going to 64 bits, we get something like 5000 centuries and  and 5 million centuries, which should be sufficent.
  uint64_t micros64() {
      return esp_timer_get_time();
  }

  uint64_t millis64() {
      return esp_timer_get_time() / millisecond;
  }

  // Code for reading the built-in CPU temperature sensor.
  #if defined(TSENS_OFFSET) && defined(TSENS_MULTIPLIER)
    #define CPU_TEMP_AVAILABLE
    extern "C" uint8_t read_tsens_register();
    float cpu_temp()
    {
      uint8_t tsens = read_tsens_register();
      
      // Assuming the TSENS reading is actually linear, this should convert it to Farenheit:
      return (tsens + TSENS_OFFSET) * TSENS_MULTIPLIER;
    }
  #endif

#else
  // no watchdog timer support
  void watchdog_init() {}
  void watchdog_start() {}
  void watchdog_reset() {}
#endif

const double ADC_DIVISOR = ADC_AREF_VOLTAGE / double(ADC_RESOLUTION);

#ifdef SERIAL_DEBUG
  #ifndef SERIAL_DEBUG_SPEED
    #define SERIAL_DEBUG_SPEED 9600
  #endif

  void debug(const char *format, ...)
  {
    char string[256];
    va_list arg;
    va_start(arg, format);
    vsnprintf(string, sizeof(string), format, arg);

    Serial.println(string);
  }
  void debug(const String& string)
  {
    Serial.println(string);
  }
  void serial_debug_init()
  {
    // delay(3000);
    Serial.begin(SERIAL_DEBUG_SPEED);
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

#if defined(OLED_MOSI)
  // Use software SPI
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
  // oled.setFont(u8x8_font_amstrad_cpc_extended_f);
  // oled.setFont(u8x8_font_pressstart2p_f);
  oled.setFont(u8x8_font_pcsenior_f);
  // oled.setFont(u8x8_font_pxplusibmcgathin_f);
  // oled.setFont(u8x8_font_pxplusibmcga_f);
  // oled.setFont(u8x8_font_pxplustandynewtv_f);

  // oled.setFont(u8x8_font_8x13_1x2_f);
}

const int display_width = 16;

void print_oled(int line, const char *format, ...)
{
  char string[display_width + 1];
  va_list arg;
  va_start(arg, format);
  vsnprintf(string, sizeof(string), format, arg);

  // pad the remainder of the string with spaces, so it clears any previous text on this line.
  strlcat(string , "                      ", sizeof(string));

  oled.drawString(0, line, string);
}
void print_oled(int line, const String& string)
{
  // pad the remainder of the string with spaces, so it clears any previous text on this line.
  String s = string + "                      ";
  oled.drawString(0, line, s.substring(0, display_width).c_str());
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

// Min and max allowed setpoints
const int temp_min = 50;
const int temp_max = 110;

// The time of the last transition. This may be used differently by different states.
uint64_t runstate_last_transition_millis = 0;

// The time of the last change to buttons pressed. This may be used differently by different states.
uint64_t buttons_last_transition_millis = 0;

// The last temperature reading we took
double last_temp = 0;
// The last valid temperature reading we took
double last_valid_temp = 0;

// Used to display temperature for a short time after the user adjusts it
bool temp_adjusted = false;
uint64_t temp_adjusted_millis = 0;
const uint64_t temp_adjusted_display_millis = 5 * 1000l;

// The amount of time to wait on startup before doing anything
const uint64_t startup_wait_seconds = 10;
// The amount of time we run the pump before believing the temperature reading
const uint64_t temp_settle_millis = 30 * 1000l; // 30 seconds
// The amount of time after stopping the pump when we no longer consider the temp valid.
const uint64_t temp_decay_millis = 60 * 1000l; // 60 seconds
// The amount of time in the idle state after which we should run to check the temperature.
const uint64_t idle_seconds = 15 * 60;  // 15 minutes
// When the user turns on the pump manually, run it for this long.
const uint64_t manual_pump_seconds = 5 * 60; // 5 minutes
// The amount of time the user has to hold buttons to escape panic state
const uint64_t panic_wait_seconds = 5;
// If smoothed readings ever disagree by this many degrees f, panic.
const int panic_sensor_difference = 10;
// If the temperature reading ever the max set temperature plus 5 degrees f, panic.
const int panic_high_temp = temp_max + 5;

// Used to flash things in panic mode
bool panic_flash;

String panic_string;

/////////////////////////////////////////////////
// other globals
const int pin_temp_count = sizeof(pin_temp) / sizeof(pin_temp[0]);

// smooth the temperature sampling over this many samples, to filter out noise.
const int temp_sample_count = 128;
int temp_sample_pointer = 0;
int temp_samples[pin_temp_count][temp_sample_count];

// Start out the display bytes with a sane value.
uint8_t display_buffer[] = { 0x02, 0x00, 0x01, 0x00, 0x00, 0x01, 0xFF};
const int32_t display_bytes = sizeof(display_buffer) / sizeof(display_buffer[0]); 
bool display_dirty = true;

// loop no faster than 16Hz
const uint64_t loop_microseconds = 1000000l / 16;

uint32_t buttons = 0;
uint32_t last_buttons = 0;

bool pump_running = false;
uint64_t pump_switch_millis = 0;
bool temp_valid = false;

int temp_setting = 100;
// stop heating at temp_setting + temp_setting_range, 
double temp_setting_range = 0.5;

enum {
  button_jets = 0x01,
  button_light = 0x02,
  button_up = 0x04,
  button_down = 0x08,
};

String dtostr(double number, signed char width = 1, unsigned char prec = 1) 
{
  char buf[32];
  dtostrf(number, width, prec, buf);
  return String(buf);
}

void temp_adjust(int amount)
{
  temp_setting += amount;
  if (temp_setting > temp_max)
    temp_setting = temp_max;
  if (temp_setting < temp_min)
    temp_setting = temp_min;
  temp_adjusted = true;
  temp_adjusted_millis = millis64();  
}

double adc_to_farenheit(double reading)
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
  runstate_last_transition_millis = millis64();
}

const char* state_name(int state)
{
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
  return name;
}

void enter_state(int state)
{
  runstate = state;
  runstate_transition();

#ifdef OLED_DISPLAY
  print_oled(0, "%s", state_name(state));
#endif
}

void panic()
{
  if (runstate != runstate_panic) {
    enter_state(runstate_panic);
  }
}

void panic(const char *format, ...)
{
  char string[1024];
  va_list arg;
  va_start(arg, format);
  vsnprintf(string, sizeof(string), format, arg);

  panic_string = String(string);
  panic();
}

// Optionally return the lowest/highest readings in the buffer
double smoothed_sensor_reading(int sensor, int *lowest = NULL, int *highest = NULL)
{
    if (sensor >= pin_temp_count) {
      sensor = 0;
    }
    if (lowest != NULL) {
      *lowest = INT_MAX;
    }
    if (highest != NULL) {
      *highest = INT_MIN;
    }

    double result = 0;
    for (int i = 0; i < temp_sample_count; i++) {
      int sample = temp_samples[sensor][i];
      result += sample;
      if (lowest != NULL) {
          *lowest = min(*lowest, sample);
      }
      if (highest != NULL) {
          *highest = max(*highest, sample);
      }
    }
    result /= temp_sample_count;

    return result;
}

void read_temp_sensors()
{
  // Advance the sample pointer in the ring buffer.
  temp_sample_pointer++;
  if (temp_sample_pointer >= temp_sample_count) {
    temp_sample_pointer = 0;
  }

  double avg_reading = 0;
  for (int i = 0; i < pin_temp_count; i++)
  {
#ifdef ARDUINO_ARCH_ESP32
    // Use the calibrated API to read actual voltage in millivolts
    uint32_t mv = analogReadMilliVolts(pin_temp[i]);
    // Convert it to our defined range.
    // The entire right-hand multiplier here is just constant values, so it should be computed at compile-time.
    int value = double(mv) * ((1.0 / ADC_AREF_VOLTAGE) * (ADC_RESOLUTION / 1000.0));
#else
    int value = analogRead(pin_temp[i]);
#endif

    // Save the current sample in the ring buffer
    temp_samples[i][temp_sample_pointer] = value;

    // Smooth the temperature sampling over temp_sample_count samples
    double smoothed_value = smoothed_sensor_reading(i);

    // The first two sensors are the water temperature sensors. Any further ones are auxiliary.
    if (i < 2) {
      avg_reading += smoothed_value;
    }

#if defined(OLED_DISPLAY)
    // dtostr(smoothed_value * ADC_DIVISOR, 1, 3);    
    print_oled(i + 1, "%s(%s/%d)"
      , dtostr(adc_to_farenheit(smoothed_value)).c_str()
      , dtostr(adc_to_farenheit(value)).c_str()
      , value
      );
#endif
  }

  // Calculate the average smoothed temp of the first two sensors and save it as the water temp.
  avg_reading /= 2;
  last_temp = adc_to_farenheit(avg_reading);

  // If the smoothed readings from sensors 0 and 1 ever differ by more than panic_sensor_difference degrees, panic.
  if (fabs(adc_to_farenheit(smoothed_sensor_reading(0)) - adc_to_farenheit(smoothed_sensor_reading(1))) > panic_sensor_difference)
  {
    panic("sensor readings diverged (%s vs %s)",
      dtostr(adc_to_farenheit(smoothed_sensor_reading(0))).c_str(),
      dtostr(adc_to_farenheit(smoothed_sensor_reading(1))).c_str());
  }
  
  // If calculated temperature is over our defined limit, panic.
  if (last_temp > panic_high_temp) 
  {
    panic("last_temp too high (%s > %d)", 
      dtostr(last_temp).c_str(),
      panic_high_temp);
  }

  // Reset the watchdog timer.
  watchdog_reset();
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

// Values in digit places:
// 0 - 9 - digit
// 0x0a - blank
// 0x0b -- "P"
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

void display_temperature(int temp)
{
  display_set_digits(temp / 100, (temp / 10) % 10, temp % 10);
}

void display_panic()
{
  display_temperature(adc_to_farenheit(smoothed_sensor_reading(panic_flash?0:1)));
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
    // dtostr(smoothed_value, 1, 0);
    print_oled(7, "VCC = %s", dtostr(vcc, 1, 3).c_str());
  #endif
#endif
}

void display_send()
{
  if (display_dirty)
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
    pump_switch_millis = millis64();
  }
}

void setup() 
{
  watchdog_init();

  serial_debug_init();

  // Make sure the pins shared with hardware SPI via the shield are high-impedance.
#ifdef QUIESCE_PINS
  QUIESCE_PINS 
#endif

  // Communications with the control panel is 2400 baud ttl serial.
  Serial1.begin(2400
#if defined(ARDUINO_ARCH_ESP32)
    // Specify format and pins
    , SERIAL_8N1
    , PANEL_RX
    , PANEL_TX
    , false // invert?
#endif
  );

#ifdef ADC_AREF_OPTION
  // Set the AREF voltage for reading from the sensors.
  analogReference(ADC_AREF_OPTION);
#endif
#ifdef ADC_ATTENUATION_FACTOR
  analogSetAttenuation(ADC_ATTENUATION_FACTOR);
#endif

  pinMode(pin_pump, OUTPUT);
  // Make really sure the pump is not running.
  digitalWrite(pin_pump, 0);
  
  start_oled();

  display_vcc();

  network_start();

  enter_state(runstate_startup);
}

void check_temp_validity()
{
  // Temperature readings are meaningless if the pump isn't running.
  // We also want to ignore temp readings when the pump was just turned on,
  // and consider the temp valid for a short time after it turns off.
  if (pump_running && !temp_valid)
  {
    if (millis64() - pump_switch_millis > temp_settle_millis)
    {
      // The pump has been running long enough for temp to be valid.
      temp_valid = true;
    }
  }
  else if (!pump_running && temp_valid)
  {
    if (millis64() - pump_switch_millis > temp_decay_millis)
    {
        // The pump has been off long enough that we should no longer consider the temp valid.
      temp_valid = false;
    }
  }

  // If the temp is currently valid, update the last valid temp
  if (temp_valid) {
    last_valid_temp = last_temp;
  }
}

// Make this a global so it can be displayed by the debug web endpoint
uint64_t loop_time = 0;

void loop() {
  uint64_t loop_start_micros = micros64();
  uint64_t loop_start_millis = millis64();
  
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
  uint64_t millis_since_last_transition = loop_start_millis - runstate_last_transition_millis;
  uint64_t seconds_since_last_transition = millis_since_last_transition / 1000l;
  uint64_t millis_since_button_change = loop_start_millis - buttons_last_transition_millis;
  uint64_t seconds_since_button_change = millis_since_button_change / 1000l;

  if (temp_adjusted)
  {
    uint64_t millis_since_temp_adjust = millis64() - temp_adjusted_millis;

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
        uint64_t repeat_time = 500;
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
      if (seconds_since_last_transition > startup_wait_seconds)
      {
        // Before starting the pump for the first time, enable the watchdog timer.
        watchdog_start();

        // While we may not have an _actual_ valid temp yet, we should at least have a smoothed version of the current sensor temp.
        // Use this for reporting stats for now, so we don't return garbage.
        last_valid_temp = last_temp;

        // Turn on the pump and enter the "finding temp" state.
        enter_state(runstate_finding_temp);
      } else {
        display_set_digits(0x0a, 0x00, 0x0a);
      }
    break;
    case runstate_finding_temp:
      if (jets_pushed) {
        // From the finding temp state, allow the user to turn off the pump manually.
        enter_state(runstate_idle);
      } else if (temp_valid) {
        // Go into the heating state. It will flip back to idle if no heating is needed.
        enter_state(runstate_heating);
      } else {
        set_pump(true);
        // indicate that we're still finding temp
        display_set_digits(0x0a, 0x0b, 0x0a);
      }
    break;
    case runstate_heating:
      if (jets_pushed) {
        // From the heating state, allow the user to turn off the pump manually.
        enter_state(runstate_idle);
      } else if (last_valid_temp > (temp_setting + temp_setting_range)) {
         // We've reached the set point. Turn the pump off and go to idle.
        enter_state(runstate_idle);
      } else {
        set_pump(true);
        display_heat(true);
      }
    break;
    case runstate_idle:
      if (jets_pushed) {
        // From the idle state, allow the user to turn on the pump manually.
        enter_state(runstate_manual_pump);
      } else if (seconds_since_last_transition > idle_seconds) {
        // We've been idle long enough that we should do a temperature check.
        enter_state(runstate_finding_temp);
      } else if ((temp_adjusted) && ((temp_setting + temp_setting_range) > last_valid_temp)) {
        // The user adjusted the temperature to above the last valid reading.
        // Transition to the finding temperature state to figure out if we need to heat.
        enter_state(runstate_finding_temp);
      } else {
        set_pump(false);
      }
    break;
    case runstate_manual_pump:
      if (jets_pushed) {
        // From the manual state, allow the user to turn off the pump.
        enter_state(runstate_idle);
      } else if (seconds_since_last_transition > manual_pump_seconds) {
        // Don't let the manual mode run for too long.
        enter_state(runstate_finding_temp);
      } else {
        set_pump(true);
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
      if (buttons == (button_jets | button_light)) {
        // User is holding down the button combo. 
        if (seconds_since_button_change >= panic_wait_seconds) {
          // They've successfully escaped.
          enter_state(runstate_startup);
        } else {
          display_panic_countdown(panic_wait_seconds -  seconds_since_button_change);
        }
      } else {
        // Not holding buttons, just flash
        if (millis_since_last_transition >= 500) {
          panic_flash = !panic_flash;
          runstate_transition();
        }
        display_panic();
      }
    break;
  }

  // If the temp was recently adjusted, override other modes and display it.
  if (temp_adjusted) {
    display_temperature(temp_setting);
  }

  // Always update the display of vcc.
  display_vcc();

  // Display the internal CPU temperature.
  #if defined(CPU_TEMP_AVAILABLE)
    print_oled(5, "CPU temp:%s", dtostr(cpu_temp()));
  #endif

  display_send();

  network_service();

  loop_time = micros64() - loop_start_micros;
  if (loop_time < loop_microseconds)
  {
    uint64_t msRemaining = loop_microseconds - loop_time;
    // debug("loop time %ld, start %ld, remaining %ld", loop_time, loop_start_micros, msRemaining);
#if defined(__AVR__)
    // On AVR, delayMicroseconds takes an int (16 bits), so it won't work properly if the delay time is over 65535ms
    if (msRemaining > UINT16_MAX) {
      delay(msRemaining / 1000l);
    } else 
#endif
    {
      delayMicroseconds(msRemaining);
    }
  }
  else
  {
    // debug("loop time %ld", loop_time);
  }
}

#if defined(NETWORK)
  #include <WiFiClient.h>
  #include <WiFi.h>
  #include <ESPmDNS.h>

  #if defined(OTA_UPDATE)
    #include <ArduinoOTA.h>
  #endif

  #if defined(WEBSERVER)
    #include <ESP32WebServer.h>
    ESP32WebServer server(80);

    void webserver_handle_stats() {
      String message;
      // Don't return stats in startup state, since we won't have a full set of temperature samples yet.
      if (runstate != runstate_startup) {
        message += dtostr(last_temp);
        message += "\n";

        message += temp_setting;
        message += "\n";

        message += pump_running?"1":"0";
        message += "\n";

        message += dtostr(last_valid_temp);
        message += "\n";

        if (pin_temp_count > 2)
        {
          // The third temperature sensor is on the shield, use that as the cpu temp.
          message += dtostr(adc_to_farenheit(smoothed_sensor_reading(2)));
          message += "\n";
        }
        #if defined(CPU_TEMP_AVAILABLE)
        else {
          // This is the internal temperature sensor on the CPU.
          message += dtostr(cpu_temp());
          message += "\n";
        }
        #endif

      }
      server.send(200, "text/plain; charset=UTF-8", message);
    }

    void webserver_handle_root() {
      String message;
      message += "<HEAD>";

      // Refresh the page once a minute, to keep temperature updated.
      message += "<meta http-equiv=\"refresh\" content=\"60\">";
      // make this look decent on a phone
      message += "<meta name=\"viewport\" content=\"width=device-width, initial-scale=1\">";

      message += "<style>\n";
      message += "html {display: inline-block; margin: 0px auto; text-align: center;}\n";
      message += ".button { background-color: #7f7f7f; border: none; color: white; padding: 12px 24px; text-decoration: none; font-size: 24px; margin: 2px; cursor: pointer;}\n";
      message += "</style>\n";

      message += "<title>Hot Tub</title>";

      message += "</HEAD>\n<BODY>\n";

      message += "<p>Status: ";
      message += state_name(runstate);
      message += "</p>\n";
      message += "<p>Current Temp: ";
      message +=  dtostr(last_temp);
      message += "</p>\n";
      message += "<p>Temp Setting: ";
      message += temp_setting;
      message += "</p>\n";

      #if defined(WEBSERVER_REMOTE_CONTROL)
        if (temp_setting < temp_max) {
          message += "<p><a href=\"/set?temp=";
          message += temp_setting + 1;
          message += "\"><button class=\"button\">Temp up</button></a></p>\n";
        }
        if (temp_setting > temp_min) {
          message += "<p><a href=\"/set?temp=";
          message += temp_setting - 1;
          message += "\"><button class=\"button\">Temp down</button></a></p>\n";
        }
        switch (runstate) {
          case runstate_idle:
            message += "<p><a href=\"/set?jets=on\"><button class=\"button\">Jets on</button></a></p>\n";
          break;
          case runstate_heating:
          case runstate_finding_temp:
          case runstate_manual_pump:
            message += "<p><a href=\"/set?jets=off\"><button class=\"button\">Jets off</button></a></p>\n";
          break;
          default:
          break;
        }
      #endif

      message += "</BODY>\n";
      server.send(200, "text/html; charset=UTF-8", message);
    }

  #if defined(WEBSERVER_REMOTE_CONTROL)
    void webserver_handle_set() {
      String message;
      // Redirect to the root page when we're done here.
      // message += "<HEAD><meta http-equiv=\"refresh\" content=\"0;url=/\"></HEAD>\n<BODY>\n";
      message += "<HEAD></HEAD>\n<BODY>\n";
      if (server.hasArg("temp")) {
        // Request to set the temperature
        String tempString = server.arg("temp");
        int temp = tempString.toInt();
        // If the conversion fails, temp will be 0. This is handled by the range check below.
        if (tempString.equals("up")) {
          temp_adjust(1);
          message += "Temp incremented to ";
          message += temp_setting;
          message += "<br>\n";
        } else if (tempString.equals("down")) {
          temp_adjust(-1);
          message += "Temp decremented to ";
          message += temp_setting;
          message += "<br>\n";
        } else if ((temp >= temp_min) && (temp <= temp_max)) {
          temp_setting = temp;
          // This handles updating the last-temp-adjust time, etc.
          temp_adjust(0);
          message += "Temp set to ";
          message += temp_setting;
          message += "<br>\n";
        } else {
          message += "Temp arg out of range: ";
          message += tempString;
          message += "<br>\n";
        }
      }
      if (server.hasArg("jets")) {
        String jetsString = server.arg("jets");
        bool accepted = false;
        switch (runstate) {
          case runstate_idle:
            if (jetsString.equals("on") || jetsString.equals("toggle")) {
              accepted = true;
              message += "Turning on manual pump<br>\n";
              enter_state(runstate_manual_pump);
            }
          break;
          case runstate_finding_temp:
          case runstate_manual_pump:
          case runstate_heating:
            if (jetsString.equals("off") || jetsString.equals("toggle")) {
              accepted = true;
              message += "Turning off manual pump<br>\n";
              enter_state(runstate_idle);
            }
          break;
          default:
          break;
        }
        if (!accepted)
        {
          message += "Jets command \"";
          message += jetsString;
          message += "\" ignored in run state ";
          message += state_name(runstate);
          message += "<br>\n";
        }
      }

      message += "</BODY>\n";

      // Redirect to the root page
      server.sendHeader("Location", "/", false);
      server.send(303, "text/html; charset=UTF-8", message);
    }

    void webserver_handle_reboot() {
      String message;
      message += "Rebooting in 5 seconds...\n";
      server.send(200, "text/plain; charset=UTF-8", message);
      delay(5 * 1000);
      ESP.restart();
    }
  #endif

    const char *reset_reason()
    {
      // Hat tip to https://www.robmiles.com/journal/2020/1/20/disabling-the-esp32-brownout-detector for this
      esp_reset_reason_t reset_reason = esp_reset_reason();
      switch (reset_reason)
      {
      case ESP_RST_POWERON:    return "Reset due to power-on event"; break;
      case ESP_RST_EXT:        return "Reset by external pin (not applicable for ESP32)"; break;
      case ESP_RST_SW:         return "Software reset via esp_restart"; break;
      case ESP_RST_PANIC:      return "Software reset due to exception/panic"; break;
      case ESP_RST_INT_WDT:    return "Reset (software or hardware) due to interrupt watchdog"; break;
      case ESP_RST_TASK_WDT:   return "Reset due to task watchdog"; break;
      case ESP_RST_WDT:        return "Reset due to other watchdogs"; break;
      case ESP_RST_DEEPSLEEP:  return "Reset after exiting deep sleep mode"; break;
      case ESP_RST_BROWNOUT:   return "Brownout reset (software or hardware)"; break;
      case ESP_RST_SDIO:       return "Reset over SDIO"; break;
      case ESP_RST_UNKNOWN:    
      default: break;
      }

      return "Reset reason can not be determined";
    }

#if defined(WEBSERVER_DEBUG)
    void webserver_handle_debug() {
      String message;

      int64_t uptime = esp_timer_get_time();
      int years = uptime / year;
      int days = (uptime % year) / day;
      message += "Uptime: ";
      if (years > 0) {
        message += years;
        message += (years != 1)?" years, ":" year, ";
      }
      if (uptime > day) {
        message += days;
        message += (days != 1)?" days, ":" day, ";
      }
      if (uptime > hour) {
        message += int((uptime % day) / hour);
        message += ":";
      }

      {
        // Evidently, the String class has no provision for integer formatting with leading zeroes.
        char buf[32];
        snprintf(buf, sizeof(buf), "%02d:%02d.%03d\n",
          int((uptime % hour) / minute),
          int((uptime % minute) / second),
          int((uptime % second) / millisecond));
        message += buf;
      }

      message += reset_reason();
      message += "\n";

      // Last loop time
      message += "Last loop time: ";
      message += uint32_t(loop_time);
      message += "/";
      message += uint32_t(loop_microseconds);
      message += " microseconds\n";

      message += "WiFI RSSI: ";
      message += WiFi.RSSI();
      message += " dBm\n";

      message += "WiFI Tx Power: ";
      wifi_power_t power = WiFi.getTxPower();
      message += power;
      message += "\n";

      message += "Run state: ";
      message += state_name(runstate);
      message += "\n";

      for (int i = 0; i < pin_temp_count; i++)
      {
        // Replicate the temperature sample reporting that goes on the OLED, 
        // plus min and max values from the smoothing buffer.
        int last_sample = temp_samples[i][temp_sample_pointer];
        int lowest, highest;
        double smoothed = smoothed_sensor_reading(i, &lowest, &highest);
        message += "Sensor ";
        message += i;

        message += ": smoothed = ";
        message += dtostr(adc_to_farenheit(smoothed));
        message += " (";
        message += dtostr(smoothed);

        message += "), cur = ";
        message += dtostr(adc_to_farenheit(last_sample));
        message += " (";
        message += last_sample;
    
        message += "), min = ";
        message += dtostr(adc_to_farenheit(lowest));
        message += " (";
        message += lowest;
    
        message += "), max = ";
        message += dtostr(adc_to_farenheit(highest));
        message += " (";
        message += highest;

        message += ")\n";
      }

      message += "Set temp: ";
      message += temp_setting;
      message += "\n";

      message += "Last temp: ";
      message += dtostr(last_temp);
      message += "\n";

      message += "Last valid temp: ";
      message += dtostr(last_valid_temp);
      message += "\n";

      #if defined(CPU_TEMP_AVAILABLE)
      {
        uint8_t tsens = read_tsens_register();
        message += "CPU temp sensor: ";
        message += dtostr((tsens + TSENS_OFFSET) * TSENS_MULTIPLIER);
        message += " (";
        message += tsens;
        message += ")\n";
      }
      #endif

      // Display panic string
      if (panic_string.length() > 0)
      {
          message += "Panic string: ";
          message += panic_string;
          message += "\n";
      }

      // Display http arguments:
      int arg_count = server.args();
      if (arg_count > 0)
      {
        message += "HTTP arguments:\n";
        for (int i = 0; i < arg_count; i++) {
          message += "   ";
          message += server.argName(i);
          message += ": ";
          message += server.arg(i);
          message += "\n";
        }
      }

      server.send(200, "text/plain; charset=UTF-8", message);
    }
#endif // WEBSERVER_DEBUG

    void webserver_handle_not_found(){
      server.send(404, "text/plain; charset=UTF-8", "404 Not Found");
   }
  #endif // WEBSERVER

void network_start()
{
  print_oled(7, "Starting WiFi");
  debug("Starting WiFi");

  // This doesn't really help, since the mDNS client doesn't appear to publish AAAA records.
  // Maybe later?
  // WiFi.enableIpV6();

  // This sets the DHCP Client ID, and probably other things.
#if defined(MDNS_NAME)
  WiFi.setHostname(MDNS_NAME);
#endif
  
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  // Instead of waiting for wifi to connect and starting services here,
  // check for connection and start them in network_service();
}

String status_string(wl_status_t status)
{
    String result;
    switch(status) {
      // Keep strings under 16 characters, so they don't truncate on the OLED.
      case WL_IDLE_STATUS: result = "WiFi idle"; break;
      case WL_NO_SSID_AVAIL: result = "WiFi no ssid"; break;
      case WL_SCAN_COMPLETED: result = "WiFi scanned"; break;
      case WL_CONNECTED: result = "WiFi connected"; break;
      case WL_CONNECT_FAILED: result = "WiFi conn failed"; break;
      case WL_CONNECTION_LOST: result = "WiFi conn lost"; break;
      case WL_DISCONNECTED: result = "WiFi disconnect"; break;
      default: result = "WiFi unknown"; break;
    }

    return result;
}

void network_service()
{
  // Only check status until the wifi connects the first time.
  static bool services_started = false;
  static wl_status_t last_status = WL_NO_SHIELD;
  wl_status_t status = WiFi.status();
  static int64_t last_status_change_time = 0;
  int64_t now = esp_timer_get_time();
  int64_t interval = now - last_status_change_time;

  if (last_status != status)
  {
    // Status has changed. 
    last_status_change_time = now;

    print_oled(7, status_string(status));
    debug("WiFI status changed from \"%s\" to \"%s\"", status_string(last_status).c_str(), status_string(status).c_str());

    if (status == WL_CONNECTED)
    {
      print_oled(7, WiFi.localIP().toString());
      debug("IP address is %s", WiFi.localIP().toString().c_str());

      if (!services_started)
      {
        // wifi has just connected, and we have not started services yet.

        // Start services.
        services_started = true;

        //////////////////////////
        // mDNS
        #if defined(MDNS_NAME)
          print_oled(6, "Starting MDNS");
          debug("Starting MDNS...");
          const char *hostname = MDNS_NAME;
          if (MDNS.begin(hostname)) {
            print_oled(6, "MDNS Started");
            debug("MDNS Started");
            #if defined(WEBSERVER)
              MDNS.addService("_http", "_tcp", 80);
            #endif
          }
          else {
            print_oled(6, "MDNS failed");
            debug("MDNS failed");
          }
        #endif // MDNS_NAME

        //////////////////////////
        // http server
        #if defined(WEBSERVER)
          server.on("/", webserver_handle_root);
          server.on("/stats", webserver_handle_stats);
          #if defined(WEBSERVER_DEBUG)
            server.on("/debug", webserver_handle_debug);
          #endif
          #if defined(WEBSERVER_REMOTE_CONTROL)
            server.on("/set", webserver_handle_set);
            server.on("/reboot", webserver_handle_reboot);
          #endif

          server.onNotFound(webserver_handle_not_found);

          print_oled(6, "Starting webserver");
          debug("Starting webserver...");
          server.begin();

          print_oled(6, "Webserver started");
          debug("Webserver started");
        #endif // WEBSERVER

        //////////////////////////
        // OTA firmware update
        #if defined(OTA_UPDATE)
          // Port defaults to 3232
          // ArduinoOTA.setPort(3232);

          // Hostname defaults to esp3232-[MAC]
          #ifdef MDNS_NAME
            ArduinoOTA.setHostname(MDNS_NAME);
            ArduinoOTA.setMdnsEnabled(true);
          #endif

          // No authentication by default
          #ifdef OTA_PASSWORD
            ArduinoOTA.setPassword(OTA_PASSWORD);
          #elif defined(OTA_PASSWORD_HASH)
            ArduinoOTA.setPasswordHash(OTA_PASSWORD_HASH);
          #endif

          ArduinoOTA.onStart([]() {
            // We're about to start an update.
            // Turn off the pump.
            set_pump(false);
            
            // Set the display to indicate the update is in progress
            display_set_digits(0x0b, 0x00, 0x00);
            display_send();
            print_oled(0, "Updating...");
            for (int i = 1; i < 8; i++) {
              print_oled(i, "");
            }
          })
          .onProgress([](unsigned int progress, unsigned int total) {
            // Tend the watchdog timer
            watchdog_reset();

            // Set the display to indicate the update is in progress
            static bool lights_toggle = false;
            int percent = (progress / (total / 100));
            lights_toggle = !lights_toggle;
            if (percent < 100) {
              display_temperature(percent);
              display_set_digit(0, 0x0b);
              display_heat(lights_toggle);
              display_filter(!lights_toggle);
            } else {
              display_set_digits(0x0b, 0x0b, 0x0b);
              display_heat(false);
              display_filter(false);
            }
            display_send();
            print_oled(1, "%d%%", percent);
          });

          debug("Starting OTA...");
          ArduinoOTA.begin();
          debug("OTA Started");
        #endif // OTA_UPDATE
      }
    }

    last_status = status;
  }
  else
  {
    // If we're in a disconnected state, we may want to try reconnecting.

    switch(last_status) {
      case WL_CONNECT_FAILED:
      case WL_CONNECTION_LOST:
      case WL_DISCONNECTED:
        if (interval > (second * 15))
        {
          // We've been disconnected long enough, try reconnecting.
          // Consider this a status change, so we don't spam it.
          last_status_change_time = now; 
          print_oled(3, "WiFi reconnect");
          debug("WiFi reconnect");
          WiFi.disconnect();
          // WiFi.reconnect();
          // I've seen reports that reconnect() doesn't work reliably, and using begin() works better.
          // ref: https://github.com/espressif/arduino-esp32/issues/653#issuecomment-572387806
          WiFi.begin();
        }
      break;
      default:
      break;
    }
  }

  // if (status == WL_CONNECTED)
  {
    print_oled(6, "RSSI: %d dBm", WiFi.RSSI());
  }

#if defined(WEBSERVER)
  server.handleClient();
#endif

#if defined(OTA_UPDATE)
  ArduinoOTA.handle();
#endif
}

#else
  // Networking is disabled. These functions are no-ops.
  void network_start() {}
  void network_service() {}
#endif
