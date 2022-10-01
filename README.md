# Replacement controller for my Softub

This code is intended to interface with the existing Softub control panel and temperature sensors, and behave similarly to the original Softub controller.
It should be able to directly replace the existing control board with an Arduino and a relay with the appropriate rating.

Details about the hardware and connections are [here](./hardware/README.md).

## CAUTION

> **⚠ Warning** 
> This project deals with high voltages and substantial power draw. You can shock yourself. Things can get hot. If you are not confident in your ability to deal with 120v AC wiring that will be carrying a ~12 amp continuous draw, I would recommend sticking to [something](https://github.com/monroewilliams/TinyKeyboard) [safer](https://github.com/monroewilliams/trackball).

When I did my initial build, I re-used the existing power cable that had run from the motor to the control board. It turned out that the cable had enough corrosion inside it that it heated up and eventually melted the wire connectors inside the high-voltage enclosure, causing a short. Fortunately for me, the enclosure contained the 
resulting fire and it just ended up [destroying](./hardware/yikes.jpg) the relay unit and wiring inside that box. 

I've since rebuilt it with new wiring and it's up and running again, but if it hadn't been in a proper high-voltage junction box things could have gotten ugly.

## Current state of the project:

I've reverse-engineered the protocol the original box uses to talk to the panel. (It's a pretty straightfoward binary protocol over TTL Serial at 2400 baud.) For anyone interested in the reverse-engineering process I went through to figure out the protocol, I've documented it [here](./hardware/reverse-engineering.md).

The plug pinout for the panel is:

1. Yellow - Gnd
2. Green - TX (controller to panel)
3. Red - RX (panel to controller)
4. Black - ???
5. (no connection)
6. Blue - +5v

The temperature sensors seem to be a linear analog sensor, probably an LM34. Supply +5v/ground on the red and blue wires, and read a voltage proportional to temperature back on the green wire. The voltage read times 100 gives the temperature in Farenheit. 

At one point I thought there was some non-linearity in the sensors and I implemented a table to correct for it, but it turned out that I hadn't set up the ADC to measure the sensor voltages accurately. After fixing that, I found that they worked just fine, so the correction table has been removed.

As a follow-on to the original project, I also added a third, surface-mount LM34 to the carrier board, to allow me to accurately monitor the temperature inside the control board enclosure. It turns out that it gets surprisingly warm in there (up to 170 degrees Farenheit) when the motor runs continuously, but all of the components seem to be able to tolerate that temperature, so it works out okay.

The control logic approximates the behavior of the original controller, with a couple of minor tweaks. Holding the temperature up/down buttons will adjust the temperature quickly. There's a "panic" mode if the two sensors go out of agreement or the temperature reading hits 5 degrees above the max allowed setpoint (currently 110 + 5 or 115 degrees), which shuts off the motor and flashes the two temperature readings on the display panel. The user can clear panic mode by holding the "light" and "jets" buttons together for 5 seconds (although if the panic condition is still present it will immediately reenter the panic state).

On ESP32, the code can optionally (dependent on `WEBSERVER` being defined) fire up a web server that has several endpoints:
- `/` -- Human-friendly status page with buttons for temperature up/down and turning the pump on/off manually (buttons are conditional on `WEBSERVER_REMOTE_CONTROL` being defined)
- `/set` -- Endpoint used to change the temp setting and pump status (conditional on `WEBSERVER_REMOTE_CONTROL` being defined)
- `/stats` -- Machine-friendly status page that reports current temp, set temp, whether the pump is running, and the last known valid water temp (I use this to build graphs with the scripts shown [here](./graphs/README.md))
- `/debug` -- Internal data useful for debugging the code (dependent on `WEBSERVER_DEBUG` being defined)

I've also included an option for using OTA update to update the sketch over the network, since pulling the board out of the pump pod to tweak settings is annoying. This support is conditional on `OTA_UPDATE` being defined.

The SSID/password for the WiFI connection and the OTA update authentication credentials can be set either inline in platformio.ini, or in a separate credentials.ini file (which is in .gitignore to prevent accidentally including them in git commits). See the `[credentials]` section in [platformio.ini](./platformio.ini) for more details on how this works.

