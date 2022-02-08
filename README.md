# WWF ARGOS Tracker

Current Status:

- Device only seems to work when compiled under DEBUG mode in the Makefile.  Core issue is related to the interface between the nRF52 and ARTIC chip.


## Documentation

Doxygen is used to build the source code documentation. Go to the [downloads page](http://www.doxygen.nl/download.html), scroll down to **Sources and Binaries** and select the latest archive for Linux, Windows, or Mac.

Or if on Linux, just do:

```
sudo apt install doxygen
```

To build the documentation:

```
cd _docs/
doxygen
```

Then use your browser to open `_docs/html/index.html`.




### Usage Guide
# main.c
This is the starting/jump off point for the code. All major threads are started/initialized here.  
You can control the sequence of thread start-up or enable/disable threads via 
```
project_settings.h
```

# Logging/CLI
These are actually two different components.  
CLI is a command line interface which allows for manual
control.  This can be used for debugging and/or user interface.  You can send Text to the terminal
through API called seen in:
```
mw_cli_thread.h
```

Logging is primary for debugging code during development. A wrapper is created around the Nordic SDK
Logging module.  it must be enabled in the SDK config file, where you can also specify either a UART 
or RTT (J-Link) physical interface. This should be a different interface than CLI
```
sdk_config.h

#define NRF_LOG_ENABLED 1

#define NRF_LOG_BACKEND_UART_ENABLED (0:1)

#define NRF_LOG_BACKEND_RTT_ENABLED (0:1)
```

# ADC
The ADC thread uses the Nordic SAADC peripheral driver and is provided as an example of ADC usage.
SAADC must be enabled in the SDK Config file
```
sdk_config.h

#define NRFX_SAADC_ENABLED 1
```

## Contributors

- Kevin Lockwood
- Sean Edmond
- Justin Lam