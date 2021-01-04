# Feather Radio Projects

The [Adafruit Feather](https://www.adafruit.com/feather) platform is an impressively flexible and extensive collection of development boards ('Feathers') and add-ons ('Wings').  Most of my Feather-based projects have used the [Feather Huzzah](https://www.adafruit.com/product/2821) to take advantage of its onboard WiFi support.  Occasionally, though, I need to transmit data from point to point in an area where there is no WiFi or available local wired network.  Fortunately there are several radio-based Feather boards offering Bluetooth, packet radio, or LoRa communications. 

Generally such projects require a matched pair of radio-based boards -- one as the transmitter and one as the receiver.

## Current Projects
### Remote Weather Station
Years ago I built a home weather station using a collection of temperature, humidity, wind, and rain sensors.  While the sensors are all outside the house the processing (and display) is done indoors where I'm able to take advantage of AC power and WiFi connectivity.  A constraint, though, is that the outdoor environmental readings are taken close to the house, which limits open exposure to airflow and other ambient conditions.

As a follow-on, I built a satellite sensor station that could use similar sensors but rely on a pair of Feather Radio devices to transmit sampled data over a short distance.  I chose the [Feather M0 RFM60HCW Packet Radio](https://www.adafruit.com/product/3176) boards given the simplicity and reasonable operating range.

There are, naturally, two sketches -- one for the transmitter and one for the receiver.  The transmitter sketch has to read from the sensors and average the data, and also manages sleep/wake states for the Feather so it makes efficient use of the solar-powered battery system also designed into the remote weather station.  This lets me position the remote station in a location far enough from the house to provide more representative environmental readings and also where exposure to the Sun is sufficent to keep the batteries charged.

The receiver sketch simply reports received data packets through its serial port, allowing a separate, more sophisticated device (in this case a Raspberry Pi) to manage and log the received data and also upload it to the cloud.
