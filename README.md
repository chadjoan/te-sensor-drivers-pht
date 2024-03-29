# C Driver for TE Connectivity PHT sensors

C driver for sensors made by TE Connectivity, including the
[MS8607 sensor](http://www.te.com/usa-en/product-CAT-BLPS0018.html) and the
[MS5840 sensor](https://www.te.com/usa-en/product-20000980-00.html).

![ms8607](http://www.te.com/content/dam/te-com/catalog/part/CAT/BLP/S00/CAT-BLPS0018-t1.jpg/jcr:content/renditions/product-details.png)

This set of drivers is based on the [MS8607 sensor driver](https://github.com/TEConnectivity/MS8607_Generic_C_Driver)
originally provided by TE Connectivity, but it has undergone modifications
(ex: removing global state, using callbacks for host integration, support
MS5840 sensor), so it isn't an official driver (or drivers).

MS8607 sensor is a self-contained pressure, humidity and temperature sensor that is fully calibrated during manufacture. The sensor can operate from 1.5V to 3.6V. The MS8607 is ideal for weather station applications embedded into compact devices and any applications in which pressure, humidity and temperature monitoring is required.

### Specifications
*	Operating pressure range: 300 to 1200 mbar
*	Measures relative humidity from 0% to 100%
*	Measures temperature from -40°C to 125°C
*	Extended pressure range 10 to 2000 mbar
*	Fast response time
*	I2C communication
*	Very low power consumption


### Driver features
* Connection test
* Reset
* Select I2C master mode
* Aquisition resolution management
* Built-in heater management
* Check battery status
* Read serial number
* Temperature, Humidty and pressure measurement
* Calculate compensated humidity
* Calculate dew point


**NB:** This driver is intended to provide an implementation example of the sensor communication protocol, in order to be usable you have to implement a proper I2C layer for your target platform.
