# IIS2DULPX
Arduino library to support the IIS2DULPX 3D sensor

## API

This sensor uses I2C or SPI to communicate.
For I2C it is then required to create a TwoWire interface before accessing to the sensors:  

    TwoWire dev_i2c(I2C_SDA, I2C_SCL);  
    dev_i2c.begin();

For SPI it is then required to create a SPI interface before accessing to the sensors:  

    SPIClass dev_spi(SPI_MOSI, SPI_MISO, SPI_SCK);  
    dev_spi.begin();

An instance can be created and enabled when the I2C bus is used following the procedure below:  

    IIS2DULPXSensor sensor(&dev_i2c);
    sensor.begin();
    sensor.Enable_X();

An instance can be created and enabled when the SPI bus is used following the procedure below:  

    IIS2DULPXSensor sensor(&dev_spi, CS_PIN);  
    sensor.begin();
    sensor.Enable_X();

The access to the sensor values is done as explained below:  

  Read sensor.  

    int32_t sensor[3];
    sensor.Get_X_Axes(sensor);  

## Examples

* IIS2DULPX_DataLog_Terminal: This application shows how to get data from IIS2DULPX sensor and print them on terminal.

* IIS2DULPX_6D_Orientation: This application shows how to use IIS2DULPX sensor to find out the 6D orientation and display data on a hyperterminal.

* IIS2DULPX_Wake_Up_Detection: This application shows how to detect the wake-up event using the IIS2DULPX sensor.

## Documentation

You can find the source files at  
https://github.com/stm32duino/IIS2DULPX

The IIS2DULPX datasheet is available at  
https://www.st.com/en/mems-and-sensors/iis2dulpx.html
