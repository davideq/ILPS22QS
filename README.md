#ILPS22QS

Arduino library to support the ILPS22QS absolute digital output barometer

## API

This sensor uses I2C or SPI to communicate.
For I2C it is then required to create a TwoWire interface before accessing to the sensors:  

    TwoWire dev_i2c(I2C_SDA, I2C_SCL);  
    dev_i2c.begin();

For SPI it is then required to create a SPI interface before accessing to the sensors:  

    SPIClass dev_spi(SPI_MOSI, SPI_MISO, SPI_SCK);  
    dev_spi.begin();

An instance can be created and enabled when the I2C bus is used following the procedure below:  

   ILPS22QSSensor sensor(&dev_i2c);
    sensor.begin();
    sensor.Enable();

An instance can be created and enabled when the SPI bus is used following the procedure below:  

   ILPS22QSSensor sensor(&dev_spi, CS_PIN);
    sensor.begin();
    sensor.Enable();

The access to the sensor values is done as explained below:  

  Read pressure and temperature.  

    float pressure;
    float temperature;
    sensor.GetPressure(&pressure);  
    sensor.GetTemperature(&temperature);

## Documentation

You can find the source files at  
https://github.com/stm32duino/ILPS22QS

TheILPS22QS datasheet is available at  
https://www.st.com/content/st_com/en/products/mems-and-sensors/pressure-sensors/ilps22qs.html
