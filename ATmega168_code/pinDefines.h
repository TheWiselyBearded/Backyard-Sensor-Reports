// ---------------
//   Pin Defines
// ---------------


// PB1 is digital laser detector.
#define LASER_DETECTOR PB0

// Soil Moisture Sensors
#define SOIL_SENSOR_ANLG_1 PC0
//#define SOIL_POWER_1 PB1
#define SOIL_SENSOR_ANLG_2 PC1

/** ADXL345 sensor */
#define ADXL345_ADDRESS_W       0xA6
#define ADXL345_ADDRESS_R       0xA7
#define ADXL345_DATA_FORMAT     0x31
#define ADXL345_POWER_CTL       0x2D
#define ADXL345_X0              0x32
#define ADXL345_X1              0x33
#define ADXL345_Y0              0x34
#define ADXL345_Y1              0x35
#define ADXL345_Z0              0x36
#define ADXL345_Z1              0x37

/** TSL2561 LUMINOSITY SENSOR */
#define TSL2561_I2C_ADDR_DEFAULT 0x39
#define TSL2561_I2C_W 0x72
#define TSL2561_I2C_R 0x73

#define TSL2561_INTEGRATION_TIME_13MS 0x00
#define TSL2561_INTEGRATION_TIME_101MS 0x01
#define TSL2561_INTEGRATION_TIME_402MS 0x02
#define TSL2561_GAIN_0X 0x00
#define TSL2561_GAIN_16X 0x10
#define TSL2561_REG_CTRL 0x00
#define TSL2561_REG_TIMING 0x01
#define TSL2561_REG_CH0_LOW 0x0C
#define TSL2561_REG_CH0_HIGH 0x0D
#define TSL2561_REG_CH1_LOW 0x0E
#define TSL2561_REG_CH1_HIGH 0x0F
#define TSL2561_CMD_BIT (0x80)
#define TSL2561_WORD_BIT (0x20)
#define TSL2561_CTRL_PWR_ON 0x03
#define TSL2561_CTRL_PWR_OFF 0x00

/** HTU21D sensor */
#define HTU21D_ADDRESS 0x40
#define HTU21D_ADDRESS_W 0x80           // appended 1 for write command
#define HTU21D_ADDRESS_R 0x81           // appended 0 for read command.
#define HTU21D_WRITE_USER_REG 0xE6      // Write command
#define HTU21D_READ_USER_REG 0xE7       // Read command.
#define SOFT_RST 0xFE                   // Perform soft reset.

#define HTU21D_TEMP_CMD 0xE3
#define HTU21D_HUMID_CMD 0xE5

#define HTU21D_TRIG_TEMP_MEAS 0xF3      // Read measurements for temp, no hold.
#define HTU21D_TRIG_HUM_MEAS 0xF5       // Read measurements for humidity, no hold.
#define HTU21D_RES_RH12_TEMP14 0x00     //RH: 12Bit, Temperature: 14Bit (by default)
#define HTU21D_RES_RH8_TEMP12  0x01   //RH: 8Bit,  Temperature: 12Bit
#define HTU21D_RES_RH10_TEMP13 0x80   //RH: 10Bit, Temperature: 13Bit
#define HTU21D_RES_RH11_TEMP11 0x81

/** DEFAULT AVRDUDE */
#define LED_PORT                PORTB
#define LED_PIN                 PINB
#define LED_DDR                 DDRB

#define LED0                    PB0
#define LED1                    PB1
#define LED2                    PB2
#define LED3                    PB3
#define LED4                    PB4
#define LED5                    PB5
#define LED6                    PB6
#define LED7                    PB7


#define BUTTON_PORT             PORTD
#define BUTTON_PIN              PIND
#define BUTTON_DDR              DDRD

#define BUTTON                  PD2
#define BUTTON2                 PD3
#define BUTTON3                 PD4

#define SPEAKER                 PD6                            /* OC0A */
#define SPEAKER_PORT            PORTD
#define SPEAKER_PIN             PIND
#define SPEAKER_DDR             DDRD

#define ANTENNA                 PD5                            /* OC0B */
#define ANTENNA_PORT            PORTD
#define ANTENNA_PIN             PIND
#define ANTENNA_DDR             DDRD

#define MODULATION              PD3                            /* OC2B */
#define MODULATION_PORT         PORTD
#define MODULATION_PIN          PIND
#define MODULATION_DDR          DDRD

#define LIGHT_SENSOR            PC0                            /* ADC0 */
#define LIGHT_SENSOR_PORT       PORTC
#define LIGHT_SENSOR_PIN        PINC
#define LIGHT_SENSOR_DDR        DDRC

#define CAP_SENSOR              PC1                            /* ADC1 */
#define CAP_SENSOR_PORT         PORTC
#define CAP_SENSOR_PIN          PINC
#define CAP_SENSOR_DDR          DDRC

#define PIEZO                   PC2                            /* ADC2 */
#define PIEZO_PORT              PORTC
#define PIEZO_PIN               PINC
#define PIEZO_DDR               DDRC

#define POT                     PC3                            /* ADC3 */
#define POT_PORT                PORTC
#define POT_PIN                 PINC
#define POT_DDR                 DDRC

//  SPI and I2C serial mode defines

#define SPI_SS                     PB2
#define SPI_SS_PORT                PORTB
#define SPI_SS_PIN                 PINB
#define SPI_SS_DDR                 DDRB

#define SPI_MOSI                     PB3
#define SPI_MOSI_PORT                PORTB
#define SPI_MOSI_PIN                 PINB
#define SPI_MOSI_DDR                 DDRB

#define SPI_MISO                     PB4
#define SPI_MISO_PORT                PORTB
#define SPI_MISO_PIN                 PINB
#define SPI_MISO_DDR                 DDRB

#define SPI_SCK                     PB5
#define SPI_SCK_PORT                PORTB
#define SPI_SCK_PIN                 PINB
#define SPI_SCK_DDR                 DDRB

#define I2C_SDA                     PC4
#define I2C_SDA_PORT                PORTC
#define I2C_SDA_PIN                 PINC
#define I2C_SDA_DDR                 DDRC

#define I2C_SCL                     PC5
#define I2C_SCL_PORT                PORTC
#define I2C_SCL_PIN                 PINC
#define I2C_SCL_DDR                 DDRC
