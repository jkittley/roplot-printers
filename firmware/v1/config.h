// PRINTER SETTINGS -  All messurements in mm
// ----------------------------------------------------------------------------------------------
#define boomRadius     510   
#define boomWidth      10
#define boomColor      "eeeeee"
#define drawStart      100
#define drawEnd        450
#define carWidth       20
#define carHeight      20
#define boomStep       0.9
#define carStep        0.9

#define NORTH_POLE     1
#define SOUTH_POLE     1

typedef struct {
 int id;
 int pole;
 char color[6];
 int width;
 int offset_x;
 int offset_y;
} pen;

pen pens[2] = { 
  {1, NORTH_POLE, '990066', 5, 20, 0},
  {1, SOUTH_POLE, '009900', 5, 20, 0}
};

// COMMON SETTINGS
// ----------------------------------------------------------------------------------------------
#define BUFSIZE                        128   // Size of the read buffer for incoming data
#define FACTORYRESET_ENABLE            1
#define MINIMUM_FIRMWARE_VERSION      "0.6.6"
#define MODE_LED_BEHAVIOUR            "MODE"

// SOFTWARE UART SETTINGS
// ----------------------------------------------------------------------------------------------
// The following macros declare the pins that will be used for 'SW' serial.
// You should use this option if you are connecting the UART Friend to an UNO
// ----------------------------------------------------------------------------------------------
#define BLUEFRUIT_SWUART_RXD_PIN       9    // Required for software serial!
#define BLUEFRUIT_SWUART_TXD_PIN       10   // Required for software serial!
#define BLUEFRUIT_UART_CTS_PIN         11   // Required for software serial!
#define BLUEFRUIT_UART_RTS_PIN         -1   // Optional, set to -1 if unused

// SHARED UART SETTINGS
// ----------------------------------------------------------------------------------------------
// The following sets the optional Mode pin, its recommended but not required
// ----------------------------------------------------------------------------------------------
#define BLUEFRUIT_UART_MODE_PIN        12    // Set to -1 if unused


// SHARED SPI SETTINGS
// ----------------------------------------------------------------------------------------------
// The following macros declare the pins to use for HW and SW SPI communication.
// SCK, MISO and MOSI should be connected to the HW SPI pins on the Uno when
// using HW SPI.  This should be used with nRF51822 based Bluefruit LE modules
// that use SPI (Bluefruit LE SPI Friend).
// ----------------------------------------------------------------------------------------------
#define BLUEFRUIT_SPI_CS               8
#define BLUEFRUIT_SPI_IRQ              7
#define BLUEFRUIT_SPI_RST              4    // Optional but recommended, set to -1 if unused

// SOFTWARE SPI SETTINGS
// ----------------------------------------------------------------------------------------------
// The following macros declare the pins to use for SW SPI communication.
// This should be used with nRF51822 based Bluefruit LE modules that use SPI
// (Bluefruit LE SPI Friend).
// ----------------------------------------------------------------------------------------------
#define BLUEFRUIT_SPI_SCK              13
#define BLUEFRUIT_SPI_MISO             12
#define BLUEFRUIT_SPI_MOSI             11

// SYSTEM STATES
// ----------------------------------------------------------------------------------------------
#define STATE_WAITING_BLE               1  
#define STATE_SETUP_STARTED             2
#define STATE_RX_BLE                    3
#define STATE_TX_BLE                    4
#define STATE_PROCESSING_CMD            5

// SYSTEM LED
#define  STATE_LED_R = 11;
#define  STATE_LED_G = 10;
#define  STATE_LED_B = 9;
