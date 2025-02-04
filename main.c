#include <16F877A.h>
#device ADC=10

#FUSES NOWDT                    //No Watch Dog Timer
#FUSES PUT                      //Power Up Timer
#FUSES NOBROWNOUT               //No brownout reset
#FUSES NOLVP                    //No low voltage prgming, B3(PIC16) or B5(PIC18) used for I/O

#use delay(crystal=4MHz)

#define LCD_ENABLE_PIN PIN_D0
#define LCD_RS_PIN PIN_D1
#define LCD_RW_PIN PIN_D2
#define LCD_DATA4 PIN_D4
#define LCD_DATA5 PIN_D5
#define LCD_DATA6 PIN_D6
#define LCD_DATA7 PIN_D7

#include <lcd.c>

// Define DHT22 pin
#define DHT22_PIN PIN_B0

// Function prototypes
void start_signal();
int8 read_data();
void read_dht22(float *temperature, float *humidity);
int wait_for_response(int timeout_us);

void main()
{
   setup_adc_ports(AN0_AN1_AN3);  // Configure ADC for AN0, AN1, and AN3
   setup_adc(ADC_CLOCK_INTERNAL); // Set ADC clock to internal
   float temperature = 0, humidity = 0;

   lcd_init();  
   output_low(PIN_B4);
   output_low(PIN_B1);
   output_low(PIN_B2);

   while(TRUE)
   {
      int16 sm, ldr;
     
      // Reading soil moisture from AN0 (A0)
      set_adc_channel(0);
      delay_us(10);  // Small delay for channel switching
      sm = read_adc();  // Read ADC value from channel 0
      if(sm > 900)
      {
         output_high(PIN_B4);  // Turn ON pump or related device
      }
      else 
      {
         output_low(PIN_B4);  // Turn OFF pump or related device
      }
      printf(lcd_putc, "\fMoisture = %ld", sm);  // Display soil moisture on LCD
      delay_ms(500);

      // Reading LDR value from AN1 (A1)
      set_adc_channel(1);
      delay_us(10);  // Small delay for channel switching
      ldr = read_adc();  // Read ADC value from channel 1
      if (ldr < 650)  // Light threshold conditions
      {
         output_high(PIN_B1);  // Turn ON light or related device
      }
      else
      {
         output_low(PIN_B1);  // Turn OFF light or related device
      }
      printf(lcd_putc, "\nLight = %ld", ldr);  // Display LDR value on LCD
      delay_ms(1000);

      // Read data from DHT22 sensor (Temperature and Humidity)
      read_dht22(&temperature, &humidity);

      // Display temperature and humidity on LCD
      lcd_gotoxy(1, 1);
      printf(lcd_putc, "\fTemp: %2.1f C", temperature);
      lcd_gotoxy(1, 2);
      printf(lcd_putc, "Hum: %2.1f %%", humidity);
      delay_ms(1000);

      // Control based on humidity and temperature thresholds
      if (humidity < 20 || temperature > 30)
      {
         output_high(PIN_B2);  // Turn ON fan or cooler
      }
      else
      {
         output_low(PIN_B2);   // Turn OFF fan or cooler
      }

      delay_ms(2000);  // Delay before the next loop
   }
}

// Function to send start signal to DHT22 sensor
void start_signal() {
   output_low(DHT22_PIN);  // Pull the data pin low
   delay_ms(20);           // Wait for at least 20 ms
   output_high(DHT22_PIN); // Pull the pin high for 30-40 microseconds
   delay_us(40);           
   output_float(DHT22_PIN); // Release the pin and wait for DHT22 response
}

// Function to wait for DHT22 response with a timeout
int wait_for_response(int timeout_us) {
   int count = 0;
   while(!input(DHT22_PIN)) {  // Wait for the pin to go high
      if (count++ > timeout_us) return 0;  // Timeout occurred
      delay_us(1);
   }
   while(input(DHT22_PIN));  // Wait for the pin to go low
   return 1;  // Response detected
}

// Function to read 8 bits of data from DHT22 sensor
int8 read_data() {
   int8 i, data = 0;
   for(i = 0; i < 8; i++) {
      while(!input(DHT22_PIN));  // Wait for pin to go high
      delay_us(30);              // Wait 30 microseconds
      if(input(DHT22_PIN)) {
         data |= (1 << (7 - i));  // Write 1 if pin is high
         while(input(DHT22_PIN)); // Wait for the pin to go low
      }
   }
   return data;
}

// Function to read temperature and humidity from DHT22 sensor
void read_dht22(float *temperature, float *humidity) {
   int8 data[5];  // Array to store 5 bytes of data
   int16 raw_humidity, raw_temperature;

   start_signal();  // Send the start signal to DHT22

   if (!wait_for_response(1000)) {  // Wait for response with timeout
      *temperature = 0;
      *humidity = 0;
      return;
   }

   // Read 5 bytes (40 bits) of data
   for(int i = 0; i < 5; i++) {
      data[i] = read_data();
   }

   // Calculate raw humidity and temperature values
   raw_humidity = (data[0] << 8) | data[1];       // Combine high and low byte for humidity
   raw_temperature = (data[2] << 8) | data[3];    // Combine high and low byte for temperature
   
   
   *humidity = raw_humidity/10.0;// Humidity in percentage
   *temperature = raw_temperature/10.0;  // Temperature in Celsius

   // Check if the temperature is negative (DHT22 can return negative temperatures)
   if(data[2] & 0x80) {
      *temperature = -(*temperature);  // Apply negative sign to temperature
   }

   // Check checksum to ensure data integrity
   int8 checksum = data[0] + data[1] + data[2] + data[3];
   if (checksum != data[4]) {
      *temperature = 0;
      *humidity = 0;
   }
}
