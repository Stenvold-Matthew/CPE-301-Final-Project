#include <DHT.h>
#include <LiquidCrystal.h>
#include <uRTCLib.h>
//#include <Stepper.h>

// Define Port B Register Pointers
volatile unsigned char *port_b = (unsigned char *)0x25;
volatile unsigned char *ddr_b = (unsigned char *)0x24;
volatile unsigned char *pin_b = (unsigned char *)0x23;

#define RDA 0x80
#define TBE 0x20

volatile unsigned char *myUCSR0A = (unsigned char *)0x00C0;
volatile unsigned char *myUCSR0B = (unsigned char *)0x00C1;
volatile unsigned char *myUCSR0C = (unsigned char *)0x00C2;
volatile unsigned int *myUBRR0 = (unsigned int *)0x00C4;
volatile unsigned char *myUDR0 = (unsigned char *)0x00C6;

volatile unsigned char *my_ADMUX = (unsigned char *)0x7C;
volatile unsigned char *my_ADCSRB = (unsigned char *)0x7B;
volatile unsigned char *my_ADCSRA = (unsigned char *)0x7A;
volatile unsigned int *my_ADC_DATA = (unsigned int *)0x78;

#define DHTPIN 9
#define DHTTYPE DHT11
DHT dht(DHTPIN, DHTTYPE);

LiquidCrystal lcd(7, 6, 5, 4, 3, 2);

uRTCLib rtc(0x68);
char daysOfTheWeek[7][12] = {"Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday"};

//const int stepsPerRevolution = 2038;
//Stepper myStepper = Stepper(stepsPerRevolution, 30, 26, 28, 22);

#define DISABLED 1
#define IDLE 2
#define ERROR 3
#define RUNNING 4

int systemMode = DISABLED;
bool systemOn = 0;
int globalTimer;
bool initialRun = 1;
float temp = 0;

void setup() {
  // setup the UART
  U0init(9600);
  // setup the GPIO
  setOutput(1); // For fan
  setInput(2);  // Reset Button
  setInput(3);  // Start/Stop Button
  setOutput(4);
  setOutput(5);
  setOutput(6);
  setOutput(7);
  // setup the ADC
  adc_init();

  // Other setup
  dht.begin();
  lcd.begin(16, 2);
  URTCLIB_WIRE.begin();
  //rtc.set(0, 10, 20, 3, 9, 5, 23);
  Serial.print("Turned on at: ");
  RTCprint();
}

void loop() {
  if (systemMode == DISABLED) {
    writePB(1, 0);
    writePB(4, 1);
    writePB(5, 0);
    writePB(6, 0);
    writePB(7, 0);
    if (readPB(3) == 1) {
      systemMode = IDLE;
      while (readPB(3) == 1) {}  //Delays changing until button is released
      Serial.print("Switched to Idle at: ");
      RTCprint();
    }
    //myStepper.setSpeed(0);
    lcd.setCursor(0, 0);
    lcd.print("                ");
    lcd.setCursor(0, 1);
    lcd.print("                ");
    initialRun = 1;

  } 
  
  else if (systemMode == IDLE) {
    writePB(1, 0);
    writePB(4, 0);
    writePB(5, 1);
    writePB(6, 0);
    writePB(7, 0);
    if (readPB(3) == 1) {
      systemMode = DISABLED;
      while (readPB(3) == 1) {}  //Delays changing until button is released
      Serial.print("Switched to Disabled at: ");
      RTCprint();
    }
    lcdTempAndHumidity();
    //myStepper.setSpeed(0);
    temp = dht.readTemperature(true);
    if (temp > 75) { // Changes to running at 75 degrees
      systemMode = RUNNING;
      Serial.print("Switched to Running at: ");
      RTCprint();
    }
    if(readWaterLevel() < 200) {
      systemMode = ERROR;
      Serial.print("Switched to Error at: ");
      RTCprint();
    }
  }

  else if (systemMode == ERROR) {
    writePB(1, 0);
    writePB(4, 0);
    writePB(5, 0);
    writePB(6, 1);
    writePB(7, 0);
    if (readPB(3) == 1) {
      systemMode = DISABLED;
      while (readPB(3) == 1) {}  //Delays changing until button is released
      Serial.print("Switched to Disabled at: ");
      RTCprint();
    }
    //myStepper.setSpeed(0);
    lcd.setCursor(0, 0);
    lcd.print("Error: Water lev");
    lcd.setCursor(0, 1);
    lcd.print("el is too low   ");
    initialRun = 1;
    if (readPB(2) == 1) { // If reset button is pushed and water level is above threshold
      if(readWaterLevel() > 200) {
        systemMode = IDLE;
        Serial.print("Switched to Idle at: ");
        RTCprint();
      }
    }
  } 

  else if (systemMode == RUNNING) {
    writePB(1, 1);
    writePB(4, 0);
    writePB(5, 0);
    writePB(6, 0);
    writePB(7, 1);
    if (readPB(3) == 1) {
      systemMode = DISABLED;
      while (readPB(3) == 1) {}  //Delays changing until button is released
      Serial.print("Switched to Disabled at: ");
      RTCprint();
    }
    lcdTempAndHumidity();
    //myStepper.setSpeed(5);
	  //myStepper.step(stepsPerRevolution);
    if (temp < 75) { // Changes to running at 75 degrees
      systemMode = IDLE;
      Serial.print("Switched to idle at: ");
      RTCprint();
    }
    if(readWaterLevel() < 200) {
      systemMode = ERROR;
      Serial.print("Switched to Error at: ");
      RTCprint();
    }
  }
}

//This is a function used to get the water level
int readWaterLevel() {
  int val = adc_read(0);
  return val;
}

void RTCprint() {
  rtc.refresh();

  Serial.print(rtc.year());
	Serial.print('/');
	Serial.print(rtc.month());
	Serial.print('/');
	Serial.print(rtc.day());

	Serial.print(' ');

	Serial.print(rtc.hour());
	Serial.print(':');
	Serial.print(rtc.minute());
	Serial.print(':');
	Serial.print(rtc.second());

	Serial.print(" (");
  Serial.print(daysOfTheWeek[rtc.dayOfWeek()-1]);
  Serial.print(") ");

	Serial.println();
}

void lcdTempAndHumidity() {
  
  if(millis() > globalTimer + 60000 || initialRun) { 
    // Updates once every minute or when it is run for the first time after 
    // switching off of disabled
    lcd.setCursor(0, 0);
    lcd.print("Temp: ");
    lcd.print(dht.readTemperature(true));
    lcd.print((char)223);
    lcd.print("F   ");
    lcd.setCursor(0, 1);
    lcd.print("Humidity: ");
    lcd.print(dht.readHumidity());
    lcd.print("%");
    globalTimer = millis();
    initialRun = 0;
  }
}

void setOutput(unsigned char pin_num) {
  *ddr_b |= 0x01 << pin_num;
}

void setInput(unsigned char pin_num) {
  *ddr_b &= ~(0x01 << pin_num);
}

int readPB(unsigned char pin_num) {
  if (*pin_b & (0x01 << pin_num)) {
    return 1;
  }
  // if the pin is low
  else {
    return 0;
  }
}

void writePB(unsigned char pin_num, unsigned char state) {
  if (state == 0) {
    *port_b &= ~(0x01 << pin_num);
  } else {
    *port_b |= 0x01 << pin_num;
  }
}

void adc_init() {
  // setup the A register
  *my_ADCSRA |= 0b10000000;  // set bit   7 to 1 to enable the ADC
  *my_ADCSRA &= 0b11011111;  // clear bit 6 to 0 to disable the ADC trigger mode
  *my_ADCSRA &= 0b11110111;  // clear bit 5 to 0 to disable the ADC interrupt
  *my_ADCSRA &= 0b11111000;  // clear bit 0-2 to 0 to set prescaler selection to slow reading
  // setup the B register
  *my_ADCSRB &= 0b11110111;  // clear bit 3 to 0 to reset the channel and gain bits
  *my_ADCSRB &= 0b11111000;  // clear bit 2-0 to 0 to set free running mode
  // setup the MUX Register
  *my_ADMUX &= 0b01111111;  // clear bit 7 to 0 for AVCC analog reference
  *my_ADMUX |= 0b01000000;  // set bit   6 to 1 for AVCC analog reference
  *my_ADMUX &= 0b11011111;  // clear bit 5 to 0 for right adjust result
  *my_ADMUX &= 0b11100000;  // clear bit 4-0 to 0 to reset the channel and gain bits
}

unsigned int adc_read(unsigned char adc_channel_num) {
  // clear the channel selection bits (MUX 4:0)
  *my_ADMUX &= 0b11100000;
  // clear the channel selection bits (MUX 5)
  *my_ADCSRB &= 0b11110111;
  // set the channel number
  if (adc_channel_num > 7) {
    // set the channel selection bits, but remove the most significant bit (bit 3)
    adc_channel_num -= 8;
    // set MUX bit 5
    *my_ADCSRB |= 0b00001000;
  }
  // set the channel selection bits
  *my_ADMUX += adc_channel_num;
  // set bit 6 of ADCSRA to 1 to start a conversion
  *my_ADCSRA |= 0x40;
  // wait for the conversion to complete
  while ((*my_ADCSRA & 0x40) != 0)
    ;
  // return the result in the ADC data register
  return *my_ADC_DATA;
}

void U0init(int U0baud) {
  unsigned long FCPU = 16000000;
  unsigned int tbaud;
  tbaud = (FCPU / 16 / U0baud - 1);
  // Same as (FCPU / (16 * U0baud)) - 1;
  *myUCSR0A = 0x20;
  *myUCSR0B = 0x18;
  *myUCSR0C = 0x06;
  *myUBRR0 = tbaud;
}
