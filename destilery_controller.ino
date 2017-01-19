#include <Wire.h>

// include the library code:
#include <LiquidCrystal.h>
#include <OneWire.h>

OneWire  ds(10);  // on pin 10

// initialize the library with the numbers of the interface pins
LiquidCrystal lcd(12, 11, 5, 4, 3, 2);

// TMP36 Pin Variables
int sensorPin = 0; //the analog pin the TMP36's Vout (sense) pin is connected to
                        //the resolution is 10 mV / degree centigrade with a
                        //500 mV offset to allow for negative temperatures

int pumpPin = 7;

void setup() {

  // Set serial port
  Serial.begin(9600);
  
  // set up the LCD's number of columns and rows:
  lcd.begin(16, 2);
  // Print a message to the LCD.
//  lcd.print("Temperatures");
  lcd.clear();

  Set pump pin.  
  pinMode(pumpPin, OUTPUT);
}

void loop() {

  int tmpTemp = getTemperatureFromTMP36(sensorPin);
  tmpTemp = getTemperatureFromDS18B20(ds); 
 
  setPump(tmpTemp, 78.0);

  // Display value
  //print(1, String(millis() / 1000));
  print(0, String("Degre:" + String(tmpTemp))); 
  
  delay(500);
}

float getTemperatureFromTMP36(int pin) {
    // getting the voltage reading from the temperature sensor
  int tmpValue = analogRead(pin);
    
  float tmpVoltage = getVoltage(tmpValue);
  float tmpTemp = getTemperature(tmpVoltage);
  
    //if (Serial.available() > 0) {
    //Serial.println("Volts: " + String(tmpValue) + " mV");
    Serial.println("Tempe: " + String(tmpTemp) + " C\t"+"Volts: " + String(tmpValue) + " mV");
  //}
  
  return tmpTemp;
}

void setPump(float temperature, float limit) {
   if(temperature < limit) {
    digitalWrite(pumpPin, LOW);
//    Serial.println("Pump stop");
  } else {
    digitalWrite(pumpPin, HIGH);
//    Serial.println("Pump start");
  }
}

float getTemperatureFromDS18B20(OneWire oneWire) {
    byte i;
  byte present = 0;
  byte type_s;
  byte data[12];
  byte addr[8];
  float celsius, fahrenheit;
  
  if ( !oneWire.search(addr)) {
    Serial.println("No more addresses.");
    Serial.println();
    oneWire.reset_search();
    delay(250);
    return;
  }
  
  Serial.print("ROM =");
  for( i = 0; i < 8; i++) {
    Serial.write(' ');
    Serial.print(addr[i], HEX);
  }

  if (OneWire::crc8(addr, 7) != addr[7]) {
      Serial.println("CRC is not valid!");
      return;
  }
  Serial.println();
 
  // the first ROM byte indicates which chip
  switch (addr[0]) {
    case 0x10:
      Serial.println("  Chip = DS18S20");  // or old DS1820
      type_s = 1;
      break;
    case 0x28:
      Serial.println("  Chip = DS18B20");
      type_s = 0;
      break;
    case 0x22:
      Serial.println("  Chip = DS1822");
      type_s = 0;
      break;
    default:
      Serial.println("Device is not a DS18x20 family device.");
      return;
  } 

  oneWire.reset();
  oneWire.select(addr);
  oneWire.write(0x44,1);         // start conversion, with parasite power on at the end
  
  delay(1000);     // maybe 750ms is enough, maybe not
  // we might do a oneWire.depower() here, but the reset will take care of it.
  
  present = oneWire.reset();
  oneWire.select(addr);    
  oneWire.write(0xBE);         // Read Scratchpad

  Serial.print("  Data = ");
  Serial.print(present,HEX);
  Serial.print(" ");
  for ( i = 0; i < 9; i++) {           // we need 9 bytes
    data[i] = oneWire.read();
    Serial.print(data[i], HEX);
    Serial.print(" ");
  }
  Serial.print(" CRC=");
  Serial.print(OneWire::crc8(data, 8), HEX);
  Serial.println();

  // convert the data to actual temperature

  unsigned int raw = (data[1] << 8) | data[0];
  if (type_s) {
    raw = raw << 3; // 9 bit resolution default
    if (data[7] == 0x10) {
      // count remain gives full 12 bit resolution
      raw = (raw & 0xFFF0) + 12 - data[6];
    }
  } else {
    byte cfg = (data[4] & 0x60);
    if (cfg == 0x00) raw = raw << 3;  // 9 bit resolution, 93.75 ms
    else if (cfg == 0x20) raw = raw << 2; // 10 bit res, 187.5 ms
    else if (cfg == 0x40) raw = raw << 1; // 11 bit res, 375 ms
    // default is 12 bit resolution, 750 ms conversion time
  }
  celsius = (float)raw / 16.0;
  fahrenheit = celsius * 1.8 + 32.0;
  Serial.print("  Temperature = ");
  Serial.print(celsius);
  Serial.print(" Celsius, ");
  Serial.print(fahrenheit);
  Serial.println(" Fahrenheit");
  
  return celsius;
}
        
/*
Converting from 10mv per degree with 500mV of to degrees ((voltage - 500mV) times 100)
*/
float getTemperature(float voltage) {
  return (voltage - 0.5) * 100;
}

float getVoltage(int value) {
  float voltage = value * 5.0;
  voltage /=1024.0;
  
  return voltage;
}

void print(int line, String message) {
  lcd.setCursor(0, line);
  lcd.print(message);
}
