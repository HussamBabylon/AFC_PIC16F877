#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <PWM.h>

LiquidCrystal_I2C lcd(0x27, 20, 4); // set the LCD address to 0x27 for a 16 chars and 2 line display

int motor = 9;
int buzzer = 10;
int32_t frequency = 100; //frequency (in Hz)

#define RT0 5684   // 25°C Degree In 5684 Ω  .
#define B 3965     // Beta sabiti
//--------------------------------------

#define VCC 5    //Supply voltage
#define R 5464  //R=5.4 KΩ

//Variables
float RT, VR, ln, T0, VRT;
int TX;

void setup() {

//  Serial.begin(9600);
  InitTimersSafe();

  //sets the frequency for the specified pin
  bool success = SetPinFrequencySafe(motor, frequency);

  //if the pin frequency was set successfully, turn pin 13 on
  if (success) {
    pinMode(buzzer, OUTPUT);
    digitalWrite(buzzer, HIGH);
    delay(2000);
    digitalWrite(buzzer, LOW);
  }

  lcd.init();
  lcd.backlight();
  lcd.setCursor(3, 0);
  lcd.print("BMW , AL-MARWA");
  lcd.setCursor(2, 1);
  lcd.print("07710854222");
  //Serial.begin(9600);
  T0 = 25 + 273.15;    //Temperature T0 from datasheet, conversion from Celsius to kelvin
}

void loop() {
  VRT = analogRead(A0);
  VRT = (5.00 / 1023.00) * VRT;      //Conversion to voltage
  VR = VCC - VRT;
  RT = VRT / (VR / R);               //Resistance of RT
  ln = log(RT / RT0);
  TX = (1 / ((ln / B) + (1 / T0))); //Temperature from thermistor
  TX = TX - 273.15;                 //Conversion to Celsius
//    Serial.print("Temperature:");
//    Serial.print(" ");
//    Serial.println(TX);
  lcd.setCursor(0, 2);
  lcd.print("Temperature: ");
  lcd.setCursor(0, 14);
  lcd.print(TX);
  lcd.setCursor(0, 17);
  lcd.print("°C");
  delay(500);

  switch (TX) {
    case 70: // enter the Temperature you want
      pwmWrite(motor, 55); // enter the speed you want
      break;
    case 75:
      pwmWrite(motor, 75);
      break;
    case 80:
      pwmWrite(motor, 75);
      break;
    case 85:
      pwmWrite(motor, 75);
      break;
    case 90:
      pwmWrite(motor, 75);
      break;
    case 91:
      pwmWrite(motor, 75);
      break;
    case 92:
      pwmWrite(motor, 75);
      break;
    case 93:
      pwmWrite(motor, 75);
      break;
    case 94:
      pwmWrite(motor, 75);
      break;
    case 95:
      pwmWrite(motor, 75);
      break;
    case -273: // whene system is eroor
      pwmWrite(motor, 75); // set the full speed
      digitalWrite(buzzer, HIGH);
      break;
    default:
      pwmWrite(motor, 25);
      break;
  }



}
