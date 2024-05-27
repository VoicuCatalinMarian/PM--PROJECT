#include <dht11.h>
#include <LiquidCrystal_I2C.h>

// Initializarea pinilor digitali
const int echoPin = 2;
const int trigPin = 3;
const int dht11Pin = 5;
const int pumpPin = 6;
const int ledPin = 10;
const int switch_buttonPin = 12;
const int on_off_buttonPin = 11;

// Initializarea pinilor analogici
const int soil_moisturePin = A0;

// Obiectul pentru senzorul DHT11
dht11 DHT11;

// Variabile pentru contorizarea apasarilor de butoane
int switch_button_counter = 0; 
bool last_switch_button_state = LOW;
int on_off_button_counter = 0; 
bool last_on_off_button_state = LOW;
bool lcd_state = true; // Starea LCD-ului

// Initializarea ecranului LCD
LiquidCrystal_I2C lcd(0x27,20,4); 
byte degree_symbol[8] = {0b00111, 0b00101, 0b00111, 0b00000, 0b00000, 0b00000, 0b00000, 0b00000};

// Variabile pentru timing
unsigned long previous_millis = 0;
const long interval = 300; // Intervalul pentru functia loop (ms)
unsigned long last_debounce_time = 0;
const unsigned long debounce_delay = 200; // Timpul pentru debounce (ms)

unsigned long last_pump_time = 0; // Timpul ultimei activari a pompei
const unsigned long pump_interval = 20000; // Intervalul de activare a pompei (20 secunde)

void setup() {
  Serial.begin(9600);

  // Configurarea pinilor
  DDRD |= (1 << DDD3); // trigPin OUTPUT
  DDRD &= ~(1 << DDD2); // echoPin INPUT

  DDRD |= (1 << DDD6); // pumpPin OUTPUT
  PORTD &= ~(1 << PORTD6); // pumpPin LOW

  DDRB |= (1 << DDB2); // ledPin OUTPUT
  DDRB &= ~(1 << DDB3); // on_off_buttonPin INPUT
  DDRB &= ~(1 << DDB4); // switch_buttonPin INPUT

  // Initializarea LCD-ului
  lcd.init();
  lcd.backlight();
  lcd.createChar(1, degree_symbol);
}

void loop() {
  unsigned long current_millis = millis();

  // Verific intrevalul pentru a executa codul periodic
  if (current_millis - previous_millis >= interval) {
    previous_millis = current_millis;
    
    // Masurarea distantei
    float distance  = getDistance();
    
    // Citirea umiditatii solului
    int soil_moisture =  analogRead(soil_moisturePin);

    // Citirea temperaturii si umiditatii din aer
    int temperature, humidity;
    readDHT(temperature, humidity);

    // Calcularea procentului de apa din rezervor
    float water = map(distance, 4, 24, 100, 0);
    water = constrain(water, 0, 100);

    // Actualizarea starii LED-ului
    updateLED(water);

    // Calcularea procentului de umiditate in sol
    float soil_hum = map(soil_moisture, 500, 950, 100, 0);
    soil_hum = constrain(soil_hum, 0, 100);

    // Actualizarea starii pompei
    updatePump(soil_hum, current_millis); 
    
    // Debouncing pentru butonul on/off
    if (debounceButton(on_off_buttonPin, last_on_off_button_state, last_debounce_time)) {
      on_off_button_counter++;
      lcd_state = !lcd_state;
    }

    // Debouncing pentru butonul de schimbare a afisajului LCD
    if (debounceButton(switch_buttonPin, last_switch_button_state, last_debounce_time)) {
      switch_button_counter++;
    }

    // Afisarea datelor pe LCD
    displayData(lcd_state, switch_button_counter, water, soil_hum);

    // Afisarea datelor de debug in Serial Monitor
    debugDisplay(distance, soil_moisture, water, soil_hum, on_off_button_counter, switch_button_counter);
  }
}

// Functia pentru masurarea distantei folosind senzorul ultrasonic
float getDistance() {
  PORTD &= ~(1 << PORTD3); // trigPin LOW
  delayMicroseconds(2);
  PORTD |= (1 << PORTD3); // trigPin HIGH
  delayMicroseconds(10);
  PORTD &= ~(1 << PORTD3); // trigPin LOW

  long duration = pulseIn(echoPin, HIGH);
  float distance = duration * 0.0343 / 2;

  return distance;
}

// Functia pentru actualizarea starii pompei in functie de umiditatea solului si a timerului
void updatePump(float soil_hum, unsigned long current_millis) {
  if (soil_hum < 50 && (current_millis - last_pump_time >= pump_interval)) {
    last_pump_time = current_millis;

    // Activez pompa
    PORTD |= (1 << PORTD6); // pumpPin HIGH
    delay(2000);

    // Dezactivez pompa
    PORTD &= ~(1 << PORTD6); // pumpPin LOW
    delay(200);
  }
}

// Functia pentru actualizarea starii LED-ului in functie de nivelul apei
void updateLED(float water) {
  if (water < 20) {
    // Activez LED-ul
    PORTB |= (1 << PORTB2); // ledPin HIGH
  } else {
    // Dezactivez LED-ul
    PORTB &= ~(1 << PORTB2); // ledPin LOW
    }
}

// Functia pentru citirea temperaturii si umiditatii folosind senzorul DHT11
void readDHT(int &temperature, int &humidity) {
  int chk = DHT11.read(dht11Pin);
  temperature = DHT11.temperature;
  humidity = DHT11.humidity;
}

// Functie pentru debounce-ul butoanelor
bool debounceButton(int pin, bool &last_button_state, unsigned long &last_debounce_time) {
  bool reading = digitalRead(pin);

  if (reading != last_button_state) {
    last_debounce_time = millis();
  }

  if ((millis() - last_debounce_time) > debounce_delay) {
    if (reading == HIGH) {
      last_button_state = reading;
      return true;
    }
  }

  last_button_state = reading;
  return false;
}

// Functia pentru afisarea datelor pe LCD
void displayData(int lcd_state, int switch_counter, float water, float soil_hum)
{
  if(lcd_state) {
    lcd.backlight();
    if(switch_button_counter % 2 == 0) {
      // Afisez temperatura din aer pe LCD
      lcd.setCursor(0, 0);
      lcd.print("Temp = ");
      lcd.setCursor(11,0);
      lcd.write(1);
      lcd.print("C");

      lcd.setCursor(7,0);
      lcd.print((float)DHT11.temperature, 1);

      // Afisez umiditatea din aer pe LCD
      lcd.setCursor(0,1);
      lcd.print("Humidity = ");
      lcd.setCursor(15,1);
      lcd.print("%");

      lcd.setCursor(11,1);
      lcd.print((float)DHT11.humidity, 1);
    } else {
      // Afisez temperatura din aer pe LCD
      lcd.setCursor(0, 0);
      lcd.print("Water = ");
      lcd.setCursor(12,0);
      lcd.print("%");

      lcd.setCursor((water == 100) ? 7 : 8, 0);
      lcd.print((float)water, 1);

      // Afisez umiditatea din aer pe LCD
      lcd.setCursor(0,1);
      lcd.print("Soil_Hum = ");
      lcd.setCursor(15,1);
      lcd.print("%");
      
      lcd.setCursor((soil_hum == 100) ? 10 : 11, 1);
      lcd.print((float)soil_hum, 1);
    }
  } else {
    lcd.clear();
    lcd.noBacklight();
  }
}

// Functia pentru afisarea datelor de debug in Serial Monitor
void debugDisplay(float distance, int soil_moisture, float water, float soil_hum, int on_off_button_counter, int switch_button_counter)
{
  // // Afisarea distantei
  // Serial.print("Distance: ");
  // Serial.print(distance);
  // Serial.print(" cm\n");

  // // Afisez umiditatea in sol
  Serial.print("Soil Moisture Level: ");
  Serial.println(soil_moisture);

  // // Afisez temperatura din aer
  // Serial.print("Temperature  (C): ");
  // Serial.println((float)DHT11.temperature, 1);

  // // Afisez umiditatea din aer
  // Serial.print("Humidity (%): ");
  // Serial.println((float)DHT11.humidity, 1);

  // // Afisez procentul de apa din bazin
  // Serial.print("Water: ");
  // Serial.print(water);
  // Serial.println(" %");

  // // Afisez procentul de umiditate in sol
  Serial.print("Soil Hum: ");
  Serial.print(soil_hum);
  Serial.println(" %");

  // // Afisez counterul pentru pronire/oprire LCD
  // Serial.println(on_off_button_counter);

  // // Afisez counterul pentru afisajul LCD
  // Serial.println(switch_button_counter);
}


