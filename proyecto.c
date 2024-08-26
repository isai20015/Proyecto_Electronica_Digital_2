#include <Arduino.h>
#include <ESP32Servo.h>
#include "AdafruitIO_WiFi.h"
#include "config.h"
#include <SPI.h>
#include <Wire.h>

// Definiciones de pines otros
#define Boton 4
#define LM35_PIN 34
#define ServoPin 2
// Definiciones de pines LEDS
#define LedVerde 27
#define LedAmarillo 26
#define LedRojo 25
// Definiciones de pines PWM
#define freqPWM 5000
#define resolutionPWM 8
#define canalPWMVerde 4
#define canalPWMAmarillo 1
#define canalPWMRojo 2
// Definiciones de pines Displays
#define SEG_A 19
#define SEG_B 5
#define SEG_C 17
#define SEG_D 16
#define SEG_E 21
#define SEG_F 22
#define SEG_G 23
#define SEG_P 33
// Definiciones de pines Transistores
#define Display1 15
#define Display2 14
#define Display3 12
#define Display4 13

// Inicialización del servo
Servo myServo;

// Arreglo de segmentos para el display de 7 segmentos
byte segmentos[] = {
    0b00111111, // 0
    0b00000110, // 1
    0b01011011, // 2
    0b01001111, // 3
    0b01100110, // 4
    0b01101101, // 5
    0b01111101, // 6
    0b00000111, // 7
    0b01111111, // 8
    0b01101111  // 9
};

// Variables para la temperatura y control del display
float temperaturaAMostrar = 0.0;
bool mostrarTemperatura = false;
const unsigned long DURACION_DISPLAY = 5000; // Duración para mostrar la temperatura
unsigned long tiempoUltimaLectura = 0; // Variable para el tiempo de última lectura

// Definir el feed de Adafruit IO
AdafruitIO_Feed *temperaturaFeed = io.feed("proyecto 1");

// Definir la cantidad de dígitos y el arreglo de dígitos
const int NUM_DIGITOS = 4;
int digitos[NUM_DIGITOS] = {0, 0, 0, 0};

// Variables de control para el multiplexado sin parpadeo
int displayActivo = 0;

// Configurar el temporizador para el multiplexado
hw_timer_t *multiplexadorTimer = NULL;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

// Declaración de la función mostrarDigito
void mostrarDigito(byte valor, bool punto);

// Función de interrupción para el multiplexado
void IRAM_ATTR multiplexarDisplay() {
    portENTER_CRITICAL_ISR(&timerMux);

    // Apagar todos los displays
    digitalWrite(Display1, LOW);
    digitalWrite(Display2, LOW);
    digitalWrite(Display3, LOW);
    digitalWrite(Display4, LOW);

    // Encender el display actual y mostrar el dígito correspondiente
    switch (displayActivo) {
        case 0:
            digitalWrite(Display1, HIGH);
            break;
        case 1:
            digitalWrite(Display2, HIGH);
            break;
        case 2:
            digitalWrite(Display3, HIGH);
            break;
        case 3:
            digitalWrite(Display4, HIGH);
            break;
    }
    mostrarDigito(segmentos[digitos[displayActivo]], (displayActivo == 2)); 

    // Pasar al siguiente display
    displayActivo = (displayActivo + 1) % NUM_DIGITOS;

    portEXIT_CRITICAL_ISR(&timerMux);
}

// Implementación de la función mostrarDigito
void mostrarDigito(byte valor, bool punto) {
    digitalWrite(SEG_A, valor & 0b00000001 ? HIGH : LOW);
    digitalWrite(SEG_B, valor & 0b00000010 ? HIGH : LOW);
    digitalWrite(SEG_C, valor & 0b00000100 ? HIGH : LOW);
    digitalWrite(SEG_D, valor & 0b00001000 ? HIGH : LOW);
    digitalWrite(SEG_E, valor & 0b00010000 ? HIGH : LOW);
    digitalWrite(SEG_F, valor & 0b00100000 ? HIGH : LOW);
    digitalWrite(SEG_G, valor & 0b01000000 ? HIGH : LOW);
    digitalWrite(SEG_P, punto ? HIGH : LOW);
}

void setup() {
    Serial.begin(115200);

    pinMode(SEG_A, OUTPUT);
    pinMode(SEG_B, OUTPUT);
    pinMode(SEG_C, OUTPUT);
    pinMode(SEG_D, OUTPUT);
    pinMode(SEG_E, OUTPUT);
    pinMode(SEG_F, OUTPUT);
    pinMode(SEG_G, OUTPUT);
    pinMode(SEG_P, OUTPUT);

    pinMode(Display1, OUTPUT);
    pinMode(Display2, OUTPUT);
    pinMode(Display3, OUTPUT);
    pinMode(Display4, OUTPUT);

    pinMode(LM35_PIN, INPUT);

    ledcSetup(canalPWMVerde, freqPWM, resolutionPWM);
    ledcAttachPin(LedVerde, canalPWMVerde);

    ledcSetup(canalPWMAmarillo, freqPWM, resolutionPWM);
    ledcAttachPin(LedAmarillo, canalPWMAmarillo);

    ledcSetup(canalPWMRojo, freqPWM, resolutionPWM);
    ledcAttachPin(LedRojo, canalPWMRojo);

    pinMode(Boton, INPUT_PULLUP);

    myServo.attach(ServoPin);

    // Conexión a Adafruit IO
    Serial.println("Conectando a Adafruit IO...");
    io.connect();

    while (io.status() < AIO_CONNECTED) {
        Serial.print(".");
        delay(500);
    }

    if (io.status() == AIO_CONNECTED) {
        Serial.println("Conectado a Adafruit IO!");
    } else {
        Serial.println("Error al conectar a Adafruit IO");
    }

    temperaturaFeed = io.feed("proyecto 1");

    Serial.print("Estado de Adafruit IO: ");
    Serial.println(io.statusText());

    // Configurar el temporizador para el multiplexado
    multiplexadorTimer = timerBegin(0, 80, true); // Temporizador 0, prescaler 80
    timerAttachInterrupt(multiplexadorTimer, &multiplexarDisplay, true);
    timerAlarmWrite(multiplexadorTimer, 5000, true); // 5000 microsegundos = 5 ms
    timerAlarmEnable(multiplexadorTimer);
}

void loop() {
    io.run();  // Actualiza el feed de Adafruit IO

    // Leer temperatura si se presiona el botón
    if (digitalRead(Boton) == LOW) {
        int valorSensor = analogRead(LM35_PIN);
        float temperatura = valorSensor * ((3.3 / 4095.0) * 100.0)+26;

        // Imprimir la temperatura leída
        Serial.print("Temperatura leida: ");
        Serial.print(temperatura);
        Serial.println(" °C");

        temperaturaAMostrar = temperatura;
        mostrarTemperatura = true;
        tiempoUltimaLectura = millis(); // Actualizar el tiempo de la última lectura

        temperaturaFeed->save(temperatura);

        // Lógica para controlar LEDs y servo según la temperatura
        if (temperatura <= 37.0) {
            ledcWrite(canalPWMVerde, 255);
            ledcWrite(canalPWMAmarillo, 0);
            ledcWrite(canalPWMRojo, 0);
            myServo.write(0);
            Serial.println("Temperatura baja. LED Verde encendido.");
        } else if (temperatura > 37.0 && temperatura <= 37.5) {
            ledcWrite(canalPWMVerde, 0);
            ledcWrite(canalPWMAmarillo, 255);
            ledcWrite(canalPWMRojo, 0);
            myServo.write(45);
            Serial.println("Temperatura media. LED Amarillo encendido.");
        } else if (temperatura > 37.5) {
            ledcWrite(canalPWMVerde, 0);
            ledcWrite(canalPWMAmarillo, 0);
            ledcWrite(canalPWMRojo, 255);
            myServo.write(90);
            Serial.println("Temperatura alta. LED Rojo encendido.");
        }

        delay(10);  // Evitar rebotes del botón
    }

    if (mostrarTemperatura) {
        int parteEntera = (int)temperaturaAMostrar;
        digitos[0] = parteEntera / 100;            // Centenas (si lo hubiera)
        digitos[1] = (parteEntera / 10) % 10;      // Decenas
        digitos[2] = parteEntera % 10;             // Unidades
        digitos[3] = (int)(temperaturaAMostrar * 10) % 10; // Decimales

        // Aquí se actualiza el display mediante la interrupción del temporizador

        if (millis() - tiempoUltimaLectura >= DURACION_DISPLAY) {
            mostrarTemperatura = false; // Dejar de mostrar temperatura después del tiempo
        }
    }
}