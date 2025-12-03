#include <Arduino.h>
#include <WiFi.h>
#include <WiFiManager.h> 
#include <BlynkSimpleEsp32.h>
#include <DHT.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "AiEsp32RotaryEncoder.h"

/* ==========================================================
   CREDENCIALES BLYNK
   ========================================================== */
#define BLYNK_TEMPLATE_ID "TMPL2sgQ_Fmpk"
#define BLYNK_TEMPLATE_NAME "TecnoTerrario"
#define BLYNK_AUTH_TOKEN "LJxbiUThf4OVm_wRUQr5z07ULiX_lHZP"

#define BLYNK_PRINT Serial

/* ==========================================================
   MAPA DE PINES (BASADO EN TU ESQUEMÁTICO)
   ========================================================== */
// MOSFETs
#define PIN_MOSFET_LED  17  // Etiqueta MOS_LED -> GPIO 17
#define PIN_MOSFET_HUM  16  // Etiqueta MOS_HUM -> GPIO 16 (Reservado)

// Sensores
#define PIN_DHT         33  // Etiqueta DHT11-01 -> GPIO 33
#define PIN_SUELO       34  // Etiqueta HW390 -> GPIO 34

// Pantalla OLED (I2C Personalizado en tu esquema)
#define OLED_SDA        18  // Etiqueta SDA -> GPIO 18
#define OLED_SCL        19  // Etiqueta SCL -> GPIO 19

// Encoder Rotativo
#define ENC_CLK         23  // Etiqueta CLK -> GPIO 23
#define ENC_DT          22  // Etiqueta DT  -> GPIO 22
#define ENC_SW          21  // Etiqueta SW  -> GPIO 21
#define ENC_VCC         -1  // Usamos 3.3V externo
#define ENC_STEPS       4

// --- OBJETOS ---
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

AiEsp32RotaryEncoder rotaryEncoder = AiEsp32RotaryEncoder(ENC_CLK, ENC_DT, ENC_SW, ENC_VCC, ENC_STEPS);

DHT dht(PIN_DHT, DHT11);
BlynkTimer timer;

// --- VARIABLES GLOBALES ---
float temperatura = 0.0;
float humedad_aire = 0.0;
int humedad_suelo = 0;
bool estado_led = false;
bool wifi_conectado = false;

// --- BITMAP ICONO WIFI (10x8 px aprox, simplificado) ---
// Dibujaremos algo simple con primitivas para ahorrar memoria de programa si prefieres,
// o usamos este pequeño mapa de bits.
const unsigned char wifi_icon[] PROGMEM = {
  0x3C, 0x00, 0x7E, 0x00, 0xFF, 0x80, 0x99, 0x00, 0x42, 0x00, 0x24, 0x00, 0x18, 0x00, 0x00, 0x00
};

// --- FUNCIÓN ACTUALIZAR PANTALLA ---
void updateDisplay() {
  display.clearDisplay();
  
  // -- BARRA SUPERIOR --
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.print("TERRARIO");

  if (wifi_conectado) {
    // Dibujar icono WiFi en esquina superior derecha
    // Usamos primitivas para simular las ondas
    display.fillCircle(120, 2, 1, SSD1306_WHITE); // Punto
    display.drawArc(120, 2, 3, 0, 180, 270, SSD1306_WHITE); // Onda 1
    display.drawArc(120, 2, 6, 0, 180, 270, SSD1306_WHITE); // Onda 2
  } else {
    display.setCursor(110, 0);
    display.print("NO");
  }
  display.drawLine(0, 10, 128, 10, SSD1306_WHITE);

  // -- DATOS SENSORES --
  display.setCursor(0, 15);
  display.print("Temp: "); display.print(temperatura, 1); display.print(" C");

  display.setCursor(0, 27);
  display.print("H.Air: "); display.print(humedad_aire, 0); display.print(" %");

  display.setCursor(0, 39);
  display.print("H.Sue: "); display.print(humedad_suelo); display.print(" %");

  // -- ESTADO LED --
  display.drawLine(0, 50, 128, 50, SSD1306_WHITE);
  display.setCursor(0, 54);
  display.print("LED Tira: ");
  if (estado_led) {
    display.setTextColor(SSD1306_BLACK, SSD1306_WHITE); // Texto invertido
    display.print(" ON ");
    display.setTextColor(SSD1306_WHITE);
  } else {
    display.print(" OFF");
  }

  display.display();
}

// --- CONTROL DEL LED ---
void toggleLed(bool nuevo_estado) {
  estado_led = nuevo_estado;
  
  // Activar Hardware Real (GPIO 17)
  digitalWrite(PIN_MOSFET_LED, estado_led ? HIGH : LOW);
  
  // Avisar a Blynk (V3)
  Blynk.virtualWrite(V3, estado_led ? 1 : 0);
  
  // Actualizar OLED
  updateDisplay();
  
  Serial.print("LED cambiado a: "); Serial.println(estado_led ? "ON" : "OFF");
}

// --- INTERRUPCIÓN ENCODER ---
void IRAM_ATTR readEncoderISR() {
  rotaryEncoder.readEncoder_ISR();
}

// --- TAREA PERIÓDICA (SIMULACIÓN DE SENSORES) ---
void sendSensorData() {
  // Simulamos datos aleatorios como pediste
  temperatura = random(220, 280) / 10.0; // 22.0 a 28.0 C
  humedad_aire = random(50, 70);
  humedad_suelo = random(30, 80);

  // Enviar a Blynk
  Blynk.virtualWrite(V0, temperatura);
  Blynk.virtualWrite(V1, humedad_aire);
  Blynk.virtualWrite(V2, humedad_suelo);
  
  updateDisplay();
}

// --- REVISAR BOTÓN ENCODER ---
void checkEncoder() {
  if (rotaryEncoder.isEncoderButtonClicked()) {
    toggleLed(!estado_led); // Invertir estado
  }
}

// --- COMANDO DESDE APP BLYNK ---
BLYNK_WRITE(V3) {
  int valor = param.asInt();
  bool nuevo_estado = (valor == 1);
  if (nuevo_estado != estado_led) {
    toggleLed(nuevo_estado);
  }
}

void setup() {
  Serial.begin(115200);

  // 1. Configurar Pines Hardware
  pinMode(PIN_MOSFET_LED, OUTPUT);
  pinMode(PIN_MOSFET_HUM, OUTPUT);
  digitalWrite(PIN_MOSFET_LED, LOW);
  digitalWrite(PIN_MOSFET_HUM, LOW);

  // 2. Iniciar I2C Personalizado (18 SDA, 19 SCL)
  Wire.begin(OLED_SDA, OLED_SCL);

  // 3. Iniciar Pantalla
  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { 
    Serial.println(F("Error OLED allocation"));
    // No bucle infinito, seguimos para que funcione el resto
  } else {
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(10, 20);
    display.println("Iniciando...");
    display.display();
  }

  // 4. Iniciar Encoder
  rotaryEncoder.begin();
  rotaryEncoder.setup(readEncoderISR);
  rotaryEncoder.setBoundaries(0, 10, false); 
  rotaryEncoder.disableAcceleration();

  // 5. WiFi Manager
  WiFiManager wm;
  // wm.resetSettings(); // Descomentar para borrar credenciales guardadas

  display.clearDisplay();
  display.setCursor(0,0);
  display.println("Conectando WiFi...");
  display.println("Si falla, busca:");
  display.println("ESP32-Terrario");
  display.display();

  if (!wm.autoConnect("ESP32-TecnoTerrario")) {
    Serial.println("Fallo conexion WiFi");
    // No reiniciamos, permitimos funcionamiento offline
  } else {
    wifi_conectado = true;
    Serial.println("WiFi Conectado");
  }

  // 6. Iniciar Blynk
  if (wifi_conectado) {
    Blynk.config(BLYNK_AUTH_TOKEN);
    Blynk.connect(2000);
  }

  // Timer cada 3 seg
  timer.setInterval(3000L, sendSensorData);
}

void loop() {
  if (wifi_conectado) Blynk.run();
  timer.run();
  rotaryEncoder.loop();
  checkEncoder();
}