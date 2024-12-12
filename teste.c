#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <ESP32Servo.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// Configurações do Servo Motor
#define TASK_IDENTIFICADOR_SERVO "ServoTask"
#define DELAY_UP_SERVO 100
#define DELAY_ENTRE_SERVO 1000
#define DELAY_DOWN_SERVO 200
#define FREQUENCIA_SERVO 50
#define REF_POSICIONAMENTO_INICIAL 500
#define REF_POSICIONAMENTO_FINAL 2500
#define DIRECAO_POSITIVA 1
#define DIRECAO_NEGATIVA -1
#define ANGULO_INICIAL 0
#define INCREMENTO 2
#define PIN_SERVO 17

// Configurações dos Botões e Potenciômetro
#define PIN_LOOP_BUTTON 19
#define PIN_MANUAL_BUTTON 18
#define PIN_SHUTDOWN_BUTTON 13
#define PIN_POTENTIOMETER 14
bool isManualMode = false;
bool loopStarted = false;
bool modoSelecionado = false;
bool isPaused = false; // Indica se o sistema está pausado

// Configurações do Display OLED
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

Servo servoMotor;

// Variáveis para debounce
unsigned long lastDebounceTime = 0;
const unsigned long debounceDelay = 50; // 50 ms para debounce

// Função para atualizar o display e o monitor serial
void updateDisplayAndSerial(const char *mode, float angulo, float incremento, const char *direcao) {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.print("Modo: ");
  display.println(mode);
  display.println("");
  display.print("Angulo: ");
  display.print(angulo);
  display.println(" graus");
  display.println("");
  display.print("Incremento: ");
  display.println(incremento);
  display.println("");
  display.print("Direcao: ");
  display.println(direcao);
  display.display();

  // Envia os mesmos dados para o monitor serial
  Serial.println("!-- ServoMotor --!");
  Serial.print("Modo: ");
  Serial.println(mode);
  Serial.print("Angulo: ");
  Serial.print(angulo);
  Serial.println(" graus");
  Serial.print("Incremento: ");
  Serial.println(incremento);
  Serial.print("Direcao: ");
  Serial.println(direcao);
  Serial.println("------------------");
}

// Task do Servo Motor
void taskServoMotor(void *params) {
  Serial.println("Iniciando Task do ServoMotor!");
  servoMotor.setPeriodHertz(FREQUENCIA_SERVO);
  servoMotor.attach(PIN_SERVO, REF_POSICIONAMENTO_INICIAL, REF_POSICIONAMENTO_FINAL);
  int angulo = ANGULO_INICIAL;
  int direcao = DIRECAO_POSITIVA;

  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  display.clearDisplay();
  display.display();
  updateDisplayAndSerial("Aguardando Modo", angulo, 0, "N/A");

  while (true) {
    unsigned long currentMillis = millis();

    // Verifica o botão de shutdown
    if (digitalRead(PIN_SHUTDOWN_BUTTON) == LOW && (currentMillis - lastDebounceTime) > debounceDelay) {
      isPaused = true; // Pausa o sistema
      loopStarted = false; // Para o movimento automático
      modoSelecionado = true; // Indica que o modo foi selecionado
      updateDisplayAndSerial("Sistema Pausado", angulo, 0, "N/A");
      lastDebounceTime = currentMillis;
    }

    // Verifica o botão manual com debounce
    if (digitalRead(PIN_MANUAL_BUTTON) == LOW && (currentMillis - lastDebounceTime) > debounceDelay) {
      isPaused = false;
      isManualMode = true;
      loopStarted = false;
      modoSelecionado = true;
      updateDisplayAndSerial("Manual", angulo, 0, "N/A");
      lastDebounceTime = currentMillis;
    }

    // Verifica o botão de loop com debounce
    if (digitalRead(PIN_LOOP_BUTTON) == LOW && (currentMillis - lastDebounceTime) > debounceDelay) {
      isPaused = false;
      isManualMode = false;
      loopStarted = true;
      modoSelecionado = true;
      updateDisplayAndSerial("Automatico", angulo, INCREMENTO * direcao, direcao > 0 ? "Positiva" : "Negativa");
      lastDebounceTime = currentMillis;
    }

    if (modoSelecionado && !isPaused) {
      if (isManualMode) {
        // Controle manual pelo potenciômetro
        int potValue = analogRead(PIN_POTENTIOMETER);
        angulo = map(potValue, 0, 4095, 0, 180);
        servoMotor.write(angulo);
        updateDisplayAndSerial("Manual", angulo, 0, "N/A");
        vTaskDelay(100 / portTICK_PERIOD_MS);
      } else if (loopStarted) {
        // Controle automático
        angulo += INCREMENTO * direcao;
        servoMotor.write(angulo);
        updateDisplayAndSerial("Automatico", angulo, INCREMENTO * direcao, direcao > 0 ? "Positiva" : "Negativa");

        if (angulo >= 180 || angulo <= 0) {
          direcao *= -1; // Inverte a direção
          vTaskDelay(DELAY_ENTRE_SERVO / portTICK_PERIOD_MS);
        } else if (direcao == DIRECAO_POSITIVA) {
          vTaskDelay(DELAY_UP_SERVO / portTICK_PERIOD_MS);
        } else if (direcao == DIRECAO_NEGATIVA) {
          vTaskDelay(DELAY_DOWN_SERVO / portTICK_PERIOD_MS);
        }
      }
    } else {
      // Aguarda até que o sistema seja reativado
      vTaskDelay(100 / portTICK_PERIOD_MS);
    }
  }
}

void setup() {
  Serial.begin(115200);
  Serial.println("Iniciando ESP32");

  // Configuração dos pinos
  pinMode(PIN_LOOP_BUTTON, INPUT_PULLUP);
  pinMode(PIN_MANUAL_BUTTON, INPUT_PULLUP);
  pinMode(PIN_SHUTDOWN_BUTTON, INPUT_PULLUP);
  pinMode(PIN_POTENTIOMETER, INPUT);

  // Inicializa botões como não pressionados
  isManualMode = false;
  loopStarted = false;
  modoSelecionado = false;

  xTaskCreatePinnedToCore(taskServoMotor, TASK_IDENTIFICADOR_SERVO, 2048, NULL, 5, NULL, 0);
}

void loop() {
  delay(10);
}
