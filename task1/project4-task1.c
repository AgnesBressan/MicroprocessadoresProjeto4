#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <ESP32Servo.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// Servo Motor Configuration
#define TASK_IDENTIFICADOR_SERVO "ServoTask"
#define DELAY_UP_SERVO 100          // Delay for upward movement
#define DELAY_ENTRE_SERVO 1000      // Delay at the end positions (0° or 180°)
#define DELAY_DOWN_SERVO 200        // Delay for downward movement
#define FREQUENCIA_SERVO 50         // Servo signal frequency
#define REF_POSICIONAMENTO_INICIAL 500
#define REF_POSICIONAMENTO_FINAL 2500
#define DIRECAO_POSITIVA 1          // Positive direction
#define DIRECAO_NEGATIVA -1         // Negative direction
#define ANGULO_INICIAL 0            // Initial angle
#define INCREMENTO 2                // Incremental step for angle
#define PIN_SERVO 17                // Pin connected to the servo motor

// Button and Potentiometer Configuration
#define PIN_LOOP_BUTTON 19          // Pin for loop mode button
#define PIN_MANUAL_BUTTON 18        // Pin for manual mode button
#define PIN_SHUTDOWN_BUTTON 13      // Pin for shutdown button
#define PIN_POTENTIOMETER 14        // Pin for potentiometer
bool isManualMode = false;          // Manual mode flag
bool loopStarted = false;           // Loop mode flag
bool modoSelecionado = false;       // Mode selected flag
bool isPaused = false;              // Pause state flag

// OLED Display Configuration
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

Servo servoMotor;

// Previous button states
bool prevLoopButton = HIGH;
bool prevManualButton = HIGH;
bool prevShutdownButton = HIGH;

// Function to update the display and log to the serial monitor
void updateDisplayAndSerial(const char *mode, float angulo, float incremento, const char *direcao) {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.print("Mode: ");
  display.println(mode);
  display.println("");
  display.print("Angle: ");
  display.print(angulo);
  display.println(" degrees");
  display.println("");
  display.print("Increment: ");
  display.println(incremento);
  display.println("");
  display.print("Direction: ");
  display.println(direcao);
  display.display();

  // Log the same data to the serial monitor
  Serial.println("!-- ServoMotor --!");
  Serial.print("Mode: ");
  Serial.println(mode);
  Serial.print("Angle: ");
  Serial.print(angulo);
  Serial.println(" degrees");
  Serial.print("Increment: ");
  Serial.println(incremento);
  Serial.print("Direction: ");
  Serial.println(direcao);
  Serial.println("------------------");
}

// Function to detect button state transitions
bool detectButtonPress(int pin, bool &prevState) {
  bool currentState = digitalRead(pin);
  if (currentState != prevState) {
    prevState = currentState;
    return true; // Button state changed
  }
  prevState = currentState;
  return false;
}

// Task to control the servo motor
void taskServoMotor(void *params) {
  Serial.println("Starting Servo Motor Task!");
  servoMotor.setPeriodHertz(FREQUENCIA_SERVO);
  servoMotor.attach(PIN_SERVO, REF_POSICIONAMENTO_INICIAL, REF_POSICIONAMENTO_FINAL);
  int angulo = ANGULO_INICIAL;
  int direcao = DIRECAO_POSITIVA;

  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  display.clearDisplay();
  display.display();
  updateDisplayAndSerial("Waiting for Mode", angulo, 0, "N/A");

  while (true) {
    // Check the shutdown button
    if (detectButtonPress(PIN_SHUTDOWN_BUTTON, prevShutdownButton)) {
      isPaused = true;              // Pause the system
      loopStarted = false;          // Stop automatic movement
      modoSelecionado = true;       // Mode is selected
      updateDisplayAndSerial("System Paused", angulo, 0, "N/A");
    }

    // Check the manual button
    if (detectButtonPress(PIN_MANUAL_BUTTON, prevManualButton)) {
      isPaused = false;
      isManualMode = true;
      loopStarted = false;
      modoSelecionado = true;
      updateDisplayAndSerial("Manual", angulo, 0, "N/A");
    }

    // Check the loop button
    if (detectButtonPress(PIN_LOOP_BUTTON, prevLoopButton)) {
      isPaused = false;
      isManualMode = false;
      loopStarted = true;
      modoSelecionado = true;
      updateDisplayAndSerial("Automatic", angulo, INCREMENTO * direcao, direcao > 0 ? "Positive" : "Negative");
    }

    if (modoSelecionado && !isPaused) {
      if (isManualMode) {
        // Manual mode: Control the angle via potentiometer
        int potValue = analogRead(PIN_POTENTIOMETER);
        angulo = map(potValue, 0, 4095, 0, 180);
        servoMotor.write(angulo);
        updateDisplayAndSerial("Manual", angulo, 0, "N/A");
        vTaskDelay(100 / portTICK_PERIOD_MS);
      } else if (loopStarted) {
        // Automatic mode: Move continuously with delays
        angulo += INCREMENTO * direcao;
        servoMotor.write(angulo);
        updateDisplayAndSerial("Automatic", angulo, INCREMENTO * direcao, direcao > 0 ? "Positive" : "Negative");

        if (angulo >= 180 || angulo <= 0) {
          direcao *= -1; // Reverse direction at boundaries
          vTaskDelay(DELAY_ENTRE_SERVO / portTICK_PERIOD_MS);
        } else if (direcao == DIRECAO_POSITIVA) {
          vTaskDelay(DELAY_UP_SERVO / portTICK_PERIOD_MS);
        } else if (direcao == DIRECAO_NEGATIVA) {
          vTaskDelay(DELAY_DOWN_SERVO / portTICK_PERIOD_MS);
        }
      }
    } else {
      // Wait until the system is reactivated
      vTaskDelay(100 / portTICK_PERIOD_MS);
    }
  }
}

void setup() {
  Serial.begin(115200);
  Serial.println("Initializing ESP32");

  // Pin configuration
  pinMode(PIN_LOOP_BUTTON, INPUT_PULLUP);
  pinMode(PIN_MANUAL_BUTTON, INPUT_PULLUP);
  pinMode(PIN_SHUTDOWN_BUTTON, INPUT_PULLUP);
  pinMode(PIN_POTENTIOMETER, INPUT);

  // Initialize system state
  isManualMode = false;
  loopStarted = false;
  modoSelecionado = false;

  // Create the servo task
  xTaskCreatePinnedToCore(taskServoMotor, TASK_IDENTIFICADOR_SERVO, 2048, NULL, 5, NULL, 0);
}

void loop() {
  delay(10); // Small delay in the main loop
}
