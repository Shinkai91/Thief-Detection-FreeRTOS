#include <Arduino_FreeRTOS.h>
#include <Wire.h>
#include <semphr.h>
#include <queue.h>
#include <LiquidCrystal_I2C.h>

// Define pin numbers
#define ULTRASONIC_TRIGGER_PIN 2
#define ULTRASONIC_ECHO_PIN 3
#define PIR_SENSOR_PIN 4
#define BUZZER_PIN_ULTRASONIC 5
#define BUZZER_PIN_PIR 7
#define LED_ULTRASONIC 6 // Updated name
#define LED_PIR 8

// Create LCD object
LiquidCrystal_I2C lcd(0x27, 16, 2);

// Task handles
TaskHandle_t ultrasonicTaskHandle;
TaskHandle_t pirTaskHandle;
TaskHandle_t ledTaskHandle;
TaskHandle_t lcdTaskHandle;

// Semaphores and queues
SemaphoreHandle_t ultrasonicSemaphore;
SemaphoreHandle_t pirSemaphore;
QueueHandle_t lcdQueue;

// Function prototypes
void ultrasonicTask(void *pvParameters);
void pirTask(void *pvParameters);
void ledTask(void *pvParameters);
void lcdTask(void *pvParameters);

void setup()
{
    // Initialize LCD
    lcd.begin(16, 2);
    lcd.backlight();

    // Create tasks
    xTaskCreate(ultrasonicTask, "UltrasonicTask", 100, NULL, 2, &ultrasonicTaskHandle);
    xTaskCreate(pirTask, "PIRTask", 100, NULL, 2, &pirTaskHandle);
    xTaskCreate(ledTask, "LEDTask", 100, NULL, 1, &ledTaskHandle);
    xTaskCreate(lcdTask, "LCDTask", 100, NULL, 1, &lcdTaskHandle);

    // Create semaphores and queues
    ultrasonicSemaphore = xSemaphoreCreateBinary();
    pirSemaphore = xSemaphoreCreateBinary();
    lcdQueue = xQueueCreate(5, sizeof(char[17]));

    // Start the scheduler
    vTaskStartScheduler();
}

void loop()
{
    // This should never be reached
}

void ultrasonicTask(void *pvParameters)
{
    pinMode(ULTRASONIC_TRIGGER_PIN, OUTPUT);
    pinMode(ULTRASONIC_ECHO_PIN, INPUT);

    char ultrasonicMessage[17];

    while (1)
    {
        // Ultrasonic sensor logic
        long duration, distance;

        digitalWrite(ULTRASONIC_TRIGGER_PIN, LOW);
        delayMicroseconds(2);
        digitalWrite(ULTRASONIC_TRIGGER_PIN, HIGH);
        delayMicroseconds(10);
        digitalWrite(ULTRASONIC_TRIGGER_PIN, LOW);

        duration = pulseIn(ULTRASONIC_ECHO_PIN, HIGH);
        distance = duration * 0.034 / 2;

        if (distance < 50)
        {
            // Menggabungkan pesan "Object Distance:" dan nilai jarak dalam satu baris
            snprintf(ultrasonicMessage, sizeof(ultrasonicMessage), "Distance: %ld cm", distance);
            xQueueSend(lcdQueue, ultrasonicMessage, portMAX_DELAY);
            xSemaphoreGive(ultrasonicSemaphore);
        }

        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

void pirTask(void *pvParameters)
{
    pinMode(PIR_SENSOR_PIN, INPUT);

    char pirMessage[17];

    while (1)
    {
        // PIR sensor logic
        if (digitalRead(PIR_SENSOR_PIN) == HIGH)
        {
            snprintf(pirMessage, sizeof(pirMessage), "Motion detected!");
            xQueueSend(lcdQueue, pirMessage, portMAX_DELAY);
            xSemaphoreGive(pirSemaphore);
        }

        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

void ledTask(void *pvParameters)
{
    pinMode(LED_ULTRASONIC, OUTPUT);
    pinMode(LED_PIR, OUTPUT);
    pinMode(BUZZER_PIN_ULTRASONIC, OUTPUT);
    pinMode(BUZZER_PIN_PIR, OUTPUT);

    while (1)
    {
        BaseType_t pirSemaphoreStatus = xSemaphoreTake(pirSemaphore, 0);               // Non-blocking
        BaseType_t ultrasonicSemaphoreStatus = xSemaphoreTake(ultrasonicSemaphore, 0); // Non-blocking

        if (pirSemaphoreStatus == pdTRUE && ultrasonicSemaphoreStatus == pdTRUE)
        {
            // Both PIR and Ultrasonic sensors are triggered
            char combinedMessage[17];
            snprintf(combinedMessage, sizeof(combinedMessage), "LARI ADA WIBU");
            xQueueSend(lcdQueue, combinedMessage, portMAX_DELAY);

            digitalWrite(LED_PIR, HIGH);
            digitalWrite(LED_ULTRASONIC, HIGH);
            digitalWrite(BUZZER_PIN_PIR, HIGH);
            digitalWrite(BUZZER_PIN_ULTRASONIC, HIGH);
        }
        else if (pirSemaphoreStatus == pdTRUE)
        {
            // PIR sensor is triggered
            digitalWrite(LED_ULTRASONIC, LOW);
            digitalWrite(LED_PIR, HIGH);
            digitalWrite(BUZZER_PIN_ULTRASONIC, LOW);
            digitalWrite(BUZZER_PIN_PIR, HIGH);
        }
        else if (ultrasonicSemaphoreStatus == pdTRUE)
        {
            // Ultrasonic sensor is triggered
            digitalWrite(LED_PIR, LOW);
            digitalWrite(LED_ULTRASONIC, HIGH);
            digitalWrite(BUZZER_PIN_PIR, LOW);
            digitalWrite(BUZZER_PIN_ULTRASONIC, HIGH);
        }
        else
        {
            // Neither sensor is triggered
            digitalWrite(LED_PIR, LOW);
            digitalWrite(LED_ULTRASONIC, LOW);
            digitalWrite(BUZZER_PIN_PIR, LOW);
            digitalWrite(BUZZER_PIN_ULTRASONIC, LOW);
        }

        // Small delay to avoid busy-waiting
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}

void lcdTask(void *pvParameters)
{
    char message[17];
    char lastMessage[17] = ""; // Variable to store the last displayed message

    while (1)
    {
        // Receive a message from the queue with a timeout of 500 milliseconds
        if (xQueueReceive(lcdQueue, message, 500 / portTICK_PERIOD_MS) == pdTRUE)
        {
            // If a new message is received, check if it is the special "LARI ADA WIBU" message
            if (strcmp(message, "LARI ADA WIBU") == 0)
            {
                // If the special message is received, clear the LCD and display it
                lcd.setCursor(0, 0);
                lcd.print(message);
                strcpy(lastMessage, message);
            }
            else
            {
                // For other messages, update the LCD without clearing it
                lcd.clear();
                lcd.setCursor(0, 1); // Set the cursor to the second line
                lcd.print(message);
                strcpy(lastMessage, message);
            }
        }
        else
        {
            // If no new message is received, clear the LCD
            lcd.clear();
        }
        // Small delay to avoid busy-waiting
        vTaskDelay(500 / portTICK_PERIOD_MS);
    }
}