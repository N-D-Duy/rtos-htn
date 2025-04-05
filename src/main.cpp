#include <ESP32Servo.h>
#include <driver/i2s.h>
#include <controller.h>
#include <Arduino.h>
#include "esp_task_wdt.h"

#define I2S_DOUT 25
#define I2S_BCLK 27
#define I2S_LRC 26
#define BUTTON_PIN 14
#define SERVO_PIN 18

constexpr unsigned long DEBOUNCE_DELAY = 3000;
constexpr unsigned long RESET_RING_TIME = 20000;
constexpr TickType_t FIREBASE_SYNC_INTERVAL = pdMS_TO_TICKS(5000);
constexpr TickType_t BUTTON_POLL_INTERVAL = pdMS_TO_TICKS(50);
constexpr int WDT_TIMEOUT = 30;

struct FirebaseUpdate
{
    enum UpdateType
    {
        DOOR_STATUS,
        RING_STATUS
    } type;
    bool value;
};

TaskHandle_t doorTaskHandle = NULL;
TaskHandle_t bellTaskHandle = NULL;
TaskHandle_t btnTaskHandle = NULL;
TaskHandle_t syncFirebaseTaskHandle = NULL;
TaskHandle_t firebaseUpdateTaskHandle = NULL;
TaskHandle_t stackTestTaskHandle = NULL;

EventGroupHandle_t systemEventGroup = NULL;

#define DOOR_RESOURCE_BIT (1 << 0)
#define BELL_RESOURCE_BIT (1 << 1)
#define FIREBASE_BIT (1 << 2)

QueueHandle_t doorCommandQueue = NULL;
QueueHandle_t bellCommandQueue = NULL;
QueueHandle_t firebaseUpdateQueue = NULL;

volatile unsigned long lastPressTime = 0;
volatile int pressCount = 0;
volatile bool buttonInterruptTriggered = false;

Servo myServo;
Controller *controller = NULL;

void initI2S();
void playTone(int frequency, int duration, int times);
void IRAM_ATTR buttonISR();
void checkTaskStacks();
bool safeSetDoorStatus(bool status);
bool safeSetRingStatus(bool status);
bool safeGetDoorStatus();

void doorTaskFunction(void *pvParameters);
void bellTaskFunction(void *pvParameters);
void btnTaskFunction(void *pvParameters);
void syncFirebaseTaskFunction(void *pvParameters);
void firebaseUpdateTaskFunction(void *pvParameters);
void stackTestTaskFunction(void *pvParameters);

void setup()
{
    Serial.begin(115200);
    Serial.println("Starting ESP32 Camera Door System");

    esp_task_wdt_init(WDT_TIMEOUT, true);
    esp_task_wdt_add(NULL);

    pinMode(BUTTON_PIN, INPUT_PULLUP);

    attachInterrupt(digitalPinToInterrupt(BUTTON_PIN), buttonISR, FALLING);

    systemEventGroup = xEventGroupCreate();
    if (systemEventGroup == NULL)
    {
        Serial.println("Failed to create event group!");
        while (1)
            ;
    }

    doorCommandQueue = xQueueCreate(5, sizeof(bool));
    bellCommandQueue = xQueueCreate(5, sizeof(bool));
    firebaseUpdateQueue = xQueueCreate(10, sizeof(FirebaseUpdate));

    if (doorCommandQueue == NULL || bellCommandQueue == NULL ||
        firebaseUpdateQueue == NULL)
    {
        Serial.println("Failed to create queues!");
        while (1)
            ;
    }

    myServo.attach(SERVO_PIN);

    controller = new Controller();
    if (controller == NULL)
    {
        Serial.println("Failed to create controller!");
        while (1)
            ;
    }

    controller->setup();

    initI2S();

    xTaskCreate(
        doorTaskFunction,
        "DoorTask",
        8192,
        NULL,
        2,
        &doorTaskHandle);

    xTaskCreate(
        bellTaskFunction,
        "BellTask",
        8192,
        NULL,
        2,
        &bellTaskHandle);

    xTaskCreate(
        btnTaskFunction,
        "ButtonTask",
        4096,
        NULL,
        1,
        &btnTaskHandle);

    xTaskCreate(
        syncFirebaseTaskFunction,
        "SyncFirebaseTask",
        16384,
        NULL,
        1,
        &syncFirebaseTaskHandle);

    xTaskCreate(
        firebaseUpdateTaskFunction,
        "FirebaseUpdateTask",
        16384,
        NULL,
        1,
        &firebaseUpdateTaskHandle);

    xTaskCreate(
        stackTestTaskFunction,
        "StackTestTask",
        8192,
        NULL,
        1,
        &stackTestTaskHandle);

    bool doorState = safeGetDoorStatus();
    Serial.println(doorState ? "Door is open" : "Door is closed");

    if (doorState)
    {
        bool command = true;
        xQueueSend(doorCommandQueue, &command, portMAX_DELAY);
    }
}

void loop()
{
    esp_task_wdt_reset();

    checkTaskStacks();

    vTaskDelay(pdMS_TO_TICKS(1000));
}

void IRAM_ATTR buttonISR()
{
    static unsigned long last_interrupt_time = 0;
    unsigned long interrupt_time = millis();

    if (interrupt_time - last_interrupt_time > 200)
    {
        buttonInterruptTriggered = true;
    }
    last_interrupt_time = interrupt_time;
}

void doorTaskFunction(void *pvParameters)
{
    esp_task_wdt_add(NULL);

    bool doorCommand;

    for (;;)
    {

        esp_task_wdt_reset();

        if (xQueueReceive(doorCommandQueue, &doorCommand, pdMS_TO_TICKS(100)) == pdPASS)
        {

            EventBits_t bits = xEventGroupWaitBits(
                systemEventGroup,
                DOOR_RESOURCE_BIT,
                pdFALSE,
                pdFALSE,
                pdMS_TO_TICKS(100));

            if ((bits & DOOR_RESOURCE_BIT) == 0)
            {

                xEventGroupSetBits(systemEventGroup, DOOR_RESOURCE_BIT);
            }

            if (doorCommand)
            {
                myServo.write(20);
                Serial.println("Door opened");
            }
            else
            {
                myServo.write(90);
                Serial.println("Door closed");
            }
            xEventGroupClearBits(systemEventGroup, DOOR_RESOURCE_BIT);
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void bellTaskFunction(void *pvParameters)
{

    esp_task_wdt_add(NULL);

    bool ringCommand;

    for (;;)
    {
        esp_task_wdt_reset();

        if (xQueueReceive(bellCommandQueue, &ringCommand, pdMS_TO_TICKS(100)) == pdPASS)
        {

            EventBits_t bits = xEventGroupWaitBits(
                systemEventGroup,
                BELL_RESOURCE_BIT,
                pdFALSE,
                pdFALSE,
                pdMS_TO_TICKS(100));

            if ((bits & BELL_RESOURCE_BIT) == 0)
            {
                xEventGroupSetBits(systemEventGroup, BELL_RESOURCE_BIT);
            }

            if (ringCommand)
            {
                playTone(2000, 500, 1);

                FirebaseUpdate update;
                update.type = FirebaseUpdate::RING_STATUS;
                update.value = true;
                xQueueSend(firebaseUpdateQueue, &update, portMAX_DELAY);
            }
            else
            {
                playTone(1500, 500, 1);
            }

            xEventGroupClearBits(systemEventGroup, BELL_RESOURCE_BIT);
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void btnTaskFunction(void *pvParameters)
{
    esp_task_wdt_add(NULL);

    unsigned long currentMillis;
    bool ringCommand;
    static bool lastButtonState = HIGH;

    for (;;)
    {
        esp_task_wdt_reset();

        if (buttonInterruptTriggered)
        {
            currentMillis = millis();

            if (currentMillis - lastPressTime > DEBOUNCE_DELAY)
            {
                lastPressTime = currentMillis;

                ringCommand = (pressCount == 0);
                xQueueSend(bellCommandQueue, &ringCommand, portMAX_DELAY);

                pressCount++;
            }

            buttonInterruptTriggered = false;
        }

        currentMillis = millis();
        if (currentMillis - lastPressTime > RESET_RING_TIME && pressCount > 0)
        {
            FirebaseUpdate update;
            update.type = FirebaseUpdate::RING_STATUS;
            update.value = false;
            xQueueSend(firebaseUpdateQueue, &update, portMAX_DELAY);

            pressCount = 0;
        }

        vTaskDelay(BUTTON_POLL_INTERVAL);
    }
}

void syncFirebaseTaskFunction(void *pvParameters)
{
    esp_task_wdt_add(NULL);
    static bool lastDoorState = false;
    for (;;)
    {
        esp_task_wdt_reset();

        EventBits_t bits = xEventGroupWaitBits(
            systemEventGroup,
            FIREBASE_BIT,
            pdFALSE,
            pdFALSE,
            pdMS_TO_TICKS(100));

        if ((bits & FIREBASE_BIT) == 0)
        {
            xEventGroupSetBits(systemEventGroup, FIREBASE_BIT);
            try
            {
                controller->streamData();
            }
            catch (...)
            {
                Serial.println("Error in Firebase stream");
            }

            bool currentDoorState = safeGetDoorStatus();

            if (currentDoorState != lastDoorState)
            {
                xQueueSend(doorCommandQueue, &currentDoorState, 0);
                lastDoorState = currentDoorState;
                Serial.println("Door state changed from Firebase: " + String(currentDoorState ? "Open" : "Closed"));
            }

            xEventGroupClearBits(systemEventGroup, FIREBASE_BIT);
        }

        vTaskDelay(FIREBASE_SYNC_INTERVAL);
    }
}

void firebaseUpdateTaskFunction(void *pvParameters)
{
    esp_task_wdt_add(NULL);
    FirebaseUpdate update;
    for (;;)
    {
        esp_task_wdt_reset();

        if (xQueueReceive(firebaseUpdateQueue, &update, pdMS_TO_TICKS(100)) == pdPASS)
        {

            EventBits_t bits = xEventGroupWaitBits(
                systemEventGroup,
                FIREBASE_BIT,
                pdFALSE,
                pdFALSE,
                pdMS_TO_TICKS(100));

            if ((bits & FIREBASE_BIT) == 0)
            {
                xEventGroupSetBits(systemEventGroup, FIREBASE_BIT);
                if (update.type == FirebaseUpdate::DOOR_STATUS)
                {
                    if (safeSetDoorStatus(update.value))
                    {
                        Serial.println("Door status updated on Firebase");
                    }
                    else
                    {
                        Serial.println("Failed to update door status");
                    }
                }
                else if (update.type == FirebaseUpdate::RING_STATUS)
                {
                    if (safeSetRingStatus(update.value))
                    {
                        Serial.println("Ring status updated on Firebase");
                    }
                    else
                    {
                        Serial.println("Failed to update ring status");
                    }
                }
                xEventGroupClearBits(systemEventGroup, FIREBASE_BIT);
            }
            else
            {
                xQueueSendToFront(firebaseUpdateQueue, &update, 0);
                vTaskDelay(pdMS_TO_TICKS(100));
            }
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void stackTestTaskFunction(void *pvParameters)
{
    esp_task_wdt_add(NULL);
    const int MARGIN = 512;
    int stackSize = 512;

    for (;;)
    {
        esp_task_wdt_reset();
        if (stackSize < 4096)
        {
            char buffer[stackSize];
            for (int i = 0; i < stackSize; i++)
            {
                buffer[i] = i % 256;
            }

            UBaseType_t remaining = uxTaskGetStackHighWaterMark(NULL);
            Serial.println("Stack test - remaining: " + String(remaining) +
                           " with buffer: " + String(stackSize));

            if (remaining > MARGIN)
            {
                stackSize += 256;
            }
        }
        vTaskDelay(pdMS_TO_TICKS(10000));
    }
}

void initI2S()
{
    i2s_config_t i2s_config = {
        .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX),
        .sample_rate = 44100,
        .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
        .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT,
        .communication_format = i2s_comm_format_t(I2S_COMM_FORMAT_STAND_I2S),
        .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
        .dma_buf_count = 8,
        .dma_buf_len = 64,
        .use_apll = false,
        .tx_desc_auto_clear = true,
        .fixed_mclk = 0};

    i2s_pin_config_t pin_config = {
        .bck_io_num = I2S_BCLK,
        .ws_io_num = I2S_LRC,
        .data_out_num = I2S_DOUT,
        .data_in_num = I2S_PIN_NO_CHANGE};

    i2s_driver_install(I2S_NUM_0, &i2s_config, 0, nullptr);
    i2s_set_pin(I2S_NUM_0, &pin_config);
}

void playTone(int frequency, int duration, int times)
{
    constexpr int SAMPLES = 64;
    static int16_t sample[SAMPLES];

    for (int i = 0; i < SAMPLES; i++)
    {
        sample[i] = 30000 * sin(2 * PI * frequency * i / 44100.0);
    }

    size_t bytes_written;
    for (int t = 0; t < times; t++)
    {
        unsigned long startMillis = millis();
        while (millis() - startMillis < duration)
        {
            i2s_write(I2S_NUM_0, sample, SAMPLES * sizeof(int16_t), &bytes_written, portMAX_DELAY);
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

void checkTaskStacks()
{
    UBaseType_t doorRemaining = uxTaskGetStackHighWaterMark(doorTaskHandle);
    UBaseType_t bellRemaining = uxTaskGetStackHighWaterMark(bellTaskHandle);
    UBaseType_t btnRemaining = uxTaskGetStackHighWaterMark(btnTaskHandle);
    UBaseType_t syncRemaining = uxTaskGetStackHighWaterMark(syncFirebaseTaskHandle);
    UBaseType_t updateRemaining = uxTaskGetStackHighWaterMark(firebaseUpdateTaskHandle);

    Serial.println("Stack remaining - Door: " + String(doorRemaining) +
                   ", Bell: " + String(bellRemaining) +
                   ", Button: " + String(btnRemaining) +
                   ", Sync: " + String(syncRemaining) +
                   ", Update: " + String(updateRemaining));
}

bool safeSetDoorStatus(bool status)
{
    bool success = false;
    try
    {
        success = controller->setDoorStatus(status);
    }
    catch (...)
    {
        Serial.println("Exception in setDoorStatus");
        success = false;
    }
    return success;
}

bool safeSetRingStatus(bool status)
{
    bool success = false;
    try
    {
        success = controller->setRingStatus(status);
    }
    catch (...)
    {
        Serial.println("Exception in setRingStatus");
        success = false;
    }
    return success;
}

bool safeGetDoorStatus()
{
    bool status = false;
    try
    {
        status = controller->getDoorStatus();
    }
    catch (...)
    {
        Serial.println("Exception in getDoorStatus");
        status = false;
    }
    return status;
}

