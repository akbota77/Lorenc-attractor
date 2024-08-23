#include <TaskScheduler.h>
#include <ArduinoQueue.h>
#include <OneWire.h>
#include <DallasTemperature.h>

// Пин, к которому подключен датчик DS18B20
#define ONE_WIRE_BUS 7

OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);

#define MAX_QUEUE_SIZE 10

struct SensorData {
    float value;
};

struct AuthorizationResult {
    bool authorized;
};

ArduinoQueue<SensorData> sensorDataQueue(MAX_QUEUE_SIZE);
ArduinoQueue<AuthorizationResult> authorizationResultQueue(MAX_QUEUE_SIZE);

Scheduler scheduler;

const int buttonPin = 3; // Pin for the button (changed from 2 to 3)
const int redLedPin = 4; // Pin for the red LED (changed from 1 to 4)
const int greenLedPin = 5; // Pin for the green LED (changed from 4 to 5)

void readSensor();
void authenticate();
void writeToAnalogOutput();

Task taskReadSensor(1000, TASK_FOREVER, &readSensor);
Task taskAuthenticate(1000, TASK_FOREVER, &authenticate);
Task taskWriteOutput(1000, TASK_FOREVER, &writeToAnalogOutput);

void setup() {
    Serial.begin(9600);
    sensors.begin();

    pinMode(buttonPin, INPUT); // Set button pin as input
    pinMode(redLedPin, OUTPUT); // Set red LED pin as output
    pinMode(greenLedPin, OUTPUT); // Set green LED pin as output

    scheduler.addTask(taskReadSensor);
    scheduler.addTask(taskAuthenticate);
    scheduler.addTask(taskWriteOutput);

    taskReadSensor.enable();
    taskAuthenticate.enable();
    taskWriteOutput.enable();
}

void loop() {
    scheduler.execute();
}

void generateLorenzKey(float& x, float& y, float& z) {
    float sigma = 10.0;
    float rho = 28.0;
    float beta = 8.0 / 3.0;
    float dt = 0.01;

    float dx = sigma * (y - x) * dt;
    float dy = (x * (rho - z) - y) * dt;
    float dz = (x * y - beta * z) * dt;

    x += dx;
    y += dy;
    z += dz;
}

int encryptData(int value, float& x, float& y, float& z) {
    generateLorenzKey(x, y, z);
    return value ^ static_cast<int>(x * 1000) ^ static_cast<int>(y * 1000) ^ static_cast<int>(z * 1000);
}

void readSensor() {
    float sensorValue = 0.0;
    sensors.requestTemperatures();
    sensorValue = sensors.getTempCByIndex(0);

    int buttonState = digitalRead(buttonPin);
    Serial.print("Button State: ");
    Serial.println(buttonState);
    
    if (buttonState == 1) {
        Serial.println("Button pressed, reading sensor...");
    }

    static float x = 0.1, y = 0.0, z = 0.0;

    sensorValue = encryptData(sensorValue, x, y, z);

    SensorData data;
    data.value = sensorValue;

    if (!sensorDataQueue.isFull()) {
        sensorDataQueue.enqueue(data);
        Serial.println("Data added to sensor queue.");
    } else {
        Serial.println("Sensor queue is full, data not added.");
    }
}

void authenticate() {
    if (!sensorDataQueue.isEmpty()) {
        SensorData data = sensorDataQueue.dequeue();
        
        bool authorized = (data.value > 512); // Replace with your authentication condition

        AuthorizationResult result;
        result.authorized = authorized;

        if (!authorizationResultQueue.isFull()) {
            authorizationResultQueue.enqueue(result);
            Serial.println("Authentication result added to queue.");
        } else {
            Serial.println("Authorization result queue is full, result not added.");
        }
    }
}

void writeToAnalogOutput() {
    if (!authorizationResultQueue.isEmpty()) {
        AuthorizationResult result = authorizationResultQueue.dequeue();

        if (result.authorized) {
            digitalWrite(greenLedPin, HIGH); // Green LED on if authorized
            digitalWrite(redLedPin, LOW); // Red LED off if authorized
            Serial.println("Authorized: Green LED ON, Red LED OFF");
        } else {
            digitalWrite(greenLedPin, LOW); // Green LED off if not authorized
            digitalWrite(redLedPin, HIGH); // Red LED on if not authorized
            Serial.println("Not Authorized: Green LED OFF, Red LED ON");
        }
    }
}
