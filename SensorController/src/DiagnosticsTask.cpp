#include "DiagnosticsTask.h"
#include "SensorController.h"
#include "../Common/CommonMessageDefinitions.h"
#include <Arduino.h>
#include <cmath>
#include <cfloat>

extern "C"
{
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
}

extern InterESPCommunication interESPComm; // used to send messages

static MessageType DIAG_TYPE = MessageType::DIAGNOSTIC_UPDATE;

static void sendDiagnostic(InterESPCommunication &comm, uint8_t healthBits)
{
    Message msg;
    msg.startByte = 0xAA;
    msg.type = DIAG_TYPE;
    msg.length = 1; // just one byte
    msg.payload[0] = healthBits;
    msg.checksum = calculateCRC16(msg.payload, msg.length);
    if (!comm.sendMessage(msg))
    {
        Serial.println("DiagnosticsTask: Failed to send DIAGNOSTIC_UPDATE message.");
    }
}

void vDiagnosticsTask(void *pvParameters)
{
    SensorController *controller = static_cast<SensorController *>(pvParameters);

    IMU *imu = controller->getIMU();
    Magnetometer *mag = controller->getMagnetometer();
    PressureSensor *press = controller->getPressureSensor();
    Sonar *fSonar = controller->getForwardSonar();
    Sonar *oSonar = controller->getOscillatingSonar();

    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(1000); // 1 Hz

    while (true)
    {
        bool imuHealthy = (imu && imu->isHealthy());
        bool magHealthy = (mag && mag->isHealthy());
        bool pressHealthy = (press && press->isHealthy());
        bool fSonarHealthy = (fSonar && fSonar->isHealthy());
        bool oSonarHealthy = (oSonar && oSonar->isHealthy());

        // Pack bits: bit0=IMU, bit1=Mag, bit2=Press, bit3=FSonar, bit4=OSonar
        uint8_t healthBits = 0;
        if (imuHealthy)
            healthBits |= (1 << 0);
        if (magHealthy)
            healthBits |= (1 << 1);
        if (pressHealthy)
            healthBits |= (1 << 2);
        if (fSonarHealthy)
            healthBits |= (1 << 3);
        if (oSonarHealthy)
            healthBits |= (1 << 4);

        // Send diagnostic every cycle
        sendDiagnostic(interESPComm, healthBits);

        vTaskDelayUntil(&xLastWakeTime, period);
    }
}
