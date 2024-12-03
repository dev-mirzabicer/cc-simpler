#include "DataAggregator.h"

#define MAX_RETRIES 3
#define RETRY_DELAY_MS 50 // Delay between retries in milliseconds

DataAggregator::DataAggregator()
    : interESPComm(nullptr)
{
    // Initialize aggregatedData with zeros or default values
    memset(&aggregatedData, 0, sizeof(SensorData));
}

void DataAggregator::init(InterESPCommunication *comm)
{
    interESPComm = comm;
}

SensorData DataAggregator::collectData()
{
#ifdef FREERTOS_ENABLED
    std::lock_guard<std::mutex> lock(dataMutex); // Ensure thread safety
#endif

    SensorData dataController1;
    SensorData dataController2;
    bool success1 = false;
    bool success2 = false;

    // Attempt to receive data with retries
    for (int attempt = 0; attempt < MAX_RETRIES; ++attempt)
    {
        success1 = interESPComm->receiveSensorData(SENSOR_CONTROLLER_ADDRESS_1, dataController1);
        if (success1)
            break;
        delay(RETRY_DELAY_MS);
    }

    for (int attempt = 0; attempt < MAX_RETRIES; ++attempt)
    {
        success2 = interESPComm->receiveSensorData(SENSOR_CONTROLLER_ADDRESS_2, dataController2);
        if (success2)
            break;
        delay(RETRY_DELAY_MS);
    }

    if (success1 && success2)
    {
        if (validateData(dataController1, dataController2))
        {
            mergeSensorData(dataController1, dataController2);
        }
        else
        {
            Serial.println("Data synchronization mismatch between controllers.");
            // Implement fallback: Use the latest timestamp data
            if (dataController1.timestamp > dataController2.timestamp)
            {
                mergeSensorData(dataController1, dataController1);
            }
            else
            {
                mergeSensorData(dataController2, dataController2);
            }
        }
    }
    else
    {
        Serial.println("Failed to receive sensor data from one or more controllers.");
        // Implement fallback: Use the last known good data
        // aggregatedData holds the last good data
    }

    return aggregatedData;
}

void DataAggregator::mergeSensorData(const SensorData &newData1, const SensorData &newData2)
{
    // - Controller 1 provides IMU, Magnetometer, and Sonar data
    // - Controller 2 provides Pressure data

    // Update timestamp to the latest
    aggregatedData.timestamp = max(newData1.timestamp, newData2.timestamp);

    // Merge IMU data from Controller 1
    aggregatedData.imuAcceleration[0] = newData1.imuAcceleration[0];
    aggregatedData.imuAcceleration[1] = newData1.imuAcceleration[1];
    aggregatedData.imuAcceleration[2] = newData1.imuAcceleration[2];
    aggregatedData.imuGyro[0] = newData1.imuGyro[0];
    aggregatedData.imuGyro[1] = newData1.imuGyro[1];
    aggregatedData.imuGyro[2] = newData1.imuGyro[2];

    // Merge Magnetometer data from Controller 1
    aggregatedData.magneticField[0] = newData1.magneticField[0];
    aggregatedData.magneticField[1] = newData1.magneticField[1];
    aggregatedData.magneticField[2] = newData1.magneticField[2];

    // Merge Sonar data from Controller 1
    aggregatedData.sonarDistance[0] = newData1.sonarDistance[0];
    aggregatedData.sonarDistance[1] = newData1.sonarDistance[1];

    // Merge Pressure and Depth data from Controller 2
    aggregatedData.pressure = newData2.pressure;
    aggregatedData.depth = newData2.depth;

    // Additional sensor data can be merged here as needed
}

bool DataAggregator::validateData(const SensorData &data1, const SensorData &data2)
{
    // Validate that data from both controllers are synchronized
    // For example, check if their timestamps are within a certain threshold
    uint32_t timeDiff = abs((int32_t)(data1.timestamp - data2.timestamp));
    const uint32_t MAX_TIME_DIFF = 50; // milliseconds

    return (timeDiff <= MAX_TIME_DIFF);
}
