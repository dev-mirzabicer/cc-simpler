#ifndef DATAAGGREGATOR_H
#define DATAAGGREGATOR_H

#include <Arduino.h>
#include "../CommonMessageDefinitions/Message.h"
#include "../Communication/InterESPCommunication.h"
#include <mutex> // Include mutex for thread safety

#ifdef FREERTOS_ENABLED
#include <mutex>
#endif

class DataAggregator
{
public:
    DataAggregator();
    void init(InterESPCommunication *comm);
    SensorData collectData();
    // Additional methods as needed

private:
    InterESPCommunication *interESPComm;
    SensorData aggregatedData;

    std::mutex dataMutex; // For thread safety

    // Private members for data handling
    void mergeSensorData(const SensorData &newData1, const SensorData &newData2);
    bool validateData(const SensorData &data1, const SensorData &data2);
};

#endif // DATAAGGREGATOR_H
