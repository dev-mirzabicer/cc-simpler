// SensorStatus.h
#ifndef SENSORSTATUS_H
#define SENSORSTATUS_H

enum class SensorStatus
{
    SUCCESS = 0,
    INIT_FAILURE = 1,
    CALIBRATION_FAILURE = 2,
    HARDWARE_ERROR = 3,
    COMMUNICATION_ERROR = 4,
    DATA_INVALID = 5,
    UNKNOWN_ERROR = 255
};

#endif // SENSORSTATUS_H
