#ifndef DIAGNOSTICSTASK_H
#define DIAGNOSTICSTASK_H

class SensorController;

/**
 * @brief DiagnosticsTask monitors sensor health and sends DIAGNOSTIC_UPDATE messages.
 *
 * Runs at 1 Hz, reads isHealthy() from each sensor, packs bits into a single byte and sends it.
 */
void vDiagnosticsTask(void *pvParameters);

#endif // DIAGNOSTICSTASK_H
