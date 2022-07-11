#include "Registers.h"
#include "mbed.h"
#include "TemperaturGraph.h"

#define BAMOCAR_CAN_TIMEOUT 0.1 // s On this side (Microcontroller)
#define BAMOCAR_CAN_T_OUT_SETPOINT 300 // ms After witch the Inverter will stop if no new message came
#define BAMOCAR_CAN_T_OUT_RESEND 1 // s After which the Timeout will be resent
#define BAMOCAR_CAN_T_OUT_MAX 60000
#define BAMOCAR_CAN_T_OUT_AUTOSEND_TIME 0.250

#define REPORT_CAN_BAMOCAR_ERROR

enum model_type_t : uint8_t {
    BAMOCAR_D3_UNSPECIFYED = 0,
    BAMOCAR_D3_400V,
    BAMOCAR_D3_700V,
};

// Constants for calculations
#define BAMOCAR_D3_700V_VALUE_TO_VOLTAGE 0.03166076879311998829817985406285 // V per 1
#define BAMOCAR_D3_400V_VALUE_TO_VOLTAGE 0.01814209030261732308377799596665 // V per 1

class BamocarD3 {
    public:
        BamocarD3(PinName CAN_RD, PinName CAN_TD, bool invertTorque = false, model_type_t modelType = BAMOCAR_D3_UNSPECIFYED, int frequency = STD_BAUD_RATE) : _can(CAN_RD, CAN_TD, frequency), _invertTorque(invertTorque), _modelType(modelType) {
            _can.attach(callback(this, &BamocarD3::_messageReceived), CAN::RxIrq);
        }

        /**
         * @brief Call this in a loop often
         * 
         */
        void run() {
            if (_timeout.begun) {
                if (_timeout.timer.read() > _timeout.resend) {
                    _sendTimeoutMsg();
                }
            }

            uint8_t tdError = _can.tderror();
            uint8_t rdError = _can.rderror();
            if (tdError > 0 || rdError > 0) {
                #ifdef REPORT_CAN_BAMOCAR_ERROR
                pcSerial.printf("[BamocarD3]@run: Detected CAN Error: td: %i\t rd: %i\n", tdError, rdError);
                #endif

                // Reset to recover from passive mode
                if (tdError >= 127 || rdError >= 127)
                    _can.reset();
            }
        }

        /**
         * @brief Begin with the communication including timeout setting and start of CAN Message monitoring. Send a CAN message every [timeout] milliseconds to keep inverter "On"
         * 
         * @param autoSendTimeout Autosend timeout messsage (with ticker) to avoid false cutoff of the inverter
         * @param timeout 
         * @param timeoutResend 
         */
        void begin(bool autoSendTimeout = false, float autoSendTime = BAMOCAR_CAN_T_OUT_AUTOSEND_TIME, uint16_t timeout = BAMOCAR_CAN_T_OUT_SETPOINT, float timeoutResend = BAMOCAR_CAN_T_OUT_RESEND) {
            if (timeout > BAMOCAR_CAN_T_OUT_MAX)
                timeout = BAMOCAR_CAN_T_OUT_MAX;

            _timeout.begun = true;
            _timeout.timeout = timeout;
            _timeout.resend = timeoutResend;
            _restartTimer(_timeout.timer);
            _sendTimeoutMsg();
                
            _restartTimer(_age.READY);
            _restartTimer(_age.N_ACTUAL);
            _restartTimer(_age.N_MAX);
            _restartTimer(_age.I_ACTUAL);
            _restartTimer(_age.I_DEVICE);
            _restartTimer(_age.I_200PC);
            _restartTimer(_age.TORQUE);
            _restartTimer(_age.TEMP_MOTOR);
            _restartTimer(_age.TEMP_IGBT);
            _restartTimer(_age.TEMP_AIR);
            _restartTimer(_age.DC_VOLTAGE);

            if (autoSendTimeout) {
                _timeout.autoSender.attach(callback(this, &BamocarD3::_sendTimeoutMsg), autoSendTime);
            }
        }

        // Request the Speed to be received once or with an interval in ms
        void requestSpeed(uint8_t interval = 0) {
            _requestRegister(REG_N_ACTUAL, interval);
            _requestRegister(REG_N_MAX, interval);
        }

        // Get the Speed last received from the Inverter
        float getSpeed() {
            return (float)_got.N_MAX * ((float)_got.N_ACTUAL / 32767.0);
        }

        // Get age of the Speed Value
        float getSpeedAge() {
            return _age.N_ACTUAL.read();
        }

        // Request the Data required for Current Calculation to be received once or with an interval in ms (3 Messages)
        void requestCurrent(uint8_t interval = 0) {
            _requestRegister(REG_I_ACTUAL, interval);
            _requestRegister(REG_I_DEVICE, interval);
            _requestRegister(REG_I_200PC, interval);
        }

        // Get the calculated Current last received from the Inverter
        float getCurrent() {
            return 0.2 * (float)_got.I_DEVICE * ((float)_got.I_ACTUAL / (float)_got.I_200PC);
        }

        // Set the Torque which then should be applied to the motor (as -1.0 to 1.0 representing -100% to 100% of I max pk)
        void setTorque(float torque) {
            int16_t intTorque = 0;

            if (_invertTorque) torque = -torque;

            if (torque >= 1.0) {
                intTorque = 32767;
            } else if (torque <= -1.0) {
                intTorque = -32768;
            } else if (torque > 0) {
                intTorque = torque * 32767.0;
                if (intTorque < 0) {
                    intTorque = 0;
                }
            } else if (torque < 0) {
                intTorque = torque * 32768.0;
                if (intTorque > 0) {
                    intTorque = 0;
                }
            }

            _send(REG_TORQUE, intTorque);
        }

        // Request the Temperature to be received once or with an interval in ms (3 Messages)
        void requestTemp(uint8_t interval = 0) {
            _requestRegister(REG_TEMP_MOTOR, interval);
            _requestRegister(REG_TEMP_IGBT, interval);
            _requestRegister(REG_TEMP_AIR, interval);
        }

        // Get the Temperature of the Motor
        float getMotorTemp() {
            return _calcTempFromValue(_got.TEMP_MOTOR, motorTempGraph);
        }

        float getMotorTempAge() {
            return _age.TEMP_MOTOR.read();
        }

        // Get the Temperature of the Servo
        float getServoTemp() {
            return _calcTempFromValue(_got.TEMP_IGBT, igbtTempGraph);
        }
        
        float getServoTempAge() {
            return _age.TEMP_IGBT.read();
        }

        // Get the Temperature of the Air (inside of the inverter)
        float getAirTemp() {
            return _calcTempFromValue(_got.TEMP_AIR, airTempGraph);
        }

        float getAirTempAge() {
            return _age.TEMP_AIR.read();
        }

        // Request the DC Voltage to be received once or with an interval in ms
        void requestDcVoltage(uint8_t interval = 0) {
            _requestRegister(REG_DC_VOLTAGE, interval);
        }

        // Get the DC Voltage (with a +- 2% accuracy)
        float getDcVoltage() {
            if (_modelType == BAMOCAR_D3_700V) {
                return _got.DC_VOLTAGE * BAMOCAR_D3_700V_VALUE_TO_VOLTAGE;
            } else if (_modelType == BAMOCAR_D3_400V) {
                return _got.DC_VOLTAGE * BAMOCAR_D3_400V_VALUE_TO_VOLTAGE;
            }

            return 0;
        }

        // Get the age of DC Voltage
        float getDcVoltageAge() {
            return _age.DC_VOLTAGE.read();
        }


    private:
        CAN _can;
        bool _invertTorque;
        model_type_t _modelType;

        struct {
            Timer timer;
            Ticker autoSender;
            bool begun = false;
            uint16_t timeout;
            float resend;
        } _timeout;

        // Motor Controller IDs
        uint16_t _rxId = STD_RX_ID,
                 _txId = STD_TX_ID;
        
        // The got Values from the Bamocar D3 will be stored here
        struct {
            int16_t READY = 0,
                    N_ACTUAL = 0, N_MAX = 0,
                    I_ACTUAL = 0, I_DEVICE = 0, I_200PC = 0,
                    TORQUE = 0,
                    TEMP_MOTOR = 0, TEMP_IGBT = 0, TEMP_AIR = 0,
                    DC_VOLTAGE = 0;
        } _got;

        // The age of got messages according to a value will be stored here
        struct {
            Timer READY,
                  N_ACTUAL, N_MAX,
                  I_ACTUAL, I_DEVICE, I_200PC,
                  TORQUE,
                  TEMP_MOTOR, TEMP_IGBT, TEMP_AIR,
                  DC_VOLTAGE;
        } _age;

        int16_t _getInt16(CANMessage &msg) {
            int16_t returnValue = 0;
            returnValue = msg.data[1];
            returnValue |= msg.data[2] << 8;

            return  returnValue;
        }

        int32_t _getInt32(CANMessage &msg) {
            int32_t returnValue = 0;
            returnValue = msg.data[1];
            returnValue |= msg.data[2] << 8;
            returnValue |= msg.data[3] << 16;
            returnValue |= msg.data[4] << 24;

            return returnValue;
        }

        void _messageReceived() {
            CANMessage msg = CANMessage();
            while(_can.read(msg)) {
                if (msg.len == 4) {
                    switch (msg.data[0]) {
                        case REG_READY: _got.READY = _getInt16(msg); _restartTimer(_age.READY); break;
                        case REG_N_ACTUAL: _got.N_ACTUAL = _getInt16(msg); _restartTimer(_age.N_ACTUAL); break;
                        case REG_N_MAX: _got.N_MAX = _getInt16(msg); _restartTimer(_age.N_MAX); break;
                        case REG_I_ACTUAL: _got.I_ACTUAL = _getInt16(msg); _restartTimer(_age.I_ACTUAL); break;
                        case REG_I_DEVICE: _got.I_DEVICE = _getInt16(msg); _restartTimer(_age.I_DEVICE); break;
                        case REG_I_200PC: _got.I_200PC = _getInt16(msg); _restartTimer(_age.I_200PC); break;
                        case REG_TORQUE: _got.TORQUE = _getInt16(msg); _restartTimer(_age.TORQUE); break;
                        case REG_TEMP_MOTOR: _got.TEMP_MOTOR = _getInt16(msg); _restartTimer(_age.TEMP_MOTOR); break;
                        case REG_TEMP_IGBT: _got.TEMP_IGBT = _getInt16(msg); _restartTimer(_age.TEMP_IGBT); break;
                        case REG_TEMP_AIR: _got.TEMP_AIR = _getInt16(msg); _restartTimer(_age.TEMP_AIR); break;
                        case REG_DC_VOLTAGE: _got.DC_VOLTAGE = _getInt16(msg); _restartTimer(_age.DC_VOLTAGE); break;
                    }
                } else if (msg.len == 6) {
                    // [il]
                } else {
                    // [il]
                }
            }
        }

        void _send(uint8_t reg, uint8_t data1, uint8_t data2) {
            CANMessage msg = CANMessage();
            msg.id = _rxId;
            msg.len = 3;
            msg.data[0] = reg;
            msg.data[1] = data1;
            msg.data[2] = data2;

            int msgSendResult = 0;
            Timer timeout;
            timeout.reset();
            timeout.start();
            while((msgSendResult != 1) && (timeout < BAMOCAR_CAN_TIMEOUT)) {
                msgSendResult = _can.write(msg);
                wait(0.000002);
            }
        }

        void _send(uint8_t reg, int16_t data) {
            _send(reg, (data & 0xFF), ((data >> 8) & 0xFF));
        }

        void _requestRegister(uint8_t reg, uint8_t interval) {
            _send(REG_GIB_MIR_DIE_BUTTER_DU_BITCH, reg, interval);
        }

        float _map(float x, float in_min, float in_max, float out_min, float out_max) {
            return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
        }

        float _calcTempFromValue(int16_t value, const int16_t graph[][2]) {
            uint16_t i = 0;

            // First check if the Value is smaller as the smallest measurement
            if (value <= graph[0][0]) {
                return graph[0][1];
            }

            while(true) {
                // Check if at the end of array
                if (graph[i + 1][0] == -1) {
                    return graph[i][1];
                }

                // Because we checked before if the the value is the smallest, it is enough to check if the value is smaller then the next value
                if (value <= graph[i + 1][0]) {
                    return _map(value, graph[i][0], graph[i + 1][0], graph[i][1], graph[i + 1][1]);
                } else {
                    i++;
                }
            }
        }

        void _sendTimeoutMsg() {
            _send(REG_T_OUT, _timeout.timeout & 0xFF, (_timeout.timeout >> 8) & 0xFF);
        }

        void _restartTimer(Timer& timer) {
            timer.reset();
            timer.start();
        }
};