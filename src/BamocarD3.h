#include "Registers.h"
#include "mbed.h"
#include "TemperaturGraph.h"

#define BAMOCAR_CAN_TIMEOUT 0.1 // s

class BamocarD3 {
    public:
        BamocarD3(PinName CAN_RD, PinName CAN_TD, int frequency = STD_BAUD_RATE) : _can(CAN_RD, CAN_TD, frequency) {
            _can.attach(callback(this, &BamocarD3::_messageReceived), CAN::RxIrq);
        }

        void requestSpeed(uint8_t interval = 0) {
            _requestRegister(REG_N_ACTUAL, interval);
            _requestRegister(REG_N_MAX, interval);
        }

        float getSpeed() {
            return (float)_got.N_MAX * ((float)_got.N_ACTUAL / 32767.0);
        }

        void requestCurrent(uint8_t interval = 0) {
            _requestRegister(REG_I_ACTUAL, interval);
            _requestRegister(REG_I_DEVICE, interval);
            _requestRegister(REG_I_200PC, interval);
        }

        float getCurrent() {
            return 0.2 * (float)_got.I_DEVICE * ((float)_got.I_ACTUAL / (float)_got.I_200PC);
        }

        void setTorque(float torque) {
            int16_t intTorque = 0;
            if (torque > 1.5) {
                intTorque = 0x7FFF;
            } else if (torque < -1.5) {
                intTorque = 0x8001;
            } else {
                intTorque = torque * 21845;
            }
            _send(REG_TORQUE, intTorque);
        }

        void requestTemp(uint8_t interval = 0) {
            _requestRegister(REG_TEMP_MOTOR, interval);
            _requestRegister(REG_TEMP_IGBT, interval);
            _requestRegister(REG_TEMP_AIR, interval);
        }

        float getMotorTemp() {
            return _calcTempFromValue(_got.TEMP_MOTOR, motorTempGraph);
        }

        float getServoTemp() {
            return _calcTempFromValue(_got.TEMP_IGBT, igbtTempGraph);
        }

        float getAirTemp() {
            return _calcTempFromValue(_got.TEMP_AIR, airTempGraph);
        }


    private:
        CAN _can;

        // Motor Controller IDs
        uint16_t _rxId = STD_RX_ID,
                 _txId = STD_TX_ID;
        
        struct _got {
            int16_t READY = 0,
                    N_ACTUAL = 0, N_MAX = 0,
                    I_ACTUAL = 0, I_DEVICE = 0, I_200PC = 0,
                    TORQUE = 0,
                    TEMP_MOTOR = 0, TEMP_IGBT = 0, TEMP_AIR = 0;
        } _got;

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
                        case REG_READY: _got.READY = _getInt16(msg); break;
                        case REG_N_ACTUAL: _got.N_ACTUAL = _getInt16(msg); break;
                        case REG_N_MAX: _got.N_MAX = _getInt16(msg); break;
                        case REG_I_ACTUAL: _got.I_ACTUAL = _getInt16(msg); break;
                        case REG_I_DEVICE: _got.I_DEVICE = _getInt16(msg); break;
                        case REG_I_200PC: _got.I_200PC = _getInt16(msg); break;
                        case REG_TORQUE: _got.TORQUE = _getInt16(msg); break;
                        case REG_TEMP_MOTOR: _got.TEMP_MOTOR = _getInt16(msg); break;
                        case REG_TEMP_IGBT: _got.TEMP_IGBT = _getInt16(msg); break;
                        case REG_TEMP_AIR: _got.TEMP_AIR = _getInt16(msg); break;
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
};