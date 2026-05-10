#include "Servo.h"

int Servo::channel_next_free = 0;

Servo::Servo(){ _resetFields();}

Servo::~Servo(){ detach();}

bool Servo::attach(int pin, int channel, 
                   int minAngle, int maxAngle, 
                   int minPulseWidth, int maxPulseWidth) 
{
    if(channel == CHANNEL_NOT_ATTACHED) {
        if(channel_next_free == CHANNEL_MAX_NUM) {
            return false;
        }
        _channel = channel_next_free;
        channel_next_free++;
    } else {
        _channel = channel;
    }

    _pin = pin;
    _minAngle = minAngle;
    _maxAngle = maxAngle;
    _minPulseWidth = minPulseWidth;
    _maxPulseWidth = maxPulseWidth;

    // ESP32 Arduino 3.x 删除 ledcSetup/ledcAttachPin，使用 ledcAttachChannel 保持原来的固定通道分配方式。
#if defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR >= 3)
    ledcAttachChannel(_pin, 50, 16, _channel); // channel X, 50 Hz, 16-bit depth
#else
    ledcSetup(_channel, 50, 16); // channel X, 50 Hz, 16-bit depth
    ledcAttachPin(_pin, _channel);
#endif
    return true;
}

bool Servo::detach() {
    if (!this->attached()) {
        return false;
    }

    if(_channel == (channel_next_free - 1))
        channel_next_free--;

    // ESP32 Arduino 3.x 的解绑接口按引脚解绑，旧版接口按 ledcDetachPin 命名。
#if defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR >= 3)
    ledcDetach(_pin);
#else
    ledcDetachPin(_pin);
#endif
    _pin = PIN_NOT_ATTACHED;
    return true;
}

void Servo::write(int degrees) {
    degrees = constrain(degrees, _minAngle, _maxAngle);
    writeMicroseconds(_angleToUs(degrees));
}

void Servo::writeMicroseconds(int pulseUs) {
    if (!attached()) {
        return;
    }
    pulseUs = constrain(pulseUs, _minPulseWidth, _maxPulseWidth);
    _pulseWidthDuty = _usToDuty(pulseUs);
    // 这里继续按通道写占空比，保证旧代码的通道管理逻辑在 3.x 下不变。
#if defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR >= 3)
    ledcWriteChannel(_channel, _pulseWidthDuty);
#else
    ledcWrite(_channel, _pulseWidthDuty);
#endif
}

int Servo::read() {
    return _usToAngle(readMicroseconds());
}

int Servo::readMicroseconds() {
    if (!this->attached()) {
        return 0;
    }
    // ESP32 Arduino 3.x 的 ledcRead 参数是 GPIO 引脚号，旧版参数是 LEDC 通道号。
#if defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR >= 3)
    int duty = ledcRead(_pin);
#else
    int duty = ledcRead(_channel);
#endif
    return _dutyToUs(duty);
}

bool Servo::attached() const { return _pin != PIN_NOT_ATTACHED; }

int Servo::attachedPin() const { return _pin; }

void Servo::_resetFields(void) {
    _pin = PIN_NOT_ATTACHED;
    _pulseWidthDuty = 0;
    _channel = CHANNEL_NOT_ATTACHED;
    _minAngle = MIN_ANGLE;
    _maxAngle = MAX_ANGLE;
    _minPulseWidth = MIN_PULSE_WIDTH;
    _maxPulseWidth = MAX_PULSE_WIDTH;
}
