#include "mbed.h"
// a new class to play a note on Speaker based on PwmOut class
class Speaker
{
public:

    Speaker(PinName pin) : _pin(pin) {
// _pin(pin) means pass pin to the Speaker Constructor
    }
// class method to play a note based on PwmOut class
  void PlayNote(float frequency, float duration, float volume) {
        if (frequency == 0) {
            _pin = 0.0; // Ensure PWM is off
            ThisThread::sleep_for(std::chrono::milliseconds(static_cast<int>(duration * 1000)));
        } else {

            _pin.period(1.0 / frequency);
            _pin = volume / 2.0;
            
            ThisThread::sleep_for(std::chrono::milliseconds(static_cast<int>(duration * 1000)));
            _pin = 0.0; // Turn off PWM after the note duration
            ThisThread::sleep_for(50ms);  // Short pause between notes

        }
    }

private:
    PwmOut _pin;
};