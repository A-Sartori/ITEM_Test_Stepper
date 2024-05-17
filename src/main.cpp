#include <Arduino.h>
#include <FastAccelStepper.h>

FastAccelStepperEngine engine = FastAccelStepperEngine();
FastAccelStepper* stepper = NULL;

#define dirPinStepper    18
#define enablePinStepper 16
#define stepPinStepper   17
#define STOP_BTN         4
#define START_BTN        5

#define M0 21
#define M1 22
#define M2 23

bool emergencyStop = false;
bool start = false;

void setMicroStepping(char n) {
    digitalWrite(M0, ((n & 1) != 0) ? HIGH : LOW);
    digitalWrite(M1, (((n >> 1) & 1) != 0) ? HIGH : LOW);
    digitalWrite(M2, (((n >> 2) & 1) != 0) ? HIGH : LOW);
}



void move(FastAccelStepper* stepper, int microsteps, int speed, int acceleration, int position) {
    setMicroStepping(microsteps);
    stepper->setDirectionPin(dirPinStepper);
    stepper->setEnablePin(enablePinStepper);
    stepper->setAutoEnable(true);
    stepper->setSpeedInHz(speed);
    stepper->setAcceleration(acceleration);
    stepper->moveTo(position);

    while(stepper->getCurrentPosition() != position) {
        if(emergencyStop) {
            stepper->forceStop();
            emergencyStop = false;
            while(!start) {
                delay(10);
            }
            start = false;
            move(stepper, microsteps, speed, acceleration, position);
        }
    }
    delay(100);
}

void stopCallback() {
    emergencyStop = true;
}

void startCallback() {
    start = true;
}

void setup() {
    Serial.begin(9600);
    pinMode(M0, OUTPUT);
    pinMode(M1, OUTPUT);
    pinMode(M2, OUTPUT);
    pinMode(STOP_BTN, INPUT_PULLUP);
    pinMode(START_BTN, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(STOP_BTN), stopCallback, FALLING);
    attachInterrupt(digitalPinToInterrupt(START_BTN), startCallback, FALLING);

    engine.init();
    stepper = engine.stepperConnectToPin(stepPinStepper);

    if(!stepper)
        return;

    
}

void loop() {
 // move(stepper, 1, 400, 100, 10000);
  setMicroStepping(1);
  stepper->setSpeedInHz(100);
  stepper->setAcceleration(200);
  stepper->runForward();
}