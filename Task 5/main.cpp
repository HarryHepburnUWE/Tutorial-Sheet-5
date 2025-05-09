#include "mbed.h"
#include "arm_book_lib.h"

#define NUMBER_OF_KEYS                           4
#define BLINKING_TIME_GAS_ALARM               1000
#define BLINKING_TIME_OVER_TEMP_ALARM          500
#define BLINKING_TIME_GAS_AND_OVER_TEMP_ALARM  100
#define NUMBER_OF_AVG_SAMPLES                   100
#define OVER_TEMP_LEVEL                         25
#define GAS_DETECTION_THRESHOLD                 0.4  // Analog voltage threshold for MQ2 (0-1.0)
#define TIME_INCREMENT_MS                       10
#define DEBOUNCE_KEY_TIME_MS                    40
#define KEYPAD_NUMBER_OF_ROWS                    4
#define KEYPAD_NUMBER_OF_COLS                    4
#define EVENT_MAX_STORAGE                        5
#define EVENT_NAME_MAX_LENGTH                   14

typedef enum {
    MATRIX_KEYPAD_SCANNING,
    MATRIX_KEYPAD_DEBOUNCE,
    MATRIX_KEYPAD_KEY_HOLD_PRESSED
} matrixKeypadState_t;

typedef struct systemEvent {
    time_t seconds;
    char typeOfEvent[EVENT_NAME_MAX_LENGTH];
} systemEvent_t;

DigitalIn alarmTestButton(BUTTON1);
AnalogIn mq2(A3);

DigitalOut alarmLed(LED1);
DigitalOut incorrectCodeLed(LED3);
DigitalOut systemBlockedLed(LED2);

DigitalInOut sirenPin(PE_10);

UnbufferedSerial uartUsb(USBTX, USBRX, 115200);

AnalogIn lm35(A1);

DigitalOut keypadRowPins[KEYPAD_NUMBER_OF_ROWS] = {PB_3, PB_5, PC_7, PA_15};
DigitalIn keypadColPins[KEYPAD_NUMBER_OF_COLS]  = {PB_12, PB_13, PB_15, PC_6};

bool alarmState    = OFF;
bool incorrectCode = false;
bool overTempDetector = OFF;

int numberOfIncorrectCodes = 0;
int numberOfHashKeyReleasedEvents = 0;
int keyBeingCompared    = 0;
char codeSequence[NUMBER_OF_KEYS]   = { '1', '8', '0', '5' };
char keyPressed[NUMBER_OF_KEYS] = { '0', '0', '0', '0' };
int accumulatedTimeAlarm = 0;

bool alarmLastState        = OFF;
bool gasLastState          = OFF;
bool tempLastState         = OFF;
bool ICLastState           = OFF;
bool SBLastState           = OFF;

bool gasDetectorState          = OFF;
bool overTempDetectorState     = OFF;

float potentiometerReading = 0.0;
float lm35ReadingsAverage  = 0.0;
float lm35ReadingsSum      = 0.0;
float lm35ReadingsArray[NUMBER_OF_AVG_SAMPLES];
float lm35TempC            = 0.0;
float mq2ReadingsAverage   = 0.0;
float mq2ReadingsSum       = 0.0;
float mq2ReadingsArray[NUMBER_OF_AVG_SAMPLES];

int accumulatedDebounceMatrixKeypadTime = 0;
int matrixKeypadCodeIndex = 0;
char matrixKeypadLastKeyPressed = '\0';
char matrixKeypadIndexToCharArray[] = {
    '1', '2', '3', 'A',
    '4', '5', '6', 'B',
    '7', '8', '9', 'C',
    '*', '0', '#', 'D',
};
matrixKeypadState_t matrixKeypadState;

int eventsIndex = 0;
systemEvent_t arrayOfStoredEvents[EVENT_MAX_STORAGE];

void inputsInit();
void outputsInit();
void alarmActivationUpdate();
void alarmDeactivationUpdate();
void uartTask();
void availableCommands();
bool areEqual();
void eventLogUpdate();
void systemElementStateUpdate(bool lastState, bool currentState, const char* elementName);
float celsiusToFahrenheit(float tempInCelsiusDegrees);
float analogReadingScaledWithTheLM35Formula(float analogReading);
void lm35ReadingsArrayInit();
void mq2ReadingsArrayInit();
void matrixKeypadInit();
char matrixKeypadScan();
char matrixKeypadUpdate();
void displayEventLog();

int main()
{
    inputsInit();
    outputsInit();
    const char* message = "Enter Code 1805 to Deactivate Alarm\r\n";
    uartUsb.write(message, strlen(message));
    while (true) {
        alarmActivationUpdate();
        alarmDeactivationUpdate();
        uartTask();
        eventLogUpdate();
        delay(TIME_INCREMENT_MS);
    }
}

void inputsInit()
{
    lm35ReadingsArrayInit();
    mq2ReadingsArrayInit();
    alarmTestButton.mode(PullDown);
    sirenPin.mode(OpenDrain);
    sirenPin.input();
    matrixKeypadInit();
}

void outputsInit()
{
    alarmLed = OFF;
    incorrectCodeLed = OFF;
    systemBlockedLed = OFF;
}

void alarmActivationUpdate()
{
    static int lm35SampleIndex = 0;
    static int mq2SampleIndex = 0;
    static bool lastOverTempDetector = OFF; // Track last state of overTempDetector
    static bool lastGasDetectorState = OFF; // Track last state of gasDetectorState
    int i = 0;

    // LM35 Temperature Sensor
    lm35ReadingsArray[lm35SampleIndex] = lm35.read();
    lm35SampleIndex++;
    if (lm35SampleIndex >= NUMBER_OF_AVG_SAMPLES) {
        lm35SampleIndex = 0;
    }
    
    lm35ReadingsSum = 0.0;
    for (i = 0; i < NUMBER_OF_AVG_SAMPLES; i++) {
        lm35ReadingsSum += lm35ReadingsArray[i];
    }
    lm35ReadingsAverage = lm35ReadingsSum / NUMBER_OF_AVG_SAMPLES;
    lm35TempC = analogReadingScaledWithTheLM35Formula(lm35ReadingsAverage);    
    
    if (lm35TempC > OVER_TEMP_LEVEL) {
        overTempDetector = ON;
    } else {
        overTempDetector = OFF;
    }

    // MQ2 Gas Sensor
    mq2ReadingsArray[mq2SampleIndex] = mq2.read();
    mq2SampleIndex++;
    if (mq2SampleIndex >= NUMBER_OF_AVG_SAMPLES) {
        mq2SampleIndex = 0;
    }
    
    mq2ReadingsSum = 0.0;
    for (i = 0; i < NUMBER_OF_AVG_SAMPLES; i++) {
        mq2ReadingsSum += mq2ReadingsArray[i];
    }
    mq2ReadingsAverage = mq2ReadingsSum / NUMBER_OF_AVG_SAMPLES;

    // Gas Detection
    if (mq2ReadingsAverage > GAS_DETECTION_THRESHOLD) {
        gasDetectorState = ON;
        if (!lastGasDetectorState) { // Print on transition to ON
            time_t currentTime = time(NULL);
            char str[100];
            sprintf(str, "Event: GAS_DET_ON, Time: %s", ctime(&currentTime));
            uartUsb.write(str, strlen(str));
        }
        if (!alarmLastState) {
            alarmState = ON;
        }
    } else {
        gasDetectorState = OFF;
    }
    lastGasDetectorState = gasDetectorState; // Update last state

    // Over-Temperature Detection
    if (overTempDetector) {
        overTempDetectorState = ON;
        if (!lastOverTempDetector) { // Print on transition to ON
            time_t currentTime = time(NULL);
            char str[100];
            sprintf(str, "Event: OVER_TEMP_ON, Time: %s", ctime(&currentTime));
            uartUsb.write(str, strlen(str));
        }
    } else {
        overTempDetectorState = OFF;
    }
    lastOverTempDetector = overTempDetector; // Update last state

    // Test Button
    if (alarmTestButton) {             
        overTempDetectorState = ON;
        gasDetectorState = ON;
        if (!lastGasDetectorState) { // Print gas event for test button
            time_t currentTime = time(NULL);
            char str[100];
            sprintf(str, "Event: GAS_DET_ON, Time: %s", ctime(&currentTime));
            uartUsb.write(str, strlen(str));
        }
        if (!lastOverTempDetector) { // Print over-temp event for test button
            time_t currentTime = time(NULL);
            char str[100];
            sprintf(str, "Event: OVER_TEMP_ON, Time: %s", ctime(&currentTime));
            uartUsb.write(str, strlen(str));
        }
        if (!alarmLastState) {
            alarmState = ON;
            time_t currentTime = time(NULL);
            char str[100];
            sprintf(str, "Event: TEST_BUTTON_ON, Time: %s", ctime(&currentTime));
            uartUsb.write(str, strlen(str));
        }
        lastGasDetectorState = ON; // Update states for test button
        lastOverTempDetector = ON;
    }

    // Alarm State Handling
    if (alarmState) { 
        accumulatedTimeAlarm += TIME_INCREMENT_MS;
        sirenPin.output();                                     
        sirenPin = LOW;                                

        if (gasDetectorState && overTempDetectorState) {
            if (accumulatedTimeAlarm >= BLINKING_TIME_GAS_AND_OVER_TEMP_ALARM) {
                accumulatedTimeAlarm = 0;
                alarmLed = !alarmLed;
            }
        } else if (gasDetectorState) {
            if (accumulatedTimeAlarm >= BLINKING_TIME_GAS_ALARM) {
                accumulatedTimeAlarm = 0;
                alarmLed = !alarmLed;
            }
        } else if (overTempDetectorState) {
            if (accumulatedTimeAlarm >= BLINKING_TIME_OVER_TEMP_ALARM) {
                accumulatedTimeAlarm = 0;
                alarmLed = !alarmLed;
            }
        }
    } else {
        alarmLed = OFF;
        gasDetectorState = OFF;
        overTempDetectorState = OFF;
        sirenPin.input();                                  
        lastGasDetectorState = OFF; // Reset last states when alarm is OFF
        lastOverTempDetector = OFF;
    }
}

void alarmDeactivationUpdate()
{
    if (numberOfIncorrectCodes < 5) {
        char keyReleased = matrixKeypadUpdate();
        if (keyReleased != '\0' && keyReleased != '#') {
            keyPressed[matrixKeypadCodeIndex] = keyReleased;
            if (matrixKeypadCodeIndex >= NUMBER_OF_KEYS - 1) {
                if (areEqual()) {
                    alarmState = OFF;
                    numberOfIncorrectCodes = 0;
                    matrixKeypadCodeIndex = 0;
                    uartUsb.write("Alarm Deactivated\r\n", 19);
                } else {
                    incorrectCodeLed = ON;
                    numberOfIncorrectCodes++;
                    matrixKeypadCodeIndex = 0;
                    uartUsb.write("Incorrect Code\r\n", 16);
                }
            } else {
                matrixKeypadCodeIndex++;
            }
        }
        if (keyReleased == '#') {
            displayEventLog();
        }
    } else {
        systemBlockedLed = ON;
    }
}

void uartTask()
{
    char receivedChar = '\0';
    char str[100];
    int stringLength;
    if (uartUsb.readable()) {
        uartUsb.read(&receivedChar, 1);
        switch (receivedChar) {
        case '1':
            if (alarmState) {
                uartUsb.write("The alarm is activated\r\n", 24);
            } else {
                uartUsb.write("The alarm is not activated\r\n", 28);
            }
            break;

        case '2':
            if (mq2ReadingsAverage > GAS_DETECTION_THRESHOLD) {
                uartUsb.write("Gas is being detected\r\n", 22);
            } else {
                uartUsb.write("Gas is not being detected\r\n", 27);
            }
            break;

        case '3':
            if (overTempDetector) {
                uartUsb.write("Temperature is above the maximum level\r\n", 40);
            } else {
                uartUsb.write("Temperature is below the maximum level\r\n", 40);
            }
            break;
            
        case '4':
            uartUsb.write("Please enter the four digits numeric code ", 42);
            uartUsb.write("to deactivate the alarm: ", 25);

            incorrectCode = false;

            for (keyBeingCompared = 0; keyBeingCompared < NUMBER_OF_KEYS; keyBeingCompared++) {
                uartUsb.read(&receivedChar, 1);
                uartUsb.write("*", 1);
                if (codeSequence[keyBeingCompared] != receivedChar) {
                    incorrectCode = true;
                }
            }

            if (!incorrectCode) {
                uartUsb.write("\r\nThe code is correct\r\n\r\n", 25);
                alarmState = OFF;
                incorrectCodeLed = OFF;
                numberOfIncorrectCodes = 0;
            } else {
                uartUsb.write("\r\nThe code is incorrect\r\n\r\n", 27);
                incorrectCodeLed = ON;
                numberOfIncorrectCodes++;
            }
            break;

        case '5':
            uartUsb.write("Please enter the new four digits numeric code ", 46);
            uartUsb.write("to deactivate the alarm: ", 25);

            for (keyBeingCompared = 0; keyBeingCompared < NUMBER_OF_KEYS; keyBeingCompared++) {
                uartUsb.write("*", 1);
                uartUsb.read(&receivedChar, 1);
                codeSequence[keyBeingCompared] = receivedChar;
            }

            uartUsb.write("\r\nNew code generated\r\n\r\n", 24);
            break;

        case 'c':
        case 'C':
            sprintf(str, "Temperature: %.2f \xB0 C\r\n", lm35TempC);
            stringLength = strlen(str);
            uartUsb.write(str, stringLength);
            break;

        case 'f':
        case 'F':
            sprintf(str, "Temperature: %.2f \xB0 F\r\n", celsiusToFahrenheit(lm35TempC));
            stringLength = strlen(str);
            uartUsb.write(str, stringLength);
            break;
            
        case 's':
        case 'S':
            struct tm rtcTime;
            int strIndex;
                    
            uartUsb.write("\r\nType four digits for the current year (YYYY): ", 48);
            for (strIndex = 0; strIndex < 4; strIndex++) {
                uartUsb.read(&str[strIndex], 1);
                uartUsb.write(&str[strIndex], 1);
            }
            str[4] = '\0';
            rtcTime.tm_year = atoi(str) - 1900;
            uartUsb.write("\r\n", 2);

            uartUsb.write("Type two digits for the current month (01-12): ", 47);
            for (strIndex = 0; strIndex < 2; strIndex++) {
                uartUsb.read(&str[strIndex], 1);
                uartUsb.write(&str[strIndex], 1);
            }
            str[2] = '\0';
            rtcTime.tm_mon = atoi(str) - 1;
            uartUsb.write("\r\n", 2);

            uartUsb.write("Type two digits for the current day (01-31): ", 45);
            for (strIndex = 0; strIndex < 2; strIndex++) {
                uartUsb.read(&str[strIndex], 1);
                uartUsb.write(&str[strIndex], 1);
            }
            str[2] = '\0';
            rtcTime.tm_mday = atoi(str);
            uartUsb.write("\r\n", 2);

            uartUsb.write("Type two digits for the current hour (00-23): ", 46);
            for (strIndex = 0; strIndex < 2; strIndex++) {
                uartUsb.read(&str[strIndex], 1);
                uartUsb.write(&str[strIndex], 1);
            }
            str[2] = '\0';
            rtcTime.tm_hour = atoi(str);
            uartUsb.write("\r\n", 2);

            uartUsb.write("Type two digits for the current minutes (00-59): ", 49);
            for (strIndex = 0; strIndex < 2; strIndex++) {
                uartUsb.read(&str[strIndex], 1);
                uartUsb.write(&str[strIndex], 1);
            }
            str[2] = '\0';
            rtcTime.tm_min = atoi(str);
            uartUsb.write("\r\n", 2);

            uartUsb.write("Type two digits for the current seconds (00-59): ", 49);
            for (strIndex = 0; strIndex < 2; strIndex++) {
                uartUsb.read(&str[strIndex], 1);
                uartUsb.write(&str[strIndex], 1);
            }
            str[2] = '\0';
            rtcTime.tm_sec = atoi(str);
            uartUsb.write("\r\n", 2);

            rtcTime.tm_isdst = -1;
            set_time(mktime(&rtcTime));
            uartUsb.write("Date and time has been set\r\n", 28);
            break;
                        
        case 't':
        case 'T':
            time_t epochSeconds;
            epochSeconds = time(NULL);
            sprintf(str, "Date and Time = %s", ctime(&epochSeconds));
            uartUsb.write(str, strlen(str));
            uartUsb.write("\r\n", 2);
            break;

        case 'e':
        case 'E':
            displayEventLog();
            break;

        default:
            availableCommands();
            break;
        }
    }
}

void availableCommands()
{
    uartUsb.write("Available commands:\r\n", 21);
    uartUsb.write("Press '1' to get the alarm state\r\n", 34);
    uartUsb.write("Press '2' to get the gas detector state\r\n", 41);
    uartUsb.write("Press '3' to get the over temperature detector state\r\n", 54);
    uartUsb.write("Press '4' to enter the code sequence\r\n", 38);
    uartUsb.write("Press '5' to enter a new code\r\n", 31);
    uartUsb.write("Press 'f' or 'F' to get lm35 reading in Fahrenheit\r\n", 52);
    uartUsb.write("Press 'c' or 'C' to get lm35 reading in Celsius\r\n", 49);
    uartUsb.write("Press 's' or 'S' to set the date and time\r\n", 43);
    uartUsb.write("Press 't' or 'T' to get the date and time\r\n", 43);
    uartUsb.write("Press 'e' or 'E' to get the stored events\r\n\r\n", 45);
}

bool areEqual()
{
    for (int i = 0; i < NUMBER_OF_KEYS; i++) {
        if (codeSequence[i] != keyPressed[i]) {
            return false;
        }
    }
    return true;
}

void eventLogUpdate()
{
    systemElementStateUpdate(alarmLastState, alarmState, "ALARM");
    alarmLastState = alarmState;

    systemElementStateUpdate(gasLastState, gasDetectorState, "GAS_DET");
    gasLastState = gasDetectorState;

    systemElementStateUpdate(tempLastState, overTempDetector, "OVER_TEMP");
    tempLastState = overTempDetector;
}

void systemElementStateUpdate(bool lastState, bool currentState, const char* elementName)
{
    char eventAndStateStr[EVENT_NAME_MAX_LENGTH] = "";

    // Only log ON transitions for ALARM, GAS_DET, and OVER_TEMP
    if (lastState != currentState && currentState) {
        strcat(eventAndStateStr, elementName);
        strcat(eventAndStateStr, "_ON");

        if (eventsIndex >= EVENT_MAX_STORAGE) {
            for (int i = 1; i < EVENT_MAX_STORAGE; i++) {
                arrayOfStoredEvents[i-1] = arrayOfStoredEvents[i];
            }
            eventsIndex = EVENT_MAX_STORAGE - 1;
        }

        arrayOfStoredEvents[eventsIndex].seconds = time(NULL);
        strcpy(arrayOfStoredEvents[eventsIndex].typeOfEvent, eventAndStateStr);
        eventsIndex++;

        uartUsb.write(eventAndStateStr, strlen(eventAndStateStr));
        uartUsb.write("\r\n", 2);
    }
}

float analogReadingScaledWithTheLM35Formula(float analogReading)
{
    return (analogReading * 3.3 / 0.01);
}

float celsiusToFahrenheit(float tempInCelsiusDegrees)
{
    return (tempInCelsiusDegrees * 9.0 / 5.0 + 32.0);
}

void lm35ReadingsArrayInit()
{
    for (int i = 0; i < NUMBER_OF_AVG_SAMPLES; i++) {
        lm35ReadingsArray[i] = 0;
    }
}

void mq2ReadingsArrayInit()
{
    for (int i = 0; i < NUMBER_OF_AVG_SAMPLES; i++) {
        mq2ReadingsArray[i] = 0;
    }
}

void matrixKeypadInit()
{
    matrixKeypadState = MATRIX_KEYPAD_SCANNING;
    for (int pinIndex = 0; pinIndex < KEYPAD_NUMBER_OF_COLS; pinIndex++) {
        keypadColPins[pinIndex].mode(PullUp);
    }
}

char matrixKeypadScan()
{
    for (int row = 0; row < KEYPAD_NUMBER_OF_ROWS; row++) {
        for (int i = 0; i < KEYPAD_NUMBER_OF_ROWS; i++) {
            keypadRowPins[i] = ON;
        }
        keypadRowPins[row] = OFF;
        for (int col = 0; col < KEYPAD_NUMBER_OF_COLS; col++) {
            if (keypadColPins[col] == OFF) {
                return matrixKeypadIndexToCharArray[row*KEYPAD_NUMBER_OF_ROWS + col];
            }
        }
    }
    return '\0';
}

char matrixKeypadUpdate()
{
    char keyDetected = '\0';
    char keyReleased = '\0';

    switch (matrixKeypadState) {
    case MATRIX_KEYPAD_SCANNING:
        keyDetected = matrixKeypadScan();
        if (keyDetected != '\0') {
            matrixKeypadLastKeyPressed = keyDetected;
            accumulatedDebounceMatrixKeypadTime = 0;
            matrixKeypadState = MATRIX_KEYPAD_DEBOUNCE;
        }
        break;

    case MATRIX_KEYPAD_DEBOUNCE:
        if (accumulatedDebounceMatrixKeypadTime >= DEBOUNCE_KEY_TIME_MS) {
            keyDetected = matrixKeypadScan();
            if (keyDetected == matrixKeypadLastKeyPressed) {
                matrixKeypadState = MATRIX_KEYPAD_KEY_HOLD_PRESSED;
            } else {
                matrixKeypadState = MATRIX_KEYPAD_SCANNING;
            }
        }
        accumulatedDebounceMatrixKeypadTime += TIME_INCREMENT_MS;
        break;

    case MATRIX_KEYPAD_KEY_HOLD_PRESSED:
        keyDetected = matrixKeypadScan();
        if (keyDetected != matrixKeypadLastKeyPressed) {
            if (keyDetected == '\0') {
                keyReleased = matrixKeypadLastKeyPressed;
            }
            matrixKeypadState = MATRIX_KEYPAD_SCANNING;
        }
        break;

    default:
        matrixKeypadInit();
        break;
    }
    return keyReleased;
}

void displayEventLog()
{
    char str[100];
    uartUsb.write("Recent Alarm Events:\r\n", 22);
    for (int i = 0; i < eventsIndex && i < EVENT_MAX_STORAGE; i++) {
        sprintf(str, "Event: %s, Time: %s", 
                arrayOfStoredEvents[i].typeOfEvent,
                ctime(&arrayOfStoredEvents[i].seconds));
        uartUsb.write(str, strlen(str));
    }
    uartUsb.write("\r\n", 2);
}