
#include <Arduino.h>
#include <TinyGPSPlus.h>
#include <SPI.h>
#include <SD.h>

// GPS hardware
HardwareSerial gpsSerial(2);
TinyGPSPlus gps;

// SD card hardware
const int SD_CS_PIN = 13;
const int SD_SCK_PIN = 18;
const int SD_MISO_PIN = 19;
const int SD_MOSI_PIN = 23;

const char* LOG_FILE = "/telemetry.csv";
bool sdReady = false;

// Main system states
enum SystemState {
    STATE_STARTUP,
    STATE_NORMAL,
    STATE_FAULT
};

// GPS subsystem states
enum GPSState {
    GPS_WAITING,
    GPS_OK,
    GPS_DEGRADED,
    GPS_LOST
};

// Fault codes
enum FaultCode {
    FAULT_NONE,
    FAULT_SENSOR_TIMEOUT,
    FAULT_CAN_COMMUNICATION,
    FAULT_GPS_LOST
};

// Current states
SystemState systemState = STATE_STARTUP;
GPSState gpsState = GPS_WAITING;
FaultCode activeFault = FAULT_NONE;

// Hardware configuration
const int FAULT_LED_PIN = 2;

// Timing configuration
const unsigned long STARTUP_TIME = 1000;
const unsigned long GPS_TIMEOUT = 3000;
const unsigned long LED_INTERVAL = 1500;
const unsigned long STATUS_INTERVAL = 2000;
const unsigned long LOG_INTERVAL = 5000;

// GPS quality thresholds
const int MIN_SATELLITES = 1;
const double MAX_HDOP = 7.5;

// Runtime tracking
unsigned long startupTime;
unsigned long lastLedToggle = 0;
unsigned long lastStatusPrint = 0;
unsigned long lastLogTime = 0;

bool ledState = false;
bool everHadFix = false;
bool faultLogged = false;

// Serial command input
String commandBuffer = "";

// State names for Serial output
const char* systemStateName(SystemState state) {
    switch (state) {
        case STATE_STARTUP: return "STARTUP";
        case STATE_NORMAL:  return "NORMAL";
        case STATE_FAULT:   return "FAULT";
        default:            return "UNKNOWN";
    }
}

const char* gpsStateName(GPSState state) {
    switch (state) {
        case GPS_WAITING:  return "WAITING";
        case GPS_OK:       return "OK";
        case GPS_DEGRADED: return "DEGRADED";
        case GPS_LOST:     return "LOST";
        default:           return "UNKNOWN";
    }
}

const char* faultCodeName(FaultCode code) {
    switch (code) {
        case FAULT_NONE:
            return "NONE";
        case FAULT_SENSOR_TIMEOUT:
            return "SENSOR_TIMEOUT";
        case FAULT_CAN_COMMUNICATION:
            return "CAN_COMMUNICATION";
        case FAULT_GPS_LOST:
            return "GPS_LOST";
        default:
            return "UNKNOWN";
    }
}

// Initialize SD card and create CSV file
void initializeSD() {
    Serial.println("[SD] Initializing card...");

    SPI.begin(
        SD_SCK_PIN,
        SD_MISO_PIN,
        SD_MOSI_PIN,
        SD_CS_PIN
    );

    if (!SD.begin(SD_CS_PIN, SPI, 10000000)) {
        Serial.println("[ERROR] SD initialization failed.");
        sdReady = false;
        return;
    }

    sdReady = true;

    Serial.print("[SD] Card initialized. Size: ");
    Serial.print(SD.cardSize() / (1024 * 1024));
    Serial.println(" MB");

    // Create the CSV file and header if it doesn't exist
    if (!SD.exists(LOG_FILE)) {
        File file = SD.open(LOG_FILE, FILE_WRITE);

        if (!file) {
            Serial.println("[ERROR] Could not create telemetry.csv");
            sdReady = false;
            return;
        }

        file.println(
            "timestamp,uptime_ms,latitude,longitude,"
            "satellites,hdop,gps_state,system_state,"
            "active_fault,event"
        );

        file.close();

        Serial.println("[SD] Created telemetry.csv");
    } else {
        Serial.println("[SD] Existing telemetry.csv found.");
    }
}

// Write a telemetry or fault event to the CSV file

void logRecord(const char* event) {
    if (!sdReady) {
        return;
    }

    File file = SD.open(LOG_FILE, FILE_APPEND);

    if (!file) {
        Serial.println("[ERROR] Could not open telemetry.csv");
        return;
    }

    // GPS date and time (UTC)
    char timestamp[24] = "NO_GPS_TIME";

    bool validDateTime =
        gps.date.isValid() &&
        gps.time.isValid() &&
        gps.date.age() <= GPS_TIMEOUT &&
        gps.time.age() <= GPS_TIMEOUT &&
        gps.date.year() >= 2020 &&
        gps.date.month() >= 1 &&
        gps.date.month() <= 12 &&
        gps.date.day() >= 1 &&
        gps.date.day() <= 31 &&
        gps.time.hour() <= 23 &&
        gps.time.minute() <= 59 &&
        gps.time.second() <= 59;

    if (validDateTime) {
        snprintf(
            timestamp,
            sizeof(timestamp),
            "%04d-%02d-%02d %02d:%02d:%02d",
            gps.date.year(),
            gps.date.month(),
            gps.date.day(),
            gps.time.hour(),
            gps.time.minute(),
            gps.time.second()
        );
    }

    file.print(timestamp);
    file.print(",");
    file.print(millis());
    file.print(",");

    // Only record coordinates from a recent fix
    bool freshFix =
        gps.location.isValid() &&
        gps.location.age() <= GPS_TIMEOUT;

    if (freshFix) {
        file.print(gps.location.lat(), 6);
        file.print(",");
        file.print(gps.location.lng(), 6);
    } else {
        file.print(",");
    }

    file.print(",");

    if (gps.satellites.isValid()) {
        file.print(gps.satellites.value());
    }

    file.print(",");

    if (gps.hdop.isValid()) {
        file.print(gps.hdop.hdop(), 2);
    }

    file.print(",");
    file.print(gpsStateName(gpsState));
    file.print(",");
    file.print(systemStateName(systemState));
    file.print(",");
    file.print(faultCodeName(activeFault));
    file.print(",");
    file.println(event);

    file.close();

    Serial.print("[SD] Logged: ");
    Serial.println(event);
}


// Check GPS health independently of system faults
GPSState checkGPSHealth() {
    bool freshFix =
        gps.location.isValid() &&
        gps.location.age() <= GPS_TIMEOUT;

    if (!freshFix) {
        return everHadFix ? GPS_LOST : GPS_WAITING;
    }

    everHadFix = true;

    bool enoughSatellites =
        gps.satellites.isValid() &&
        gps.satellites.age() <= GPS_TIMEOUT &&
        gps.satellites.value() >= MIN_SATELLITES;

    bool acceptableHDOP =
        gps.hdop.isValid() &&
        gps.hdop.age() <= GPS_TIMEOUT &&
        gps.hdop.hdop() <= MAX_HDOP;

    if (enoughSatellites && acceptableHDOP) {
        return GPS_OK;
    }

    return GPS_DEGRADED;
}

// Display current system information
void printStatus() {
    Serial.println();
    Serial.println("----- SYSTEM STATUS -----");

    Serial.print("System: ");
    Serial.println(systemStateName(systemState));

    Serial.print("GPS: ");
    Serial.println(gpsStateName(gpsState));

    Serial.print("Active fault: ");
    Serial.println(faultCodeName(activeFault));

    Serial.print("SD logging: ");
    Serial.println(sdReady ? "ACTIVE" : "UNAVAILABLE");

    if (gps.location.isValid() &&
        gps.location.age() <= GPS_TIMEOUT) {
        Serial.print("Latitude: ");
        Serial.println(gps.location.lat(), 6);

        Serial.print("Longitude: ");
        Serial.println(gps.location.lng(), 6);

        Serial.print("Satellites: ");
        Serial.println(gps.satellites.value());

        Serial.print("HDOP: ");
        Serial.println(gps.hdop.hdop(), 2);
    } else {
        Serial.println("Location: No recent GPS fix");
    }

    Serial.println("-------------------------");
    Serial.println();
}

// List available commands
void printHelp() {
    Serial.println();
    Serial.println("Available commands:");
    Serial.println("  help        - Show commands");
    Serial.println("  status      - Show system status");
    Serial.println("  fault sensor - Simulate sensor timeout");
    Serial.println("  fault can   - Simulate CAN failure");
    Serial.println("  fault gps   - Simulate GPS-loss fault");
    Serial.println("  clear       - Clear active fault");
    Serial.println();
}

// Register a simulated fault
void triggerFault(FaultCode code) {
    if (systemState == STATE_FAULT) {
        Serial.println(
            "[ERROR] A fault is already active. Use 'clear' first."
        );
        return;
    }

    activeFault = code;
    systemState = STATE_FAULT;
    faultLogged = false;

    Serial.print("[FAULT] Simulated fault triggered: ");
    Serial.println(faultCodeName(activeFault));

    logRecord("FAULT_TRIGGERED");
}

// Clear the active fault
void clearFault() {
    if (systemState != STATE_FAULT) {
        Serial.println("[SYSTEM] No active fault to clear.");
        return;
    }

    Serial.print("[SYSTEM] Clearing fault: ");
    Serial.println(faultCodeName(activeFault));

    activeFault = FAULT_NONE;
    systemState = STATE_NORMAL;
    faultLogged = false;

    ledState = false;
    digitalWrite(FAULT_LED_PIN, LOW);

    Serial.println("[SYSTEM] FAULT -> NORMAL");

    logRecord("FAULT_CLEARED");
}

// Process a complete Serial Monitor command
void processCommand(String command) {
    command.trim();
    command.toLowerCase();

    if (command.length() == 0) {
        return;
    }

    Serial.print("[COMMAND] ");
    Serial.println(command);

    if (command == "help") {
        printHelp();
    } else if (command == "status") {
        printStatus();
    } else if (command == "fault sensor") {
        triggerFault(FAULT_SENSOR_TIMEOUT);
    } else if (command == "fault can") {
        triggerFault(FAULT_CAN_COMMUNICATION);
    } else if (command == "fault gps") {
        triggerFault(FAULT_GPS_LOST);
    } else if (command == "clear") {
        clearFault();
    } else {
        Serial.println(
            "[ERROR] Unknown command. Type 'help' for commands."
        );
    }
}

// Read commands ending with a newline
void readSerialCommands() {
    while (Serial.available() > 0) {
        char incoming = Serial.read();

        if (incoming == '\n') {
            processCommand(commandBuffer);
            commandBuffer = "";
        } else if (incoming != '\r') {
            if (commandBuffer.length() < 80) {
                commandBuffer += incoming;
            } else {
                commandBuffer = "";
                Serial.println("[ERROR] Command too long.");
            }
        }
    }
}

void setup() {
    pinMode(FAULT_LED_PIN, OUTPUT);
    digitalWrite(FAULT_LED_PIN, LOW);

    Serial.begin(115200);
    gpsSerial.begin(9600, SERIAL_8N1, 15, 17);

    startupTime = millis();

    Serial.println();
    Serial.println("==============================");
    Serial.println("     ENGINE-CHECK-MATE");
    Serial.println("==============================");
    Serial.println("[SYSTEM] Starting up...");
    Serial.println("[SYSTEM] Initializing GPS...");

    initializeSD();

    Serial.println("[SYSTEM] Type 'help' for commands.");
}

void loop() {
    // Keep reading GPS data continuously
    while (gpsSerial.available() > 0) {
        gps.encode(gpsSerial.read());
    }

    static unsigned long lastGPSDebug = 0;

    if (millis() - lastGPSDebug >= 5000) {
        lastGPSDebug = millis();

        Serial.println("--- GPS DIAGNOSTICS ---");

        Serial.print("Characters received: ");
        Serial.println(gps.charsProcessed());

        Serial.print("Satellites reported: ");

        if (gps.satellites.isValid()) {
            Serial.println(gps.satellites.value());
        } else {
            Serial.println("Not available");
        }

        Serial.print("Location valid: ");
        Serial.println(gps.location.isValid() ? "YES" : "NO");

        if (gps.location.isValid()) {
            Serial.print("Latitude: ");
            Serial.println(gps.location.lat(), 6);

            Serial.print("Longitude: ");
            Serial.println(gps.location.lng(), 6);
        }

        Serial.print("HDOP: ");

        if (gps.hdop.isValid()) {
            Serial.println(gps.hdop.hdop(), 2);
        } else {
            Serial.println("Not available");
        }

        Serial.print("Location age (ms): ");

        if (gps.location.isValid()) {
            Serial.println(gps.location.age());
        } else {
            Serial.println("No valid location yet");
        }

        Serial.println("-----------------------");
    }

    // Handle incoming Serial Monitor commands
    readSerialCommands();

    unsigned long now = millis();

    // Update GPS health independently
    gpsState = checkGPSHealth();

    // Main system state machine
    switch (systemState) {
        case STATE_STARTUP:
            if (now - startupTime >= STARTUP_TIME) {
                systemState = STATE_NORMAL;
                Serial.println("[SYSTEM] STARTUP -> NORMAL");
            }
            break;

        case STATE_NORMAL:
            // Waiting for real fault detection logic
            break;

        case STATE_FAULT:
            if (!faultLogged) {
                Serial.println("[SYSTEM] Entered FAULT state");
                Serial.print("[FAULT] Code: ");
                Serial.println(faultCodeName(activeFault));
                faultLogged = true;
            }
            break;
    }

    // Blink onboard LED while a fault is active
    if (systemState == STATE_FAULT) {
        if (now - lastLedToggle >= LED_INTERVAL) {
            lastLedToggle = now;
            ledState = !ledState;
            digitalWrite(FAULT_LED_PIN, ledState);
        }
    } else {
        ledState = false;
        digitalWrite(FAULT_LED_PIN, LOW);
    }

    // Periodically log telemetry
    if (now - lastLogTime >= LOG_INTERVAL) {
        lastLogTime = now;
        logRecord("TELEMETRY");
    }

    // Periodic status output
    if (now - lastStatusPrint >= STATUS_INTERVAL) {
        lastStatusPrint = now;

        Serial.print("[SYSTEM] State: ");
        Serial.println(systemStateName(systemState));

        Serial.print("[GPS] State: ");
        Serial.println(gpsStateName(gpsState));

        Serial.print("[FAULT] Active code: ");
        Serial.println(faultCodeName(activeFault));
    }
}
