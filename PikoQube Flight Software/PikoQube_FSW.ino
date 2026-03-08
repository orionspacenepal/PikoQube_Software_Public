/*
 * PROJECT: PikoQube FSW (Flight Software)
 * STATUS:  FLIGHT READY
 * DESCRIPTION: Main control software for the PikoQube satellite.
 * Handles sensors, radio transmission, power management,
 * and antenna deployment.
 */

// --- LIBRARIES ---
#include <Arduino.h>             // Core Arduino library (required for all Arduino sketches)
#include <Wire.h>                // Library for I2C communication (used by Sensors & Memory)
#include <SPI.h>                 // Library for SPI communication (used by the Radio)
#include <RH_RF24.h>             // RadioHead library driver for the RFM24/RFM22 radio module
#include <Adafruit_INA219.h>     // Library for the Power Sensor (Voltage/Current)
#include <Adafruit_BMP085.h>     // Library for the Pressure/Temperature Sensor
#include <Adafruit_MPU6050.h>    // Library for the Gyroscope/Accelerometer (IMU)
#include <Adafruit_Sensor.h>     // Unified sensor library helper
#include <avr/sleep.h>           // AVR Library to control processor sleep modes (Power Saving)
#include <avr/wdt.h>             // AVR Library to control the Watchdog Timer (System Reset Safety)
#include <math.h>                // Standard math library for calculations (sqrt, atan2, etc.)

// --- PIN MAPPING (Hardware Connections) ---
#define PIN_WDI         4        // Watchdog Interface Pin: Toggles to tell the external watchdog "I'm alive"
#define PIN_RADIO_SDN   9        // Radio Shutdown Pin: HIGH = Radio OFF, LOW = Radio ON
#define PIN_RADIO_CS    10       // Radio Chip Select: Selects the radio on the SPI bus
#define PIN_RADIO_IRQ   2        // Radio Interrupt Request: Radio tells Arduino "I received data!"
#define PIN_ANT_BURN    A3       // Antenna Burn Wire: High current pin to melt the fishing line
#define PIN_LED_VISUAL  7        // Status LED: Blinks when transmitting or deploying

#define PIN_DEPLOY_SENSE A6      // Deployment Switch: Analog pin to check if antenna is open
#define PIN_LIGHT_SENS   A7      // Light Sensor: Analog pin connected to the photoresistor

// --- USER CONFIGURATION ---
#define MY_CALLSIGN     "PQ"     // The name of your satellite (Morse Code ID)
#define RADIO_FREQ      436.0    // The frequency to transmit on (436.000 MHz)
#define UNIT_LENGTH     120       // Morse code speed (Speed of a single "dot" in milliseconds)

// --- TRANSMISSION POWER LEVELS ---
// The radio module uses hex codes to set power (0x00 to 0x7F)
#define TX_POWER_MAX    0x7F     // +20dBm (Max Power): Used for data packets to get max range
#define TX_POWER_MID    0x40     // +15dBm: Mid power for power saving
#define TX_POWER_MIN    0x05     // +1dBm:  Low power. Used for Morse to prevent battery crashes.

// --- LOGIC THRESHOLDS (Voltage Decision Points) ---
#define V_LOW_THRESH    3.40     // Above 3.4V = Normal Mode (High Speed Data)
#define V_CRIT_THRESH   3.20     // Above 3.2V = Low Power Mode (Slower Data)
                                 // Below 3.2V = Critical Mode (Beacon Only)
#define V_BURN_SAFE     3.60     // Minimum voltage required to attempt antenna deployment safely

// --- TIMERS (Sleep Durations) ---
#define TIME_NORM_SEC   30       // Sleep for 30 seconds in Normal Mode
#define TIME_LOW_SEC    300      // Sleep for 5 minutes (300s) in Low Power Mode
#define TIME_CRIT_SEC   600      // Sleep for 10 minutes (600s) in Critical Mode

// --- GFSK SETTINGS ---
#define TIME_GFSK_DURATION_MS 10000UL // Duration to blast data packets (10 seconds)

// --- DEPLOYMENT SETTINGS ---
#define DEPLOY_WAIT_MS    2000UL      // Wait 2 seconds before starting burn (Safety buffer)
#define BURN_CYCLE_MS     30000UL     // Try burning for 30 seconds total
#define MAX_BURN_ATTEMPTS 3           // Only try to burn 3 times max (to save battery)

// --- MEMORY MAP (Where data is stored in FRAM) ---
#define MEM_PTR_ADDR    0x10     // Address that stores "Where should I write the next log?"
#define MEM_DEPLOY_FLAG 0x20     // Address that stores "Did I successfully deploy?" (0xAA = Yes)
#define MEM_BURN_COUNT  0x21     // Address that stores "How many times have I tried to burn?"
#define MEM_DATA_START  0x30     // The memory address where data logging begins
#define MEM_MAX_ADDR    32000    // The end of the memory stick (32KB)

// --- SENSOR FUSION VARIABLES (Math for Orientation) ---
float pitch = 0.0;               // Stores the calculated Pitch angle (Tilt up/down)
float roll  = 0.0;               // Stores the calculated Roll angle (Tilt left/right)
unsigned long lastImuTime = 0;   // Timestamp of the last time we calculated angles
const float alpha = 0.96;        // Filter constant: 96% Gyro data, 4% Accelerometer data

// --- GENERIC MEMORY CLASS ---
// This is a custom helper to make talking to the FRAM memory chip easier
class Generic_I2C_Memory {
  private:
    uint8_t _addr;               // Variable to hold the I2C address of the chip
  public:
    // Function to start the memory connection
    bool begin(uint8_t addr = 0x50) {
      _addr = addr;              // Set the address
      Wire.begin();              // Start I2C bus
      Wire.beginTransmission(_addr); // Ping the device
      return (Wire.endTransmission() == 0); // Return TRUE if device answers
    }
    // Function to write a single byte to a specific address
    void write(uint16_t memAddr, uint8_t value) {
      Wire.beginTransmission(_addr);
      Wire.write((uint8_t)(memAddr >> 8));   // Send High Byte of address
      Wire.write((uint8_t)(memAddr & 0xFF)); // Send Low Byte of address
      Wire.write(value);                     // Send the actual data
      Wire.endTransmission();                // Stop talking
    }
    // Function to read a single byte from a specific address
    uint8_t read(uint16_t memAddr) {
      Wire.beginTransmission(_addr);
      Wire.write((uint8_t)(memAddr >> 8));   // Send High Byte of address
      Wire.write((uint8_t)(memAddr & 0xFF)); // Send Low Byte of address
      Wire.endTransmission();
      Wire.requestFrom(_addr, (uint8_t)1);   // Request 1 byte back
      if (Wire.available()) return Wire.read(); // If data comes, return it
      return 0; // If fail, return 0
    }
};

// Create "objects" for each hardware device so we can control them
RH_RF24 radio(PIN_RADIO_CS, PIN_RADIO_IRQ, PIN_RADIO_SDN); // The Radio
Adafruit_INA219 pwr;           // The Power Sensor
Adafruit_BMP085 bmp;           // The Pressure Sensor
Adafruit_MPU6050 imu;          // The IMU (Gyro/Accel)
Generic_I2C_Memory fram;       // The Memory Chip

// --- GLOBAL VARIABLES ---
bool imuHealthy = false;       // Flag to remember if the IMU is broken or working

// --- OOK CONFIGURATION (Morse Code Settings) ---
// This is a "Register Dump" provided by the radio manufacturer to enable OOK (On-Off Keying)
// It tells the radio chip exactly how to pulse electricity to make Morse Code.
RH_RF24::ModemConfig OOKAsync = {
  0x89, 0x00, 0xc3, 0x50, 0x01, 0x00, 0x00, 0x00, 0x00, 0x34,
  0x10, 0x00, 0x3f, 0x08, 0x31, 0x27, 0x04, 0x10, 0x02, 0x12,
  0x00, 0x2c, 0x03, 0xf9, 0x62, 0x11, 0x0e, 0x0e, 0x00, 0x02,
  0xff, 0xff, 0x00, 0x27, 0x00, 0x00, 0x07, 0xff, 0x40, 0xcc,
  0xa1, 0x30, 0xa0, 0x21, 0xd1, 0xb9, 0xc9, 0xea, 0x05, 0x12,
  0x11, 0x0a, 0x04, 0x15, 0xfc, 0x03, 0x00, 0xcc, 0xa1, 0x30,
  0xa0, 0x21, 0xd1, 0xb9, 0xc9, 0xea, 0x05, 0x12, 0x11, 0x0a,
  0x04, 0x15, 0xfc, 0x03, 0x00, 0x3f, 0x2c, 0x0e, 0x04, 0x0c,
  0x73
};

// --- TELEMETRY PACKET STRUCTURE ---
// This defines exactly what one "Chunk" of data looks like when sent over radio
struct TelemetryPacket {
    uint8_t  startByte;      // A marker (0xFE) to say "Packet starts here"
    uint16_t packetCount;    // ID number of the packet (1, 2, 3...)
    uint32_t timestamp;      // Time since boot (in milliseconds)
    uint8_t  mode;           // Current Mode (2=Normal, 1=Low, 0=Crit)
    uint8_t  deploy_status;  // 1 = Deployed, 0 = Stowed
    uint16_t v_batt_mv;      // Battery voltage in millivolts
    int16_t  current_ma;     // Current usage in milliamps
    int16_t  temp_obc_c;     // Board temperature
    int16_t  temp_ext_c;     // External temperature
    uint16_t pressure_hpa;   // Atmospheric pressure
    uint16_t light_val;      // Light sensor reading (0-1023)
    int16_t  fused_pitch;    // Pitch angle x100 (to send as integer)
    int16_t  fused_roll;     // Roll angle x100
    uint16_t crc;            // Checksum (Math to verify data isn't corrupted)
} __attribute__((packed));   // Tells compiler "Don't add empty spaces between variables"

TelemetryPacket dataPacket;      // Create an instance of the packet to fill with data
uint16_t globalPacketCount = 0;  // Counter for how many packets we've sent
uint16_t framLogAddr = MEM_DATA_START; // Variable to track where we are writing in memory

// Internal struct to hold raw sensor numbers (floats) before we pack them
struct RawSensorData {
    float v_batt;
    float current;
    float temp_obc;
    float temp_ext;
    float pressure;
    uint16_t light;
    uint8_t deploy_status;
    float ax, ay, az; // Accelerometer X, Y, Z
    float gx, gy, gz; // Gyroscope X, Y, Z
} rawData;

// --- MORSE CODE DICTIONARY ---
// A look-up table. If I want 'A', this table gives me ".-"
static const struct {const char letter, *code;} MorseMap[] =
{
  { 'A', ".-" },    { 'B', "-..." },  { 'C', "-.-." },
  { 'D', "-.." },   { 'E', "." },     { 'F', "..-." },
  { 'G', "--." },   { 'H', "...." },  { 'I', ".." },
  { 'J', ".---" },  { 'K', "-.-" },  { 'L', ".-.." },
  { 'M', "--" },    { 'N', "-." },    { 'O', "---" },
  { 'P', ".--." },  { 'Q', "--.-" },  { 'R', ".-." },
  { 'S', "..." },   { 'T', "-" },     { 'U', "..-" },
  { 'V', "...-" },  { 'W', ".--" },   { 'X', "-..-" },
  { 'Y', "-.--" },  { 'Z', "--.." },
  { '0', "-----" }, { '1', ".----" }, { '2', "..---" },
  { '3', "...--" }, { '4', "....-" }, { '5', "....." },
  { '6', "-...." }, { '7', "--..." }, { '8', "---.." },
  { '9', "----." }, { '.', ".-.-.-" },{ '-', "-....-" },
  { ' ', " " }
};

// --- WATCHDOG FUNCTION ---
// The external Watchdog Timer will reset the whole system if we don't toggle Pin 4 often.
void kickWatchdog() {
   static bool s = 0;
   digitalWrite(PIN_WDI, s); // Toggle pin HIGH/LOW
   s = !s;                   // Flip state for next time
}

// Interrupt Service Routine for Watchdog (Required by AVR library, even if empty)
ISR(WDT_vect) { }

// --- DEEP SLEEP FUNCTION ---
// Puts the microcontroller into a "Coma" to save power for X milliseconds.
void deepSleepDelay(unsigned long ms) {
    unsigned long loops = ms / 1000; // Convert milliseconds to seconds (loops)
    if (loops == 0) loops = 1;       // Sleep at least 1 second

    Serial.print(F("SLEEP: "));      // Debug message
    Serial.println(loops);
    Serial.flush();                  // Ensure message is sent before sleeping

    for (unsigned long i = 0; i < loops; i++) {
        kickWatchdog(); // Tell watchdog "I'm still alive" before napping
        
        // AVR Magic commands to configure sleep registers:
        MCUSR &= ~(1 << WDRF); 
        WDTCSR |= (1 << WDCE) | (1 << WDE);
        WDTCSR = (1 << WDIE) | (1 << WDP2) | (1 << WDP1); // Set internal wakeup timer ~1 sec
        
        set_sleep_mode(SLEEP_MODE_PWR_DOWN); // Select deepest sleep mode
        sleep_enable();
        sleep_bod_disable(); // Turn off "Brown Out Detection" to save more power
        sleep_cpu();         // actually Go To Sleep Here!
        
        // ... CPU Wakes up here after 1 second ...
        sleep_disable();
        
        // Reset Watchdog config
        WDTCSR |= (1 << WDCE) | (1 << WDE);
        WDTCSR = 0x00;
    }
    Serial.begin(115200); // Restart Serial port (it shuts down during sleep)
    delay(10); 
}

// --- CRC CALCULATION ---
// Maths to generate a "checksum" to detect data errors
uint16_t calculateCRC(uint8_t *data, size_t len) {
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (uint8_t j = 0; j < 8; j++) {
            if (crc & 0x0001) crc = (crc >> 1) ^ 0xA001;
            else crc >>= 1;
        }
    }
    return crc;
}

// --- DEPLOYMENT CHECK ---
// Reads the deployment switch to see if the antenna is open
bool isAntennaDeployed() {
    int reading = analogRead(PIN_DEPLOY_SENSE);
    return (reading > 500); // If switch is open, analog value goes HIGH
}

// --- MORSE TRANSMIT FUNCTION ---
// Takes a single character, finds its Morse code, and blinks the radio
void transmitMorseChar(char c) {
  kickWatchdog(); 
  
  if (c == ' ') { // If Space
      Serial.print("   "); 
      delay(UNIT_LENGTH * 4); // Wait 4 units
      return; 
  }
  
  const char* code = "";
  // Find character in our Morse Map array
  for( unsigned int j = 0; j < sizeof(MorseMap) / sizeof(*MorseMap); ++j ) {
      if( toupper(c) == MorseMap[j].letter ) { code = MorseMap[j].code; break; }
  }
  if (strlen(code) == 0) return; // If char not found, skip

  // Iterate through dots and dashes
  for (int i=0; code[i] != 0; i++) {
      kickWatchdog(); 
      
      Serial.print(code[i]); // Print dot/dash to debug
      
      digitalWrite(PIN_LED_VISUAL, HIGH); // LED ON
      radio.setTxPower(TX_POWER_MIN);// <<Key down (Carrier ON)
      
      // Hold for correct duration
      if (code[i] == '.') delay(UNIT_LENGTH);       // Dot = 1 unit
      else delay(UNIT_LENGTH * 3);                  // Dash = 3 units
      
      radio.setTxPower(0x00); // Key up (Carrier OFF)
      digitalWrite(PIN_LED_VISUAL, LOW);  // LED OFF
      
      delay(UNIT_LENGTH); // Wait 1 unit between parts of a letter
  }
  Serial.print(" "); // Space in debug log
  
  // Wait 3 units between letters (keeping watchdog happy)
  unsigned long startGap = millis();
  while (millis() - startGap < (UNIT_LENGTH * 3)) {
      kickWatchdog();
  }
}

// Helper to send a whole sentence in Morse
void transmitMorseString(const char* str) {
  for(int i=0; str[i] != 0; i++) transmitMorseChar(str[i]);
}

// --- SENSOR FUSION ---
// Reads IMU and calculates Pitch/Roll using a Complementary Filter
void updateAttitude() {
    // If battery is critically low, skip this math to save power
    if (rawData.v_batt < V_CRIT_THRESH && rawData.v_batt > 0.1) {
        return; 
    }

    if (!imuHealthy) return; // Don't read broken sensor

    imu.enableSleep(false); // Wake up IMU
    
    sensors_event_t a, g, t;
    if (imu.getEvent(&a, &g, &t)) { // Read Accel, Gyro, Temp
        
        unsigned long currentTime = millis();
        float dt = (currentTime - lastImuTime) / 1000.0; // Calculate time passed
        lastImuTime = currentTime;

        if (dt > 1.0) dt = 0.05; // Safety check for huge time gaps

        // Calculate angle from Accelerometer (Gravity vector)
        float accelPitch = atan2(a.acceleration.y, a.acceleration.z) * RAD_TO_DEG;
        float accelRoll  = atan2(-a.acceleration.x, sqrt(a.acceleration.y * a.acceleration.y + a.acceleration.z * a.acceleration.z)) * RAD_TO_DEG;

        // Get Gyro rates
        float gyroPitchRate = g.gyro.x * RAD_TO_DEG;
        float gyroRollRate  = g.gyro.y * RAD_TO_DEG;

        // FUSE THEM: Combine reliable slow Accel with fast Gyro
        pitch = alpha * (pitch + gyroPitchRate * dt) + (1.0 - alpha) * accelPitch;
        roll  = alpha * (roll  + gyroRollRate  * dt) + (1.0 - alpha) * accelRoll;

        // Store raw values for telemetry
        rawData.temp_obc = t.temperature;
        rawData.ax = a.acceleration.x; rawData.ay = a.acceleration.y; rawData.az = a.acceleration.z;
        rawData.gx = g.gyro.x; rawData.gy = g.gyro.y; rawData.gz = g.gyro.z;
    }
    
    imu.enableSleep(true); // Put IMU back to sleep
}

// --- DATA GATHERING ---
// Reads all sensors and packs them into the data structure
void preparePacketAndTelemetry() {
    radio.setModeTx(); // Turn on Radio TX briefly to load the battery
    delay(10); 
    
    rawData.current = pwr.getCurrent_mA(); // Read Current under load
    
    radio.setModeIdle(); // Turn off Radio

    // Read remaining sensors
    rawData.v_batt = pwr.getBusVoltage_V();
    rawData.temp_ext = bmp.readTemperature();
    rawData.pressure = bmp.readPressure();
    rawData.light = analogRead(PIN_LIGHT_SENS);
    rawData.deploy_status = (isAntennaDeployed()) ? 1 : 0;
    
    // Debug Print
    Serial.print(F("MODE: "));
    if (rawData.v_batt >= V_LOW_THRESH) Serial.print(F("NORMAL"));
    else if (rawData.v_batt >= V_CRIT_THRESH) Serial.print(F("LOW POWER"));
    else Serial.print(F("CRITICAL"));
    Serial.print(F(" | Batt: ")); Serial.print(rawData.v_batt); Serial.print(F("V"));
    Serial.println();

    rawData.temp_obc = 0; 
    
    updateAttitude(); // Run sensor fusion math
    Serial.print(F("ATTITUDE: P=")); Serial.print(pitch);
    Serial.print(F(" R=")); Serial.println(roll);

    // Pack data into the binary packet structure
    dataPacket.startByte = 0xFE;
    dataPacket.packetCount = globalPacketCount++;
    dataPacket.timestamp = millis();

    // Set Mode Flag
    if (rawData.v_batt >= V_LOW_THRESH) dataPacket.mode = 2; 
    else if (rawData.v_batt >= V_CRIT_THRESH) dataPacket.mode = 1; 
    else dataPacket.mode = 0; 

    // Convert Floats to Integers to save space in packet
    dataPacket.deploy_status = rawData.deploy_status;
    dataPacket.v_batt_mv  = (uint16_t)(rawData.v_batt * 1000);
    dataPacket.current_ma = (int16_t)(rawData.current);
    dataPacket.temp_obc_c = (int16_t)(rawData.temp_obc * 100);
    dataPacket.temp_ext_c = (int16_t)(rawData.temp_ext * 100);
    dataPacket.pressure_hpa = (uint16_t)(rawData.pressure / 100);
    dataPacket.light_val = rawData.light;
    
    dataPacket.fused_pitch = (int16_t)(pitch * 100);
    dataPacket.fused_roll  = (int16_t)(roll * 100);

    // Calculate Checksum
    dataPacket.crc = calculateCRC((uint8_t*)&dataPacket, sizeof(dataPacket) - 2);
}

// --- MORSE BEACON ---
// Formats sensor data into a text string and transmits it via Morse
void transmitAllSensorsMorse() {
    Serial.print(F("[TX] Morse: "));
    
    char morseBuf[100];
    // Create string: "PQ V3900 C15 T25..."
    snprintf(morseBuf, sizeof(morseBuf), "%s V%d C%d T%d P%d L%d O%d",
             MY_CALLSIGN,
             dataPacket.v_batt_mv / 100,
             dataPacket.current_ma,
             dataPacket.temp_ext_c / 100,
             dataPacket.pressure_hpa,
             dataPacket.light_val,
             dataPacket.fused_pitch / 100
             );

    radio.setModemRegisters(&OOKAsync); // Load OOK Radio Config
    
    // Set Power to Minimum to prevent crashing the battery during long Morse transmissions
    radio.setTxPower(0x00); 
    radio.setModeTx();
    delay(5);

    Serial.println(); 
    Serial.print(F("Readable: "));
    Serial.println(morseBuf);
    Serial.flush();

    transmitMorseString(morseBuf); // Send it!
    
    radio.sleep(); // stop transmitter cleanly
}

// --- GFSK PACKET TX ---
// Sends the binary data packet (High Speed Data)
void transmitGFSKPacket(uint8_t powerLevel) {
    Serial.print(F("[TX] GFSK (Size: ")); Serial.print(sizeof(dataPacket)); Serial.print(F(")... "));
    if (!radio.init()) Serial.print(" (Reset)"); // Re-init radio if it got confused
    radio.setFrequency(RADIO_FREQ);
    radio.setModemConfig(RH_RF24::GFSK_Rb5Fd10);
    radio.setTxPower(powerLevel); 
    
    radio.send((uint8_t*)&dataPacket, sizeof(dataPacket)); // Send binary data
    delay(150); // Wait for transmission to finish
    radio.sleep(); // Sleep the radio chip
    Serial.println(F("[SENT]"));
}

// --- ANTENNA DEPLOYMENT ---
// The sequence to burn the wire and release the antenna
void runDeploymentSequence() {
    Serial.println(F("!!! STARTING SLOW DEPLOYMENT (30s Pulsed) !!!"));
    
    // 1. Initial Safety Wait
    unsigned long startWait = millis();
    while (millis() - startWait < DEPLOY_WAIT_MS) { kickWatchdog(); }

    // Check FRAM: Have we tried this too many times?
    uint8_t previousAttempts = fram.read(MEM_BURN_COUNT);
    if (previousAttempts >= MAX_BURN_ATTEMPTS) {
        Serial.println(F("[DEP] MAX ATTEMPTS EXCEEDED. ABORTING."));
        return; 
    }

    // 2. Start Attempt Loop
    for (int attempt = 1; attempt <= MAX_BURN_ATTEMPTS; attempt++) {
        kickWatchdog();
        
        // Log attempt count to FRAM
        if (previousAttempts < 255) fram.write(MEM_BURN_COUNT, previousAttempts + attempt);

        // Check if already open
        if (isAntennaDeployed()) {
            Serial.println(F("[DEP] CONFIRMED DEPLOYED."));
            fram.write(MEM_DEPLOY_FLAG, 0xAA); // Write Success Flag
            break; 
        }

        Serial.print(F("[DEP] Attempt ")); Serial.print(attempt);
        Serial.println(F(" - PULSING BURN WIRE (Slow Heat)..."));

        digitalWrite(PIN_LED_VISUAL, HIGH);
        
        // 3. THE SLOW BURN LOOP (30 Seconds)
        unsigned long burnStart = millis();
        while(millis() - burnStart < BURN_CYCLE_MS) { 
             kickWatchdog(); 
             
             // --- PULSE LOGIC (Heat management) ---
             digitalWrite(PIN_ANT_BURN, HIGH); // Heat ON
             delay(30);                        // Wait 30ms (Short Burst)
             
             digitalWrite(PIN_ANT_BURN, LOW);  // Heat OFF
             delay(70);                        // Wait 70ms (Cool/Rest)
             // -------------------------------------

             // If it snaps mid-burn, stop immediately
             if (isAntennaDeployed()) {
                 Serial.println(F("[DEP] SNAP DETECTED! Stopping early."));
                 break; 
             }
        }
        
        digitalWrite(PIN_ANT_BURN, LOW); // Ensure Heat is OFF
        digitalWrite(PIN_LED_VISUAL, LOW);

        if (isAntennaDeployed()) {
            fram.write(MEM_DEPLOY_FLAG, 0xAA);
            break; 
        }
        
        Serial.println(F("[DEP] Cooling down..."));
        for(int i=0; i<50; i++) { delay(100); kickWatchdog(); } // Cool down delay
    }
    
    Serial.println(F("!!! DEPLOYMENT END !!!"));
}

// --- SETUP (Runs Once at Power On) ---
void setup() {
    pinMode(PIN_WDI, OUTPUT);
    pinMode(PIN_LED_VISUAL, OUTPUT);
    pinMode(PIN_ANT_BURN, OUTPUT);
    digitalWrite(PIN_ANT_BURN, LOW); // Ensure Burn Wire is OFF
    
    Serial.begin(115200);
    Wire.begin();
    kickWatchdog(); // Start Watchdog heartbeat

    Serial.println(F("--- SYSTEM START ---"));

    pwr.begin(); // Start Power Sensor
    float startupVolts = pwr.getBusVoltage_V();
    
    bmp.begin(); // Start Pressure Sensor
    // Start IMU, if fails, mark as unhealthy
    if (!imu.begin(0x68)) { Serial.println(F("IMU FAIL")); imuHealthy = false; } 
    else { Serial.println(F("IMU OK")); imuHealthy = true; imu.enableSleep(true); }

    lastImuTime = millis();

    // Check Memory (FRAM)
    if (fram.begin(0x50)) {
        Serial.println(F("FRAM Found!"));
        fram.write(MEM_BURN_COUNT, 0); // Reset burn counter (for testing)
        
        // Check Deployment Flag
        uint8_t flagVal = fram.read(MEM_DEPLOY_FLAG);
        bool flagSet = (flagVal == 0xAA);
        bool physicallyDeployed = isAntennaDeployed();
        
        // If Logic says "Stowed" but Voltage is good -> Try Deploying
        if (!flagSet && !physicallyDeployed) {
             if (startupVolts >= V_BURN_SAFE) {
                 runDeploymentSequence();
             } else {
                 Serial.print(F("SKIP DEPLOYMENT: Low Voltage"));
             }
        } 
        // If physically open but flag not set -> Update flag
        else if (physicallyDeployed && !flagSet) {
             fram.write(MEM_DEPLOY_FLAG, 0xAA);
        }
        
        // Restore Memory Log Pointer
        uint16_t savedAddr = (fram.read(MEM_PTR_ADDR + 1) << 8) | fram.read(MEM_PTR_ADDR);
        if (savedAddr >= MEM_DATA_START && savedAddr < MEM_MAX_ADDR) framLogAddr = savedAddr;
    }

    // Initialize Radio
    pinMode(PIN_RADIO_SDN, OUTPUT); digitalWrite(PIN_RADIO_SDN, LOW); delay(100);
    if (!radio.init()) radio.init(); radio.setFrequency(RADIO_FREQ); radio.sleep();
}

// --- MAIN LOOP (Runs Forever) ---
void loop() {
    kickWatchdog();
    
    pwr.begin(); 
    float voltage = pwr.getBusVoltage_V(); // Check Battery
    
    // --- MODE SELECTION ---
    
    // 1. NORMAL MODE (Battery > 3.4V)
    if (voltage >= V_LOW_THRESH) {
        Serial.println(F("--- STARTING 10s GFSK BURST ---"));
        
        unsigned long startTime = millis();
        // Loop for 10 seconds sending fast data packets
        while (millis() - startTime < TIME_GFSK_DURATION_MS) {
            kickWatchdog(); 
            digitalWrite(PIN_LED_VISUAL, HIGH);
            
            preparePacketAndTelemetry(); 
            transmitGFSKPacket(TX_POWER_MAX); // Max Power!

            // Save data to FRAM memory
            uint8_t *ptr = (uint8_t *)&dataPacket;
            for (uint16_t j = 0; j < sizeof(dataPacket); j++) {
                fram.write(framLogAddr + j, ptr[j]);
            }
            // Update memory address pointer
            framLogAddr += sizeof(dataPacket);
            if (framLogAddr > MEM_MAX_ADDR) framLogAddr = MEM_DATA_START; // Wrap around if full
            fram.write(MEM_PTR_ADDR, lowByte(framLogAddr));
            fram.write(MEM_PTR_ADDR + 1, highByte(framLogAddr));

            digitalWrite(PIN_LED_VISUAL, LOW);
            
            // Wait 1 second between packets
            for (int d = 0; d < 10; d++) { delay(100); kickWatchdog(); }
        }

        Serial.println(F("[TX] 10s Window Complete. Sending Morse..."));
        delay(1000); 
        kickWatchdog();
        
        // Send One Morse Beacon (Low Power Safe)
        transmitAllSensorsMorse();
        
        // Sleep for 30 seconds
        deepSleepDelay(TIME_NORM_SEC * 1000UL);
    }
    
    // 2. LOW POWER MODE (Battery 3.2V - 3.4V)
    else if (voltage >= V_CRIT_THRESH) {
        digitalWrite(PIN_LED_VISUAL, HIGH);
        preparePacketAndTelemetry();
        digitalWrite(PIN_LED_VISUAL, LOW);
        
        transmitGFSKPacket(TX_POWER_MAX); // Send only ONE data packet
        
        transmitAllSensorsMorse(); // Send Morse Beacon
        
        deepSleepDelay(TIME_LOW_SEC * 1000UL); // Sleep 5 minutes
    }
    
    // 3. CRITICAL MODE (Battery < 3.2V)
    else {
        preparePacketAndTelemetry();
        Serial.println(F("[TX] CRIT MODE (BEACON ONLY)"));
        
        transmitAllSensorsMorse(); // Send ONLY Morse (Saves power)
        deepSleepDelay(TIME_CRIT_SEC * 1000UL); // Sleep 10 minutes
    } 
}