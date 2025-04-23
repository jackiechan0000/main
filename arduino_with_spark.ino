// Define digital output pins for solenoids valves
int v1 = 13;
int v2 = 6;
int v3 = 14;
int v4 = 21;

// Pin used to trigger a coil
int coil = 4; 

// Analog input pins for pressure transducers
int OPD02  = A7;
int pt2  = A8;
int OPD01  = A10;
int pt4  = A5;
int EPD01  = A2;
int FPD01 = A14;

// Raw analog values
int d1, d2, d3, d4, d5;

// Number of samples to average for pressure reading
int pres_samples = 10;

// Pressure transducer full-scale pressure ratings
int Pmax = 500;     // PSI for voltage-output sensors
int Pmax1k = 1000;  // PSI for current-output sensors

// Transducer electrical characteristics
float R = 250;      // Load resistor for 4-20 mA sensors
float I0 = 0.004;   // 4 mA minimum
float Imax = 0.02;  // 20 mA maximum
float V0 = 0.5;     // Voltage transducer min output (V)
float Vmax = 4.5;   // Voltage transducer max output (V)

// Pressure smoothing and offset calibration variables
float pres_sum, pres_sum1, pres_sum2, pres_sum3, pres_sum4, pres_sum5;
float sum1, sum2, sum3, sum4, sum5;
float offset, offset1, offset2, offset3, offset4, offset5;

unsigned long startTime;

// Flags to track time-based sequence steps for test sequencing
bool did0s = false;
bool did20s = false;
bool did21s = false;
bool did29_95s = false;
bool did30s = false;
bool did30_5s = false;
bool did10s = false;
bool did15s = false;

void setup() {
  // Set valve and coil pins as outputs
    pinMode(v1, OUTPUT);
    pinMode(v2, OUTPUT);
    pinMode(v3, OUTPUT);
    pinMode(v4, OUTPUT);
    pinMode(coil, OUTPUT);

  // Set pressure transducer pins as inputs
    pinMode(OPD02, INPUT);
    pinMode(pt2, INPUT);
    pinMode(OPD01, INPUT);
    pinMode(FPD01, INPUT);
    pinMode(EPD01, INPUT);

  // Allow sensors to stabilize
    delay(3000);

  // Calibrate voltage-based sensors (pt3 and pt5)
    for (int i = 0; i < 100; i++) {
        double d1 = analogRead(OPD01);
        double d2 = analogRead(EPD01);
        float Vread1 = d1 * (5.0 / 1024.0);
        float Vread2 = d2 * (5.0 / 1024.0);
        float pressure1 = Pmax * ((Vread1 - V0) / (Vmax - V0));
        float pressure2 = Pmax * ((Vread2 - V0) / (Vmax - V0));
        sum1 += pressure1;
        sum2 += pressure2;
    }

   // Compute ambient offsets to convert gauge to absolute pressure
    offset1 = 14.7 - (sum1 / 100);
    offset2 = 14.7 - (sum2 / 100);

   // Calibrate current-based sensors (pt1, pt2, pt4)
    for (int i = 0; i < 100; i++) {
        float d3 = analogRead(OPD02);
        float d4 = analogRead(FPD01);
        float d5 = analogRead(pt4);
        float Vread3 = d3 * (5.0 / 1024.0);
        float Vread4 = d4 * (5.0 / 1024.0);
        float Vread5 = d5 * (5.0 / 1024.0);
        float pressure3 = (Pmax1k * (Vread3 - (I0 * R))) / (R * (Imax - I0));
        float pressure4 = (Pmax1k * (Vread4 - (I0 * R))) / (R * (Imax - I0));
        float pressure5 = (Pmax1k * (Vread5 - (I0 * R))) / (R * (Imax - I0));
        sum3 += pressure3;
        sum4 += pressure4;
        sum5 += pressure5;
    }

    offset3 = 14.7 - (sum3 / 100);
    offset4 = 14.7 - (sum4 / 100);
    offset5 = 14.7 - (sum5 / 100);

   //This probably should be changed to 115200 but then the pi
   //should also be changed
    Serial.begin(9600);

}

void loop() {


  //checks for avaialable data from pi
  if (Serial.available()) 
  {
        //reads 1 character from pi
        char c = Serial.read();
        delay(1); // slight pause for command stability
        //Serial.print('Arduino recieved command'); // echo received command

        // Manual valve controls via serial commands
        if (c == '1') digitalWrite(v1, HIGH);
        if (c == '!') digitalWrite(v1, LOW);
        if (c == '2') digitalWrite(v2, HIGH);
        if (c == '@') digitalWrite(v2, LOW);
        if (c == '3') digitalWrite(v3, HIGH);
        if (c == '#') digitalWrite(v3, LOW);
        if (c == '4') digitalWrite(v4, HIGH);
        if (c == '$') digitalWrite(v4, LOW);

        if (c == '9') {
            long start_time = millis();
            long endtime = start_time;
            void while((endtime-start_time) <= 300) {
                digitalWrite(coil, HIGH);
                delay(16.667)
                digitalWrite(coil, LOW);
                delay(1)

                endtime = millis();
            }
            
        }

        // Read data from valvues
        if (c == '5') {
            float pressure = ReadOPD01();
            Serial.println(pressure);  // Send value back to Pi
        }
        if (c == '6') {
            float pressure = ReadOPD02();
            Serial.println(pressure);
        }
        if (c == '7') {
            float pressure = ReadEPD01();
            Serial.println(pressure);
        }
        if (c == '8') {
            float pressure = ReadFPD01();
            Serial.println(pressure);
        }
   }

   
}

// ------------------- TEST SEQUENCES -------------------

// Sequence of valve/pressure actions at specific times
void dry_test() {
    unsigned long startTime = millis();
    while (!did30_5s) {
        unsigned long currentTime = millis();
        float elapsedSeconds = (currentTime - startTime) / 1000.0;

        if (!did0s && elapsedSeconds >= 0) {
            // Start_Count(); // Placeholder
            did0s = true;
        }

        if (!did10s && elapsedSeconds >= 10.0) {
            ReadOPD01();
            ReadOPD02();
            ReadFPD01();
            ReadEPD01();
            FV_02_CLOSE();
            did10s = true;
        }

        if (!did15s && elapsedSeconds >= 15.0) {
            NV_02_OPEN();
            did15s = true;
        }

        if (!did29_95s && elapsedSeconds >= 29.95) {
            OV_03_OPEN();
            did29_95s = true;
        }

        if (!did30s && elapsedSeconds >= 30.0) {
            FV_03_OPEN();
            did30s = true;
        }

        if (!did30_5s && elapsedSeconds >= 30.5) {
            BLP_Abort(); // Safety or abort shutdown
            did30_5s = true;
        }
    }
}

// ------------------- PRESSURE SENSOR FUNCTIONS -------------------

// Read pressure from OPD (pt1)
float ReadOPD02() {
    pres_sum3 = 0;
    for (int i = 0; i < pres_samples; i++) {
        // 5 V / 1024 bits
        float Vread = analogRead(OPD02) * (5.0 / 1024.0);
        float pressure = (Pmax1k * (Vread - (I0 * R))) / (R * (Imax - I0));
        pres_sum3 += pressure;
    }
    return pres_sum3 / pres_samples;
}
float ReadOPD01() {
    pres_sum3 = 0;
    for (int i = 0; i < pres_samples; i++) {
        // 5 V / 1024 bits
        float Vread = analogRead(OPD01) * (5.0 / 1024.0);
        float pressure = (Pmax1k * (Vread - (I0 * R))) / (R * (Imax - I0));
        pres_sum3 += pressure;
    }
    return pres_sum3 / pres_samples;
}


// Read pressure from FPD (pt4)
float ReadFPD01() {
    pres_sum3 = 0;
    for (int i = 0; i < pres_samples; i++) {
        // 5 V / 1024 bits
        float Vread = analogRead(FPD01) * (5.0 / 1024.0);
        float pressure = (Pmax1k * (Vread - (I0 * R))) / (R * (Imax - I0));
        pres_sum3 += pressure;
    }
    return pres_sum3 / pres_samples;
}

// Read pressure from EPD (pt5)
float ReadEPD01() {
    pres_sum2 = 0;
    for (int i = 0; i < pres_samples; i++) {
        // 5 V / 1024 bits
        float Vread = analogRead(EPD01) * (5.0 / 1024.0);
        float pressure = Pmax * ((Vread - V0) / (Vmax - V0));
        pres_sum2 += pressure;
    }
    return pres_sum2 / pres_samples;
}

// ------------------- VALVE CONTROL -------------------

void OV_03_OPEN()  { digitalWrite(v3, HIGH); }
void FV_03_OPEN()  { digitalWrite(v2, HIGH); }
void NV_02_OPEN()  { digitalWrite(v4, HIGH); }
void FV_02_OPEN()  { digitalWrite(v1, HIGH); }
void FV_02_CLOSE() { digitalWrite(v1, LOW); }
void OV_03_CLOSE() { digitalWrite(v3, LOW); }
void FV_03_CLOSE() { digitalWrite(v2, LOW); }
void NV_02_CLOSE() { digitalWrite(v4, LOW); }

// ------------------- ABORT SEQUENCE -------------------
void BLP_Abort() {
    OV_03_CLOSE();
    FV_03_CLOSE();
    FV_02_OPEN();  // Unusual: opens instead of closing? Confirm logic
    NV_02_CLOSE();
}
