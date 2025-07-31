#include <ModbusMaster.h>
#include <SPI.h>
#include <SD.h>
#include <Wire.h>
#include <Adafruit_INA219.h>
#include <ArduinoJson.h>

// Pin MAX485
#define MAX485_DE      PA0
#define MAX485_RE      PA1

// Pin Relay
#define RELAY_Inv      PB15
#define RELAY_Batt     PB14
#define RELAY_ATS_N    PB13
#define RELAY_ATS_F    PB12

// SPI
#define SPI1_NSS_PIN PA4  // SPI_1 digunakan untuk komunikasi dengan SD Card


// Alamat slave untuk PZEM
static uint8_t pzemSlaveAddrPanel = 0x01;
static uint8_t pzemSlaveAddrBattery = 0x02;
static uint8_t pzemSlaveAddrAC = 0x03;

// Objek ModbusMaster
ModbusMaster nodePanel;
ModbusMaster nodeBattery;
ModbusMaster nodeAC;
Adafruit_INA219 ina219;
 
// Variabel untuk PZEM Panel
float PZEMVoltagePanel = 0.00, PZEMCurrentPanel = 0.00, PZEMPowerPanel = 0.00, PZEMEnergyPanel = 0.00;

// Variabel untuk PZEM Baterai
float PZEMVoltageBattery = 0.00, PZEMCurrentBattery = 0.00, PZEMPowerBattery = 0.00, PZEMEnergyBattery = 0.00;

// Variable untuk PSEM AC
float voltageAC, currentAC, powerAC, energyAC, frequencyAC, powerFactorAC;

// Variabel untuk PZEM AC PLTS
float PLTSVoltage = 0.00, PLTSCurrent = 0.00, PLTSPower = 0.00, PLTSEnergy = 5, PLTSHz = 0.00, PLTSPf = 0.00;

// Variabel untuk PZEM AC Grid
float GridVoltage = 0.00, GridCurrent = 0.00, GridPower = 0.00, GridEnergy = 4, GridHz = 0.00, GridPf = 0.00;

// Variabel untuk INA219
float ShuntVoltage = 0.00, INA219Voltage = 0.00, INA219Current = 0.00;

// Variabel untuk BMS
String receivedData = "";
float total_voltage = 0.0, total_current = 0.0, power = 0.0, temp1 = 0.0, temp2 = 0.0, v1 = 0.0, v2 = 0.0, v3 = 0.0, v4 = 0.0, soc = 0.0;

// Initial SOC
bool soc_initialized = false;  // Global flag

//Initial Waktu
bool waktu_initialized = false;
char waktu[10];
int jam = 0, menit = 0, detik = 0, hour = 0, minute = 0, second = 0;

// Pengolahan Data
int previousStatus = -1;  // Inisialisasi dengan nilai tidak valid
int count = 0, SOCt = 0, Ah = 100, interval = 0;
float deltaT = 0.0, SOCo = 0.0, kapasitas = 0.0;

// Status Alat Terkini
int statusbatt = 0, statuscsv = 0;

// Variabel global yang dibutuhkan
bool isPLTS = false;                  // true = relay ON (pakai PLTS)
bool waitToTurnOn = false;           // status delay sebelum ON
unsigned long timeToTurnOn = 0;      // waktu mulai delay

//Timer untuk Pembaharuan Data
unsigned long startMillisPZEM;
unsigned long currentMillisPZEM;
const unsigned long periodPZEM = 1000; // 1 detik

//Timer untuk ESP
unsigned long startMillis;
unsigned long currentMillis;
const unsigned long period = 5000;  // 5 detik

//Timer untuk SD CARD
unsigned long startMillisSOC;
unsigned long currentMillisSOC;
const unsigned long periodSOC = 600000;  // 10 menit

unsigned long baseTime = 0;      // waktu dasar dalam detik
unsigned long baseMillis = 0;    // millis saat waktu diterima

void setup() {
  // Serial Monitor
  Serial.begin(9600);
  while (!Serial);

  // Komunikasi Antar Perangkat
  Serial2.begin(9600, SERIAL_8N2); // Untuk komunikasi PZEM
  Serial3.begin(9600); // Untuk komunikasi dengan ESP
  Serial.println("STM32 siap mengirim dan menerima data");

  pinMode(SPI1_NSS_PIN, OUTPUT);

  // Inisialisasi SD Card
  Serial.print("Initializing SD card...");
  delay(500);
  if (!SD.begin(SPI1_NSS_PIN)) {
    Serial.println("initialization failed!");
    while (1);
  }
  Serial.println("initialization done.");

  // Konfigurasi MAX485
  pinMode(MAX485_RE, OUTPUT);
  pinMode(MAX485_DE, OUTPUT);
  digitalWrite(MAX485_RE, 0);
  digitalWrite(MAX485_DE, 0);

  // Konfigurasi Relay
  pinMode(RELAY_ATS_F, OUTPUT);
  pinMode(RELAY_Inv, OUTPUT);
  pinMode(RELAY_Batt, OUTPUT);
  pinMode(RELAY_ATS_N, OUTPUT);

  digitalWrite(RELAY_ATS_F, HIGH);
  digitalWrite(RELAY_Inv, HIGH);
  digitalWrite(RELAY_Batt, HIGH);
  digitalWrite(RELAY_ATS_N, HIGH);

  // Inisialisasi Modbus
  nodePanel.begin(pzemSlaveAddrPanel, Serial2);
  nodePanel.preTransmission(preTransmission);
  nodePanel.postTransmission(postTransmission);

  nodeBattery.begin(pzemSlaveAddrBattery, Serial2);
  nodeBattery.preTransmission(preTransmission);
  nodeBattery.postTransmission(postTransmission);

  nodeAC.begin(pzemSlaveAddrAC, Serial2);
  nodeAC.preTransmission(preTransmission);
  nodeAC.postTransmission(postTransmission);

  // // Inisialisasi INA219
  // if (!ina219.begin()) {
  //   Serial.println("Failed to find INA219 chip");
  //   while (1) { delay(10); }
  // }
  // ina219.setCalibration_32V_50A();

  startMillisPZEM = millis();
  startMillis = millis();
  startMillisSOC = millis();

  soc_initialized = false;
}

void preTransmission() {
  digitalWrite(MAX485_RE, 1);
  digitalWrite(MAX485_DE, 1);
  delay(1);
}

void postTransmission() {
  delay(3);
  digitalWrite(MAX485_RE, 0);
  digitalWrite(MAX485_DE, 0);
}

void readPZEMDC(ModbusMaster &node, float &voltage, float &current, float &power, float &energy) {
    while (Serial2.available()) Serial2.read(); // Bersihkan buffer serial
    delay(100); // Beri waktu switching RS485
    uint8_t result = node.readInputRegisters(0x0000, 6);
    if (result == node.ku8MBSuccess) {
        uint32_t tempdouble = 0x00000000;
        voltage = node.getResponseBuffer(0x0000) / 100.0;
        current = node.getResponseBuffer(0x0001) / 100.0;
        tempdouble = (node.getResponseBuffer(0x0003) << 16) + node.getResponseBuffer(0x0002);
        power = tempdouble / 10.0;
        tempdouble = (node.getResponseBuffer(0x0005) << 16) + node.getResponseBuffer(0x0004);
        energy = tempdouble;
    }
}

void readPZEMAC(ModbusMaster &node, float &voltageAC, float &currentAC, float &powerAC, float &energyAC, float &frequencyAC, float &powerFactorAC) {
    while (Serial2.available()) Serial2.read(); // Bersihkan buffer serial
    delay(100); // Beri waktu switching RS485
    uint8_t result = node.readInputRegisters(0x0000, 9);
    if (result == node.ku8MBSuccess) {
        uint32_t tempdouble = 0x00000000;
        voltageAC = node.getResponseBuffer(0x0000) / 10.0;
        tempdouble = (node.getResponseBuffer(0x0002) << 16) + node.getResponseBuffer(0x0001);
        currentAC = tempdouble / 1000.00;
        tempdouble = (node.getResponseBuffer(0x0004) << 16) + node.getResponseBuffer(0x0003);
        powerAC = tempdouble / 10.0;
        tempdouble = (node.getResponseBuffer(0x0006) << 16) + node.getResponseBuffer(0x0005);
        energyAC = tempdouble;
        frequencyAC = node.getResponseBuffer(0x0007) / 10.0;
        powerFactorAC = node.getResponseBuffer(0x0008) / 100.0;
    }
}

void readPZEMData() {
    readPZEMDC(nodePanel, PZEMVoltagePanel, PZEMCurrentPanel, PZEMPowerPanel, PZEMEnergyPanel);
    Serial.println("PZEM-017 Panel:");
    Serial.print("Voltage Panel: "); Serial.print(PZEMVoltagePanel, 1); Serial.print(" V   ");
    Serial.print("Current Panel: "); Serial.print(PZEMCurrentPanel, 3); Serial.print(" A   ");
    Serial.print("Power Panel: "); Serial.print(PZEMPowerPanel, 1); Serial.print(" W   ");
    Serial.print("Energy Panel: "); Serial.print(PZEMEnergyPanel, 0); Serial.println(" Wh");
    delay(500);

    readPZEMDC(nodeBattery, PZEMVoltageBattery, PZEMCurrentBattery, PZEMPowerBattery, PZEMEnergyBattery);
    Serial.println("PZEM-017 Baterai:");
    Serial.print("Voltage Batt: "); Serial.print(PZEMVoltageBattery, 1); Serial.print(" V   ");
    Serial.print("Current Batt: "); Serial.print(PZEMCurrentBattery, 3); Serial.print(" A   ");
    Serial.print("Power Batt: "); Serial.print(PZEMPowerBattery, 1); Serial.print(" W   ");
    Serial.print("Energy Batt: "); Serial.print(PZEMEnergyBattery, 0); Serial.println(" Wh");
    delay(500);

    readPZEMAC(nodeAC, voltageAC, currentAC, powerAC, energyAC, frequencyAC, powerFactorAC);
    if (isPLTS) {
        PLTSVoltage = voltageAC;
        PLTSCurrent = currentAC;
        PLTSPower = powerAC;
        PLTSEnergy = energyAC;
        PLTSHz = frequencyAC;
        PLTSPf = powerFactorAC;
        Serial.println("PZEM-016 AC PLTS:");
        Serial.print("Voltage PLTS: "); Serial.print(PLTSVoltage, 1); Serial.print(" V   ");
        Serial.print("Current PLTS: "); Serial.print(PLTSCurrent, 3); Serial.print(" A   ");
        Serial.print("Power PLTS: "); Serial.print(PLTSPower, 1); Serial.print(" W   ");
        Serial.print("Energy PLTS: "); Serial.print(PLTSEnergy, 0); Serial.println(" Wh");
        Serial.print("Frekuensi PLTS: "); Serial.print(PLTSHz, 1); Serial.print(" Hz   ");
        Serial.print("PF PLTS: "); Serial.print(PLTSPf, 0); Serial.println(" pF");
    } else {
        GridVoltage = voltageAC;
        GridCurrent = currentAC;
        GridPower = powerAC;
        GridEnergy = energyAC;
        GridHz = frequencyAC;
        GridPf = powerFactorAC;
        Serial.println("PZEM-016 AC Grid:");
        Serial.print("Voltage Grid: "); Serial.print(GridVoltage, 1); Serial.print(" V   ");
        Serial.print("Current Grid: "); Serial.print(GridCurrent, 3); Serial.print(" A   ");
        Serial.print("Power Grid: "); Serial.print(GridPower, 1); Serial.print(" W   ");
        Serial.print("Energy Grid: "); Serial.print(GridEnergy, 0); Serial.println(" Wh");
        Serial.print("Frekuensi Grid: "); Serial.print(GridHz, 1); Serial.print(" Hz   ");
        Serial.print("PF Grid: "); Serial.print(GridPf, 0); Serial.println(" pF");
    }
    delay(500); 
}

void readINA219Data() {
    ShuntVoltage = ina219.getShuntVoltage_mV();
    INA219Voltage = ina219.getBusVoltage_V();
    INA219Current = ina219.getCurrent_mA() / 1000.0;
    Serial.println("INA219 Batt:");
    Serial.print("Shunt Voltage: "); Serial.print(ShuntVoltage); Serial.print(" mV   ");
    Serial.print("INA219 Voltage: "); Serial.print(INA219Voltage, 2); Serial.print(" V   ");
    Serial.print("INA219 Current: "); Serial.print(INA219Current, 3); Serial.println(" A");
}

void urgent() {
    if (PZEMCurrentBattery > 10.0) {
        digitalWrite(RELAY_Batt, LOW);
        Serial.println("Relay ON - Overcurrent detected!");
        Serial.print(PZEMCurrentBattery);
        Serial.print("A > 10.0 A");
    } else {
        digitalWrite(RELAY_Batt, HIGH);
        Serial.println("Relay OFF - Normal condition.");
    }
}

void calculateSOC() {
    deltaT = period / 3600000.0;
    kapasitas = ((PZEMCurrentBattery * deltaT) / Ah) * 100.0;

    if (INA219Current < -0.2) { // charging
        if (previousStatus != statusbatt) count = 0;
        count++;
        SOCo += kapasitas;
        statusbatt = 1;
        Serial.println("Baterai Charge");
    } else if (INA219Current > 0.2) { // discharging
        if (previousStatus != statusbatt) count = 0;
        count++;
        SOCo -= kapasitas;
        statusbatt = 0;
        Serial.println("Baterai Discharge");
    }
    previousStatus = statusbatt;
    interval = (count * period);
    SOCo = constrain(SOCo, 0, 100);
    SOCt = static_cast<int>(SOCo);
    Serial.print("Interval Perhitungan : ");
    Serial.print(deltaT,8);
    Serial.print(" kapasitas : ");
    Serial.println(kapasitas,8);
    Serial.print("SOC Baterai (float) : ");
    Serial.println(SOCo);
    Serial.print("SOC Baterai (int) : ");
    Serial.println(SOCt);
}

void sendDataToESP() {
    String data = String(PZEMVoltagePanel, 2) + "," +
                  String(PZEMCurrentPanel, 2) + "," +
                  String(PZEMPowerPanel, 2) + "," +
                  String(PZEMEnergyPanel, 2) + "," +
                  String(PZEMVoltageBattery, 2) + "," +
                  String(PZEMCurrentBattery, 2) + "," +
                  String(PZEMPowerBattery, 2) + "," +
                  String(PZEMEnergyBattery, 2) + "," +
                  String(SOCo, 2) + "," +
                  String(PLTSVoltage, 2) + "," +
                  String(PLTSCurrent, 2) + "," +
                  String(PLTSPower, 2) + "," +
                  String(PLTSEnergy, 2) + "," +
                  String(PLTSHz, 2) + "," +
                  String(PLTSPf, 2) + "," +
                  String(GridVoltage, 2) + "," +
                  String(GridCurrent, 2) + "," +
                  String(GridPower, 2) + "," +
                  String(GridEnergy, 2) + "," +
                  String(GridHz, 2) + "," +
                  String(GridPf, 2) + "," +
                  String(INA219Voltage, 2) + "," +
                  String(INA219Current, 2) + "," +
                  String(ShuntVoltage, 2) + "\n";
    Serial3.println(data);
    Serial.println("Data to ESP: " + data);
}

void receiveESPData() {
    while (Serial3.available()) {
      char c = Serial3.read();
      Serial.print(c); // Debugging karakter yang diterima
      if (c == '\n') {
        Serial.println("\nReceived (Raw Data): " + receivedData); // Menampilkan data mentah
        parseData(receivedData);  // Parsing data yang diterima
        receivedData = ""; // Reset buffer
      } else {
        receivedData += c;
      }
    }  
}

// Fungsi untuk parsing data CSV
void parseData(String data) {
  Serial.println("Raw Data for Parsing: " + data);  // Debug log

  int index = 0;
  float values[13]; // 10 data + 3 untuk jam, menit, detik
  char *ptr = strtok((char*)data.c_str(), ",");

  while (ptr != NULL && index < 13) {
    values[index] = atof(ptr);
    ptr = strtok(NULL, ",");
    index++;
  }

  Serial.print("Total Parsed Values: ");
  Serial.println(index);

  if (index == 13) {
    total_voltage = values[0];
    total_current = values[1];
    power = values[2];
    temp1 = values[3];
    temp2 = values[4];
    v1 = values[5];
    v2 = values[6];
    v3 = values[7];
    v4 = values[8];
    soc = values[9];

    // Tambahan parsing waktu
    jam = (int)values[10];
    menit = (int)values[11];
    detik = (int)values[12];

    Serial.println("Final Parsed Data:");
    Serial.print("Total Voltage: " + String(total_voltage) + " V   ");
    Serial.print("Total Current: " + String(total_current) + " A   ");
    Serial.print("Power: " + String(power) + " W   ");
    Serial.print("Temp1: " + String(temp1) + " °C   ");
    Serial.println("Temp2: " + String(temp2) + " °C");
    Serial.print("V1: " + String(v1) + " V   ");
    Serial.print("V2: " + String(v2) + " V   ");
    Serial.print("V3: " + String(v3) + " V   ");
    Serial.print("V4: " + String(v4) + " V   ");
    Serial.println("SOC: " + String(soc) + "%");

    Serial.print("Waktu: ");
    Serial.print(jam);
    Serial.print(":");
    Serial.print(menit);
    Serial.print(":");
    Serial.println(detik);

    initialSOC();  // << Panggil di sini
    initialwaktu();
    

  } else {
    Serial.println("Data tidak lengkap! Hanya " + String(index) + " nilai yang terbaca.");
  }
}

// Fungsi untuk membaca SOC Awal dari BMS
void initialSOC () {
  SOCo = soc;
  //   if (!soc_initialized && soc > 0) {  // Pastikan SOC dari ESP valid
  //   SOCo = soc;
  //   soc_initialized = true;
  //   Serial.println("Inisialisasi SOC Awal dari ESP:");
  //   Serial.println("SOCo = " + String(SOCo) + " %");
  // }
}

// Fungsi untuk membaca Waktu Awal dari ESP NTP
void initialwaktu() {
  if (!waktu_initialized) {
    if (jam >= 0 && jam <= 23 && menit >= 0 && menit <= 59 && detik >= 0 && detik <= 59) {
      hour = jam;
      minute = menit;
      second = detik;

      baseTime = hour * 3600UL + minute * 60UL + second;
      baseMillis = millis();

      sprintf(waktu, "%02d:%02d:%02d", hour, minute, second);
      Serial.println("🕒 Update Waktu dari ESP: " + String(waktu));
      
      waktu_initialized = true; // tandai sudah inisialisasi waktu
    } else {
      Serial.println("⚠️ Waktu dari ESP tidak valid!");
    }
  } else {
    Serial.println("Waktu sudah diinisialisasi, skip.");
  }
}

void sendiEI() {
  unsigned long elapsed = (millis() - baseMillis) / 1000;
  unsigned long totalSeconds = baseTime + elapsed;

  hour = (totalSeconds / 3600) % 24;
  minute = (totalSeconds / 60) % 60;
  second = totalSeconds % 60;

  sprintf(waktu, "%02d:%02d:%02d", hour, minute, second);
  Serial.println("⏱️ Waktu Internal STM32: " + String(waktu));
}

void ATS() {
  Serial.print("🕒 Jam sekarang: ");
  Serial.println(waktu);

  bool waktuAktif = (hour  >= 18 || hour  < 6);

  if (waktuAktif) {
    if (soc >= 30.00) {
      // Jam aktif & SOC cukup => pakai PLTS
      digitalWrite(RELAY_Inv, HIGH);
      digitalWrite(RELAY_ATS_F, HIGH);
      digitalWrite(RELAY_ATS_N, HIGH);
      isPLTS = true;
      Serial.println("✅ Jam aktif & SOC ≥ 30% - Relay ON (PLTS)");
    } else {
      // Jam aktif tapi SOC rendah => PLN
      digitalWrite(RELAY_Inv, LOW);
      digitalWrite(RELAY_ATS_F, LOW);
      digitalWrite(RELAY_ATS_N, LOW);
      isPLTS = false;
      Serial.println("⛔ Jam aktif tapi SOC < 30% - Relay OFF (PLN)");
    }
  } else {
    // Di luar jam aktif (misalnya siang hari), pakai PLN tapi relay ATS tetap ON
    digitalWrite(RELAY_Inv, LOW);
    digitalWrite(RELAY_ATS_F, HIGH);
    digitalWrite(RELAY_ATS_N, HIGH);
    isPLTS = false;
    Serial.println("🌤 Di luar jam aktif - Inverter OFF, Relay ON (PLTS)");
  }

  // Log akhir
  Serial.print("📶 SOC ");
  Serial.print(soc);
  Serial.print("% - Relay ");
  Serial.print(isPLTS ? "NYALA ✅" : "MATI ❌");
  Serial.println(" - Sumber: " + String(isPLTS ? "PLTS" : "PLN"));
}

void logtoSDcard() {
    if (!SD.exists("/Maul.csv")) {
        File file3 = SD.open("/Maul.csv", FILE_WRITE);
        file3.println("No,Waktu (s),Baterai Energi (Wh),PLTS Energi (Wh),Grid Energi (Wh)");
        file3.close();
    }

    File file3 = SD.open("/Maul.csv", FILE_WRITE);
    if (file3) {
        file3.print(count); file3.print(", ");
        file3.print(waktu); file3.print(", ");
        file3.print(PZEMEnergyBattery); file3.print(", ");
        file3.print(PLTSEnergy); file3.print(", ");
        file3.print(GridEnergy); file3.println();
        file3.close();
        Serial.println("Data disimpan ke Maul");
    } else {
        Serial.println("Gagal membuka Maul");
    }
}


void loop() {
    unsigned long now = millis();

    if (now  - startMillisPZEM >= periodPZEM) {
        startMillisPZEM += periodPZEM;
        readPZEMData();
    }

    readINA219Data();
    urgent();
    receiveESPData();
    sendiEI(); // update dan tampilkan waktu internal STM32
    ATS();

    if (now  - startMillis >= period) {
        startMillis += period;
        sendDataToESP();
        // calculateSOC();
    }

    if (now  - startMillisSOC >= periodSOC) {
        startMillisSOC += periodSOC;;
        logtoSDcard();
    }
}