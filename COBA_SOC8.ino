#include <ModbusMaster.h>
#include <SPI.h>
#include <SD.h>
#include <Wire.h>
#include <Adafruit_INA219.h>
#include <Ethernet.h>
#include <PubSubClient.h>
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
#define SPI2_NSS_PIN PB0 // SPI_2 digunakan untuk komunikasi dengan Ethernet

//IP Static
byte mac[] = { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED };
#define MYIPADDR 192,168,1,167
#define MYIPMASK 255,255,255,0
#define MYDNS 192,168,1,1
#define MYGW 192,168,1,1

//MQTT
EthernetClient ethClient;
PubSubClient client(ethClient);
const char* mqttServer = "192.168.0.62";
const int mqttPort = 1883;
const char* mqttUser = "mqtt";
const char* mqttPassword = "mqtt";
const char* topic_publish1 = "iot/data1";
const char* topic_publish2 = "iot/data3";
const char* topic_subscribe = "iot/data2";

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
float total_voltage, total_current, power, temp1, temp2, v1, v2, v3, v4, soc;

// Initial SOC
bool soc_initialized = false;  // Global flag

// Pengolahan Data
int previousStatus = -1;  // Inisialisasi dengan nilai tidak valid
int count = 0, SOCt = 0, Ah = 100, interval = 0;
float deltaT = 0.0, SOCo = 0.0, kapasitas = 0.0;
char waktu[10];

// Status Alat Terkini
int statusbatt = 0, statuscsv = 0;

// Variable callback
const char* hari = "";
const char* bulan = "";
int tanggal, tahun, jam, menit, detik;

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

//Timer untuk SOC, MQTT, dan SD CARD
unsigned long startMillisSOC;
unsigned long currentMillisSOC;
const unsigned long periodSOC = 600000;  // 10 menit

void setup() {
  // Serial Monitor
  Serial.begin(9600);
  while (!Serial);

  // Komunikasi Antar Perangkat
  Serial2.begin(9600, SERIAL_8N2); // Untuk komunikasi PZEM
  Serial3.begin(9600); // Untuk komunikasi dengan ESP
  Serial.println("STM32 siap mengirim dan menerima data");

  pinMode(SPI1_NSS_PIN, OUTPUT);
  pinMode(SPI2_NSS_PIN, OUTPUT);
     
  digitalWrite(SPI1_NSS_PIN, HIGH);
  digitalWrite(SPI2_NSS_PIN, HIGH);

  // Inisialisasi SD Card
  Serial.print("Initializing SD card...");
  digitalWrite(SPI1_NSS_PIN, LOW);
  if (!SD.begin(SPI1_NSS_PIN)) {
    Serial.println("initialization failed!");
    while (1);
  }
  Serial.println("initialization done.");

  digitalWrite(SPI1_NSS_PIN, HIGH);
  delay(500);
  
  // Serial.println("Begin Ethernet");
  // delay(500);
  // digitalWrite(SPI2_NSS_PIN, LOW);
  // Ethernet.init(SPI2_NSS_PIN);
  // if (Ethernet.begin(mac)) {
  //     Serial.println("DHCP OK!");
  // } else {
  //     Serial.println("Failed DHCP, using static IP");
  //     IPAddress ip(MYIPADDR);
  //     IPAddress dns(MYDNS);
  //     IPAddress gw(MYGW);
  //     IPAddress sn(MYIPMASK);
  //     Ethernet.begin(mac, ip, dns, gw, sn);
  // }
  // Serial.print("Local IP : ");
  // Serial.println(Ethernet.localIP());

  // // Inisialisasi MQTT
  // client.setServer(mqttServer, mqttPort);
  // client.setCallback(callback);
  // reconnectMQTT();

  // digitalWrite(SPI2_NSS_PIN, HIGH);

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

  // Inisialisasi INA219
  if (!ina219.begin()) {
    Serial.println("Failed to find INA219 chip");
    while (1) { delay(10); }
  }
  ina219.setCalibration_32V_50A();

  startMillisPZEM = millis();
  startMillisSOC = millis();
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

// Fungsi untuk membaca SOC Awal berdasarkan tegangan baterai
void initialSOC () {
    if (!soc_initialized && soc > 0) {  // Pastikan SOC dari ESP valid
    SOCo = soc;
    soc_initialized = true;
    Serial.println("Inisialisasi SOC Awal dari ESP:");
    Serial.println("SOCo = " + String(SOCo) + " %");
  }
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
    deltaT = periodSOC / 3600000.0;
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
    interval = (count * periodSOC)/60000;
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
    String data = String(PZEMVoltagePanel) + "," + String(PZEMCurrentPanel) + "," + String(PZEMPowerPanel) + "," + 
                   String(PZEMVoltageBattery) + "," + String(PZEMCurrentBattery) + "," + String(PZEMPowerBattery) + "," + 
                   String(INA219Voltage) + "," + String(INA219Current) + "\n"; 
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
  Serial.println("Raw Data for Parsing: " + data);  // Tambahkan log ini untuk debugging

  int index = 0;
  float values[10]; // Pastikan array cukup untuk 10 nilai
  char *ptr = strtok((char*)data.c_str(), ",");

  while (ptr != NULL && index < 10) {  // Loop sampai 10 elemen
    values[index] = atof(ptr);
    ptr = strtok(NULL, ",");
    index++;
  }

  Serial.print("Total Parsed Values: ");
  Serial.println(index);  // Debug jumlah elemen yang berhasil diparsing

  if (index == 10) {
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

    // Coba inisialisasi
    initialSOC();  // << Panggil di sini

  } else {
    Serial.println("Data tidak lengkap! Hanya " + String(index) + " nilai yang terbaca.");
  }
}

void reconnectMQTT() {
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    if (client.connect("STM32_UTAMA", mqttUser, mqttPassword)) {
      Serial.println("Connected!");
      if (client.subscribe(topic_subscribe)) {
                Serial.println("Subscribed to topic successfully");
            } else {
                Serial.println("Failed to subscribe to topic");
            }
    } else {
      Serial.print("Failed, rc=");
      Serial.print(client.state());
      Serial.println(" Retrying in 5 seconds");
      delay(5000);
    }
  }
}

void publishMQTT(){
  digitalWrite(SPI2_NSS_PIN, LOW);
    if (!client.connected()) {
          reconnectMQTT();
      }
      client.loop(); 

      //Payload 1 Topik 1
      String jsonPayload1 = "{";
      jsonPayload1 += "\"Daya Baterai\":" + String(PZEMPowerBattery, 2) + ",";
      jsonPayload1 += "\"SOC\":" + String(soc, 2) + ",";
      jsonPayload1 += "\"Energi Baterai\":" + String(PZEMEnergyBattery, 2) + ",";
      jsonPayload1 += "\"Daya PLTS\":" + String(PLTSPower, 2) + ",";
      jsonPayload1 += "\"Energi PLTS\":" + String(PLTSEnergy, 2) + ",";
      jsonPayload1 += "\"Daya Grid\":" + String(GridPower, 2)+ ",";
      jsonPayload1 += "\"Energi Grid\":" + String(GridEnergy, 2);
      jsonPayload1 += "}";
      // Publish ke topik MQTT
      client.publish(topic_publish1, jsonPayload1.c_str());
      Serial.println("Data Dikirim ke MQTT topik 1");

      //Payload 2 Topik 2
      String jsonPayload2 = "{";
      jsonPayload2 += "\"PZEMVoltagePanel\":" + String(PZEMVoltagePanel, 2) + ",";
      jsonPayload2 += "\"PZEMCurrentPanel\":" + String(PZEMCurrentPanel, 2) + ",";
      jsonPayload2 += "\"PZEMPowerPanel\":" + String(PZEMPowerPanel, 2) + ",";
      jsonPayload2 += "\"PZEMEnergyPanel\":" + String(PZEMEnergyPanel, 2) + ",";

      jsonPayload2 += "\"PZEMVoltageBattery\":" + String(PZEMVoltageBattery, 2) + ",";
      jsonPayload2 += "\"PZEMCurrentBattery\":" + String(PZEMCurrentBattery, 2) + ",";
      jsonPayload2 += "\"PZEMPowerBattery\":" + String(PZEMPowerBattery, 2) + ",";
      jsonPayload2 += "\"PZEMEnergyBattery\":" + String(PZEMEnergyBattery, 2) + ",";

      jsonPayload2 += "\"SOCo\":" + String(SOCo, 2) + ",";

      jsonPayload2 += "\"PLTSVoltage\":" + String(PLTSVoltage, 2) + ",";
      jsonPayload2 += "\"PLTSCurrent\":" + String(PLTSCurrent, 2) + ",";
      jsonPayload2 += "\"PLTSPower\":" + String(PLTSPower, 2) + ",";
      jsonPayload2 += "\"PLTSEnergy\":" + String(PLTSEnergy, 2) + ",";
      jsonPayload2 += "\"PLTSHz\":" + String(PLTSHz, 2) + ",";
      jsonPayload2 += "\"PLTSPf\":" + String(PLTSPf, 2) + ",";

      jsonPayload2 += "\"GridVoltage\":" + String(GridVoltage, 2) + ",";
      jsonPayload2 += "\"GridCurrent\":" + String(GridCurrent, 2) + ",";
      jsonPayload2 += "\"GridPower\":" + String(GridPower, 2) + ",";
      jsonPayload2 += "\"GridEnergy\":" + String(GridEnergy, 2) + ",";
      jsonPayload2 += "\"GridHz\":" + String(GridHz, 2) + ",";
      jsonPayload2 += "\"GridPf\":" + String(GridPf, 2) + ",";

      jsonPayload2 += "\"INA219Voltage\":" + String(INA219Voltage, 2) + ",";
      jsonPayload2 += "\"INA219Current\":" + String(INA219Current, 2) + ",";
      jsonPayload2 += "\"ShuntVoltage\":" + String(ShuntVoltage, 2) + ",";

      jsonPayload2 += "\"total_voltage\":" + String(total_voltage, 2) + ",";
      jsonPayload2 += "\"total_current\":" + String(total_current, 2) + ",";
      jsonPayload2 += "\"power\":" + String(power, 2) + ",";
      jsonPayload2 += "\"temp1\":" + String(temp1, 2) + ",";
      jsonPayload2 += "\"temp2\":" + String(temp2, 2) + ",";
      jsonPayload2 += "\"v1\":" + String(v1, 2) + ",";
      jsonPayload2 += "\"v2\":" + String(v2, 2) + ",";
      jsonPayload2 += "\"v3\":" + String(v3, 2) + ",";
      jsonPayload2 += "\"v4\":" + String(v4, 2) + ",";
      jsonPayload2 += "\"soc\":" + String(soc, 2);
      jsonPayload2 += "}";

      client.publish(topic_publish2, jsonPayload2.c_str());
      Serial.println("Data Dikirim ke MQTT topik 2");

    digitalWrite(SPI2_NSS_PIN, HIGH);
}

void callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message received on topic: ");
  Serial.println(topic);
  Serial.print("Raw payload: ");
  for (unsigned int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
  }
  Serial.println();
  
  char jsonBuffer[length + 1];
  memcpy(jsonBuffer, payload, length);
  jsonBuffer[length] = '\0';
  
  StaticJsonDocument<256> doc;
  DeserializationError error = deserializeJson(doc, jsonBuffer);
  if (error) {
    Serial.print("JSON Parsing Failed: ");
    Serial.println(error.f_str());
    return;
  }
  
  hari = doc["hari"];
  tanggal = doc["tanggal"];
  bulan = doc["bulan"];
  tahun = doc["tahun"];
  jam = doc["jam"];
  menit = doc["menit"];
  detik = doc["detik"];

  sprintf(waktu, "%02d:%02d:%02d", jam, menit, detik);
  
  Serial.println("Parsed Data:");
  Serial.print("Hari: "); Serial.print(hari); Serial.print(", ");Serial.print(tanggal); 
  Serial.print(" ");Serial.print(bulan); Serial.print(" ");Serial.print(tahun); Serial.print ("   ");
  Serial.print("Pukul: "); Serial.print(jam); Serial.print(":");Serial.print(menit);
  Serial.print(":");Serial.println(detik);
}


void ATS() {
  unsigned long now = millis();
  bool waktuAktif = (jam >= 18 || jam < 11);  // jam 18:00 s/d 05:59

  // === 1. Jika di luar jam aktif → matikan semua relay ===
  if (!waktuAktif) {
    digitalWrite(RELAY_Inv, LOW);
    digitalWrite(RELAY_ATS_F, LOW);
    digitalWrite(RELAY_ATS_N, LOW);
    isPLTS = false;
    waitToTurnOn = false;
    Serial.println("⛔ Di luar jam aktif - Relay OFF (PLN)");
    return;
  }

  // === 2. Jika SOC < 30% → matikan relay dan reset status ===
  if (SOCt < 30) {
    digitalWrite(RELAY_Inv, LOW);
    digitalWrite(RELAY_ATS_F, LOW);
    digitalWrite(RELAY_ATS_N, LOW);
    isPLTS = false;
    waitToTurnOn = false;
    Serial.println("🔋 SOC < 30% - Relay OFF (PLN)");
    return;
  }

  // === 3. Jika SOC ≥ 90% dan belum ON → tunggu 5 detik ===
  if (SOCt >= 90 && !isPLTS) {
    if (!waitToTurnOn) {
      waitToTurnOn = true;
      timeToTurnOn = now;
      Serial.println("⏳ SOC ≥ 90% - Tunggu 5 detik sebelum Relay ON");
    }

    if (now - timeToTurnOn >= 5000) {
      digitalWrite(RELAY_Inv, HIGH);
      digitalWrite(RELAY_ATS_F, HIGH);
      digitalWrite(RELAY_ATS_N, HIGH);
      isPLTS = true;
      waitToTurnOn = false;
      Serial.println("✅ SOC ≥ 90% stabil - Relay ON (PLTS)");
    } else {
      digitalWrite(RELAY_Inv, LOW);
      digitalWrite(RELAY_ATS_F, LOW);
      digitalWrite(RELAY_ATS_N, LOW);
    }
    return;
  }

  // === 4. SOC 30–89% → pertahankan status sebelumnya ===
  digitalWrite(RELAY_Inv, isPLTS ? HIGH : LOW);
  digitalWrite(RELAY_ATS_F, isPLTS ? HIGH : LOW);
  digitalWrite(RELAY_ATS_N, isPLTS ? HIGH : LOW);

  Serial.print("📶 SOC ");
  Serial.print(SOCt);
  Serial.print("% - Relay ");
  Serial.print(isPLTS ? "NYALA ✅" : "MATI ❌");
  Serial.println(" - Sumber: " + String(isPLTS ? "PLTS" : "PLN"));
}



void logtoSDcard() {
    digitalWrite(SPI1_NSS_PIN, LOW);

    // === 1. TULIS DATA UTAMA KE Farhan  ===
    if (!SD.exists("/SOCi2.csv")) {
        File file = SD.open("/SOCi2.csv", FILE_WRITE);
        file.println("No,Waktu,Interval (s),V PV (V),I PV (A),P PV (W),E PV (Wh),V1 (V),V2 (V),V3 (V),V4 (V),V Batt (V),I Batt (A),P Batt (W),E Batt (Wh),SOC1 (%),SOC2 (%),Temp1 (°C),Temp2(°C)");
        file.close();
    }

    File file = SD.open("/SOCi2.csv", FILE_WRITE);
    if (file) {
        file.print(count); file.print(", ");
        file.print(waktu); file.print(", ");
        file.print(interval); file.print(", ");
        file.print(PZEMVoltagePanel); file.print(", ");
        file.print(PZEMCurrentPanel); file.print(", ");
        file.print(PZEMPowerPanel); file.print(", ");
        file.print(PZEMEnergyPanel); file.print(", ");
        file.print(v1); file.print(", ");
        file.print(v2); file.print(", ");
        file.print(v3); file.print(", ");
        file.print(v4); file.print(", ");
        file.print(PZEMVoltageBattery); file.print(", ");
        file.print(PZEMCurrentBattery); file.print(", ");
        file.print(PZEMPowerBattery); file.print(", ");
        file.print(PZEMEnergyBattery); file.print(", ");
        file.print(soc); file.print(", ");
        file.print(SOCo); file.print(", ");
        file.print(temp1); file.print(", ");
        file.print(temp2); file.println();
        file.close();
        Serial.println("Data utama disimpan ke Farhan.");
        statuscsv = 1;
    } else {
        Serial.println("Gagal membuka Farhan");
        statuscsv = 0;
    }

    // === 2. TULIS DATA PLTS & GRID KE Maul csv ===
    if (!SD.exists("/Energi.csv")) {
        File file3 = SD.open("/Energi.csv", FILE_WRITE);
        file3.println("No,Waktu (s),P Batt (W),E Batt (Wh),SOC1 (%),PLTS Power (W),PLTS Energy (Wh),Grid Power (W),Grid Energy (Wh)");
        file3.close();
    }

    File file3 = SD.open("/Energi.csv", FILE_WRITE);
    if (file3) {
        file3.print(count); file3.print(", ");
        file3.print(waktu); file3.print(", ");
        file3.print(PZEMPowerBattery); file.print(", ");
        file3.print(PZEMEnergyBattery); file.print(", ");
        file3.print(soc); file.print(", ");
        file3.print(PLTSPower); file3.print(", ");
        file3.print(PLTSEnergy); file3.print(", ");
        file3.print(GridPower); file3.print(", ");
        file3.print(GridEnergy); file3.println();
        file3.close();
        Serial.println("Data PLTS & Grid disimpan ke Maul");
    } else {
        Serial.println("Gagal membuka Maul");
    }

    digitalWrite(SPI1_NSS_PIN, HIGH);  
}


void loop() {
    currentMillisPZEM = millis();
    currentMillis = millis();
    currentMillisSOC = millis();

    if (currentMillisPZEM - startMillisPZEM >= periodPZEM) {
        startMillisPZEM += periodPZEM;
        readPZEMData();
    }
    readINA219Data();
    urgent();
    // ATS();

    if ((currentMillis - startMillis) >= period) {
        startMillis += period;
        sendDataToESP();
        receiveESPData();
    }

    if ((currentMillisSOC - startMillisSOC) >= periodSOC) {
        startMillisSOC += periodSOC;;
        calculateSOC();
        logtoSDcard();
        // publishMQTT();
    }
}
