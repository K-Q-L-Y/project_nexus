#include <WiFi.h>
#include <esp_now.h>

// --------------------------------------------------
// Change this for each sensor ESP32:
// Sensor 1 = 1
// Sensor 2 = 2
// Sensor 3 = 3
// --------------------------------------------------

constexpr uint8_t NODE_ID = 1;

// --------------------------------------------------
// Wi-Fi configuration
// Must use the same network as the main ESP32.
// --------------------------------------------------

const char* WIFI_SSID = "UM-WiFi-Guest";
const char* WIFI_PASSWORD = "";

// Main ESP32 station MAC:
// 7C:E8:B1:A7:47:A8

uint8_t mainEspMac[] = {
  0x7C, 0xE8, 0xB1,
  0xA7, 0x47, 0xA8
};

constexpr uint32_t SEND_INTERVAL_MS = 2000;

// --------------------------------------------------
// ESP-NOW packet
// Must be identical to the main ESP32 structure.
// --------------------------------------------------

struct SensorPacket {
  uint8_t nodeId;
  uint32_t sequence;
  float vibrationFrequency;
  uint8_t pipeCondition;
};

enum PipeCondition : uint8_t {
  PIPE_NORMAL = 0,
  PIPE_SMALL_LEAK = 1,
  PIPE_LARGE_LEAK = 2
};

// Replace these values after calibrating your vibration sensor.
constexpr float SMALL_LEAK_THRESHOLD_HZ = 40.0;
constexpr float LARGE_LEAK_THRESHOLD_HZ = 80.0;

uint32_t sequenceNumber = 0;
uint32_t lastSendTime = 0;

// --------------------------------------------------
// Replace these with your actual sensor functions.
// The sample values let you test the dashboard first.
// --------------------------------------------------

float readVibrationFrequency() {
  // Replace this demonstration value with the frequency
  // calculated from your real vibration sensor.
  if (NODE_ID == 1) return 25.0;
  if (NODE_ID == 2) return 55.0;
  return 95.0;
}

PipeCondition determinePipeCondition(float frequency) {
  if (frequency >= LARGE_LEAK_THRESHOLD_HZ) {
    return PIPE_LARGE_LEAK;
  }

  if (frequency >= SMALL_LEAK_THRESHOLD_HZ) {
    return PIPE_SMALL_LEAK;
  }

  return PIPE_NORMAL;
}

const char* pipeConditionText(PipeCondition condition) {
  switch (condition) {
    case PIPE_NORMAL:
      return "Normal";
    case PIPE_SMALL_LEAK:
      return "Small leak";
    case PIPE_LARGE_LEAK:
      return "Large leak";
    default:
      return "Unknown";
  }
}

// --------------------------------------------------
// Connect to router
// --------------------------------------------------

void connectToWiFi() {
  WiFi.mode(WIFI_STA);
  WiFi.setAutoReconnect(true);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  Serial.print("Connecting to Wi-Fi");

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println();
  Serial.println("Wi-Fi connected");

  Serial.print("Sensor IP address: ");
  Serial.println(WiFi.localIP());

  Serial.print("Wi-Fi channel: ");
  Serial.println(WiFi.channel());

  Serial.print("Sensor MAC address: ");
  Serial.println(WiFi.macAddress());
}

// --------------------------------------------------
// Configure ESP-NOW
// --------------------------------------------------

bool configureEspNow() {
  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW initialization failed");
    return false;
  }

  esp_now_peer_info_t peerInfo = {};

  memcpy(
    peerInfo.peer_addr,
    mainEspMac,
    sizeof(mainEspMac)
  );

  // Channel 0 means use the current Wi-Fi channel.
  peerInfo.channel = 0;
  peerInfo.ifidx = WIFI_IF_STA;
  peerInfo.encrypt = false;

  if (!esp_now_is_peer_exist(mainEspMac)) {
    if (esp_now_add_peer(&peerInfo) != ESP_OK) {
      Serial.println(
        "Failed to add main ESP32 as peer"
      );
      return false;
    }
  }

  return true;
}

// --------------------------------------------------
// Setup
// --------------------------------------------------

void setup() {
  Serial.begin(115200);

  unsigned long serialStart = millis();

  while (!Serial && millis() - serialStart < 3000) {
    delay(10);
  }

  Serial.println();
  Serial.printf(
    "Starting Project Nexus sensor %u\n",
    NODE_ID
  );

  connectToWiFi();

  if (!configureEspNow()) {
    Serial.println("Setup failed");
    return;
  }

  Serial.printf(
    "Sensor node %u is ready\n",
    NODE_ID
  );
}

// --------------------------------------------------
// Main loop
// --------------------------------------------------

void loop() {
  if (
    millis() - lastSendTime <
    SEND_INTERVAL_MS
  ) {
    delay(10);
    return;
  }

  lastSendTime = millis();

  SensorPacket packet = {};

  packet.nodeId = NODE_ID;
  packet.sequence = sequenceNumber++;
  packet.vibrationFrequency = readVibrationFrequency();

  PipeCondition condition = determinePipeCondition(
    packet.vibrationFrequency
  );
  packet.pipeCondition = static_cast<uint8_t>(condition);

  esp_err_t result = esp_now_send(
    mainEspMac,
    reinterpret_cast<uint8_t*>(&packet),
    sizeof(packet)
  );

  if (result == ESP_OK) {
    Serial.printf(
      "Packet %lu queued: %.1f Hz, %s\n",
      static_cast<unsigned long>(packet.sequence),
      packet.vibrationFrequency,
      pipeConditionText(condition)
    );
  } else {
    Serial.printf(
      "ESP-NOW send error: %d\n",
      result
    );
  }
}
