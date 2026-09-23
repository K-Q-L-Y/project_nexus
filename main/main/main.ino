#include <WiFi.h>
#include <WebServer.h>
#include <ESPmDNS.h>
#include <esp_now.h>

// --------------------------------------------------
// Wi-Fi configuration
// --------------------------------------------------

const char* WIFI_SSID = "UM-WiFi-Guest";
const char* WIFI_PASSWORD = "";

constexpr uint32_t OFFLINE_AFTER_MS = 10000;

WebServer server(80);

// --------------------------------------------------
// ESP-NOW data structures
// Must be identical in the sensor sketch.
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

struct NodeState {
  SensorPacket reading;
  uint32_t lastSeen;
  bool received;
};

NodeState nodes[3] = {};
portMUX_TYPE dataMux = portMUX_INITIALIZER_UNLOCKED;

// --------------------------------------------------
// Dashboard webpage
// --------------------------------------------------

const char DASHBOARD[] PROGMEM = R"HTML(
<!DOCTYPE html>
<html lang="en">
<head>
  <meta charset="UTF-8">
  <meta
    name="viewport"
    content="width=device-width, initial-scale=1"
  >

  <title>Project Nexus</title>

  <style>
    * {
      box-sizing: border-box;
    }

    body {
      margin: 0;
      padding: 24px;
      font-family: Arial, sans-serif;
      background: #0f172a;
      color: #f8fafc;
    }

    header {
      max-width: 1000px;
      margin: 0 auto 24px;
    }

    h1 {
      margin-bottom: 5px;
    }

    .subtitle {
      margin-top: 0;
      color: #94a3b8;
    }

    #cards {
      display: grid;
      grid-template-columns:
        repeat(auto-fit, minmax(240px, 1fr));
      gap: 18px;
      max-width: 1000px;
      margin: auto;
    }

    .card {
      padding: 22px;
      border: 1px solid #334155;
      border-radius: 16px;
      background: #1e293b;
      box-shadow: 0 8px 25px rgba(0, 0, 0, 0.25);
    }

    .card h2 {
      margin-top: 0;
    }

    .status {
      display: inline-block;
      margin-bottom: 16px;
      padding: 6px 11px;
      border-radius: 20px;
      font-weight: bold;
    }

    .online {
      color: #86efac;
      background: rgba(34, 197, 94, 0.15);
    }

    .offline {
      color: #fca5a5;
      background: rgba(239, 68, 68, 0.15);
    }

    .reading {
      display: flex;
      justify-content: space-between;
      padding: 10px 0;
      border-bottom: 1px solid #334155;
    }

    .label {
      color: #94a3b8;
    }

    .value {
      font-weight: bold;
    }

    .condition {
      display: inline-block;
      padding: 5px 10px;
      border-radius: 20px;
      font-weight: bold;
    }

    .condition-normal {
      color: #86efac;
      background: rgba(34, 197, 94, 0.15);
    }

    .condition-small {
      color: #fde68a;
      background: rgba(245, 158, 11, 0.15);
    }

    .condition-large {
      color: #fca5a5;
      background: rgba(239, 68, 68, 0.15);
    }

    .condition-unknown {
      color: #cbd5e1;
      background: rgba(148, 163, 184, 0.15);
    }

    #connection {
      max-width: 1000px;
      margin: 20px auto 0;
      color: #94a3b8;
    }
  </style>
</head>

<body>
  <header>
    <h1>Project Nexus</h1>
    <p class="subtitle">ESP32-S3 Sensor Dashboard</p>
  </header>

  <main id="cards"></main>
  <div id="connection">Connecting to ESP32...</div>

  <script>
    function displayFrequency(value) {
      if (value === null) {
        return "--";
      }

      return Number(value).toFixed(1) + " Hz";
    }

    function conditionClass(condition) {
      if (condition === "Normal") return "condition-normal";
      if (condition === "Small leak") return "condition-small";
      if (condition === "Large leak") return "condition-large";
      return "condition-unknown";
    }

    async function updateDashboard() {
      const connection =
        document.getElementById("connection");

      try {
        const response = await fetch(
          "/data?t=" + Date.now(),
          { cache: "no-store" }
        );

        if (!response.ok) {
          throw new Error("HTTP error");
        }

        const data = await response.json();

        document.getElementById("cards").innerHTML =
          data.nodes.map(node => `
            <section class="card">
              <h2>Sensor ${node.id}</h2>

              <div class="status ${
                node.online ? "online" : "offline"
              }">
                ${node.online ? "● Online" : "● Offline"}
              </div>

              <div class="reading">
                <span class="label">Vibration frequency</span>
                <span class="value">
                  ${displayFrequency(node.vibrationFrequency)}
                </span>
              </div>

              <div class="reading">
                <span class="label">Pipe condition</span>
                <span class="condition ${conditionClass(node.pipeCondition)}">
                  ${node.pipeCondition}
                </span>
              </div>

            </section>
          `).join("");

        connection.textContent =
          "Dashboard connected — updated " +
          new Date().toLocaleTimeString();

      } catch (error) {
        connection.textContent =
          "Dashboard connection lost. Retrying...";
      }
    }

    updateDashboard();
    setInterval(updateDashboard, 1000);
  </script>
</body>
</html>
)HTML";

// --------------------------------------------------
// ESP-NOW receive callback
// Arduino-ESP32 3.x callback format.
// --------------------------------------------------

void onDataReceived(
  const esp_now_recv_info_t* info,
  const uint8_t* incomingData,
  int length
) {
  (void)info;

  if (length != sizeof(SensorPacket)) {
    return;
  }

  SensorPacket packet;
  memcpy(&packet, incomingData, sizeof(packet));

  if (packet.nodeId < 1 || packet.nodeId > 3) {
    return;
  }

  const uint8_t index = packet.nodeId - 1;

  portENTER_CRITICAL(&dataMux);

  nodes[index].reading = packet;
  nodes[index].lastSeen = millis();
  nodes[index].received = true;

  portEXIT_CRITICAL(&dataMux);
}

// --------------------------------------------------
// Web server handlers
// --------------------------------------------------

void sendDashboard() {
  server.send_P(200, "text/html", DASHBOARD);
}

const char* pipeConditionText(uint8_t condition) {
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

void sendData() {
  NodeState snapshot[3];

  portENTER_CRITICAL(&dataMux);
  memcpy(snapshot, nodes, sizeof(snapshot));
  portEXIT_CRITICAL(&dataMux);

  String json;
  json.reserve(500);
  json = "{\"nodes\":[";

  for (int i = 0; i < 3; i++) {
    if (i > 0) {
      json += ",";
    }

    bool online =
      snapshot[i].received &&
      (millis() - snapshot[i].lastSeen <
       OFFLINE_AFTER_MS);

    json += "{";
    json += "\"id\":";
    json += String(i + 1);

    json += ",\"online\":";
    json += online ? "true" : "false";

    if (snapshot[i].received) {
      json += ",\"vibrationFrequency\":";
      json += String(
        snapshot[i].reading.vibrationFrequency, 1
      );

      json += ",\"pipeCondition\":\"";
      json += pipeConditionText(
        snapshot[i].reading.pipeCondition
      );
      json += "\"";

      json += ",\"sequence\":";
      json += String(
        snapshot[i].reading.sequence
      );
    } else {
      json += ",\"vibrationFrequency\":null";
      json += ",\"pipeCondition\":\"No data\"";
      json += ",\"sequence\":0";
    }

    json += "}";
  }

  json += "]}";

  server.sendHeader("Cache-Control", "no-store");
  server.send(200, "application/json", json);
}

void sendTestPage() {
  server.send(
    200,
    "text/plain",
    "Project Nexus ESP32 web server is working."
  );
}

void sendNotFound() {
  Serial.print("Unknown page requested: ");
  Serial.println(server.uri());

  server.send(404, "text/plain", "Page not found");
}

// --------------------------------------------------
// Connect to the router
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

  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());

  Serial.print("Wi-Fi channel: ");
  Serial.println(WiFi.channel());

  Serial.print("Main ESP32 station MAC: ");
  Serial.println(WiFi.macAddress());
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
  Serial.println("Starting Project Nexus main ESP32-S3");

  connectToWiFi();

  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW initialization failed");
    return;
  }

  if (
    esp_now_register_recv_cb(onDataReceived)
    != ESP_OK
  ) {
    Serial.println(
      "Could not register ESP-NOW receive callback"
    );
    return;
  }

  server.on("/", sendDashboard);
  server.on("/data", sendData);
  server.on("/test", sendTestPage);
  server.onNotFound(sendNotFound);
  server.begin();

  Serial.println("Web server started");

  if (MDNS.begin("nexus")) {
    Serial.println(
      "Dashboard: http://nexus.local"
    );
  }

  Serial.print("Dashboard IP: http://");
  Serial.println(WiFi.localIP());

  Serial.println("Main ESP32-S3 is ready");
}

// --------------------------------------------------
// Main loop
// --------------------------------------------------

void loop() {
  server.handleClient();
  delay(2);

  static uint32_t lastStatus = 0;

  if (millis() - lastStatus >= 10000) {
    lastStatus = millis();

    Serial.print("Wi-Fi status: ");

    if (WiFi.status() == WL_CONNECTED) {
      Serial.print("connected, IP ");
      Serial.println(WiFi.localIP());
    } else {
      Serial.println("disconnected");
    }
  }
}
