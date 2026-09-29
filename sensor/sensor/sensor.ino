#include <WiFi.h>
#include <esp_now.h>
#include <algorithm>
#include <math.h>

#include "leak_model_data.h"

constexpr uint8_t NODE_ID = 3;       // Change to 1, 2, or 3.
constexpr uint8_t PIEZO_ADC_PIN = 4; // Amplifier AO/OUT.

const char* WIFI_SSID = "33";
const char* WIFI_PASSWORD = "limkimsay";

// Main ESP32 station MAC: 7C:E8:B1:A7:47:A8
// D4:E9:F4:E6:EB:30 NEW MAC ERIN HSE

// uint8_t mainEspMac[] = {
//   0x7C, 0xE8, 0xB1,
//   0xA7, 0x47, 0xA8
// };

uint8_t mainEspMac[] = {
  0xD4, 0xE9, 0xF4,
  0xE6, 0xEB, 0x30
};

constexpr uint32_t MEASUREMENT_INTERVAL_MS = 30000;
constexpr uint32_t SAMPLE_RATE_HZ = 2000;
constexpr uint32_t SAMPLE_PERIOD_US = 1000000UL / SAMPLE_RATE_HZ;
constexpr size_t SAMPLES_PER_WINDOW = 2000;
constexpr uint8_t WINDOWS_PER_PREDICTION = 5;
constexpr uint32_t INTER_WINDOW_DELAY_MS = 250;
constexpr float MINIMUM_CONFIDENCE = 0.65f;

static uint16_t adcWindows[WINDOWS_PER_PREDICTION][SAMPLES_PER_WINDOW];
static float sortScratch[SAMPLES_PER_WINDOW];

// Must remain byte-for-byte identical to the main ESP32 structure.
struct SensorPacket {
  uint8_t nodeId;
  uint32_t sequence;
  float vibrationFrequency;
  uint8_t pipeCondition;
};

enum PipeCondition : uint8_t {
  PIPE_NORMAL = 0,
  PIPE_SMALL_LEAK = 1,
  PIPE_LARGE_LEAK = 2,
  PIPE_UNKNOWN = 255
};

enum FeatureIndex : uint8_t {
  FEATURE_MEAN_ADC = 0,
  FEATURE_STD_ADC,
  FEATURE_RMS_AC,
  FEATURE_MAD_ADC,
  FEATURE_P2P_ADC,
  FEATURE_P95_ABS,
  FEATURE_P99_ABS,
  FEATURE_DIFF_RMS,
  FEATURE_DIFF_MAD,
  FEATURE_BAND_0_50,
  FEATURE_BAND_50_150,
  FEATURE_BAND_150_300,
  FEATURE_BAND_300_600,
  FEATURE_BAND_600_1000,
};

static_assert(LEAK_FEATURE_COUNT == 14, "Model feature count is not 14");

uint32_t sequenceNumber = 0;
uint32_t lastMeasurementStart = 0;
bool firstMeasurement = true;

void captureWindow(uint16_t* samples) {
  uint32_t nextSampleUs = micros();

  for (size_t index = 0; index < SAMPLES_PER_WINDOW; ++index) {
    while ((int32_t)(micros() - nextSampleUs) < 0) {
    }
    samples[index] = analogRead(PIEZO_ADC_PIN);
    nextSampleUs += SAMPLE_PERIOD_US;
  }
}

float interpolatedPercentile(
  float* sortedValues,
  size_t count,
  float percentile
) {
  const float position = (count - 1) * percentile;
  const size_t lower = (size_t)floorf(position);
  const size_t upper = (size_t)ceilf(position);

  if (lower == upper) {
    return sortedValues[lower];
  }

  const float fraction = position - lower;
  return sortedValues[lower] * (1.0f - fraction)
         + sortedValues[upper] * fraction;
}

float goertzelPower(
  const uint16_t* samples,
  uint16_t frequencyBin,
  float mean
) {
  if (frequencyBin == SAMPLES_PER_WINDOW / 2) {
    float alternatingSum = 0.0f;
    for (size_t index = 0; index < SAMPLES_PER_WINDOW; ++index) {
      const float centered = (float)samples[index] - mean;
      alternatingSum += (index & 1U) ? -centered : centered;
    }
    return alternatingSum * alternatingSum;
  }

  const float omega =
    2.0f * PI * frequencyBin / SAMPLES_PER_WINDOW;
  const float coefficient = 2.0f * cosf(omega);
  float previous = 0.0f;
  float previous2 = 0.0f;

  for (size_t index = 0; index < SAMPLES_PER_WINDOW; ++index) {
    const float centered = (float)samples[index] - mean;
    const float current =
      centered + coefficient * previous - previous2;
    previous2 = previous;
    previous = current;
  }

  const float power =
    previous * previous
    + previous2 * previous2
    - coefficient * previous * previous2;
  return power > 0.0f ? power : 0.0f;
}

void calculateSpectrum(
  const uint16_t* samples,
  float mean,
  float bandFractions[5],
  float& dominantFrequencyHz
) {
  double bandPower[5] = {0, 0, 0, 0, 0};
  double totalPower = 0.0;
  float strongestPower = -1.0f;
  uint16_t strongestBin = 0;

  // N and Fs are both 2000, so each bin represents exactly 1 Hz.
  for (uint16_t bin = 1; bin <= SAMPLES_PER_WINDOW / 2; ++bin) {
    const float power = goertzelPower(samples, bin, mean);
    totalPower += power;

    if (power > strongestPower) {
      strongestPower = power;
      strongestBin = bin;
    }

    if (bin < 50)
      bandPower[0] += power;
    else if (bin < 150)
      bandPower[1] += power;
    else if (bin < 300)
      bandPower[2] += power;
    else if (bin < 600)
      bandPower[3] += power;
    else
      bandPower[4] += power;
  }

  for (uint8_t band = 0; band < 5; ++band) {
    bandFractions[band] =
      totalPower > 0.0
        ? (float)(bandPower[band] / totalPower)
        : 0.0f;
  }

  dominantFrequencyHz = (float)strongestBin;
}

void extractWindowFeatures(
  const uint16_t* samples,
  float features[LEAK_FEATURE_COUNT],
  float& dominantFrequencyHz
) {
  double sum = 0.0;
  uint16_t minimum = UINT16_MAX;
  uint16_t maximum = 0;

  for (size_t index = 0; index < SAMPLES_PER_WINDOW; ++index) {
    const uint16_t value = samples[index];
    sum += value;
    minimum = min(minimum, value);
    maximum = max(maximum, value);
  }

  const float mean = (float)(sum / SAMPLES_PER_WINDOW);
  double squaredSum = 0.0;
  double absoluteSum = 0.0;
  double differenceSquaredSum = 0.0;
  double differenceAbsoluteSum = 0.0;

  for (size_t index = 0; index < SAMPLES_PER_WINDOW; ++index) {
    const float centered = (float)samples[index] - mean;
    const float absolute = fabsf(centered);
    squaredSum += centered * centered;
    absoluteSum += absolute;
    sortScratch[index] = absolute;

    if (index > 0) {
      const float difference =
        (float)samples[index] - samples[index - 1];
      differenceSquaredSum += difference * difference;
      differenceAbsoluteSum += fabsf(difference);
    }
  }

  std::sort(sortScratch, sortScratch + SAMPLES_PER_WINDOW);

  features[FEATURE_MEAN_ADC] = mean;
  features[FEATURE_STD_ADC] =
    sqrtf((float)(squaredSum / SAMPLES_PER_WINDOW));
  features[FEATURE_RMS_AC] = features[FEATURE_STD_ADC];
  features[FEATURE_MAD_ADC] =
    (float)(absoluteSum / SAMPLES_PER_WINDOW);
  features[FEATURE_P2P_ADC] = (float)(maximum - minimum);
  features[FEATURE_P95_ABS] =
    interpolatedPercentile(sortScratch, SAMPLES_PER_WINDOW, 0.95f);
  features[FEATURE_P99_ABS] =
    interpolatedPercentile(sortScratch, SAMPLES_PER_WINDOW, 0.99f);
  features[FEATURE_DIFF_RMS] =
    sqrtf((float)(differenceSquaredSum / (SAMPLES_PER_WINDOW - 1)));
  features[FEATURE_DIFF_MAD] =
    (float)(differenceAbsoluteSum / (SAMPLES_PER_WINDOW - 1));

  float bands[5];
  calculateSpectrum(samples, mean, bands, dominantFrequencyHz);
  features[FEATURE_BAND_0_50] = bands[0];
  features[FEATURE_BAND_50_150] = bands[1];
  features[FEATURE_BAND_150_300] = bands[2];
  features[FEATURE_BAND_300_600] = bands[3];
  features[FEATURE_BAND_600_1000] = bands[4];
}

void predictLeak(
  const float features[LEAK_FEATURE_COUNT],
  float probabilities[3]
) {
  probabilities[0] = 0.0f;
  probabilities[1] = 0.0f;
  probabilities[2] = 0.0f;

  for (uint16_t treeIndex = 0;
       treeIndex < LEAK_TREE_COUNT;
       ++treeIndex) {
    const uint16_t treeOffset = LEAK_TREE_OFFSETS[treeIndex];
    int16_t nodeIndex = 0;

    while (true) {
      const LeakTreeNode& node =
        LEAK_NODES[treeOffset + nodeIndex];

      if (node.left < 0) {
        const float total =
          node.probability[0]
          + node.probability[1]
          + node.probability[2];

        if (total > 0.0f) {
          for (uint8_t classIndex = 0;
               classIndex < 3;
               ++classIndex) {
            probabilities[classIndex] +=
              node.probability[classIndex] / total;
          }
        }
        break;
      }

      nodeIndex =
        features[node.feature] <= node.threshold
          ? node.left
          : node.right;
    }
  }

  for (uint8_t classIndex = 0; classIndex < 3; ++classIndex) {
    probabilities[classIndex] /= LEAK_TREE_COUNT;
  }
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

PipeCondition modelClassToCondition(const char* modelClass) {
  if (strcmp(modelClass, "no_leak") == 0) {
    return PIPE_NORMAL;
  }
  if (strcmp(modelClass, "small_leak") == 0) {
    return PIPE_SMALL_LEAK;
  }
  if (strcmp(modelClass, "large_leak") == 0) {
    return PIPE_LARGE_LEAK;
  }
  return PIPE_UNKNOWN;
}

PipeCondition takeLeakMeasurement(
  float& vibrationFrequency,
  float& confidence
) {
  Serial.println("Capturing five piezo windows...");

  for (uint8_t window = 0;
       window < WINDOWS_PER_PREDICTION;
       ++window) {
    captureWindow(adcWindows[window]);
    Serial.printf(
      "Window %u/%u captured\n",
      window + 1,
      WINDOWS_PER_PREDICTION
    );

    if (window + 1 < WINDOWS_PER_PREDICTION) {
      delay(INTER_WINDOW_DELAY_MS);
    }
  }

  float averagedFeatures[LEAK_FEATURE_COUNT] = {0};
  float frequencySum = 0.0f;

  Serial.println("Extracting features and evaluating model...");

  for (uint8_t window = 0;
       window < WINDOWS_PER_PREDICTION;
       ++window) {
    float windowFeatures[LEAK_FEATURE_COUNT];
    float windowFrequency = 0.0f;

    extractWindowFeatures(
      adcWindows[window],
      windowFeatures,
      windowFrequency
    );

    frequencySum += windowFrequency;

    for (uint8_t feature = 0;
         feature < LEAK_FEATURE_COUNT;
         ++feature) {
      averagedFeatures[feature] +=
        windowFeatures[feature] / WINDOWS_PER_PREDICTION;
    }
  }

  vibrationFrequency =
    frequencySum / WINDOWS_PER_PREDICTION;

  float probabilities[3];
  predictLeak(averagedFeatures, probabilities);

  uint8_t bestClass = 0;
  for (uint8_t classIndex = 1; classIndex < 3; ++classIndex) {
    if (probabilities[classIndex] > probabilities[bestClass]) {
      bestClass = classIndex;
    }
  }

  confidence = probabilities[bestClass];

  Serial.printf(
    "Model scores: large=%.1f%%, normal=%.1f%%, small=%.1f%%\n",
    probabilities[0] * 100.0f,
    probabilities[1] * 100.0f,
    probabilities[2] * 100.0f
  );

  if (confidence < MINIMUM_CONFIDENCE) {
    return PIPE_UNKNOWN;
  }

  return modelClassToCondition(LEAK_CLASS_NAMES[bestClass]);
}

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

bool configureEspNow() {
  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW initialization failed");
    return false;
  }

  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, mainEspMac, sizeof(mainEspMac));
  peerInfo.channel = 0; // Use the connected Wi-Fi network's channel.
  peerInfo.ifidx = WIFI_IF_STA;
  peerInfo.encrypt = false;

  if (!esp_now_is_peer_exist(mainEspMac)) {
    if (esp_now_add_peer(&peerInfo) != ESP_OK) {
      Serial.println("Failed to add main ESP32 as peer");
      return false;
    }
  }

  return true;
}

void publishMeasurement() {
  float vibrationFrequency = 0.0f;
  float confidence = 0.0f;

  const PipeCondition condition =
    takeLeakMeasurement(vibrationFrequency, confidence);

  SensorPacket packet = {};
  packet.nodeId = NODE_ID;
  packet.sequence = sequenceNumber++;
  packet.vibrationFrequency = vibrationFrequency;
  packet.pipeCondition = static_cast<uint8_t>(condition);

  const esp_err_t result = esp_now_send(
    mainEspMac,
    reinterpret_cast<uint8_t*>(&packet),
    sizeof(packet)
  );

  if (result == ESP_OK) {
    Serial.printf(
      "Published packet %lu: %.1f Hz, %s, confidence %.1f%%\n",
      static_cast<unsigned long>(packet.sequence),
      packet.vibrationFrequency,
      pipeConditionText(condition),
      confidence * 100.0f
    );
  } else {
    Serial.printf("ESP-NOW send error: %d\n", result);
  }
}

void setup() {
  Serial.begin(115200);

  const unsigned long serialStart = millis();
  while (!Serial && millis() - serialStart < 3000) {
    delay(10);
  }

  Serial.println();
  Serial.printf("Starting Project Nexus leak sensor %u\n", NODE_ID);

  analogReadResolution(12);
  analogSetPinAttenuation(PIEZO_ADC_PIN, ADC_11db);

  connectToWiFi();

  if (!configureEspNow()) {
    Serial.println("Setup failed");
    return;
  }

  Serial.printf(
    "Sensor ready: %u trees, measurement every %lu seconds\n",
    LEAK_TREE_COUNT,
    static_cast<unsigned long>(MEASUREMENT_INTERVAL_MS / 1000)
  );
}

void loop() {
  const uint32_t now = millis();

  if (
    !firstMeasurement
    && now - lastMeasurementStart < MEASUREMENT_INTERVAL_MS
  ) {
    delay(10);
    return;
  }

  firstMeasurement = false;
  lastMeasurementStart = now;
  publishMeasurement();
}
