#include <Arduino.h>
#include "esp_heap_caps.h"
#include "model_data.h"

// Create shorter aliases for the auto-generated xxd variable names
const unsigned char *kModelData =
    model_training_runs_detect_model_p2_960_safe_weights_best_int8_tflite;
const unsigned int kModelSize =
    model_training_runs_detect_model_p2_960_safe_weights_best_int8_tflite_len;

void setup() {
  Serial.begin(115200);
  delay(2000);

  // 1. Verify Memory Status
  psramInit();
  Serial.printf("[MEM]   Internal SRAM Free: %u KB\n", ESP.getFreeHeap() / 1024);
  Serial.printf("[MEM]   External PSRAM Free: %.2f MB\n",
                ESP.getFreePsram() / (1024.0 * 1024.0));

  // 2. Verify Model in Flash (.rodata / DROM)
  Serial.printf("[MODEL] Flash Pointer Addr: %p\n", kModelData);
  Serial.printf("[MODEL] Model Size:         %u bytes (%.2f MB)\n",
                kModelSize, kModelSize / (1024.0 * 1024.0));

  // 3. Verify TFLite FlatBuffer Magic Header (Bytes 4..7 should spell "TFL3")
  char magic[5] = {
      (char)kModelData[4],
      (char)kModelData[5],
      (char)kModelData[6],
      (char)kModelData[7],
      '\0'
  };
  Serial.printf("[MODEL] FlatBuffer Magic:   %s\n", magic);

  if (strcmp(magic, "TFL3") == 0) {
    Serial.println("[MODEL] VALID TFLite v3 Model Confirmed in Flash!");
  } else {
    Serial.println("[MODEL] ERROR: Invalid TFLite Header!");
  }

  // 4. Allocate 2.5 MB Tensor Arena in PSRAM
  const size_t kArenaSize = 2500 * 1024;
  uint8_t *tensor_arena = (uint8_t *)heap_caps_malloc(
      kArenaSize, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);

  if (tensor_arena != nullptr) {
    Serial.printf("[ARENA] 2.50 MB Allocated at: %p (Remaining PSRAM: %.2f MB)\n",
                  tensor_arena, ESP.getFreePsram() / (1024.0 * 1024.0));
  }
}

void loop() {
  Serial.println("Heartbeat: Model Mounted & System Ready.");
  delay(3000);
}