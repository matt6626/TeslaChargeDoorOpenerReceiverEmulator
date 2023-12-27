// Based on RepeatTimer example and Ray Wang RF temperature sniffer http://rayshobby.net/?p=8827

#define RX_PIN GPIO_NUM_26
#define RING_BUFFER_SIZE 1024

#define CLK_PERIOD_US 400
// #define CLK_TOLERANCE_US 50

/* Comparison Message */
const uint8_t sequence[] = { // based on https://github.com/akrutsinger/tesla-charge-port-signal
  0xAA,0xAA,0xAA,0x8A,0xCB,0x32,0xCC,0xCC,0xCB,0x4D,0x2D,
  0x4A,0xD3,0x4C,0xAB,0x4B,0x15,0x96,0x65,0x99,0x99,0x96,
  0x9A,0x5A,0x95,0xA6,0x99,0x56,0x96,0x2B,0x2C,0xCB,0x33,
  0x33,0x2D,0x34,0xB5,0x2B,0x4D,0x32,0xAD,0x28,
};
const uint8_t sequenceLength = sizeof(sequence) / sizeof(uint8_t);
/* End Comparison Message */

QueueHandle_t rxQueue;
const int rxQueueSize = 20;
typedef struct {
  uint32_t timings[RING_BUFFER_SIZE];
  uint8_t levels[RING_BUFFER_SIZE];
  uint32_t messageLength;
} rx_message_t;

#define MSG_MAX_SIZE 1024
typedef struct {
  uint8_t levels[MSG_MAX_SIZE];
  uint8_t byteSequence[MSG_MAX_SIZE];
  uint32_t length;
  uint32_t byteLength;
} decoded_message_t;

hw_timer_t * timer = NULL;
volatile SemaphoreHandle_t rfReceiveSemaphore;
portMUX_TYPE rfReceiveMux = portMUX_INITIALIZER_UNLOCKED;

volatile uint32_t isrCounter = 0;
volatile uint32_t lastIsrAt = 0;
volatile uint32_t duration = 0;
volatile uint32_t lastTime = 0;
volatile uint32_t ringIndex = 0;
volatile bool received = false;
volatile uint32_t timings[RING_BUFFER_SIZE];
volatile uint8_t levels[RING_BUFFER_SIZE];

typedef enum {
  preamble,
  postamble
} decodeState_t;

bool detectPreamble(uint32_t index, uint32_t *startIndex) {
  static const uint32_t PREAMBLE_MSG_COUNT = 25;
  static const uint32_t PREAMBLE_PERIOD_US = 400;
  if (levels[index % RING_BUFFER_SIZE] != 0) {
    return false;
  }
  for (uint32_t i = 0; i < PREAMBLE_MSG_COUNT; i++, index = (index - 1) % RING_BUFFER_SIZE) {
    if (timings[index % RING_BUFFER_SIZE] != PREAMBLE_PERIOD_US) {
      return false;
    }
  }
  *startIndex = index;
  return true;
}

bool detectPostamble(uint32_t index, uint32_t *endIndex) {
  // 3 * 400us is at the end of packet, anything more should indicate end
  static const uint32_t POSTAMBLE_PERIOD_US = 4 * 400;
  if (levels[index] != 1) {
    return false;
  }
  if (timings[index % RING_BUFFER_SIZE] < POSTAMBLE_PERIOD_US) {
    return false;
  }
  *endIndex = index % RING_BUFFER_SIZE;
  return true;
}

uint32_t roundToNearest(uint32_t val, uint32_t multiple) {
  uint32_t quotient, remainder, correction;
  quotient = val / multiple;
  remainder = val % multiple;
  correction = remainder / ((multiple + 1) / 2);
  return (quotient + correction) * multiple;
}

void ARDUINO_ISR_ATTR rfReceive(){
  static decodeState_t decodeState = preamble;
  static uint32_t startIndex = 0;
  static uint32_t endIndex = 0;
  static uint32_t messageLength = 0;

  // Increment the counter and set the time of ISR
  portENTER_CRITICAL_ISR(&rfReceiveMux);

  // Buffer overflow before detecting message
  if (messageLength > RING_BUFFER_SIZE) {
    messageLength = 0;
    decodeState = preamble;
  }

  uint32_t time = micros();
  uint8_t rxState = digitalRead(RX_PIN);
  duration = time - lastTime;
  lastTime = time;

  timings[ringIndex] = roundToNearest(duration, 100);
  levels[ringIndex] = rxState;

  switch (decodeState) {
    case preamble:
      if (detectPreamble(ringIndex, &startIndex)) {
        decodeState = postamble;
        messageLength += (ringIndex - startIndex + 1) % RING_BUFFER_SIZE;
      }
      break;
    case postamble:
      messageLength++;
      if (detectPostamble(ringIndex, &endIndex)) {
        decodeState = preamble;
        if (rxQueue != NULL) {
          static rx_message_t rxMessage;
          rxMessage.messageLength = 0;
          BaseType_t xHigherPriorityTaskWokenByPost = pdFALSE;
          // memcpy(rxMessage.levels, (const void*) levels, sizeof(levels));
          // memcpy(rxMessage.timings, (const void*) timings, sizeof(timings));

          for (uint32_t i = 0; i < messageLength; i++) {
            uint32_t levelsIndex = (startIndex + i) % RING_BUFFER_SIZE;
            uint32_t timingsIndex = (levelsIndex + 1) % RING_BUFFER_SIZE;
            rxMessage.levels[i] = (uint32_t)levels[levelsIndex];
            rxMessage.timings[i] = (uint32_t)timings[timingsIndex];
          }

          rxMessage.messageLength = messageLength;
          
          xQueueSendFromISR(rxQueue, &rxMessage, &xHigherPriorityTaskWokenByPost);
          
          messageLength = 0;
        }
      }
      break;
  }

  ringIndex = (ringIndex + 1) % RING_BUFFER_SIZE;
  

  isrCounter++;
  lastIsrAt = millis();

  portEXIT_CRITICAL_ISR(&rfReceiveMux);
  // Give a semaphore that we can check in the loop
  xSemaphoreGiveFromISR(rfReceiveSemaphore, NULL);
  // It is safe to use digitalRead/Write here if you want to toggle an output
}

void setup() {
  Serial.begin(115200);

  // Set BTN_STOP_ALARM to input mode
  pinMode(RX_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(RX_PIN), rfReceive, CHANGE);

  // // Create semaphore to inform us when the timer has fired
  rfReceiveSemaphore = xSemaphoreCreateBinary();

  rxQueue = xQueueCreate(rxQueueSize, sizeof(rx_message_t));
  if (rxQueue == NULL) {
    Serial.printf("Queue creation error.\n");
    while (1) delay(1000);
  }
}

bool decode_message(rx_message_t * rxMessage, decoded_message_t * decodedMessage) {

  // Ensure output variable is cleared
  memset(decodedMessage, 0, sizeof(decoded_message_t));

  for (uint32_t i = 0; i < (rxMessage->messageLength - 1); i++) {
    if (i == 0) {
      decodedMessage->levels[0] = rxMessage->levels[0];
      decodedMessage->length++;
    } else if (i == rxMessage->messageLength-2) {
      uint32_t level = rxMessage->levels[i];
      for (uint32_t endcount = 3; endcount > 0; endcount--) {
        decodedMessage->levels[decodedMessage->length] = level;
        decodedMessage->length++;
      }
      
    } else {
      uint32_t level = rxMessage->levels[i];
      for (uint32_t clock_pulses = 0; clock_pulses < rxMessage->timings[i] / 400; clock_pulses++) {
        decodedMessage->levels[decodedMessage->length] = level;
        decodedMessage->length++;
      }
    }
  }

  // Get byte representation from bit string
  decodedMessage->byteLength = decodedMessage->length / 8;
  for (uint32_t i = 0; i < decodedMessage->length; i++) {
    decodedMessage->byteSequence[i / 8] |= ( decodedMessage->levels[i] << (7 - (i % 8)) );
  }

  return true;
}

void clear_rx_message_t(rx_message_t * rxMessage) {
  rxMessage->messageLength = 0;
  for (uint32_t i = 0; i < sizeof(rxMessage->levels); i++) {
    rxMessage->levels[i] = 0;
    rxMessage->timings[i] = 0;
  }
}

void loop() {
  
  static rx_message_t rxMessage;
  static decoded_message_t decodedMessage;
  static uint32_t receivedMessageCount = 0;

  memset(&rxMessage, 0, sizeof(rx_message_t));
  memset(&decodedMessage, 0, sizeof(decoded_message_t));

  if (rxQueue != NULL && xQueueReceive(rxQueue, &rxMessage, 0)) {
    receivedMessageCount++;
    Serial.printf("Message Count: %u\n", receivedMessageCount);
    Serial.printf("Message Received (length=%u):\n", rxMessage.messageLength);
    Serial.printf("levels:");
    for (uint32_t i = 0; i < rxMessage.messageLength; i++) {
      Serial.printf("%u,", rxMessage.levels[i]);
    }
    Serial.printf("\n");
    Serial.flush();
    Serial.printf("timings:");
    for (uint32_t i = 0; i < rxMessage.messageLength; i++) {
      Serial.printf("%u,", rxMessage.timings[i]);
    }
    Serial.printf("\n");
    Serial.flush();
    decode_message(&rxMessage, &decodedMessage);
    Serial.printf("decoded (length=%u):", decodedMessage.length);
    for (uint32_t i = 0; i < decodedMessage.length; i++) {
      Serial.printf("%u", decodedMessage.levels[i]);
    }
    Serial.printf("\n");
    Serial.flush();
    Serial.printf("bytes (hex) - length %u:", decodedMessage.byteLength);
    for (uint32_t i = 0; i < decodedMessage.byteLength; i++) {
      Serial.printf("%02hhX", decodedMessage.byteSequence[i]);
    }
    Serial.printf("\n");
    Serial.flush();

    bool sequenceMatch = true;
    for (uint32_t i = 0; i < sequenceLength; i++) {
      if (decodedMessage.byteSequence[i] != sequence[i]) {
        sequenceMatch = false;
      }
    }
    if (sequenceMatch) {
      Serial.printf("Sequence Matched Transmit Message!");
    }
    Serial.printf("\n\n");
    Serial.flush();
  }
}
