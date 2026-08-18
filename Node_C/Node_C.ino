#include <Arduino.h>
#include <Preferences.h>
#include <string.h>

#define ROLE_SOURCE 1
#define ROLE_RELAY 2
#define ROLE_DESTINATION 3

#define NODE_ID 3
#define NODE_ROLE ROLE_DESTINATION

#define NRF_CE 4
#define NRF_CSN 5
#define NRF_SCK 18
#define NRF_MISO 19
#define NRF_MOSI 23

#define NETWORK_ID 0xA731

#define RENDEZVOUS_CHANNEL 76

const uint8_t HOP_TABLE[] = {
  10,25,40,55,70,85,100,115
};

#define HOP_COUNT 8

/*
 * IMPORTANT:
 * Never hop faster than this.
 */
#define EPOCH_MS 600

/*
 * Time before FHSS starts after START.
 */
#define START_DELAY_MS 2500

/*
 * Slots inside one 600 ms epoch.
 */
#define SLOT_A_TO_B 0
#define SLOT_B_TO_C 1

#define SLOT_MS 250

#define SLOT_GUARD_MS 20

/*
 * Periodic synchronization.
 */
#define RESYNC_EPOCHS 10

/*
 * Maximum packets stored by B.
 */
#define FLASH_QUEUE_SIZE 24

/*
 * Payload carried by one packet.
 */
#define MAX_PAYLOAD 16

#define SYNC_TIMEOUT_MS 5000
#define READY_TIMEOUT_MS 300

#define ACK_TIMEOUT_MS 35

#define CMD_R_REGISTER 0x00
#define CMD_W_REGISTER 0x20
#define CMD_R_RX_PAYLOAD 0x61
#define CMD_W_TX_PAYLOAD 0xA0
#define CMD_FLUSH_TX 0xE1
#define CMD_FLUSH_RX 0xE2

#define REG_CONFIG 0x00
#define REG_EN_AA 0x01
#define REG_EN_RXADDR 0x02
#define REG_SETUP_AW 0x03
#define REG_SETUP_RETR 0x04
#define REG_RF_CH 0x05
#define REG_RF_SETUP 0x06
#define REG_STATUS 0x07
#define REG_RX_ADDR_P0 0x0A
#define REG_TX_ADDR 0x10
#define REG_RX_PW_P0 0x11

#define CONFIG_EN_CRC 0x08
#define CONFIG_CRCO 0x04
#define CONFIG_PWR_UP 0x02
#define CONFIG_PRIM_RX 0x01

#define STATUS_RX_DR 0x40
#define STATUS_TX_DS 0x20
#define STATUS_MAX_RT 0x10

#define PKT_SYNC 1
#define PKT_READY 2
#define PKT_START 3
#define PKT_DATA 4
#define PKT_ACK 5

#define ACK_FOR_DATA 1
#define ACK_FOR_READY 2

const uint8_t RADIO_ADDRESS[5] = {
  'M','E','S','H','1'
};

const uint32_t XTEA_KEY[4] = {
  0xA56BABCD,
  0x000FF123,
  0x13579BDF,
  0x2468ACE0
};

struct __attribute__((packed)) MeshPacket {
  uint8_t type;
  uint8_t source;
  uint8_t destination;
  uint16_t sequence;
  uint32_t epoch;
  uint8_t hop;
  uint8_t flags;
  uint8_t payloadLength;
  uint8_t payload[MAX_PAYLOAD];
  uint32_t crc;
};

static_assert(
  sizeof(MeshPacket) == 32,
  "Packet must be exactly 32 bytes"
);

Preferences flashStore;

MeshPacket flashQueue[FLASH_QUEUE_SIZE];

uint8_t queueCount = 0;

uint16_t txSequence = 0;

uint32_t networkEpoch = 0;

uint32_t epochStartMillis = 0;

uint8_t currentHop = 0;

uint8_t currentChannel =
  RENDEZVOUS_CHANNEL;

bool synchronized = false;

bool bReady = false;

bool cReady = false;

bool startReceived = false;

uint32_t lastSyncEpoch = 0;

uint32_t lastStatus = 0;

uint32_t lastDataSendEpoch =
  0xFFFFFFFF;

uint32_t lastForwardEpoch =
  0xFFFFFFFF;

uint16_t lastReceivedSequence =
  0xFFFF;

bool waitingForB = false;

bool waitingForC = false;

uint32_t readyWaitStart = 0;

uint32_t serialSequence = 0;

void waitMicros(uint32_t us) {
  uint32_t start = micros();

  while (
    (uint32_t)(micros() - start) <
    us
  ) {
    yield();
  }
}

uint8_t spiTransfer(uint8_t data) {
  uint8_t received = 0;

  for (int8_t bit = 7; bit >= 0; bit--) {

    digitalWrite(
      NRF_MOSI,
      (data & (1 << bit)) ?
      HIGH :
      LOW
    );

    waitMicros(1);

    digitalWrite(
      NRF_SCK,
      HIGH
    );

    waitMicros(1);

    if (
      digitalRead(NRF_MISO)
    ) {
      received |=
        (1 << bit);
    }

    digitalWrite(
      NRF_SCK,
      LOW
    );

    waitMicros(1);
  }

  return received;
}

uint8_t readRegister(
  uint8_t reg
) {
  uint8_t value;

  digitalWrite(
    NRF_CSN,
    LOW
  );

  spiTransfer(
    CMD_R_REGISTER |
    (reg & 0x1F)
  );

  value =
    spiTransfer(0xFF);

  digitalWrite(
    NRF_CSN,
    HIGH
  );

  return value;
}

void writeRegister(
  uint8_t reg,
  uint8_t value
) {
  digitalWrite(
    NRF_CSN,
    LOW
  );

  spiTransfer(
    CMD_W_REGISTER |
    (reg & 0x1F)
  );

  spiTransfer(value);

  digitalWrite(
    NRF_CSN,
    HIGH
  );
}

void writeRegisterMulti(
  uint8_t reg,
  const uint8_t *data,
  uint8_t length
) {
  digitalWrite(
    NRF_CSN,
    LOW
  );

  spiTransfer(
    CMD_W_REGISTER |
    (reg & 0x1F)
  );

  for (
    uint8_t i = 0;
    i < length;
    i++
  ) {
    spiTransfer(data[i]);
  }

  digitalWrite(
    NRF_CSN,
    HIGH
  );
}

void command(
  uint8_t cmd
) {
  digitalWrite(
    NRF_CSN,
    LOW
  );

  spiTransfer(cmd);

  digitalWrite(
    NRF_CSN,
    HIGH
  );
}

void flushTX() {
  command(CMD_FLUSH_TX);
}

void flushRX() {
  command(CMD_FLUSH_RX);
}

void writePayload(
  const uint8_t *data,
  uint8_t length
) {
  digitalWrite(
    NRF_CSN,
    LOW
  );

  spiTransfer(
    CMD_W_TX_PAYLOAD
  );

  for (
    uint8_t i = 0;
    i < length;
    i++
  ) {
    spiTransfer(data[i]);
  }

  digitalWrite(
    NRF_CSN,
    HIGH
  );
}

void readPayload(
  uint8_t *data,
  uint8_t length
) {
  digitalWrite(
    NRF_CSN,
    LOW
  );

  spiTransfer(
    CMD_R_RX_PAYLOAD
  );

  for (
    uint8_t i = 0;
    i < length;
    i++
  ) {
    data[i] =
      spiTransfer(0xFF);
  }

  digitalWrite(
    NRF_CSN,
    HIGH
  );
}

void nrfInitialize() {

  digitalWrite(
    NRF_CE,
    LOW
  );

  delay(10);

  writeRegister(
    REG_CONFIG,
    CONFIG_EN_CRC |
    CONFIG_CRCO
  );

  /*
   * We implement ACK ourselves.
   */
  writeRegister(
    REG_EN_AA,
    0x00
  );

  writeRegister(
    REG_EN_RXADDR,
    0x01
  );

  writeRegister(
    REG_SETUP_AW,
    0x03
  );

  writeRegister(
    REG_SETUP_RETR,
    0x00
  );

  /*
   * 1 Mbps, 0 dBm.
   */
  writeRegister(
    REG_RF_SETUP,
    0x06
  );

  writeRegister(
    REG_RF_CH,
    RENDEZVOUS_CHANNEL
  );

  writeRegisterMulti(
    REG_RX_ADDR_P0,
    RADIO_ADDRESS,
    5
  );

  writeRegisterMulti(
    REG_TX_ADDR,
    RADIO_ADDRESS,
    5
  );

  writeRegister(
    REG_RX_PW_P0,
    32
  );

  writeRegister(
    REG_STATUS,
    STATUS_RX_DR |
    STATUS_TX_DS |
    STATUS_MAX_RT
  );

  flushTX();
  flushRX();

  writeRegister(
    REG_CONFIG,
    CONFIG_EN_CRC |
    CONFIG_CRCO |
    CONFIG_PWR_UP
  );

  delay(5);
}

void setChannel(
  uint8_t channel
) {
  digitalWrite(
    NRF_CE,
    LOW
  );

  writeRegister(
    REG_RF_CH,
    channel
  );

  currentChannel =
    channel;

  /*
   * Radio settling guard.
   */
  waitMicros(150);
}

void enterRX() {

  digitalWrite(
    NRF_CE,
    LOW
  );

  writeRegister(
    REG_CONFIG,
    CONFIG_EN_CRC |
    CONFIG_CRCO |
    CONFIG_PWR_UP |
    CONFIG_PRIM_RX
  );

  writeRegister(
    REG_STATUS,
    STATUS_RX_DR |
    STATUS_TX_DS |
    STATUS_MAX_RT
  );

  waitMicros(150);

  digitalWrite(
    NRF_CE,
    HIGH
  );

  waitMicros(150);
}

void enterTX() {

  digitalWrite(
    NRF_CE,
    LOW
  );

  writeRegister(
    REG_CONFIG,
    CONFIG_EN_CRC |
    CONFIG_CRCO |
    CONFIG_PWR_UP
  );

  waitMicros(150);
}

bool radioTransmit(
  const MeshPacket &packet
) {
  enterTX();

  flushTX();

  writeRegister(
    REG_STATUS,
    STATUS_RX_DR |
    STATUS_TX_DS |
    STATUS_MAX_RT
  );

  writePayload(
    (const uint8_t *)&packet,
    sizeof(MeshPacket)
  );

  /*
   * CE HIGH > 10 us.
   */
  waitMicros(15);

  digitalWrite(
    NRF_CE,
    HIGH
  );

  waitMicros(20);

  digitalWrite(
    NRF_CE,
    LOW
  );

  uint32_t start =
    millis();

  while (
    millis() - start <
    ACK_TIMEOUT_MS
  ) {

    uint8_t status =
      readRegister(
        REG_STATUS
      );

    if (
      status &
      STATUS_TX_DS
    ) {

      writeRegister(
        REG_STATUS,
        STATUS_TX_DS
      );

      return true;
    }

    if (
      status &
      STATUS_MAX_RT
    ) {

      writeRegister(
        REG_STATUS,
        STATUS_MAX_RT
      );

      flushTX();

      return false;
    }

    delayMicroseconds(100);
  }

  flushTX();

  return false;
}

bool packetAvailable() {
  return (
    readRegister(
      REG_STATUS
    ) &
    STATUS_RX_DR
  );
}

bool radioReceive(
  MeshPacket &packet
) {

  if (
    !packetAvailable()
  ) {
    return false;
  }

  readPayload(
    (uint8_t *)&packet,
    sizeof(MeshPacket)
  );

  writeRegister(
    REG_STATUS,
    STATUS_RX_DR
  );

  return true;
}

/* =========================
   CRC
   ========================= */

uint32_t crc32(
  const uint8_t *data,
  size_t length
) {

  uint32_t crc =
    0xFFFFFFFF;

  for (
    size_t i = 0;
    i < length;
    i++
  ) {

    crc ^= data[i];

    for (
      uint8_t j = 0;
      j < 8;
      j++
    ) {

      if (
        crc & 1
      ) {
        crc =
          (crc >> 1) ^
          0xEDB88320;
      } else {
        crc >>= 1;
      }
    }
  }

  return ~crc;
}

/* =========================
   XTEA
   ========================= */

void xteaEncrypt(
  uint32_t *v
) {

  uint32_t v0 = v[0];
  uint32_t v1 = v[1];

  uint32_t sum = 0;

  const uint32_t delta =
    0x9E3779B9;

  for (
    uint8_t i = 0;
    i < 32;
    i++
  ) {

    v0 += (
      (
        (v1 << 4) ^
        (v1 >> 5)
      ) + v1
    ) ^
    (
      sum +
      XTEA_KEY[
        sum & 3
      ]
    );

    sum += delta;

    v1 += (
      (
        (v0 << 4) ^
        (v0 >> 5)
      ) + v0
    ) ^
    (
      sum +
      XTEA_KEY[
        (sum >> 11) & 3
      ]
    );
  }

  v[0] = v0;
  v[1] = v1;
}

void xteaDecrypt(
  uint32_t *v
) {

  uint32_t v0 = v[0];
  uint32_t v1 = v[1];

  const uint32_t delta =
    0x9E3779B9;

  uint32_t sum =
    delta * 32;

  for (
    uint8_t i = 0;
    i < 32;
    i++
  ) {

    v1 -= (
      (
        (v0 << 4) ^
        (v0 >> 5)
      ) + v0
    ) ^
    (
      sum +
      XTEA_KEY[
        (sum >> 11) & 3
      ]
    );

    sum -= delta;

    v0 -= (
      (
        (v1 << 4) ^
        (v1 >> 5)
      ) + v1
    ) ^
    (
      sum +
      XTEA_KEY[
        sum & 3
      ]
    );
  }

  v[0] = v0;
  v[1] = v1;
}

void encryptPayload(
  const uint8_t *plain,
  uint8_t *encrypted
) {

  memcpy(
    encrypted,
    plain,
    16
  );

  for (
    uint8_t i = 0;
    i < 16;
    i += 8
  ) {

    xteaEncrypt(
      (uint32_t *)&encrypted[i]
    );
  }
}

void decryptPayload(
  const uint8_t *encrypted,
  uint8_t *plain
) {

  memcpy(
    plain,
    encrypted,
    16
  );

  for (
    uint8_t i = 0;
    i < 16;
    i += 8
  ) {

    xteaDecrypt(
      (uint32_t *)&plain[i]
    );
  }
}

/* =========================
   PACKET
   ========================= */

uint32_t packetCRC(
  MeshPacket &packet
) {

  return crc32(
    (uint8_t *)&packet,
    sizeof(MeshPacket) -
    sizeof(uint32_t)
  );
}

bool verifyPacket(
  MeshPacket &packet
) {

  uint32_t received =
    packet.crc;

  uint32_t calculated =
    packetCRC(packet);

  return (
    received ==
    calculated
  );
}

/* =========================
   FLASH STORE
   ========================= */

void flashSave() {

  flashStore.begin(
    "meshbuf",
    false
  );

  flashStore.putUChar(
    "count",
    queueCount
  );

  for (
    uint8_t i = 0;
    i < queueCount;
    i++
  ) {

    char key[8];

    snprintf(
      key,
      sizeof(key),
      "p%02u",
      i
    );

    flashStore.putBytes(
      key,
      &flashQueue[i],
      sizeof(MeshPacket)
    );
  }

  flashStore.end();
}

void flashLoad() {

  flashStore.begin(
    "meshbuf",
    true
  );

  queueCount =
    flashStore.getUChar(
      "count",
      0
    );

  if (
    queueCount >
    FLASH_QUEUE_SIZE
  ) {
    queueCount = 0;
  }

  for (
    uint8_t i = 0;
    i < queueCount;
    i++
  ) {

    char key[8];

    snprintf(
      key,
      sizeof(key),
      "p%02u",
      i
    );

    size_t read =
      flashStore.getBytes(
        key,
        &flashQueue[i],
        sizeof(MeshPacket)
      );

    if (
      read !=
      sizeof(MeshPacket)
    ) {
      queueCount = i;
      break;
    }
  }

  flashStore.end();

  Serial.print(
    "[FLASH] Stored packets: "
  );

  Serial.println(
    queueCount
  );
}

bool flashPush(
  MeshPacket &packet
) {

  if (
    queueCount >=
    FLASH_QUEUE_SIZE
  ) {

    Serial.println(
      "[FLASH] BUFFER FULL"
    );

    return false;
  }

  flashQueue[
    queueCount
  ] = packet;

  queueCount++;

  flashSave();

  Serial.print(
    "[FLASH] Stored packet "
  );

  Serial.print(
    packet.sequence
  );

  Serial.print(
    " | count="
  );

  Serial.println(
    queueCount
  );

  return true;
}

bool flashPeek(
  MeshPacket &packet
) {

  if (
    queueCount == 0
  ) {
    return false;
  }

  packet =
    flashQueue[0];

  return true;
}

void flashPop() {

  if (
    queueCount == 0
  ) {
    return;
  }

  for (
    uint8_t i = 1;
    i < queueCount;
    i++
  ) {

    flashQueue[i - 1] =
      flashQueue[i];
  }

  queueCount--;

  flashSave();

  Serial.print(
    "[FLASH] Packet removed | remaining="
  );

  Serial.println(
    queueCount
  );
}

/* =========================
   PACKET CREATION
   ========================= */

void makeDataPacket(
  MeshPacket &packet,
  const char *text
) {

  memset(
    &packet,
    0,
    sizeof(packet)
  );

  packet.type =
    PKT_DATA;

  packet.source =
    1;

  packet.destination =
    3;

  packet.sequence =
    txSequence++;

  packet.epoch =
    networkEpoch;

  packet.hop =
    currentHop;

  packet.flags =
    0;

  uint8_t length =
    strlen(text);

  if (
    length >
    MAX_PAYLOAD
  ) {
    length =
      MAX_PAYLOAD;
  }

  packet.payloadLength =
    length;

  uint8_t plain[16];

  memset(
    plain,
    0,
    sizeof(plain)
  );

  memcpy(
    plain,
    text,
    length
  );

  encryptPayload(
    plain,
    packet.payload
  );

  packet.crc =
    packetCRC(packet);
}

/* =========================
   ACK
   ========================= */

void makeACK(
  MeshPacket &ack,
  uint8_t destination,
  uint16_t sequence
) {

  memset(
    &ack,
    0,
    sizeof(ack)
  );

  ack.type =
    PKT_ACK;

  ack.source =
    NODE_ID;

  ack.destination =
    destination;

  ack.sequence =
    sequence;

  ack.epoch =
    networkEpoch;

  ack.hop =
    currentHop;

  ack.flags =
    ACK_FOR_DATA;

  ack.crc =
    packetCRC(ack);
}

/* =========================
   SYNC
   ========================= */

void makeSync(
  MeshPacket &packet
) {

  memset(
    &packet,
    0,
    sizeof(packet)
  );

  packet.type =
    PKT_SYNC;

  packet.source =
    1;

  packet.destination =
    0xFF;

  packet.sequence =
    txSequence++;

  packet.epoch =
    networkEpoch;

  packet.hop =
    currentHop;

  packet.flags =
    0;

  packet.crc =
    packetCRC(packet);
}

void makeReady(
  MeshPacket &packet
) {

  memset(
    &packet,
    0,
    sizeof(packet)
  );

  packet.type =
    PKT_READY;

  packet.source =
    NODE_ID;

  packet.destination =
    1;

  packet.sequence =
    0;

  packet.epoch =
    networkEpoch;

  packet.hop =
    currentHop;

  packet.flags =
    ACK_FOR_READY;

  packet.crc =
    packetCRC(packet);
}

void makeStart(
  MeshPacket &packet,
  uint32_t epoch
) {

  memset(
    &packet,
    0,
    sizeof(packet)
  );

  packet.type =
    PKT_START;

  packet.source =
    1;

  packet.destination =
    0xFF;

  packet.sequence =
    txSequence++;

  packet.epoch =
    epoch;

  packet.hop =
    epoch % HOP_COUNT;

  packet.crc =
    packetCRC(packet);
}

/* =========================
   SERIAL INPUT
   ========================= */

void processSerial() {

  if (
    NODE_ROLE !=
    ROLE_SOURCE
  ) {
    return;
  }

  if (
    !Serial.available()
  ) {
    return;
  }

  String input =
    Serial.readStringUntil(
      '\n'
    );

  input.trim();

  if (
    input.length() == 0
  ) {
    return;
  }

  Serial.print(
    "[APP] Input: "
  );

  Serial.println(
    input
  );

  MeshPacket packet;

  makeDataPacket(
    packet,
    input.c_str()
  );

  /*
   * Queue the application packet.
   *
   * It is sent during the next
   * A->B slot.
   */
  flashPush(packet);
}

/* =========================
   FHSS
   ========================= */

uint32_t localEpoch() {

  if (
    !synchronized
  ) {
    return 0;
  }

  return
    networkEpoch +
    (
      (
        millis() -
        epochStartMillis
      ) /
      EPOCH_MS
    );
}

uint32_t epochElapsed() {

  if (
    !synchronized
  ) {
    return 0;
  }

  return (
    millis() -
    epochStartMillis
  ) %
  EPOCH_MS;
}

void setHop(
  uint32_t epoch
) {

  currentHop =
    epoch %
    HOP_COUNT;

  uint8_t channel =
    HOP_TABLE[
      currentHop
    ];

  setChannel(
    channel
  );

  enterRX();

  Serial.print(
    "[FHSS] Epoch="
  );

  Serial.print(
    epoch
  );

  Serial.print(
    " Channel="
  );

  Serial.println(
    channel
  );
}

void maintainFHSS() {

  if (
    !synchronized
  ) {
    return;
  }

  uint32_t epoch =
    localEpoch();

  if (
    epoch ==
    networkEpoch
  ) {
    return;
  }

  networkEpoch =
    epoch;

  setHop(
    networkEpoch
  );
}

/* =========================
   SOURCE
   ========================= */

void sourceSend() {

  if (
    NODE_ROLE !=
    ROLE_SOURCE
  ) {
    return;
  }

  /*
   * A->B slot.
   */
  uint32_t elapsed =
    epochElapsed();

  if (
    elapsed <
    SLOT_GUARD_MS
  ) {
    return;
  }

  if (
    elapsed >
    SLOT_MS -
    SLOT_GUARD_MS
  ) {
    return;
  }

  if (
    lastDataSendEpoch ==
    networkEpoch
  ) {
    return;
  }

  if (
    queueCount == 0
  ) {
    return;
  }

  MeshPacket packet;

  if (
    !flashPeek(packet)
  ) {
    return;
  }

  lastDataSendEpoch =
    networkEpoch;

  Serial.print(
    "[A->B] Sending seq="
  );

  Serial.println(
    packet.sequence
  );

  bool success =
    radioTransmit(packet);

  if (success) {

    Serial.println(
      "[A->B] ACK received"
    );

    flashPop();

  } else {

    Serial.println(
      "[A->B] FAILED - retained"
    );
  }
}

/* =========================
   RELAY RECEIVE
   ========================= */

void relayReceive() {

  if (
    NODE_ROLE !=
    ROLE_RELAY
  ) {
    return;
  }

  MeshPacket packet;

  while (
    radioReceive(packet)
  ) {

    if (
      packet.type ==
      PKT_SYNC
    ) {

      if (
        verifyPacket(packet)
      ) {

        /*
         * Stay on CH 76 while
         * synchronization happens.
         */
        synchronized =
          false;

        setChannel(
          RENDEZVOUS_CHANNEL
        );

        enterRX();

        makeReady(packet);

        /*
         * The relay's ready packet
         * tells A that B is alive.
         */
        radioTransmit(
          packet
        );

        Serial.println(
          "[SYNC] B READY"
        );
      }

      continue;
    }

    if (
      packet.type ==
      PKT_START
    ) {

      if (
        !verifyPacket(packet)
      ) {
        continue;
      }

      networkEpoch =
        packet.epoch;

      epochStartMillis =
        millis() +
        START_DELAY_MS;

      synchronized =
        true;

      startReceived =
        true;

      Serial.print(
        "[START] FHSS begins in "
      );

      Serial.print(
        START_DELAY_MS
      );

      Serial.println(
        " ms"
      );

      continue;
    }

    if (
      packet.type !=
      PKT_DATA
    ) {
      continue;
    }

    if (
      packet.destination !=
      3
    ) {
      continue;
    }

    if (
      packet.source !=
      1
    ) {
      continue;
    }

    if (
      !verifyPacket(packet)
    ) {

      Serial.println(
        "[B] CRC FAILED"
      );

      continue;
    }

    /*
     * Avoid duplicate storage.
     */
    if (
      packet.sequence ==
      lastReceivedSequence
    ) {
      continue;
    }

    lastReceivedSequence =
      packet.sequence;

    Serial.print(
      "[B] Received A packet seq="
    );

    Serial.println(
      packet.sequence
    );

    /*
     * STORE FIRST.
     *
     * This is the important
     * store-forward behavior.
     */
    if (
      flashPush(packet)
    ) {

      MeshPacket ack;

      makeACK(
        ack,
        1,
        packet.sequence
      );

      radioTransmit(
        ack
      );

      Serial.println(
        "[B] Stored + ACKed A"
      );
    }
  }
}

/* =========================
   RELAY FORWARD
   ========================= */

void relayForward() {

  if (
    NODE_ROLE !=
    ROLE_RELAY
  ) {
    return;
  }

  /*
   * B->C slot.
   */
  uint32_t elapsed =
    epochElapsed();

  if (
    elapsed <
    SLOT_MS +
    SLOT_GUARD_MS
  ) {
    return;
  }

  if (
    elapsed >
    (2 * SLOT_MS) -
    SLOT_GUARD_MS
  ) {
    return;
  }

  if (
    lastForwardEpoch ==
    networkEpoch
  ) {
    return;
  }

  if (
    queueCount == 0
  ) {
    return;
  }

  MeshPacket packet;

  if (
    !flashPeek(packet)
  ) {
    return;
  }

  /*
   * B does NOT remove it yet.
   */
  lastForwardEpoch =
    networkEpoch;

  Serial.print(
    "[B->C] Sending stored seq="
  );

  Serial.println(
    packet.sequence
  );

  bool success =
    radioTransmit(packet);

  if (success) {

    Serial.println(
      "[B->C] C ACK received"
    );

    /*
     * NOW remove from flash.
     */
    flashPop();

  } else {

    Serial.println(
      "[B->C] C unavailable"
    );

    Serial.println(
      "[B->C] Packet remains in FLASH"
    );
  }
}

/* =========================
   DESTINATION
   ========================= */

void destinationReceive() {

  if (
    NODE_ROLE !=
    ROLE_DESTINATION
  ) {
    return;
  }

  MeshPacket packet;

  while (
    radioReceive(packet)
  ) {

    if (
      packet.type ==
      PKT_SYNC
    ) {

      if (
        verifyPacket(packet)
      ) {

        setChannel(
          RENDEZVOUS_CHANNEL
        );

        enterRX();

        MeshPacket ready;

        makeReady(
          ready
        );

        radioTransmit(
          ready
        );

        Serial.println(
          "[SYNC] C READY"
        );
      }

      continue;
    }

    if (
      packet.type ==
      PKT_START
    ) {

      if (
        !verifyPacket(packet)
      ) {
        continue;
      }

      networkEpoch =
        packet.epoch;

      epochStartMillis =
        millis() +
        START_DELAY_MS;

      synchronized =
        true;

      startReceived =
        true;

      Serial.print(
        "[START] FHSS begins in "
      );

      Serial.print(
        START_DELAY_MS
      );

      Serial.println(
        " ms"
      );

      continue;
    }

    if (
      packet.type !=
      PKT_DATA
    ) {
      continue;
    }

    if (
      packet.destination !=
      3
    ) {
      continue;
    }

    if (
      packet.source !=
      2
    ) {
      continue;
    }

    if (
      !verifyPacket(packet)
    ) {

      Serial.println(
        "[C] CRC FAILED"
      );

      continue;
    }

    uint8_t plaintext[16];

    decryptPayload(
      packet.payload,
      plaintext
    );

    Serial.println();
    Serial.println(
      "========== MESSAGE =========="
    );

    Serial.print(
      "Source: "
    );

    Serial.println(
      1
    );

    Serial.print(
      "Relay: "
    );

    Serial.println(
      2
    );

    Serial.print(
      "Destination: "
    );

    Serial.println(
      3
    );

    Serial.print(
      "Sequence: "
    );

    Serial.println(
      packet.sequence
    );

    Serial.print(
      "Epoch: "
    );

    Serial.println(
      packet.epoch
    );

    Serial.print(
      "Channel: "
    );

    Serial.println(
      currentChannel
    );

    Serial.print(
      "Message: "
    );

    for (
      uint8_t i = 0;
      i < packet.payloadLength;
      i++
    ) {

      Serial.print(
        (char)plaintext[i]
      );
    }

    Serial.println();

    Serial.println(
      "CRC: OK"
    );

    Serial.println(
      "============================"
    );

    /*
     * Tell B that the packet arrived.
     */
    MeshPacket ack;

    makeACK(
      ack,
      2,
      packet.sequence
    );

    radioTransmit(
      ack
    );

    Serial.println(
      "[C] ACK -> B"
    );
  }
}

/* =========================
   STATUS
   ========================= */

void printStatus() {

  Serial.println();
  Serial.println(
    "========== STATUS =========="
  );

  Serial.print(
    "Node: "
  );

  Serial.println(
    NODE_ID
  );

  Serial.print(
    "Role: "
  );

  if (
    NODE_ROLE ==
    ROLE_SOURCE
  ) {
    Serial.println(
      "SOURCE"
    );
  }

  else if (
    NODE_ROLE ==
    ROLE_RELAY
  ) {
    Serial.println(
      "RELAY"
    );
  }

  else {
    Serial.println(
      "DESTINATION"
    );
  }

  Serial.print(
    "SYNC: "
  );

  Serial.println(
    synchronized ?
    "YES" :
    "NO"
  );

  Serial.print(
    "Epoch: "
  );

  Serial.println(
    networkEpoch
  );

  Serial.print(
    "Channel: "
  );

  Serial.println(
    currentChannel
  );

  Serial.print(
    "Hop: "
  );

  Serial.println(
    currentHop
  );

  if (
    NODE_ROLE ==
    ROLE_RELAY
  ) {

    Serial.print(
      "FLASH BUFFER: "
    );

    Serial.print(
      queueCount
    );

    Serial.print(
      "/"
    );

    Serial.println(
      FLASH_QUEUE_SIZE
    );
  }

  if (
    NODE_ROLE ==
    ROLE_SOURCE
  ) {

    Serial.print(
      "OUTGOING QUEUE: "
    );

    Serial.println(
      queueCount
    );
  }

  Serial.println(
    "============================"
  );
}

/* =========================
   SOURCE SYNCHRONIZATION
   ========================= */

void sourceSynchronization() {

  /*
   * A is NOT allowed to hop yet.
   */
  synchronized =
    false;

  setChannel(
    RENDEZVOUS_CHANNEL
  );

  enterRX();

  Serial.println();
  Serial.println(
    "[SYNC] A waiting on CH 76"
  );

  uint32_t start =
    millis();

  bool gotB = false;
  bool gotC = false;

  while (
    millis() - start <
    SYNC_TIMEOUT_MS
  ) {

    MeshPacket sync;

    makeSync(
      sync
    );

    /*
     * Broadcast sync.
     */
    enterTX();

    radioTransmit(
      sync
    );

    setChannel(
      RENDEZVOUS_CHANNEL
    );

    enterRX();

    uint32_t waitStart =
      millis();

    while (
      millis() - waitStart <
      READY_TIMEOUT_MS
    ) {

      MeshPacket packet;

      if (
        radioReceive(packet)
      ) {

        if (
          packet.type ==
          PKT_READY &&
          packet.source == 2 &&
          verifyPacket(packet)
        ) {

          gotB = true;

          Serial.println(
            "[SYNC] B READY"
          );
        }

        if (
          packet.type ==
          PKT_READY &&
          packet.source == 3 &&
          verifyPacket(packet)
        ) {

          gotC = true;

          Serial.println(
            "[SYNC] C READY"
          );
        }
      }

      if (
        gotB &&
        gotC
      ) {
        break;
      }

      delay(1);
    }

    if (
      gotB &&
      gotC
    ) {
      break;
    }

    Serial.println(
      "[SYNC] Waiting for B/C..."
    );

    delay(200);
  }

  if (
    !gotB ||
    !gotC
  ) {

    Serial.println(
      "[SYNC] FAILED"
    );

    return;
  }

  /*
   * Both nodes are present.
   *
   * Now choose a future epoch.
   */
  uint32_t startEpoch =
    1;

  MeshPacket startPacket;

  makeStart(
    startPacket,
    startEpoch
  );

  /*
   * START is still transmitted
   * on CH 76.
   */
  setChannel(
    RENDEZVOUS_CHANNEL
  );

  enterTX();

  Serial.println(
    "[SYNC] B and C READY"
  );

  Serial.println(
    "[SYNC] Sending START"
  );

  radioTransmit(
    startPacket
  );

  /*
   * Every node receives START
   * and waits START_DELAY_MS.
   *
   * A does exactly the same.
   */
  networkEpoch =
    startEpoch;

  epochStartMillis =
    millis() +
    START_DELAY_MS;

  synchronized =
    true;

  Serial.print(
    "[SYNC] FHSS starts in "
  );

  Serial.print(
    START_DELAY_MS
  );

  Serial.println(
    " ms"
  );
}

/* =========================
   SETUP
   ========================= */

void setup() {

  Serial.begin(
    115200
  );

  delay(1000);

  Serial.println();
  Serial.println(
    "================================"
  );

  Serial.println(
    "3 NODE FHSS STORE-FORWARD MESH"
  );

  Serial.println(
    "================================"
  );

  Serial.print(
    "NODE ID: "
  );

  Serial.println(
    NODE_ID
  );

  pinMode(
    NRF_CE,
    OUTPUT
  );

  pinMode(
    NRF_CSN,
    OUTPUT
  );

  pinMode(
    NRF_SCK,
    OUTPUT
  );

  pinMode(
    NRF_MOSI,
    OUTPUT
  );

  pinMode(
    NRF_MISO,
    INPUT
  );

  digitalWrite(
    NRF_CE,
    LOW
  );

  digitalWrite(
    NRF_CSN,
    HIGH
  );

  digitalWrite(
    NRF_SCK,
    LOW
  );

  digitalWrite(
    NRF_MOSI,
    LOW
  );

  nrfInitialize();

  setChannel(
    RENDEZVOUS_CHANNEL
  );

  enterRX();

  /*
   * B restores its queue after reboot.
   */
  if (
    NODE_ROLE ==
    ROLE_RELAY
  ) {

    flashLoad();
  }

  /*
   * A starts the network.
   */
  if (
    NODE_ROLE ==
    ROLE_SOURCE
  ) {

    sourceSynchronization();
  }

  Serial.println();
  Serial.println(
    "READY"
  );

  if (
    NODE_ROLE ==
    ROLE_SOURCE
  ) {

    Serial.println(
      "Type a message and press ENTER."
    );

    Serial.println(
      "Example: hello from A"
    );
  }
}

/* =========================
   LOOP
   ========================= */

void loop() {

  /*
   * APPLICATION INPUT
   */
  processSerial();

  /*
   * BEFORE FHSS:
   *
   * All nodes remain on CH 76.
   */
  if (
    !synchronized
  ) {

    if (
      NODE_ROLE ==
      ROLE_SOURCE
    ) {

      sourceSynchronization();
    }

    else if (
      NODE_ROLE ==
      ROLE_RELAY
    ) {

      relayReceive();
    }

    else {

      destinationReceive();
    }

    delay(1);

    return;
  }

  /*
   * Wait for common future
   * FHSS start time.
   */
  if (
    millis() <
    epochStartMillis
  ) {

    if (
      NODE_ROLE ==
      ROLE_RELAY
    ) {

      relayReceive();
    }

    else if (
      NODE_ROLE ==
      ROLE_DESTINATION
    ) {

      destinationReceive();
    }

    delay(1);

    return;
  }

  /*
   * FHSS has started.
   */
  maintainFHSS();

  /*
   * SOURCE
   */
  if (
    NODE_ROLE ==
    ROLE_SOURCE
  ) {

    sourceSend();
  }

  /*
   * RELAY
   */
  else if (
    NODE_ROLE ==
    ROLE_RELAY
  ) {

    /*
     * Receive A->B.
     */
    relayReceive();

    /*
     * Forward B->C.
     */
    relayForward();
  }

  /*
   * DESTINATION
   */
  else if (
    NODE_ROLE ==
    ROLE_DESTINATION
  ) {

    destinationReceive();
  }

  /*
   * Periodic status.
   */
  if (
    millis() -
    lastStatus >
    3000
  ) {

    lastStatus =
      millis();

    printStatus();
  }

  delay(1);
}