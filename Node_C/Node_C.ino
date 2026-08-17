#include <Arduino.h>
#include <string.h>

#define NODE_ID 3
#define NODE_ROLE ROLE_DESTINATION

#define NRF_CE   4
#define NRF_CSN  5
#define NRF_SCK  18
#define NRF_MISO 19
#define NRF_MOSI 23

#define ROLE_SOURCE      1
#define ROLE_RELAY       2
#define ROLE_DESTINATION 3

#define NETWORK_ID 0xA731
#define RENDEZVOUS_CHANNEL 76

const uint8_t HOP_TABLE[] = {
  10,25,40,55,70,85,100,115
};

#define HOP_COUNT 8

#define EPOCH_MS 120
#define SLOT_MS 30
#define GUARD_MS 5

#define SLOT_A_TO_B 0
#define SLOT_B_TO_C 1
#define SLOT_RESERVED 2

#define SYNC_PERIOD_EPOCHS 16
#define SYNC_CHANNEL 76

#define SYNC_WAIT_MS 40
#define DATA_WAIT_MS 18

#define MAX_TX_RETRIES 2

#define CMD_R_REGISTER   0x00
#define CMD_W_REGISTER   0x20
#define CMD_R_RX_PAYLOAD 0x61
#define CMD_W_TX_PAYLOAD 0xA0
#define CMD_FLUSH_TX     0xE1
#define CMD_FLUSH_RX     0xE2
#define CMD_NOP          0xFF

#define REG_CONFIG       0x00
#define REG_EN_AA        0x01
#define REG_EN_RXADDR    0x02
#define REG_SETUP_AW     0x03
#define REG_SETUP_RETR   0x04
#define REG_RF_CH        0x05
#define REG_RF_SETUP     0x06
#define REG_STATUS       0x07
#define REG_RX_ADDR_P0   0x0A
#define REG_TX_ADDR      0x10
#define REG_RX_PW_P0     0x11

#define CONFIG_EN_CRC    0x08
#define CONFIG_CRCO      0x04
#define CONFIG_PWR_UP    0x02
#define CONFIG_PRIM_RX   0x01

#define STATUS_RX_DR     0x40
#define STATUS_TX_DS     0x20
#define STATUS_MAX_RT    0x10

#define PKT_SYNC         1
#define PKT_DATA         2
#define PKT_ACK          3

struct __attribute__((packed)) MeshPacket {
  uint8_t type;
  uint8_t source;
  uint8_t destination;
  uint16_t sequence;
  uint32_t epoch;
  uint8_t hop;
  uint8_t flags;
  uint8_t payloadLength;
  uint8_t encryptedPayload[16];
  uint32_t crc;
};

static_assert(
  sizeof(MeshPacket) == 32,
  "MeshPacket must be exactly 32 bytes"
);

const uint8_t RADIO_ADDRESS[5] = {
  'M','E','S','H','1'
};

const uint32_t XTEA_KEY[4] = {
  0xA56BABCD,
  0x000FF123,
  0x13579BDF,
  0x2468ACE0
};

uint32_t networkEpoch = 0;
uint32_t epochStartMillis = 0;

uint8_t currentHop = 0;
uint8_t currentChannel = RENDEZVOUS_CHANNEL;

uint16_t txSequence = 0;

uint32_t lastSyncEpoch = 0;

bool synchronized = false;

uint16_t lastReceivedSequence = 0;

uint32_t lastPrint = 0;

const char demoMessage[] =
  "A_TO_C_TEST";

void waitMicros(uint32_t us) {
  uint32_t start = micros();

  while ((uint32_t)(micros() - start) < us)
    yield();
}

uint8_t spiTransfer(uint8_t data) {
  uint8_t received = 0;

  for (int8_t bit = 7; bit >= 0; bit--) {
    digitalWrite(
      NRF_MOSI,
      (data & (1 << bit)) ? HIGH : LOW
    );

    waitMicros(1);

    digitalWrite(
      NRF_SCK,
      HIGH
    );

    waitMicros(1);

    if (digitalRead(NRF_MISO))
      received |= (1 << bit);

    digitalWrite(
      NRF_SCK,
      LOW
    );

    waitMicros(1);
  }

  return received;
}

uint8_t readRegister(uint8_t reg) {
  uint8_t value;

  digitalWrite(NRF_CSN, LOW);

  spiTransfer(
    CMD_R_REGISTER |
    (reg & 0x1F)
  );

  value = spiTransfer(0xFF);

  digitalWrite(NRF_CSN, HIGH);

  return value;
}

void writeRegister(
  uint8_t reg,
  uint8_t value
) {
  digitalWrite(NRF_CSN, LOW);

  spiTransfer(
    CMD_W_REGISTER |
    (reg & 0x1F)
  );

  spiTransfer(value);

  digitalWrite(NRF_CSN, HIGH);
}

void writeRegisterMulti(
  uint8_t reg,
  const uint8_t *data,
  uint8_t length
) {
  digitalWrite(NRF_CSN, LOW);

  spiTransfer(
    CMD_W_REGISTER |
    (reg & 0x1F)
  );

  for (uint8_t i = 0; i < length; i++)
    spiTransfer(data[i]);

  digitalWrite(NRF_CSN, HIGH);
}

void command(uint8_t cmd) {
  digitalWrite(NRF_CSN, LOW);

  spiTransfer(cmd);

  digitalWrite(NRF_CSN, HIGH);
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
  digitalWrite(NRF_CSN, LOW);

  spiTransfer(CMD_W_TX_PAYLOAD);

  for (uint8_t i = 0; i < length; i++)
    spiTransfer(data[i]);

  digitalWrite(NRF_CSN, HIGH);
}

void readPayload(
  uint8_t *data,
  uint8_t length
) {
  digitalWrite(NRF_CSN, LOW);

  spiTransfer(CMD_R_RX_PAYLOAD);

  for (uint8_t i = 0; i < length; i++)
    data[i] = spiTransfer(0xFF);

  digitalWrite(NRF_CSN, HIGH);
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

  waitMicros(10);

  digitalWrite(
    NRF_CE,
    HIGH
  );

  waitMicros(15);

  digitalWrite(
    NRF_CE,
    LOW
  );

  uint32_t start =
    millis();

  while (
    millis() - start <
    DATA_WAIT_MS
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
  if (!packetAvailable())
    return false;

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

uint32_t crc32(
  const uint8_t *data,
  size_t length
) {
  uint32_t crc =
    0xFFFFFFFF;

  for (size_t i = 0;
       i < length;
       i++) {

    crc ^= data[i];

    for (uint8_t j = 0;
         j < 8;
         j++) {

      if (crc & 1)
        crc =
          (crc >> 1) ^
          0xEDB88320;
      else
        crc >>= 1;
    }
  }

  return ~crc;
}

void xteaEncrypt(
  uint32_t *v
) {
  uint32_t v0 = v[0];
  uint32_t v1 = v[1];

  uint32_t sum = 0;

  const uint32_t delta =
    0x9E3779B9;

  for (uint8_t i = 0;
       i < 32;
       i++) {

    v0 += (
      ((v1 << 4) ^
       (v1 >> 5)) +
      v1
    ) ^
    (sum +
     XTEA_KEY[
       sum & 3
     ]);

    sum += delta;

    v1 += (
      ((v0 << 4) ^
       (v0 >> 5)) +
      v0
    ) ^
    (sum +
     XTEA_KEY[
       (sum >> 11) & 3
     ]);
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

  for (uint8_t i = 0;
       i < 32;
       i++) {

    v1 -= (
      ((v0 << 4) ^
       (v0 >> 5)) +
      v0
    ) ^
    (sum +
     XTEA_KEY[
       (sum >> 11) & 3
     ]);

    sum -= delta;

    v0 -= (
      ((v1 << 4) ^
       (v1 >> 5)) +
      v1
    ) ^
    (sum +
     XTEA_KEY[
       sum & 3
     ]);
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

  for (uint8_t i = 0;
       i < 16;
       i += 8) {

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

  for (uint8_t i = 0;
       i < 16;
       i += 8) {

    xteaDecrypt(
      (uint32_t *)&plain[i]
    );
  }
}

uint32_t packetCRC(
  MeshPacket &packet
) {
  return crc32(
    (uint8_t *)&packet,
    sizeof(MeshPacket) -
    sizeof(uint32_t)
  );
}

void buildDataPacket(
  MeshPacket &packet,
  const char *message
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
    0x01;

  packet.payloadLength =
    strlen(message);

  if (
    packet.payloadLength >
    16
  )
    packet.payloadLength =
      16;

  uint8_t plain[16];

  memset(
    plain,
    0,
    sizeof(plain)
  );

  memcpy(
    plain,
    message,
    packet.payloadLength
  );

  encryptPayload(
    plain,
    packet.encryptedPayload
  );

  packet.crc =
    packetCRC(packet);
}

bool verifyPacket(
  MeshPacket &packet
) {
  uint32_t receivedCRC =
    packet.crc;

  packet.crc = 0;

  uint32_t calculatedCRC =
    crc32(
      (uint8_t *)&packet,
      sizeof(MeshPacket) -
      sizeof(uint32_t)
    );

  packet.crc =
    receivedCRC;

  return (
    receivedCRC ==
    calculatedCRC
  );
}

void printDecrypted(
  MeshPacket &packet
) {
  uint8_t plain[16];

  decryptPayload(
    packet.encryptedPayload,
    plain
  );

  Serial.print(
    "[DATA] Message: "
  );

  for (
    uint8_t i = 0;
    i < packet.payloadLength;
    i++
  ) {
    Serial.print(
      (char)plain[i]
    );
  }

  Serial.println();
}

void synchronizeFromPacket(
  MeshPacket &packet
) {
  networkEpoch =
    packet.epoch;

  epochStartMillis =
    millis();

  currentHop =
    networkEpoch %
    HOP_COUNT;

  setChannel(
    HOP_TABLE[currentHop]
  );

  enterRX();

  synchronized =
    true;

  Serial.print(
    "[SYNC] Epoch="
  );

  Serial.print(
    networkEpoch
  );

  Serial.print(
    " CH="
  );

  Serial.println(
    currentChannel
  );
}

void sendSync() {
  MeshPacket packet;

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

  packet.payloadLength =
    0;

  packet.crc =
    packetCRC(packet);

  radioTransmit(
    packet
  );
}

void processSync() {
  MeshPacket packet;

  if (!radioReceive(packet))
    return;

  if (
    packet.type !=
    PKT_SYNC
  )
    return;

  if (
    packet.source !=
    1
  )
    return;

  if (!verifyPacket(packet))
    return;

  synchronizeFromPacket(
    packet
  );
}

void processDataAtRelay(
  MeshPacket &packet
) {
  if (
    packet.destination !=
    3
  )
    return;

  if (
    packet.source !=
    1
  )
    return;

  if (
    packet.sequence ==
    lastReceivedSequence
  )
    return;

  lastReceivedSequence =
    packet.sequence;

  Serial.print(
    "[RELAY] A->B packet seq="
  );

  Serial.println(
    packet.sequence
  );

  if (!verifyPacket(packet)) {
    Serial.println(
      "[RELAY] CRC FAILED"
    );

    return;
  }

  Serial.println(
    "[RELAY] CRC OK"
  );

  printDecrypted(
    packet
  );

  packet.source =
    2;

  packet.destination =
    3;

  packet.epoch =
    networkEpoch;

  packet.hop =
    currentHop;

  packet.flags |=
    0x02;

  packet.crc =
    packetCRC(packet);

  Serial.println(
    "[RELAY] Forwarding B->C"
  );

  bool result =
    radioTransmit(packet);

  if (result)
    Serial.println(
      "[RELAY] Forward OK"
    );
  else
    Serial.println(
      "[RELAY] Forward FAILED"
    );
}

void processDataAtDestination(
  MeshPacket &packet
) {
  if (
    packet.destination !=
    3
  )
    return;

  if (
    packet.source !=
    2
  )
    return;

  if (!verifyPacket(packet)) {
    Serial.println(
      "[DEST] CRC FAILED"
    );

    return;
  }

  Serial.println();
  Serial.println(
    "========== DATA RECEIVED =========="
  );

  Serial.print(
    "Source: "
  );

  Serial.println(
    1
  );

  Serial.print(
    "Forwarded by: "
  );

  Serial.println(
    2
  );

  Serial.print(
    "Destination: "
  );

  Serial.println(
    packet.destination
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
    "Hop: "
  );

  Serial.println(
    packet.hop
  );

  Serial.println(
    "CRC: OK"
  );

  printDecrypted(
    packet
  );

  Serial.println(
    "==================================="
  );
}

void processData() {
  MeshPacket packet;

  while (
    radioReceive(packet)
  ) {
    if (
      packet.type !=
      PKT_DATA
    )
      continue;

    if (
      packet.destination !=
      NODE_ID
    )
      continue;

    if (
      NODE_ROLE ==
      ROLE_RELAY
    ) {
      processDataAtRelay(
        packet
      );
    }

    else if (
      NODE_ROLE ==
      ROLE_DESTINATION
    ) {
      processDataAtDestination(
        packet
      );
    }
  }
}

void setHop(
  uint32_t epoch
) {
  currentHop =
    epoch %
    HOP_COUNT;

  setChannel(
    HOP_TABLE[currentHop]
  );

  enterRX();
}

uint32_t localEpoch() {
  if (!synchronized)
    return 0;

  uint32_t elapsed =
    millis() -
    epochStartMillis;

  return networkEpoch +
    (elapsed / EPOCH_MS);
}

uint32_t epochElapsed() {
  if (!synchronized)
    return 0;

  return (
    millis() -
    epochStartMillis
  ) % EPOCH_MS;
}

bool isSlot(
  uint8_t slot
) {
  uint32_t elapsed =
    epochElapsed();

  uint32_t start =
    slot * SLOT_MS;

  return (
    elapsed >=
    start + GUARD_MS &&
    elapsed <
    start + SLOT_MS -
    GUARD_MS
  );
}

void sourceTransmit() {
  if (
    !isSlot(
      SLOT_A_TO_B
    )
  )
    return;

  static uint32_t lastTXEpoch =
    0xFFFFFFFF;

  if (
    localEpoch() ==
    lastTXEpoch
  )
    return;

  lastTXEpoch =
    localEpoch();

  MeshPacket packet;

  buildDataPacket(
    packet,
    demoMessage
  );

  Serial.println();
  Serial.println(
    "[SOURCE] A->B"
  );

  Serial.print(
    "Epoch="
  );

  Serial.println(
    networkEpoch
  );

  Serial.print(
    "Channel="
  );

  Serial.println(
    currentChannel
  );

  Serial.print(
    "Sequence="
  );

  Serial.println(
    packet.sequence
  );

  bool result =
    radioTransmit(packet);

  if (result)
    Serial.println(
      "[SOURCE] TX OK"
    );
  else
    Serial.println(
      "[SOURCE] TX FAILED"
    );
}

void coordinatorSync() {
  uint32_t elapsed =
    epochElapsed();

  if (
    elapsed >
    3 * SLOT_MS
  )
    return;

  if (
    networkEpoch %
    SYNC_PERIOD_EPOCHS !=
    0
  )
    return;

  if (
    lastSyncEpoch ==
    networkEpoch
  )
    return;

  lastSyncEpoch =
    networkEpoch;

  setChannel(
    SYNC_CHANNEL
  );

  enterRX();

  delay(2);

  Serial.print(
    "[SYNC] Beacon epoch="
  );

  Serial.println(
    networkEpoch
  );

  enterTX();

  sendSync();

  setHop(
    networkEpoch
  );
}

void maintainFHSS() {
  if (!synchronized)
    return;

  uint32_t calculatedEpoch =
    localEpoch();

  if (
    calculatedEpoch !=
    networkEpoch
  ) {
    networkEpoch =
      calculatedEpoch;

    setHop(
      networkEpoch
    );

    Serial.print(
      "[FHSS] Epoch="
    );

    Serial.print(
      networkEpoch
    );

    Serial.print(
      " Channel="
    );

    Serial.println(
      currentChannel
    );
  }
}

void startupSync() {
  setChannel(
    SYNC_CHANNEL
  );

  enterRX();

  Serial.println(
    "[SYNC] Waiting on CH 76"
  );

  uint32_t start =
    millis();

  while (
    millis() - start <
    5000
  ) {
    processSync();

    if (synchronized)
      return;

    delay(1);
  }
}

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
  )
    Serial.println(
      "SOURCE"
    );

  else if (
    NODE_ROLE ==
    ROLE_RELAY
  )
    Serial.println(
      "RELAY"
    );

  else
    Serial.println(
      "DESTINATION"
    );

  Serial.print(
    "Synchronized: "
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
    "Hop: "
  );

  Serial.println(
    currentHop
  );

  Serial.print(
    "Channel: "
  );

  Serial.println(
    currentChannel
  );

  Serial.println(
    "============================"
  );
}

void setup() {
  Serial.begin(
    115200
  );

  delay(1000);

  Serial.println();
  Serial.println(
    "3-NODE FHSS MESH"
  );

  Serial.print(
    "Node ID: "
  );

  Serial.println(
    NODE_ID
  );

  Serial.print(
    "Packet size: "
  );

  Serial.println(
    sizeof(MeshPacket)
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

  if (
    NODE_ROLE ==
    ROLE_SOURCE
  ) {
    networkEpoch =
      0;

    epochStartMillis =
      millis();

    synchronized =
      true;

    Serial.println(
      "[SOURCE] Coordinator"
    );

    Serial.println(
      "[SOURCE] Starting FHSS"
    );
  }

  else {
    startupSync();
  }

  printStatus();
}

void loop() {
  if (
    NODE_ROLE ==
    ROLE_SOURCE
  ) {
    maintainFHSS();

    coordinatorSync();

    sourceTransmit();
  }

  else {
    if (!synchronized) {
      startupSync();
    }

    maintainFHSS();

    if (
      NODE_ROLE ==
      ROLE_RELAY
    ) {
      if (
        isSlot(
          SLOT_A_TO_B
        )
      ) {
        enterRX();
        processData();
      }

      if (
        isSlot(
          SLOT_B_TO_C
        )
      ) {
        enterRX();
        processData();
      }
    }

    if (
      NODE_ROLE ==
      ROLE_DESTINATION
    ) {
      enterRX();

      processData();
    }
  }

  if (
    millis() -
    lastPrint >
    2000
  ) {
    lastPrint =
      millis();

    printStatus();
  }

  delay(1);
}