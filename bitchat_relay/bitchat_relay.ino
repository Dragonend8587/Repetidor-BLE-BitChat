/*
 * BitChat ESP32 BLE Relay
 * ========================
 * Repite mensajes de la app BitChat via BLE para extender el rango.
 *
 * Hardware requerido: cualquier ESP32 con BLE (ESP32-WROOM, ESP32-S3, etc.)
 * IDE: Arduino IDE 2.x con "esp32 by Espressif Systems" instalado
 * Baud monitor serie: 115200
 *
 * Basado en: https://github.com/ScreamingEagleUSA/bitchat-esp32
 * Protocolo: https://github.com/permissionlesstech/bitchat
 */

#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <BLEScan.h>
#include <set>
#include <map>
#include <vector>

// ── UUIDs del protocolo BitChat (BluetoothMeshService.swift) ──────────────
#define SERVICE_UUID        "F47B5E2D-4A9E-4C5A-9B3F-8E1D2C3A4B5C"
#define CHARACTERISTIC_UUID "A1B2C3D4-E5F6-4A5B-8C9D-0E1F2A3B4C5D"

// ── Constantes del protocolo (BinaryProtocol.swift) ───────────────────────
#define HEADER_SIZE       13
#define SENDER_ID_SIZE     8
#define RECIPIENT_ID_SIZE  8
#define SIGNATURE_SIZE    64
#define MAX_PACKET_SIZE  512   // BLE 5.0; ajusta según tu MTU
#define CACHE_TIMEOUT  43200   // 12 horas (segundos) para store-and-forward
#define MAX_CACHE_SIZE   100   // Máximo mensajes cacheados
#define TTL_INDEX          2   // Posición del byte TTL en el header

// Tipos de fragmento (MessageType)
#define FRAGMENT_START    0x05
#define FRAGMENT_CONTINUE 0x06
#define FRAGMENT_END      0x07

// Flags (BinaryProtocol)
#define FLAG_HAS_RECIPIENT 0x01
#define FLAG_HAS_SIGNATURE 0x02
#define FLAG_IS_COMPRESSED 0x04

// ── Estado global ──────────────────────────────────────────────────────────
std::set<std::string> seenMessages;   // Dedup por senderID + timestamp + type
std::map<std::string, std::pair<std::string, unsigned long>> messageCache;
                                       // recipientID -> (paquete, timestamp)

BLEServer*         pServer         = nullptr;
BLECharacteristic* pCharacteristic = nullptr;

// ── Declaraciones anticipadas ──────────────────────────────────────────────
void processAndRelay(std::string packet);
void forwardPacket(std::string packet);
void cleanupCache();

// ── Callbacks del servidor BLE (modo periférico) ───────────────────────────
class MyServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer* pServer) override {
    Serial.println("[BLE] Dispositivo conectado como periférico.");
  }
  void onDisconnect(BLEServer* pServer) override {
    Serial.println("[BLE] Dispositivo desconectado.");
    // Reinicia advertising para aceptar nuevas conexiones
    BLEDevice::startAdvertising();
  }
};

// ── Callback de escritura en característica ────────────────────────────────
class MyCharacteristicCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic* pCharacteristic) override {
    String rxValue = pCharacteristic->getValue();
    if (rxValue.length() > 0) {
      Serial.printf("[RX] Paquete recibido: %d bytes\n", rxValue.length());
      std::string packet(rxValue.c_str(), rxValue.length());
      processAndRelay(packet);
    }
  }
};

// ── Procesa y reenvía un paquete BitChat ───────────────────────────────────
void processAndRelay(std::string packet) {
  if ((int)packet.length() < HEADER_SIZE + SENDER_ID_SIZE) {
    Serial.println("[WARN] Paquete demasiado corto, descartando.");
    return;
  }

  // Parseo del header
  // uint8_t version    = packet[0];   // reservado para uso futuro
  uint8_t type       = packet[1];
  uint8_t ttl        = packet[TTL_INDEX];
  std::string timestamp_str = packet.substr(3, 8);
  uint8_t flags      = packet[11];
  // uint16_t payloadLen = ((uint8_t)packet[12] << 8) | (uint8_t)packet[13]; // reservado

  int offset = HEADER_SIZE;
  std::string senderID = packet.substr(offset, SENDER_ID_SIZE);
  offset += SENDER_ID_SIZE;

  std::string recipientID = "";
  bool hasRecipient = (flags & FLAG_HAS_RECIPIENT) != 0;
  if (hasRecipient && (int)packet.length() >= offset + RECIPIENT_ID_SIZE) {
    recipientID = packet.substr(offset, RECIPIENT_ID_SIZE);
    offset += RECIPIENT_ID_SIZE;
  }

  // ID único del mensaje para deduplicación
  std::string messageID = senderID + timestamp_str + std::string(1, (char)type);

  if (seenMessages.count(messageID)) {
    Serial.println("[DEDUP] Paquete duplicado, ignorando.");
    return;
  }
  seenMessages.insert(messageID);

  // Limita el set de dedup para no desbordar la RAM
  if (seenMessages.size() > 1000) {
    seenMessages.clear();
    Serial.println("[DEDUP] Cache de dedup limpiado (límite 1000).");
  }

  // Verifica TTL
  if (ttl == 0) {
    Serial.println("[TTL] Expirado, descartando paquete.");
    return;
  }
  ttl--;
  packet[TTL_INDEX] = (char)ttl;
  Serial.printf("[TTL] Restante: %d\n", ttl);

  // Los fragmentos se reenvían tal cual sin reensamblar
  if (type == FRAGMENT_START || type == FRAGMENT_CONTINUE || type == FRAGMENT_END) {
    Serial.println("[FRAG] Fragmento detectado, reenviando sin reensamblar.");
  }

  // Store-and-forward: cachea mensajes privados
  if (hasRecipient) {
    std::string broadcastID(RECIPIENT_ID_SIZE, (char)0xFF);
    if (recipientID != broadcastID) {
      unsigned long now = millis() / 1000;
      messageCache[recipientID] = { packet, now };
      Serial.println("[CACHE] Mensaje privado cacheado para destinatario offline.");
    }
  }

  forwardPacket(packet);
}

// ── Escanea y reenvía el paquete a dispositivos BitChat cercanos ───────────
void forwardPacket(std::string packet) {
  Serial.println("[FWD] Escaneando dispositivos BitChat...");

  BLEScan* pBLEScan = BLEDevice::getScan();
  pBLEScan->setActiveScan(true);
  pBLEScan->setInterval(100);
  pBLEScan->setWindow(99);
  BLEScanResults* results = pBLEScan->start(5, false);  // 5 segundos de escaneo

  if (!results) {
    Serial.println("[FWD] Sin resultados de escaneo.");
    return;
  }

  int forwarded = 0;
  for (int i = 0; i < results->getCount(); i++) {
    BLEAdvertisedDevice device = results->getDevice(i);

    if (!device.haveServiceUUID() ||
        !device.isAdvertisingService(BLEUUID(SERVICE_UUID))) {
      continue;
    }

    Serial.printf("[FWD] Dispositivo BitChat encontrado: %s\n",
                  device.getAddress().toString().c_str());

    BLEClient* pClient = BLEDevice::createClient();
    pClient->setClientCallbacks(nullptr);

    if (!pClient->connect(&device)) {
      Serial.println("[FWD] No se pudo conectar.");
      pClient->disconnect();
      delete pClient;
      continue;
    }

    BLERemoteService* pRemoteService =
        pClient->getService(BLEUUID(SERVICE_UUID));
    if (!pRemoteService) {
      Serial.println("[FWD] Servicio no encontrado en dispositivo.");
      pClient->disconnect();
      delete pClient;
      continue;
    }

    BLERemoteCharacteristic* pRemoteChar =
        pRemoteService->getCharacteristic(BLEUUID(CHARACTERISTIC_UUID));
    if (!pRemoteChar || !pRemoteChar->canWriteNoResponse()) {
      Serial.println("[FWD] Característica no disponible.");
      pClient->disconnect();
      delete pClient;
      continue;
    }

    // Envía respetando el MTU (fragmenta si el paquete es más grande)
    size_t mtu = pClient->getMTU();
    size_t chunkSize = (mtu > 3) ? (mtu - 3) : 20;

    if (packet.length() > chunkSize) {
      for (size_t pos = 0; pos < packet.length(); pos += chunkSize) {
        std::string chunk = packet.substr(pos, chunkSize);
        pRemoteChar->writeValue(
            (uint8_t*)chunk.c_str(), chunk.size(), false);
        delay(10);  // pequeña pausa entre chunks
      }
      Serial.printf("[FWD] Enviado en %d chunks (MTU=%d).\n",
                    (int)((packet.length() + chunkSize - 1) / chunkSize),
                    (int)mtu);
    } else {
      pRemoteChar->writeValue(
          (uint8_t*)packet.c_str(), packet.size(), false);
      Serial.println("[FWD] Paquete enviado completo.");
    }

    forwarded++;
    pClient->disconnect();
    delete pClient;
  }

  pBLEScan->clearResults();
  Serial.printf("[FWD] Reenviado a %d dispositivos.\n", forwarded);
}

// ── Limpieza periódica del cache (mensajes expirados) ─────────────────────
void cleanupCache() {
  unsigned long now = millis() / 1000;
  int removed = 0;
  for (auto it = messageCache.begin(); it != messageCache.end(); ) {
    if (now - it->second.second > CACHE_TIMEOUT ||
        messageCache.size() > MAX_CACHE_SIZE) {
      it = messageCache.erase(it);
      removed++;
    } else {
      ++it;
    }
  }
  if (removed > 0) {
    Serial.printf("[CACHE] Limpiados %d mensajes expirados.\n", removed);
  }
}

// ── setup() ───────────────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  Serial.println("\n╔══════════════════════════════════╗");
  Serial.println("║  BitChat ESP32 BLE Relay          ║");
  Serial.println("╚══════════════════════════════════╝");

  // Inicializa BLE con nombre visible en la red BitChat
  BLEDevice::init("bitchat-relay");
  BLEDevice::setMTU(512);

  // ── Modo Periférico (acepta conexiones entrantes) ──
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  BLEService* pService = pServer->createService(SERVICE_UUID);

  pCharacteristic = pService->createCharacteristic(
      CHARACTERISTIC_UUID,
      BLECharacteristic::PROPERTY_READ  |
      BLECharacteristic::PROPERTY_WRITE |
      BLECharacteristic::PROPERTY_WRITE_NR |
      BLECharacteristic::PROPERTY_NOTIFY
  );
  pCharacteristic->setCallbacks(new MyCharacteristicCallbacks());
  pService->start();

  // ── Advertising ──
  BLEAdvertising* pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(true);
  pAdvertising->setMinPreferred(0x06);
  pAdvertising->setMaxPreferred(0x12);
  BLEDevice::startAdvertising();

  Serial.println("[BLE] Advertising iniciado — visible como 'bitchat-relay'");
  Serial.println("[BLE] Esperando mensajes...\n");
}

// ── loop() ────────────────────────────────────────────────────────────────
unsigned long lastCleanup = 0;

void loop() {
  // Limpieza del cache cada 10 minutos
  unsigned long now = millis();
  if (now - lastCleanup > 600000UL) {
    cleanupCache();
    lastCleanup = now;
  }

  delay(100);
}
