#ifndef A7680_THINGSBOARD_H_
#define A7680_THINGSBOARD_H_

#include <Arduino.h>

class A7680Modem {
 public:
  explicit A7680Modem(Stream& serial);

  void setDebug(Stream& debug);
  void setCommandTimeout(uint32_t timeoutMs);

  bool testAT(uint32_t timeoutMs = 2000);
  bool beginNetwork(const char* apn,
                    const char* simPin = nullptr,
                    uint32_t networkTimeoutMs = 60000,
                    int cid = 1);
  bool setAPN(const char* apn, int cid = 1);
  bool activatePDP(int cid = 1);
  bool deactivatePDP(int cid = 1);
  bool isPDPActive(int cid = 1);
  bool waitForSIMReady(const char* simPin = nullptr, uint32_t timeoutMs = 30000);
  bool waitForNetwork(uint32_t timeoutMs = 60000);
  int  getRSSI();
  String getOperator();
  String getModel();
  String getFirmware();

  bool sendAT(const String& cmd);
  bool sendCommandExpectOK(const String& cmd, uint32_t timeoutMs = 5000);
  bool waitForPrompt(uint32_t timeoutMs = 5000);
  bool waitForOK(uint32_t timeoutMs = 5000);
  bool waitForResult(const String& wantedPrefix, String& matchedLine, uint32_t timeoutMs = 5000);
  bool readLine(String& line, uint32_t timeoutMs = 1000);
  size_t readBytesExact(uint8_t* buffer, size_t len, uint32_t timeoutMs = 5000);
  void purgeInput(uint32_t quietMs = 50);
  void writeRaw(const uint8_t* data, size_t len);
  void writeRaw(const char* data);

  int lastError() const;
  String lastErrorText() const;

 private:
  Stream& _serial;
  Stream* _debug;
  uint32_t _defaultTimeoutMs;
  int _lastError;
  String _lastErrorText;

  void setError(int code, const String& text);
  bool parseRegLine(const String& line, int& statOut) const;
  void debugPrint(const String& s);
};

class A7680MqttClient {
 public:
  typedef void (*MessageCallback)(const String& topic, const String& payload);

  explicit A7680MqttClient(A7680Modem& modem, int clientIndex = 0);

  void setCallback(MessageCallback cb);
  void setKeepAlive(uint16_t keepAliveSec);
  void setCleanSession(bool cleanSession);

  bool start();
  bool stop();
  bool acquire(const char* clientId, bool useSSL = false);
  bool release();
  bool connect(const char* host,
               uint16_t port,
               const char* username = nullptr,
               const char* password = nullptr);
  bool disconnect(uint16_t timeoutSec = 60);
  bool isConnected();

  bool publish(const char* topic,
               const char* payload,
               uint8_t qos = 1,
               uint8_t pubTimeoutSec = 60);
  bool publish(const String& topic,
               const String& payload,
               uint8_t qos = 1,
               uint8_t pubTimeoutSec = 60);
  bool subscribe(const char* topic, uint8_t qos = 1);
  bool unsubscribe(const char* topic);

  void loop(uint32_t timeoutMs = 20);
  int lastError() const;
  String lastErrorText() const;

 private:
  A7680Modem& _modem;
  MessageCallback _callback;
  String _clientId;
  int _clientIndex;
  bool _serviceStarted;
  bool _acquired;
  bool _connected;
  bool _useSSL;
  uint16_t _keepAliveSec;
  bool _cleanSession;

  bool waitForMqttUrc(const String& prefix, String& line, uint32_t timeoutMs = 15000);
  bool sendDataAfterPrompt(const uint8_t* data, size_t len, uint32_t timeoutMs = 5000);
  bool readIncomingMessage(uint32_t timeoutMs = 2000);
};

class A7680ThingsBoard {
 public:
  typedef void (*RpcCallback)(uint32_t requestId, const String& payload);
  typedef void (*AttributesCallback)(const String& payload);
  typedef void (*AttrResponseCallback)(uint32_t requestId, const String& payload);

  explicit A7680ThingsBoard(A7680MqttClient& mqtt);

  void setRpcCallback(RpcCallback cb);
  void setAttributesCallback(AttributesCallback cb);
  void setAttributesResponseCallback(AttrResponseCallback cb);

  bool connect(const char* host,
               uint16_t port,
               const char* deviceToken,
               const char* clientId = "esp32-a7680");
  bool disconnect();

  bool sendTelemetryJson(const String& json, uint8_t qos = 1);
  bool sendAttributesJson(const String& json, uint8_t qos = 1);
  bool subscribeAttributes();
  bool subscribeRpc();
  bool requestAttributes(uint32_t requestId,
                         const String& clientKeys,
                         const String& sharedKeys);
  bool sendRpcResponse(uint32_t requestId, const String& json, uint8_t qos = 1);

  void loop(uint32_t timeoutMs = 20);
  int lastError() const;
  String lastErrorText() const;

 private:
  static A7680ThingsBoard* _activeInstance;
  A7680MqttClient& _mqtt;
  RpcCallback _rpcCb;
  AttributesCallback _attrCb;
  AttrResponseCallback _attrRespCb;

  static void onMqttMessageStatic(const String& topic, const String& payload);
  void onMqttMessage(const String& topic, const String& payload);
  uint32_t parseTrailingId(const String& topic, const String& prefix) const;
};

#endif
