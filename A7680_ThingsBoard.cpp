#include "A7680_ThingsBoard.h"

namespace {

static String trimLine(const String& in) {
  String s = in;
  s.trim();
  return s;
}

static String jsonEscapeList(const String& csv) {
  String out;
  int start = 0;
  while (start < csv.length()) {
    int comma = csv.indexOf(',', start);
    if (comma < 0) comma = csv.length();
    String token = csv.substring(start, comma);
    token.trim();
    if (token.length() > 0) {
      if (out.length() > 0) out += ',';
      out += '"';
      out += token;
      out += '"';
    }
    start = comma + 1;
  }
  return out;
}

}  // namespace

A7680ThingsBoard* A7680ThingsBoard::_activeInstance = nullptr;

// ========================= A7680Modem =========================

A7680Modem::A7680Modem(Stream& serial)
    : _serial(serial),
      _debug(nullptr),
      _defaultTimeoutMs(5000),
      _lastError(0),
      _lastErrorText("") {}

void A7680Modem::setDebug(Stream& debug) { _debug = &debug; }

void A7680Modem::setCommandTimeout(uint32_t timeoutMs) { _defaultTimeoutMs = timeoutMs; }

void A7680Modem::debugPrint(const String& s) {
  if (_debug) {
    _debug->println(s);
  }
}

void A7680Modem::setError(int code, const String& text) {
  _lastError = code;
  _lastErrorText = text;
  if (_debug) {
    _debug->print(F("[A7680] "));
    _debug->println(text);
  }
}

int A7680Modem::lastError() const { return _lastError; }
String A7680Modem::lastErrorText() const { return _lastErrorText; }

void A7680Modem::purgeInput(uint32_t quietMs) {
  uint32_t lastSeen = millis();
  while (millis() - lastSeen < quietMs) {
    while (_serial.available()) {
      _serial.read();
      lastSeen = millis();
    }
    delay(1);
  }
}

bool A7680Modem::sendAT(const String& cmd) {
  debugPrint(String(F(">> ")) + cmd);
  _serial.print(cmd);
  _serial.print("\r\n");
  return true;
}

void A7680Modem::writeRaw(const uint8_t* data, size_t len) {
  if (len > 0) {
    _serial.write(data, len);
  }
}

void A7680Modem::writeRaw(const char* data) {
  if (data) {
    _serial.write(reinterpret_cast<const uint8_t*>(data), strlen(data));
  }
}

bool A7680Modem::readLine(String& line, uint32_t timeoutMs) {
  line = "";
  uint32_t start = millis();
  while (millis() - start < timeoutMs) {
    while (_serial.available()) {
      char c = static_cast<char>(_serial.read());
      if (c == '\r') {
        continue;
      }
      if (c == '\n') {
        if (line.length() == 0) {
          continue;
        }
        debugPrint(String(F("<< ")) + line);
        return true;
      }
      line += c;
      if (line == ">") {
        debugPrint(String(F("<< ")) + line);
        return true;
      }
    }
    delay(1);
  }
  return false;
}


size_t A7680Modem::readBytesExact(uint8_t* buffer, size_t len, uint32_t timeoutMs) {
  size_t got = 0;
  uint32_t start = millis();
  while (got < len && millis() - start < timeoutMs) {
    while (_serial.available() && got < len) {
      buffer[got++] = static_cast<uint8_t>(_serial.read());
      start = millis();
    }
    delay(1);
  }
  return got;
}

bool A7680Modem::waitForPrompt(uint32_t timeoutMs) {
  uint32_t start = millis();
  String line;
  while (millis() - start < timeoutMs) {
    while (_serial.available()) {
      char c = static_cast<char>(_serial.read());
      if (c == '>') {
        debugPrint(F("<< >"));
        return true;
      }
      line += c;
      if (line.indexOf("ERROR") >= 0) {
        setError(-10, trimLine(line));
        return false;
      }
    }
    delay(1);
  }
  setError(-11, F("Timeout waiting for prompt"));
  return false;
}

bool A7680Modem::waitForOK(uint32_t timeoutMs) {
  String line;
  uint32_t start = millis();
  while (millis() - start < timeoutMs) {
    if (!readLine(line, 100)) {
      continue;
    }
    line = trimLine(line);
    if (line.length() == 0) continue;
    if (line == "OK") return true;
    if (line == "ERROR" || line.startsWith("+CME ERROR") || line.startsWith("+CMS ERROR")) {
      setError(-12, line);
      return false;
    }
  }
  setError(-13, F("Timeout waiting for OK"));
  return false;
}

bool A7680Modem::waitForResult(const String& wantedPrefix, String& matchedLine, uint32_t timeoutMs) {
  matchedLine = "";
  String line;
  uint32_t start = millis();
  while (millis() - start < timeoutMs) {
    if (!readLine(line, 100)) {
      continue;
    }
    line = trimLine(line);
    if (line.length() == 0) continue;
    if (wantedPrefix.length() && line.startsWith(wantedPrefix)) {
      matchedLine = line;
      return true;
    }
    if (line == "OK") continue;
    if (line == "ERROR" || line.startsWith("+CME ERROR") || line.startsWith("+CMS ERROR")) {
      setError(-14, line);
      return false;
    }
  }
  setError(-15, String(F("Timeout waiting for: ")) + wantedPrefix);
  return false;
}

bool A7680Modem::sendCommandExpectOK(const String& cmd, uint32_t timeoutMs) {
  purgeInput();
  sendAT(cmd);
  return waitForOK(timeoutMs);
}

bool A7680Modem::testAT(uint32_t timeoutMs) {
  purgeInput();
  for (int i = 0; i < 5; ++i) {
    if (sendCommandExpectOK(F("AT"), timeoutMs)) {
      setError(0, F("OK"));
      return true;
    }
    delay(300);
  }
  setError(-1, F("Modem did not answer AT"));
  return false;
}

bool A7680Modem::parseRegLine(const String& line, int& statOut) const {
  int comma = line.lastIndexOf(',');
  if (comma < 0 || comma + 1 >= line.length()) return false;
  statOut = line.substring(comma + 1).toInt();
  return true;
}

bool A7680Modem::waitForSIMReady(const char* simPin, uint32_t timeoutMs) {
  uint32_t start = millis();
  while (millis() - start < timeoutMs) {
    purgeInput();
    sendAT(F("AT+CPIN?"));
    String line;
    bool gotReady = false;
    uint32_t stepStart = millis();
    while (millis() - stepStart < 3000) {
      if (!readLine(line, 150)) continue;
      line = trimLine(line);
      if (line.startsWith("+CPIN:")) {
        if (line.indexOf("READY") >= 0) {
          gotReady = true;
        } else if (line.indexOf("SIM PIN") >= 0 && simPin && strlen(simPin) > 0) {
          if (!sendCommandExpectOK(String(F("AT+CPIN=\"")) + simPin + '"', 10000)) {
            return false;
          }
        }
      }
      if (line == "OK") {
        if (gotReady) return true;
        break;
      }
      if (line == "ERROR" || line.startsWith("+CME ERROR")) {
        break;
      }
    }
    delay(500);
  }
  setError(-2, F("SIM not ready"));
  return false;
}

bool A7680Modem::waitForNetwork(uint32_t timeoutMs) {
  uint32_t start = millis();
  while (millis() - start < timeoutMs) {
    String line;
    int stat = 0;

    purgeInput();
    sendAT(F("AT+CEREG?"));
    uint32_t t1 = millis();
    while (millis() - t1 < 2000) {
      if (!readLine(line, 100)) continue;
      line = trimLine(line);
      if (line.startsWith("+CEREG:") && parseRegLine(line, stat)) {
        if (stat == 1 || stat == 5) return true;
      }
      if (line == "OK" || line == "ERROR") break;
    }

    purgeInput();
    sendAT(F("AT+CGREG?"));
    uint32_t t2 = millis();
    while (millis() - t2 < 2000) {
      if (!readLine(line, 100)) continue;
      line = trimLine(line);
      if (line.startsWith("+CGREG:") && parseRegLine(line, stat)) {
        if (stat == 1 || stat == 5) return true;
      }
      if (line == "OK" || line == "ERROR") break;
    }

    delay(1000);
  }
  setError(-3, F("Network registration timeout"));
  return false;
}

bool A7680Modem::setAPN(const char* apn, int cid) {
  if (!apn || !strlen(apn)) {
    setError(-4, F("APN empty"));
    return false;
  }
  return sendCommandExpectOK(String(F("AT+CGDCONT=")) + cid + F(",\"IP\",\"") + apn + '"', 5000);
}

bool A7680Modem::activatePDP(int cid) {
  if (!sendCommandExpectOK(String(F("AT+CGACT=1,")) + cid, 15000)) {
    return false;
  }
  return isPDPActive(cid);
}

bool A7680Modem::deactivatePDP(int cid) {
  return sendCommandExpectOK(String(F("AT+CGACT=0,")) + cid, 15000);
}

bool A7680Modem::isPDPActive(int cid) {
  purgeInput();
  sendAT(F("AT+CGACT?"));
  String line;
  uint32_t start = millis();
  while (millis() - start < 3000) {
    if (!readLine(line, 100)) continue;
    line = trimLine(line);
    if (line.startsWith("+CGACT:")) {
      int colon = line.indexOf(':');
      int comma = line.indexOf(',', colon + 1);
      if (colon > 0 && comma > colon) {
        int gotCid = line.substring(colon + 1, comma).toInt();
        int state = line.substring(comma + 1).toInt();
        if (gotCid == cid && state == 1) {
          return true;
        }
      }
    }
    if (line == "OK") break;
    if (line == "ERROR") return false;
  }
  setError(-5, F("PDP not active"));
  return false;
}

bool A7680Modem::beginNetwork(const char* apn,
                              const char* simPin,
                              uint32_t networkTimeoutMs,
                              int cid) {
  if (!testAT()) return false;
  sendCommandExpectOK(F("ATE0"), 3000);
  sendCommandExpectOK(F("AT+CMEE=2"), 3000);
  if (!waitForSIMReady(simPin, 30000)) return false;
  if (!waitForNetwork(networkTimeoutMs)) return false;

  // APN co the de trong neu modem da duoc luu cau hinh tu truoc.
  if (apn && strlen(apn) > 0) {
    if (!setAPN(apn, cid)) return false;
  }

  if (isPDPActive(cid)) {
    setError(0, F("PDP already active"));
    return true;
  }

  if (!activatePDP(cid)) return false;
  return true;
}

int A7680Modem::getRSSI() {
  purgeInput();
  sendAT(F("AT+CSQ"));
  String line;
  uint32_t start = millis();
  while (millis() - start < 3000) {
    if (!readLine(line, 100)) continue;
    line = trimLine(line);
    if (line.startsWith("+CSQ:")) {
      int colon = line.indexOf(':');
      int comma = line.indexOf(',', colon + 1);
      if (colon > 0 && comma > colon) {
        return line.substring(colon + 1, comma).toInt();
      }
    }
    if (line == "OK" || line == "ERROR") break;
  }
  return -1;
}

String A7680Modem::getOperator() {
  purgeInput();
  sendAT(F("AT+COPS?"));
  String line;
  uint32_t start = millis();
  while (millis() - start < 3000) {
    if (!readLine(line, 100)) continue;
    line = trimLine(line);
    if (line.startsWith("+COPS:")) return line;
    if (line == "OK" || line == "ERROR") break;
  }
  return "";
}

String A7680Modem::getModel() {
  purgeInput();
  sendAT(F("AT+CGMM"));
  String line, value;
  uint32_t start = millis();
  while (millis() - start < 3000) {
    if (!readLine(line, 100)) continue;
    line = trimLine(line);
    if (line == "OK") break;
    if (line != "AT+CGMM" && line != "ERROR") value = line;
  }
  return value;
}

String A7680Modem::getFirmware() {
  purgeInput();
  sendAT(F("AT+CGMR"));
  String line, value;
  uint32_t start = millis();
  while (millis() - start < 3000) {
    if (!readLine(line, 100)) continue;
    line = trimLine(line);
    if (line == "OK") break;
    if (line != "AT+CGMR" && line != "ERROR") value = line;
  }
  return value;
}

// ========================= A7680MqttClient =========================

A7680MqttClient::A7680MqttClient(A7680Modem& modem, int clientIndex)
    : _modem(modem),
      _callback(nullptr),
      _clientId(""),
      _clientIndex(clientIndex),
      _serviceStarted(false),
      _acquired(false),
      _connected(false),
      _useSSL(false),
      _keepAliveSec(60),
      _cleanSession(true) {}

void A7680MqttClient::setCallback(MessageCallback cb) { _callback = cb; }
void A7680MqttClient::setKeepAlive(uint16_t keepAliveSec) { _keepAliveSec = keepAliveSec; }
void A7680MqttClient::setCleanSession(bool cleanSession) { _cleanSession = cleanSession; }
int A7680MqttClient::lastError() const { return _modem.lastError(); }
String A7680MqttClient::lastErrorText() const { return _modem.lastErrorText(); }

bool A7680MqttClient::start() {
  if (_serviceStarted) return true;
  _modem.purgeInput();
  _modem.sendAT(F("AT+CMQTTSTART"));
  if (!_modem.waitForOK(5000)) return false;
  String urc;
  if (!_modem.waitForResult(F("+CMQTTSTART:"), urc, 15000)) return false;
  int result = urc.substring(13).toInt();
  if (result != 0) {
    _modem.sendCommandExpectOK(F("AT+CMQTTSTOP"), 3000);
    _modem.purgeInput();
    _modem.sendAT(F("AT+CMQTTSTART"));
    if (!_modem.waitForOK(5000)) return false;
    if (!_modem.waitForResult(F("+CMQTTSTART:"), urc, 15000)) return false;
    result = urc.substring(13).toInt();
    if (result != 0) return false;
  }
  _serviceStarted = true;
  return true;
}

bool A7680MqttClient::stop() {
  _modem.purgeInput();
  _modem.sendAT(F("AT+CMQTTSTOP"));
  if (!_modem.waitForOK(5000)) return false;
  String urc;
  if (!_modem.waitForResult(F("+CMQTTSTOP:"), urc, 15000)) return false;
  _serviceStarted = false;
  _acquired = false;
  _connected = false;
  return true;
}

bool A7680MqttClient::acquire(const char* clientId, bool useSSL) {
  if (!_serviceStarted && !start()) return false;
  _clientId = clientId ? clientId : "esp32-a7680";
  _useSSL = useSSL;

  String cmd = String(F("AT+CMQTTACCQ=")) + _clientIndex + F(",\"") + _clientId + '"';
  if (_useSSL) cmd += F(",1");

  if (!_modem.sendCommandExpectOK(cmd, 5000)) return false;
  _acquired = true;
  return true;
}

bool A7680MqttClient::release() {
  if (!_acquired) return true;
  if (!_modem.sendCommandExpectOK(String(F("AT+CMQTTREL=")) + _clientIndex, 5000)) return false;
  _acquired = false;
  _connected = false;
  return true;
}

bool A7680MqttClient::connect(const char* host,
                              uint16_t port,
                              const char* username,
                              const char* password) {
  if (!_acquired && !acquire(_clientId.length() ? _clientId.c_str() : "esp32-a7680", _useSSL)) {
    return false;
  }

  String server = String(F("tcp://")) + host + ':' + String(port);
  String cmd = String(F("AT+CMQTTCONNECT=")) + _clientIndex + F(",\"") + server + F("\",") +
               _keepAliveSec + ',' + (_cleanSession ? '1' : '0');
  if (username && strlen(username) > 0) {
    cmd += F(",\"");
    cmd += username;
    cmd += '"';
    if (password) {
      cmd += F(",\"");
      cmd += password;
      cmd += '"';
    }
  }

  _modem.purgeInput();
  _modem.sendAT(cmd);
  if (!_modem.waitForOK(5000)) return false;

  String urc;
  if (!_modem.waitForResult(F("+CMQTTCONNECT:"), urc, 30000)) return false;

  int comma = urc.lastIndexOf(',');
  int result = (comma >= 0) ? urc.substring(comma + 1).toInt() : -1;
  _connected = (result == 0);
  return _connected;
}

bool A7680MqttClient::disconnect(uint16_t timeoutSec) {
  _modem.purgeInput();
  _modem.sendAT(String(F("AT+CMQTTDISC=")) + _clientIndex + ',' + timeoutSec);
  if (!_modem.waitForOK(5000)) return false;
  String urc;
  if (!_modem.waitForResult(F("+CMQTTDISC:"), urc, (uint32_t)timeoutSec * 1000UL + 5000UL)) return false;
  _connected = false;
  return urc.endsWith(",0");
}

bool A7680MqttClient::isConnected() { return _connected; }

bool A7680MqttClient::sendDataAfterPrompt(const uint8_t* data, size_t len, uint32_t timeoutMs) {
  if (!_modem.waitForPrompt(timeoutMs)) return false;
  _modem.writeRaw(data, len);
  return _modem.waitForOK(timeoutMs);
}

bool A7680MqttClient::publish(const char* topic,
                              const char* payload,
                              uint8_t qos,
                              uint8_t pubTimeoutSec) {
  if (!_connected) return false;
  if (!topic || !payload) return false;

  const size_t tlen = strlen(topic);
  const size_t plen = strlen(payload);

  _modem.purgeInput();
  _modem.sendAT(String(F("AT+CMQTTTOPIC=")) + _clientIndex + ',' + tlen);
  if (!sendDataAfterPrompt(reinterpret_cast<const uint8_t*>(topic), tlen, 5000)) return false;

  _modem.purgeInput();
  _modem.sendAT(String(F("AT+CMQTTPAYLOAD=")) + _clientIndex + ',' + plen);
  if (!sendDataAfterPrompt(reinterpret_cast<const uint8_t*>(payload), plen, 5000)) return false;

  _modem.purgeInput();
  _modem.sendAT(String(F("AT+CMQTTPUB=")) + _clientIndex + ',' + qos + ',' + pubTimeoutSec);
  if (!_modem.waitForOK(5000)) return false;
  String urc;
  if (!_modem.waitForResult(F("+CMQTTPUB:"), urc, (uint32_t)pubTimeoutSec * 1000UL + 5000UL)) return false;
  return urc.endsWith(",0");
}

bool A7680MqttClient::publish(const String& topic,
                              const String& payload,
                              uint8_t qos,
                              uint8_t pubTimeoutSec) {
  return publish(topic.c_str(), payload.c_str(), qos, pubTimeoutSec);
}

bool A7680MqttClient::subscribe(const char* topic, uint8_t qos) {
  if (!_connected || !topic) return false;
  const size_t tlen = strlen(topic);

  _modem.purgeInput();
  _modem.sendAT(String(F("AT+CMQTTSUBTOPIC=")) + _clientIndex + ',' + tlen + ',' + qos);
  if (!sendDataAfterPrompt(reinterpret_cast<const uint8_t*>(topic), tlen, 5000)) return false;

  _modem.purgeInput();
  _modem.sendAT(String(F("AT+CMQTTSUB=")) + _clientIndex);
  if (!_modem.waitForOK(5000)) return false;
  String urc;
  if (!_modem.waitForResult(F("+CMQTTSUB:"), urc, 15000)) return false;
  return urc.endsWith(",0");
}

bool A7680MqttClient::unsubscribe(const char* topic) {
  if (!_connected || !topic) return false;
  const size_t tlen = strlen(topic);

  _modem.purgeInput();
  _modem.sendAT(String(F("AT+CMQTTUNSUB=")) + _clientIndex + ',' + tlen + F(",0"));
  if (!sendDataAfterPrompt(reinterpret_cast<const uint8_t*>(topic), tlen, 5000)) return false;

  String urc;
  if (!_modem.waitForResult(F("+CMQTTUNSUB:"), urc, 15000)) return false;
  return urc.endsWith(",0");
}

bool A7680MqttClient::waitForMqttUrc(const String& prefix, String& line, uint32_t timeoutMs) {
  uint32_t start = millis();
  while (millis() - start < timeoutMs) {
    if (!_modem.readLine(line, 100)) continue;
    line = trimLine(line);
    if (line.startsWith(prefix)) return true;
    if (line == "ERROR" || line.startsWith("+CME ERROR")) return false;
  }
  return false;
}

bool A7680MqttClient::readIncomingMessage(uint32_t timeoutMs) {
  String line;
  if (!waitForMqttUrc(F("+CMQTTRXSTART:"), line, timeoutMs)) return false;

  int firstComma = line.indexOf(',');
  int secondComma = line.indexOf(',', firstComma + 1);
  int thirdComma = line.indexOf(',', secondComma + 1);
  int topicLen = 0;
  int payloadLen = 0;
  if (secondComma > 0 && thirdComma > secondComma) {
    topicLen = line.substring(secondComma + 1, thirdComma).toInt();
    payloadLen = line.substring(thirdComma + 1).toInt();
  } else if (firstComma > 0 && secondComma > firstComma) {
    topicLen = line.substring(firstComma + 1, secondComma).toInt();
    payloadLen = line.substring(secondComma + 1).toInt();
  }

  if (topicLen < 0 || payloadLen < 0) {
    return false;
  }

  if (!waitForMqttUrc(F("+CMQTTRXTOPIC:"), line, 2000)) return false;
  char* topicBuf = new char[topicLen + 1];
  if (!topicBuf) return false;
  memset(topicBuf, 0, topicLen + 1);
  if (_modem.readBytesExact(reinterpret_cast<uint8_t*>(topicBuf), topicLen, 3000) != (size_t)topicLen) {
    delete[] topicBuf;
    return false;
  }

  String dummy;
  _modem.readLine(dummy, 200);

  if (!waitForMqttUrc(F("+CMQTTRXPAYLOAD:"), line, 2000)) {
    delete[] topicBuf;
    return false;
  }
  char* payloadBuf = new char[payloadLen + 1];
  if (!payloadBuf) {
    delete[] topicBuf;
    return false;
  }
  memset(payloadBuf, 0, payloadLen + 1);
  if (_modem.readBytesExact(reinterpret_cast<uint8_t*>(payloadBuf), payloadLen, 5000) != (size_t)payloadLen) {
    delete[] topicBuf;
    delete[] payloadBuf;
    return false;
  }

  _modem.readLine(dummy, 200);
  waitForMqttUrc(F("+CMQTTRXEND:"), line, 2000);

  if (_callback) {
    _callback(String(topicBuf), String(payloadBuf));
  }

  delete[] topicBuf;
  delete[] payloadBuf;
  return true;
}

void A7680MqttClient::loop(uint32_t timeoutMs) {
  if (_connected) {
    readIncomingMessage(timeoutMs);
  }
}

// ========================= A7680ThingsBoard =========================

A7680ThingsBoard::A7680ThingsBoard(A7680MqttClient& mqtt)
    : _mqtt(mqtt), _rpcCb(nullptr), _attrCb(nullptr), _attrRespCb(nullptr) {
  _activeInstance = this;
  _mqtt.setCallback(&A7680ThingsBoard::onMqttMessageStatic);
}

void A7680ThingsBoard::setRpcCallback(RpcCallback cb) { _rpcCb = cb; }
void A7680ThingsBoard::setAttributesCallback(AttributesCallback cb) { _attrCb = cb; }
void A7680ThingsBoard::setAttributesResponseCallback(AttrResponseCallback cb) { _attrRespCb = cb; }

void A7680ThingsBoard::onMqttMessageStatic(const String& topic, const String& payload) {
  if (_activeInstance) {
    _activeInstance->onMqttMessage(topic, payload);
  }
}

uint32_t A7680ThingsBoard::parseTrailingId(const String& topic, const String& prefix) const {
  if (!topic.startsWith(prefix)) return 0;
  return static_cast<uint32_t>(topic.substring(prefix.length()).toInt());
}

void A7680ThingsBoard::onMqttMessage(const String& topic, const String& payload) {
  if (topic == F("v1/devices/me/attributes")) {
    if (_attrCb) _attrCb(payload);
    return;
  }

  if (topic.startsWith(F("v1/devices/me/attributes/response/"))) {
    uint32_t id = parseTrailingId(topic, F("v1/devices/me/attributes/response/"));
    if (_attrRespCb) _attrRespCb(id, payload);
    return;
  }

  if (topic.startsWith(F("v1/devices/me/rpc/request/"))) {
    uint32_t id = parseTrailingId(topic, F("v1/devices/me/rpc/request/"));
    if (_rpcCb) _rpcCb(id, payload);
    return;
  }
}

bool A7680ThingsBoard::connect(const char* host,
                               uint16_t port,
                               const char* deviceToken,
                               const char* clientId) {
  if (!_mqtt.start()) return false;
  if (!_mqtt.acquire(clientId, false)) return false;
  return _mqtt.connect(host, port, deviceToken, nullptr);
}

bool A7680ThingsBoard::disconnect() { return _mqtt.disconnect(); }

bool A7680ThingsBoard::sendTelemetryJson(const String& json, uint8_t qos) {
  return _mqtt.publish(F("v1/devices/me/telemetry"), json, qos, 60);
}

bool A7680ThingsBoard::sendAttributesJson(const String& json, uint8_t qos) {
  return _mqtt.publish(F("v1/devices/me/attributes"), json, qos, 60);
}

bool A7680ThingsBoard::subscribeAttributes() {
  return _mqtt.subscribe("v1/devices/me/attributes", 1) &&
         _mqtt.subscribe("v1/devices/me/attributes/response/+", 1);
}

bool A7680ThingsBoard::subscribeRpc() {
  return _mqtt.subscribe("v1/devices/me/rpc/request/+", 1);
}

bool A7680ThingsBoard::requestAttributes(uint32_t requestId,
                                         const String& clientKeys,
                                         const String& sharedKeys) {
  String topic = String(F("v1/devices/me/attributes/request/")) + requestId;
  String payload = "{";
  bool hasPrev = false;

  if (clientKeys.length()) {
    payload += F("\"clientKeys\":\"");
    payload += clientKeys;
    payload += '"';
    hasPrev = true;
  }

  if (sharedKeys.length()) {
    if (hasPrev) payload += ',';
    payload += F("\"sharedKeys\":\"");
    payload += sharedKeys;
    payload += '"';
  }

  payload += '}';
  return _mqtt.publish(topic, payload, 1, 60);
}

bool A7680ThingsBoard::sendRpcResponse(uint32_t requestId, const String& json, uint8_t qos) {
  String topic = String(F("v1/devices/me/rpc/response/")) + requestId;
  return _mqtt.publish(topic, json, qos, 60);
}

void A7680ThingsBoard::loop(uint32_t timeoutMs) { _mqtt.loop(timeoutMs); }
int A7680ThingsBoard::lastError() const { return _mqtt.lastError(); }
String A7680ThingsBoard::lastErrorText() const { return _mqtt.lastErrorText(); }

