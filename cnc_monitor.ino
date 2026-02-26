/**
 * Standard libs
 */
#include <stdio.h>
#include <stdarg.h>

/**
 * Finder 7M 
 */
#include <Finder7M.h>

/**
 * RTOS Mbed
 */
#include <rtos.h>
using namespace rtos;

/**
 * Bibliotecas MQTT e WiFi
*/
#include <ArduinoMqttClient.h>
#include <WiFi.h>

/**
 * Bibliotecas de Tempo
 */
#include <NTPClient.h>
#include <WiFiUdp.h>
#include <mbed_mktime.h> // Permite usar set_time() no RTC do Opta

/**
 * LOG (0 - Não; 1 - Sim)
 */
#define LOG_ENABLE 0
#if LOG_ENABLE
  #define LOG(fmt, ...) safePrint(fmt, ##__VA_ARGS__)
#else
  #define LOG(fmt, ...) do {} while (0)
#endif

/**
 * WiFi Credentials
 */
constexpr const char WIFI_SSID[] = "Pos_Sist_Emb";
constexpr const char WIFI_PASS[] = "LPFS@senai109";

/**
 * MQTT Configuration
 */
constexpr const char MQTT_BROKER_IP[] = "broker.app.wnology.io";
constexpr uint16_t MQTT_BROKER_PORT = 1883;

constexpr const char MQTT_CLIENT_ID[] = "694170f6d09b5139ea5d287d";
constexpr const char MQTT_USERNAME[] = "962fdf25-b123-4c75-a6e4-59b368596049";
constexpr const char MQTT_PASSWORD[] = "83154899704548b9c3fc418daa40476d186e03032f4b76d8baf563f0ed32d5c2";

/**
 * Tópicos MQTT
 */
constexpr const char MQTT_TOPIC_STATE[] = "wnology/694170f6d09b5139ea5d287d/state";

/**
 * MQTT Reconnection
 */
static uint32_t lastReconnectAttempt = 0;
constexpr uint32_t RECONNECT_INTERVAL_MS = 5000;

/**
 * Modbus Configuration
 */
constexpr uint8_t MODBUS_7M_ADDRESS = 1;

/**
 * Estrutura de dados de energia
 */
typedef struct {
  float current;              // Corrente média (A)
  float powerFactor;          // Fator de potência
  int16_t voltage;           // Tensão média (V)
  int16_t activePower;        // Potência ativa total (kW)
  int16_t reactivePower;      // Potência reativa total (kVAr)
  int16_t apparentPower;      // Potência aparente total (kVA)
  int16_t frequency;          // Frequência (Hz)
  int16_t activeEnergy;       // Energia ativa acumulada (kWh)
} energy_data_t;

/**
 * Estrutura de dados da máquina
 */
typedef struct {
  uint16_t spindleSpeed;  // Velocidade Fuso   
  uint8_t machineOn;      // Máquina Ligada
  uint8_t doorOpen;       // Porta Aberta
  uint8_t doorClosed;     // Porta Fechada
  uint8_t clampOn;        // Peça Fixada
} machine_data_t;

typedef struct {
  uint32_t machineOnTimeMs;   // Tempo acumulado de máquina ligada (ms)
  uint32_t cuttingTimeMs;     // Tempo acumulado em corte (ms)
  uint32_t partCount;         // Contagem total de peças
  uint32_t lastCycleTimeMs;   // Tempo do último ciclo completo (ms)
  float utilizationRate;      // Taxa de utilização (%)
  float cadence;              // Peças por hora (projetado)
} kpi_data_t;

/**
 * Enum de estados
 */
enum class LedState
{
  OFF,
  ON,
  BLINK_SLOW,
  BLINK_FAST,
  PULSE
};

/**
 * Estrutura de LED
 */
typedef struct{
  uint8_t   pin;
  LedState  state;
  uint32_t  lastToggleMs;
} led_t;

/**
 * Cria variáveis LED
 */
led_t systemLed;
led_t wifiLed;
led_t mqttLed;
led_t modbusLed;

// Variáveis Globais das Estruturas (Tabela de Estados)
machine_data_t globalMachineData = {0};
kpi_data_t     globalKpis        = {0};
energy_data_t  globalEnergyData  = {0};

/*
 * Cria objetos WiFiClient e MqttClient
 * Cria objeto finder7m
 */
WiFiClient wifiClient;
MqttClient mqttClient(wifiClient);
Finder7M f7m;

/**
 * Mutex da Serial para compartilhamento entre Threads
 */
Mutex serialMutex;

/**
 * Mutexes de proteção de variáveis globais
 */
Mutex machineMutex;
Mutex energyMutex;

/**
 * Configuração NTP
 * UTC (offset 0) para o RTC interno. 
 */
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "pool.ntp.org", 0);

/**
 * Protótipos de função
 */
void mqttThread();
void modbusThread();
void machineThread();
void ledManagerThread();
bool readEnergyParameters(energy_data_t *data);
void publishDeviceState(const energy_data_t* energy, const machine_data_t* machine, const kpi_data_t* kpi);
void connectWifi();
void connectMqtt();
void handleMqttRecon();
void updateLed(led_t &led);
void syncRtcWithNtp();
void getIsoTime(char* buffer, size_t size);
void safePrint(const char *fmt, ...);

/**
 * Função de callback chamada quando uma mensagem é publicada em um tópico assinado
 */
void onMqttMessage(int messageSize) 
{
  while (mqttClient.available())
  {
    mqttClient.read();
  }
}

/**
 * setup
 */
void setup() 
{
  /**
   * Inicia a Serial
   */
  Serial.begin(9600);

  /**
   * Configuração dos terminais do Relé OPTA
   */
  pinMode(PIN_A0, INPUT);
  pinMode(PIN_A1, INPUT);
  pinMode(PIN_A2, INPUT);
  pinMode(PIN_A3, INPUT);
  pinMode(PIN_A4, INPUT);
  analogReadResolution(12);

  pinMode(LED_D0, OUTPUT);
  pinMode(LED_D1, OUTPUT);
  pinMode(LED_D2, OUTPUT);
  pinMode(LED_D3, OUTPUT);  

  /**
   * Cria e inicia as threads
   */
  static Thread ledTh;
  ledTh.start(&ledManagerThread);

  static Thread mbTh;
  mbTh.start(&modbusThread);

  static Thread machineTh;
  machineTh.start(&machineThread);  

  static Thread mqttTh;
  mqttTh.start(&mqttThread);
}

/**
 * loop()
 */
void loop()
{
  delay(1000);
}

/**
 * Thread para publicar os dados de energia ao broker MQTT
 */
void mqttThread()
{
  // Conexão WiFi
  connectWifi();

  // Sincroniza o relógio
  syncRtcWithNtp();  

  // Conexao MQTT
  connectMqtt();

  // Aumenta o buffer MQTT para 1024 bytes
  mqttClient.setTxPayloadSize(1024);

  // Variáveis de escopo
  uint32_t lastPublishTime = 0;
  uint8_t lastReportedClamp = 0;
  uint8_t lastReportedDoorOpen = 0;
  uint8_t lastReportedDoorClosed = 0;
  uint8_t lastReportedMachineOn = 0;
  uint16_t lastReportedSpindle = 0;

  for(;;)
  {
    // Keep Alive MQTT
    mqttClient.poll();  

    // Verifica conexão MQTT
    handleMqttRecon();

    if (mqttClient.connected()) 
    {
      // Copia estado atual das estruturas protegidas por mutex
      machine_data_t snapMachine;
      kpi_data_t snapKpi;
      energy_data_t snapEnergy;

      machineMutex.lock();
      snapMachine = globalMachineData;
      snapKpi     = globalKpis;
      machineMutex.unlock();

      energyMutex.lock();
      snapEnergy = globalEnergyData;
      energyMutex.unlock();

      // Avalia regras de gatilho para publicação
      bool forcePublish = false;
      uint32_t now = Kernel::get_ms_count();

      // Regra: Mudança de Estado
      if (snapMachine.clampOn != lastReportedClamp ||
          snapMachine.doorOpen != lastReportedDoorOpen ||
          snapMachine.doorClosed != lastReportedDoorClosed ||
          snapMachine.machineOn != lastReportedMachineOn ||
          (abs(snapMachine.spindleSpeed - lastReportedSpindle)) > 50) 
          {
          forcePublish = true;
      }

      // Regra: Publicação periódica a cada 60 segundos
      if (now - lastPublishTime >= 60000) {
          forcePublish = true;
      }

      // Publicação
      if (forcePublish) {

        // Atualiza controles
        lastPublishTime = now;
        lastReportedClamp = snapMachine.clampOn;
        lastReportedDoorOpen = snapMachine.doorOpen;
        lastReportedDoorClosed = snapMachine.doorClosed;
        lastReportedMachineOn = snapMachine.machineOn;
        lastReportedSpindle = snapMachine.spindleSpeed;

        // Publica estruturas
        publishDeviceState(&snapEnergy, &snapMachine, &snapKpi);
      }
    }

    // Delay
    ThisThread::sleep_for(50);  
  }
}

/**
 * Thread de leitura dos dados via Modbus RTU
 */
void modbusThread() 
{
  /**
   * Inicia cliente (master) Modbus RTU
   */
  if (!f7m.init(38400)) {
    LOG("Finder 7M Init Fault!");
    modbusLed.state = LedState::BLINK_FAST;
  } else {
    LOG("Finder 7M Ready!");
  }

  for(;;)
  {
    energy_data_t localEnergy;
    
    if (readEnergyParameters(&localEnergy)) {
      // Atualiza tabela de estado se leitura foi OK
      energyMutex.lock();
      globalEnergyData = localEnergy;
      energyMutex.unlock();
      
      modbusLed.state = LedState::PULSE;
    } else {
      LOG("Modbus Read Error!");
      modbusLed.state = LedState::BLINK_FAST;
    }

    ThisThread::sleep_for(5000);
  }
}

/**
 * Thread de leitura dos dados da máquina e cálculo de kpis
 */
void machineThread()
{
  uint32_t lastUpdateTime = millis();
  uint8_t lastClampState = 0;
  uint32_t lastCycleStartMs = 0;

  // Parâmetros da média móvel exponencial
  float smoothedCycleTimeMs = 0.0f;
  const float alpha = 0.2f;

  for(;;)
  {
    // Leitura rápida de hardware
    uint8_t mOn      = digitalRead(PIN_A0);
    uint16_t spindle = analogRead(PIN_A1) * 0.9766;    
    uint8_t doorC    = digitalRead(PIN_A2);
    uint8_t doorO    = digitalRead(PIN_A3);
    uint8_t cOn      = digitalRead(PIN_A4);

    uint32_t now = millis();
    uint32_t dt = now - lastUpdateTime;
    lastUpdateTime = now;

    // Trava Mutex para atualizar variáveis globais
    machineMutex.lock();

    globalMachineData.machineOn    = mOn;
    globalMachineData.spindleSpeed = spindle;
    globalMachineData.doorClosed   = doorC;
    globalMachineData.doorOpen     = doorO;
    globalMachineData.clampOn      = cOn;

    // Integração de Tempos
    if (mOn) {
      globalKpis.machineOnTimeMs += dt;
    }

    if (spindle > 100 && cOn) {
      globalKpis.cuttingTimeMs += dt;
    }

    // Borda de subida (Contagem e Tempo de Ciclo)
    if (cOn && !lastClampState) {
      globalKpis.partCount++;
      
      if (lastCycleStartMs > 0) {
          uint32_t currentCycleMs = now - lastCycleStartMs;
          
          if (currentCycleMs > 0) {
              // Inicializa o filtro na primeira peça válida
              if (smoothedCycleTimeMs == 0.0f) {
                  smoothedCycleTimeMs = currentCycleMs;
              } else {
                  // Aplica a média móvel exponencial
                  smoothedCycleTimeMs = (alpha * currentCycleMs) + ((1.0f - alpha) * smoothedCycleTimeMs);
              }
              
              globalKpis.lastCycleTimeMs = currentCycleMs; // Mantém o real para o log
              globalKpis.cadence = 3600000.0f / smoothedCycleTimeMs; // Cadência suavizada
          }
      }
      lastCycleStartMs = now;
    }
    lastClampState = cOn;

    // Taxa de utilização
    if (globalKpis.machineOnTimeMs > 0) {
      globalKpis.utilizationRate = ((float)globalKpis.cuttingTimeMs / (float)globalKpis.machineOnTimeMs) * 100.0f;
    } else {
      globalKpis.utilizationRate = 0.0f;
    }

    machineMutex.unlock();

    // Taxa de amostragem cravada
    ThisThread::sleep_for(50);
  }
}

/**
 * Thread para atualização dos LEDs de diagnóstico
 */
void ledManagerThread()
{
  systemLed.pin = LED_D0;
  systemLed.state = LedState::OFF;

  wifiLed.pin = LED_D1;
  wifiLed.state = LedState::OFF;

  mqttLed.pin = LED_D2;
  mqttLed.state = LedState::OFF;

  modbusLed.pin = LED_D3;
  modbusLed.state = LedState::OFF;

  for (;;)
  {
    systemLed.state = LedState::PULSE;
    updateLed(systemLed);
    updateLed(wifiLed);
    updateLed(mqttLed);
    updateLed(modbusLed);

    ThisThread::sleep_for(50);
  }
}

/**
 * Função para ler todos os parâmetros de energia do Finder 7M
 */
bool readEnergyParameters(energy_data_t *data)
{
  bool success = true;

  // Tensão
  Measure voltage = f7m.getVoltagePhase1(MODBUS_7M_ADDRESS);
  data->voltage = int16_t(voltage.toFloat());
  success &= !voltage.isReadError();
  delay(10);

  // Potência Ativa
  Measure activePower = f7m.getActivePowerTotal(MODBUS_7M_ADDRESS);
  data->activePower = int16_t(activePower.toFloat());
  success &= !activePower.isReadError();
  delay(10);

  // Potência Reativa
  Measure reactivePower = f7m.getReactivePowerTotal(MODBUS_7M_ADDRESS);
  data->reactivePower = int16_t(reactivePower.toFloat());
  success &= !reactivePower.isReadError();
  delay(10);

  // Potência Aparente
  Measure apparentPower = f7m.getApparentPowerTotal(MODBUS_7M_ADDRESS);
  data->apparentPower = int16_t(apparentPower.toFloat());
  success &= !apparentPower.isReadError();
  delay(10);

  // Fator de Potência
  Measure powerFactor = f7m.getPowerFactorTotal(MODBUS_7M_ADDRESS);
  data->powerFactor = powerFactor.toFloat();
  success &= !powerFactor.isReadError();
  delay(10);

  // Frequência
  Measure frequency = f7m.getFrequency(MODBUS_7M_ADDRESS);
  data->frequency = int16_t(frequency.toFloat());
  success &= !frequency.isReadError();
  delay(10);

  // Corrente
  Measure current = f7m.getCurrentPhase1(MODBUS_7M_ADDRESS);
  data->current = current.toFloat();
  success &= !current.isReadError();
  delay(10);

  // Energia (Ativa)
  Measure activeEnergy = f7m.getMIDInActiveEnergy(MODBUS_7M_ADDRESS);
  data->activeEnergy = int16_t(activeEnergy.toFloat());
  success &= !activeEnergy.isReadError();

  return success;
}

/**
 * Função para publicar todos os dados via MQTT
 */
void publishDeviceState(const energy_data_t* energy,
                        const machine_data_t* machine,
                        const kpi_data_t* kpi)
{
  if (!mqttClient.connected()){
    return;
  }

  //Obter o Timestamp atual
  char timeStr[32];
  getIsoTime(timeStr, sizeof(timeStr));

  char payload[800];

  int len = snprintf(
    payload,
    sizeof(payload),
    "{"
      "\"time\":\"%s\","
      "\"data\":{"
        "\"voltage\":%d,"
        "\"current\":%.2f,"
        "\"activePower\":%d,"
        "\"reactivePower\":%d,"
        "\"apparentPower\":%d,"
        "\"powerFactor\":%.2f,"
        "\"frequency\":%d,"
        "\"activeEnergy\":%d,"
        "\"machineOn\":%u,"
        "\"doorOpen\":%u,"
        "\"doorClosed\":%u,"
        "\"clampOn\":%u,"
        "\"spindleSpeed\":%d,"
        "\"machineOnTimeS\":%lu,"
        "\"cuttingTimeS\":%lu,"
        "\"utilizationRate\":%.1f,"
        "\"partCount\":%lu,"
        "\"cycleTimeS\":%.1f,"
        "\"cadence\":%.1f"        
      "}"
    "}",
    // Timestamp
    timeStr,
    // Energy
    (int)energy->voltage,
    (double)energy->current,
    (int)energy->activePower,
    (int)energy->reactivePower,
    (int)energy->apparentPower,
    (double)energy->powerFactor,
    (int)energy->frequency,
    (int)energy->activeEnergy,
    // Machine
    (unsigned int)machine->machineOn,
    (unsigned int)machine->doorOpen,
    (unsigned int)machine->doorClosed,
    (unsigned int)machine->clampOn,
    (int)machine->spindleSpeed,
    // KPI
    (unsigned long)(kpi->machineOnTimeMs / 1000), 
    (unsigned long)(kpi->cuttingTimeMs / 1000),
    (double)kpi->utilizationRate,
    (unsigned long)kpi->partCount,
    (double)(kpi->lastCycleTimeMs / 1000.0),     
    (double)kpi->cadence
  );

  if (len <= 0 || len >= (int)sizeof(payload)){
    LOG("State JSON formatting error");
    return;
  }

  if (!mqttClient.beginMessage(MQTT_TOPIC_STATE, (unsigned long)len)){
    LOG("MQTT beginMessage failed (state)");
    return;
  }

  mqttClient.print(payload);

  if (!mqttClient.endMessage()){
    LOG("MQTT publish failed (state)");
    return;
  }

  LOG("Device state published");
}

/**
 * Conecta-se a um AP Wi-Fi
 */
void connectWifi()
{
  /**
   * Tentativa de conexão WiFi com o AP
   */
  int retries = 0;
  while (WiFi.begin(WIFI_SSID, WIFI_PASS) != WL_CONNECTED)
  {
    LOG("Connecting to AP: %s", WIFI_SSID);
    wifiLed.state = LedState::BLINK_SLOW;

    if (++retries > 10){
      LOG("WiFi connection timeout");
      return;
    }
    ThisThread::sleep_for(5000);
  }

  // Conectado ao AP
  String ip = WiFi.localIP().toString();
  LOG("WiFi Connected! Got IP: %s", ip.c_str());
  wifiLed.state = LedState::ON;
}

/**
 * Conecta-se a um broker MQTT
 */
void connectMqtt()
{
  /**
   * Credenciais e Client ID (Wegnology)
   */
  mqttClient.setId(MQTT_CLIENT_ID);
  mqttClient.setUsernamePassword(MQTT_USERNAME, MQTT_PASSWORD);

  /**
   * Tentativa de conexão com o broker MQTT
  */
  if (!mqttClient.connect(MQTT_BROKER_IP, MQTT_BROKER_PORT)) {
    LOG("MQTT Connection Error: %d", mqttClient.connectError());
  } else {
    LOG("Connected to MQTT Broker!"); 
    mqttLed.state = LedState::ON;
  }

  /**
   * Registra função de callback para eventos
   * Assina tópicos de interesse
   */
  mqttClient.onMessage(onMqttMessage);  
}

/**
 * Monitora conexão MQTT
 */
void handleMqttRecon()
{
  /**
    * Lógica de reconexão com backoff
    */
  uint32_t now = Kernel::get_ms_count();

  if (!mqttClient.connected())
  {
    if (now - lastReconnectAttempt >= RECONNECT_INTERVAL_MS)
    {
      lastReconnectAttempt = now;

      LOG("MQTT disconnected, trying reconnect...");
      mqttLed.state = LedState::BLINK_SLOW;

      if (mqttClient.connect(MQTT_BROKER_IP, MQTT_BROKER_PORT)){
        LOG("MQTT reconnected");
        mqttLed.state = LedState::ON;
        } else {
          LOG("MQTT reconnect failed (%d)",
              mqttClient.connectError());
      }
    }
  }
}

/**
 * Atualiza estado dos LEDs
 */
void updateLed(led_t &led)
{
  uint32_t now = Kernel::get_ms_count();

  switch (led.state)
  {
  case LedState::OFF:
    digitalWrite(led.pin, LOW);
    break;

  case LedState::ON:
    digitalWrite(led.pin, HIGH);
    break;

    case LedState::BLINK_SLOW:
      if ((now - led.lastToggleMs) >= 500)
      {
        digitalWrite(led.pin, !digitalRead(led.pin));
        led.lastToggleMs = now;
      }
      break;

    case LedState::BLINK_FAST:
      if ((now - led.lastToggleMs) >= 150)
      {
        digitalWrite(led.pin, !digitalRead(led.pin));
        led.lastToggleMs = now;
      }
      break;

    case LedState::PULSE:
      digitalWrite(led.pin, HIGH);
      led.lastToggleMs = now;
      led.state = LedState::OFF;
      break;
  }
}

/**
 * Sincroniza o RTC interno do Opta com o servidor NTP
 */
void syncRtcWithNtp() 
{
  LOG("Syncing RTC with NTP...");
  timeClient.begin();
  
  // Tenta atualizar. Se falhar, não trava o sistema.
  if (timeClient.update()) {
    unsigned long epoch = timeClient.getEpochTime();
    set_time(epoch); // Função do Mbed OS que ajusta o RTC do hardware
    LOG("RTC Updated! Epoch: %lu", epoch);
  } else {
    LOG("NTP Sync Failed");
  }
  
  timeClient.end(); // Libera a porta UDP
}

/**
 * Gera timestamp no formato ISO-8601 (UTC)
 * Exemplo: "2023-10-27T12:34:56Z"
 */
void getIsoTime(char* buffer, size_t size) 
{
  time_t now = time(NULL); // Lê o RTC interno do Opta
  struct tm * t = gmtime(&now); // Converte para estrutura de tempo UTC
  
  strftime(buffer, size, "%Y-%m-%dT%H:%M:%SZ", t);
}

/**
 * Função segura para escrita na serial
 */
void safePrint(const char *fmt, ...)
{
  char buffer[128];

  va_list args;
  va_start(args, fmt);
  vsnprintf(buffer, sizeof(buffer), fmt, args);
  va_end(args);

  if (serialMutex.trylock_for(50))
  {
    Serial.println(buffer);
    serialMutex.unlock();
  }
}
