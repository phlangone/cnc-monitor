# Monitoramento de CNC via Internet Industrial das Coisas

## Descrição do Projeto

Este projeto implementa um firmware para monitoramento de uma máquina CNC com um **Arduino Opta**, utilizando conceitos de Internet Industrial das Coisas. O Opta lê sinais de estado da máquina, coleta métricas de energia de um **medidor de energia Finder 7M** via **Modbus RTU/RS485**, calcula KPIs de produção na borda e publica a telemetria em um broker MQTT.

O firmware é desenvolvido com **PlatformIO**, utilizando o **framework Arduino** sobre o runtime baseado em Mbed do Opta. A aplicação é organizada como um firmware RTOS, com tarefas independentes para aquisição dos sinais da máquina, aquisição de energia, telemetria e diagnóstico.

---

## Arquitetura de Hardware e E/S

O mapeamento físico dos sinais no **Arduino Opta** é configurado em `include/Config.h` e pode ser sobrescrito por meio de flags de compilação do PlatformIO.

Mapeamento padrão das entradas:

- **A0:** Máquina ligada
- **A1:** Fim de programa
- **A2:** Porta fechada
- **A3:** Porta aberta
- **A4:** Peça fixada
- **A5:** Máquina pronta
- **RS485:** Comunicação Modbus RTU com o medidor de energia Finder 7M a **38400 baud**

Mapeamento padrão dos LEDs de diagnóstico:

- **D0:** Estado do sistema
- **D1:** Estado do Wi-Fi
- **D2:** Estado do MQTT
- **D3:** Estado do Modbus/Finder 7M

---

## Montagem Física

Abaixo está a representação da montagem física do sistema, detalhando a integração entre o painel da máquina CNC, o Arduino Opta e o medidor de energia Finder 7M dentro do quadro elétrico.

![montagem](https://github.com/user-attachments/assets/bd2ca228-f3fa-41ba-86a2-1b2bc3e4a9c3)

---

## Arquitetura de Software

O firmware é dividido em camadas de domínio, aplicação e plataforma:

- `src/domain` e `include/domain` contêm regras independentes de hardware, como processamento de KPIs, serialização da telemetria e política de publicação.
- `src/application` e `include/application` compõem o firmware e coordenam o estado compartilhado.
- `src/platform` e `include/platform` contêm adaptadores de hardware para as entradas do Opta, leituras do Finder 7M, Wi-Fi/MQTT, diagnóstico, logging e wrappers de RTOS.

A aplicação inicia quatro threads principais do RTOS:

- **machineThread**  
  Amostra os sinais da máquina CNC a cada **50 ms**, atualiza o estado da máquina e recalcula os KPIs.

- **energyThread**  
  Consulta o medidor de energia Finder 7M a cada **5000 ms** via RS485/Modbus RTU.

- **telemetryThread**  
  Gerencia Wi-Fi, sincronização NTP, reconexão MQTT, serialização JSON e publicação da telemetria.

- **diagnosticThread**  
  Atualiza os LEDs locais de diagnóstico do sistema, Wi-Fi, MQTT e Modbus.

---

## Modelo de Dados

O firmware mantém um snapshot coerente do sistema composto por:

- **MachineData**  
  Estados de máquina ligada, máquina pronta, porta fechada/aberta, peça fixada e fim de programa.

- **EnergyData**  
  Tensão, corrente, potência ativa, potência reativa, potência aparente, fator de potência, frequência e energia ativa acumulada.

- **KpiData**  
  Tempo de máquina ligada, tempo de operação, taxa de utilização, contagem de peças, tempo de ciclo e cadência de produção.

O estado compartilhado é centralizado por meio do `StateStore`, mantendo separadas as lógicas de aquisição, processamento e publicação.

---

## Processamento de KPIs e Inteligência na Borda

O firmware processa os indicadores de produção diretamente na borda:

- **Contagem de peças**  
  Incrementada quando uma borda de subida é detectada no sinal de peça fixada.

- **Tempo de operação**  
  Acumulado quando a porta está fechada, a peça está fixada e o sinal de fim de programa não está ativo.

- **Tempo de máquina ligada**  
  Acumulado enquanto o sinal de máquina ligada está ativo.

- **Taxa de utilização (%)**  
  Calculada como o tempo de operação dividido pelo tempo de máquina ligada.

- **Tempo de ciclo e cadência**  
  Calculados a partir do intervalo entre eventos de fixação detectados. A cadência utiliza um filtro de média móvel exponencial com **alpha = 0,2**.

---

## Telemetria e Publicação MQTT

A telemetria é publicada em formato JSON no broker e tópico MQTT configurados.

A publicação é orientada a eventos:

- Uma mensagem é publicada imediatamente após alterações relevantes no estado da máquina.
- Uma mensagem de heartbeat é publicada a cada **60 segundos** caso não ocorram alterações de estado.
- As tentativas de reconexão Wi-Fi e MQTT utilizam um intervalo de **5000 ms**.
- Os timestamps são gerados no formato ISO-8601 após a sincronização NTP.
- Os payloads são serializados em um buffer local de **600 bytes**.

Exemplo de payload:

```json
{
  "time": "2026-08-01T12:00:00Z",
  "data": {
    "voltageV": 220.00,
    "currentA": 4.20,
    "activePowerW": 850.00,
    "reactivePowerVAr": 120.00,
    "apparentPowerVA": 910.00,
    "powerFactor": 0.930,
    "frequencyHz": 60.00,
    "activeEnergyKWh": 15.420,
    "machineOn": 1,
    "machineReady": 1,
    "doorClosed": 1,
    "doorOpened": 0,
    "partClamped": 1,
    "programEnd": 0,
    "machineOnTimeS": 3600,
    "operationTimeS": 2700,
    "utilizationRate": 75.0,
    "partCount": 120,
    "cycleTimeS": 30.000,
    "cadence": 120.0
  }
}
```

---

## Configuração do PlatformIO

Ambiente PlatformIO necessário:

```ini
[env:opta]
platform = ststm32
board = opta
framework = arduino
```

Dependências do projeto:

```ini
lib_deps =
	dndg/Finder 7M for Finder Opta@^1.1.5
	arduino-libraries/ArduinoMqttClient@^0.1.8
	arduino-libraries/NTPClient@^3.2.1
```

Comandos típicos:

```bash
pio run
pio run --target upload
pio device monitor
```

---

## Dashboards de Monitoramento

Abaixo estão exemplos dos dados processados na borda e visualizados na plataforma em nuvem.

### Dashboard de Produção e OEE

Painel exibindo os principais indicadores operacionais, incluindo contagem de peças, taxa de utilização da máquina, tempo de ciclo e cadência de produção em tempo real.

<img width="1920" height="1080" alt="Dashboard de produção e OEE" src="https://github.com/user-attachments/assets/7022a33d-7407-4e7e-ace1-81decd611fe7" />

---

### Dashboard de Energia

Painel dedicado ao monitoramento elétrico do equipamento, registrando consumo de potência, fator de potência, tensão, corrente, frequência e energia acumulada por meio da integração Modbus RTU.

<img width="1920" height="1080" alt="Dashboard de energia" src="https://github.com/user-attachments/assets/16fef7fa-5992-45d8-a39d-82e004d0d672" />

---

## Diagnóstico Visual

Os indicadores LED do Opta simplificam o diagnóstico em campo:

- **LED do sistema (D0)**  
  Piscando lentamente indica que o firmware está em execução.

- **LED do Wi-Fi (D1)**  
  Pisca lentamente durante a conexão e permanece aceso quando conectado.

- **LED do MQTT (D2)**  
  Pisca lentamente enquanto desconectado e permanece aceso quando conectado ao broker.

- **LED do Modbus (D3)**  
  Um pulso indica leituras bem-sucedidas do Finder 7M. Piscadas rápidas indicam erros de inicialização ou de leitura.

---

## Fluxo Local de Segredos

Este repositório inclui `secrets.example.ini` como modelo seguro. O arquivo real `secrets.ini` é ignorado pelo Git e deve permanecer local, pois contém as credenciais de Wi-Fi e MQTT.

Antes de compilar ou gravar o firmware, copie o arquivo de exemplo:

```bash
cp secrets.example.ini secrets.ini
```

No Windows PowerShell:

```powershell
Copy-Item secrets.example.ini secrets.ini
```

Depois, edite `secrets.ini` com os valores reais para:

- SSID e senha do Wi-Fi
- Endereço e porta do broker MQTT
- ID do cliente MQTT
- Usuário e senha do MQTT
- Tópico MQTT

---

## Licença

Este projeto é licenciado sob a Licença MIT. Consulte `LICENSE` para mais detalhes.
