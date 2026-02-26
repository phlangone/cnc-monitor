# Monitoramento CNC via IoT Industrial

## Descrição do Projeto
Este projeto implementa a arquitetura de firmware para o monitoramento de uma máquina CNC utilizando conceitos de IoT Industrial. O sistema utiliza um Arduino Opta como dispositivo mestre para coletar dados físicos da máquina e métricas de energia de um medidor Finder 7M, transmitindo as informações para a nuvem através do Wegnology MQTT Broker. O firmware é baseado no Mbed OS, garantindo uma operação multitarefa em tempo real (RTOS).

## Arquitetura de Hardware e I/O
O mapeamento físico dos sinais no Arduino Opta é estruturado da seguinte forma:
* **A0:** Máquina Ligada (Digital).
* **A1:** Velocidade do Spindle (Entrada analógica 0-10V).
* **A2:** Porta Fechada (Digital).
* **A3:** Porta Aberta (Digital).
* **A4:** Peça Fixada / Clamp (Digital).
* **RS485:** Comunicação Modbus RTU com o medidor Finder 7M a 38400 baud.

## Montagem Física
Abaixo está a representação da montagem física do sistema, detalhando a integração entre o painel da máquina CNC, o Arduino Opta e o medidor de energia Finder 7M no painel elétrico:

![montagem](https://github.com/user-attachments/assets/bd2ca228-f3fa-41ba-86a2-1b2bc3e4a9c3)

## Arquitetura de Software (RTOS)
O sistema é dividido em quatro threads principais para garantir a execução multitarefa em tempo real:
* **machineThread:** Responsável pela amostragem rápida (ciclos de 50ms) e leitura dos sensores da máquina CNC.
* **modbusThread:** Realiza o polling via comunicação serial RS485 para obter os dados do medidor de energia (tensão, corrente, potência e frequência).
* **mqttThread:** Gerencia a conectividade WiFi, sincronização de tempo (NTP Sync via UDP) e telemetria para a nuvem.
* **ledManagerThread:** Controla o feedback visual e o diagnóstico de status local.

## Estruturas de Dados e Concorrência
O gerenciamento de estado global utiliza o conceito de fonte única da verdade (Single Source of Truth), dividindo os dados em estruturas:
* **machine_data_t:** Armazena a velocidade do spindle e os estados digitais da máquina.
* **energy_data_t:** Armazena grandezas elétricas como tensão, corrente, potência ativa e fator de potência.
* **kpi_data_t:** Armazena os tempos de operação, contagem de peças e taxas de utilização.

Para garantir a integridade dos dados e prevenir condições de corrida entre as threads de leitura e escrita, o sistema implementa Mutexes (`machineMutex` e `energyMutex`) no acesso à memória global.

## Processamento de KPI e Inteligência na Borda
O firmware processa métricas de eficiência (OEE) diretamente na borda:
* **Contagem de Peças:** Incrementada na detecção de borda de subida do sinal de Clamp quando o Spindle está ativo.
* **Cadência (Peças/Hora):** Calculada utilizando um filtro de média móvel exponencial para estabilidade (fator Alpha = 0.2).
* **Taxa de Utilização (%):** Calculada pela relação entre o tempo de corte e o tempo com a máquina ligada.

## Telemetria e Publicação (MQTT)
A publicação de dados no broker MQTT Wegnology é baseada em eventos para otimização de banda:
* **Publicação Imediata:** Ocorre ao detectar mudança de estado na porta, máquina, clamp ou se houver variação na velocidade do spindle superior a 50 rpm.
* **Keep-alive:** Publicação automática (Heartbeat) a cada 60 segundos caso não haja mudanças nos estados físicos.
* O payload é construído em formato JSON, alocado em um buffer de 1024 bytes, incluindo um timestamp ISO-8601 sincronizado via NTP. 
* A conectividade possui um loop de resiliência com tentativas de reconexão automática MQTT (5000ms de backoff) para garantir operação contínua 24/7.

## Dashboards de Monitoramento

Abaixo estão as visualizações dos dados processados na borda e enviados para a plataforma em nuvem:

### Dashboard de Produção e OEE
Painel com os indicadores principais de operação, exibindo a contagem de peças, taxa de utilização da máquina e cadência produtiva atualizada em tempo real.

<img width="1920" height="1080" alt="Captura de tela 2026-02-26 130714" src="https://github.com/user-attachments/assets/7022a33d-7407-4e7e-ace1-81decd611fe7" />

### Dashboard de Energia
Painel dedicado ao monitoramento elétrico do equipamento, registrando o consumo de potência, fator de potência, tensão e corrente via integração Modbus RTU.

<img width="1920" height="1080" alt="Captura de tela 2026-02-26 130533" src="https://github.com/user-attachments/assets/16fef7fa-5992-45d8-a39d-82e004d0d672" />

## Diagnóstico Visual
Indicadores em LED na placa facilitam a análise do estado do sistema em campo:
* **System LED (D0):** Sinal em pulso indica que o sistema operacional está ativo.
* **WiFi LED (D1):** Piscando lento ao conectar e fixo (ON) quando conectado à rede.
* **Modbus LED (D3):** Pulso contínuo para leitura OK e piscando rápido em caso de erro de leitura com o medidor Finder.

