# Handoff Técnico Para Próximo Agente

## Resumo do estado atual

Este repositório contém um firmware para NodeMCU v2 (ESP8266) com:

- Seleção de protocolo ativo via setup web: `modbustcp`, `modbusrtu`, `mqtt` ou `opcua` (modo exclusivo).
- Modbus TCP na porta 502 por padrão — configurável no setup web e persistida (`modbusTcpPort`) — (protocolo `modbustcp`) e Modbus RTU via RS485 (protocolo `modbusrtu`).
- Modo AP para configuração via página web.
- Persistência de configuração em LittleFS (`/config.txt`) com escrita atômica.
- Leitura de ADC e DS18B20.
- Saídas digitais por coils e saídas PWM por holding registers.

O arquivo principal de firmware é `src/main.cpp`.

## Implementado até agora

### Comunicação e estados

- Máquina de estados principal:
  - `BOOT`
  - `AP_SETUP`
  - `WIFI_CONNECTING`
  - `RUN_PROTOCOL`
  - `FAILSAFE_AP`
- Modo AP com SSID no formato `NodeMCU_Setup_<ChipID>`.
- Página web local para configuração de Wi-Fi, rede estática, protocolo e parâmetros por protocolo.
- Retorno para AP em caso de falha persistente de conexão Wi-Fi.
- Apenas um protocolo é executado por vez (não há execução simultânea de Modbus TCP, Modbus RTU, MQTT ou OPC UA).

### Persistência

- Configuração salva em `LittleFS` no arquivo `/config.txt`.
- Escrita atômica: grava em `/config.tmp` e renomeia para `/config.txt` ao final.
- Campos persistidos:
  - `protocol`
  - `ssid`
  - `psk`
  - `ip`
  - `mask`
  - `gateway`
  - `dns`
  - `deviceId`
  - `modbusTcpPort`
  - `baud`
  - `parity`
  - `stopBits`
  - `mqttBroker`
  - `mqttPort`
  - `mqttUser`
  - `mqttPass`
  - `mqttTopicBase`
  - `mqttMode`
  - `mqttPublishTopic`
  - `mqttSubscribeTopic`
  - `opcUaPublisherId`
  - `opcUaDataSetWriterId`
- Parse numérico estrito para campos numéricos no `loadConfig`.
- Saneamento (`trim`) em campos de rede/MQTT/OPC UA antes de validar e salvar.

### Modbus TCP

- Implementação manual da camada Modbus TCP em `serviceModbusTcp()`.
- Suporte a **múltiplos clientes simultâneos** (array fixo `modbusTcpClients[MODBUS_TCP_MAX_CLIENTS]`, padrão 4) — Python, Node-RED e SCADA/IHM ao mesmo tempo, sem travamento de conexão.
- Funções suportadas:
  - `0x01` Read Coils
  - `0x02` Read Discrete Inputs
  - `0x03` Read Holding Registers
  - `0x04` Read Input Registers
  - `0x05` Write Single Coil
  - `0x06` Write Single Register

### Modbus RTU

- Banco de registradores mantido pela lib local `ModbusSerial`.
- Sincronização direcional (modo RTU exclusivo):
  - `syncModelInputsToRtu()` — entradas (`ists`/`iregs`) fluem modelo → banco RTU a cada período de sensor.
  - `syncRtuOutputsToModel()` — saídas (`coils`/`hregs`) escritas pelo mestre fluem banco RTU → modelo a cada 25 ms.
  - `syncModelToRtu()` — usada apenas na inicialização para popular o banco.
- Não há mais sincronização bidirecional no runtime; cada protocolo opera de forma independente.

### MQTT

- Cliente MQTT implementado com `PubSubClient`.
- Configuração persistente do protocolo MQTT:
  - `mqttMode`: `geral` ou `thingsboard` (caixa de seleção na interface web).
  - `mqttPublishTopic`: tópico principal de publicação (usado no código para publicar as entradas).
  - `mqttSubscribeTopic`: tópico de subscrição (comandos de saída).
  - `mqttPort`: porta persistente do broker.
- Modo **MQTT Geral** (tópicos configuráveis; se vazios, usam a base + `/inputs` e `/outputs/set`):
  - Publicação periódica de entradas em JSON:
    - tópico: `<mqttPublishTopic>` (padrão `<topico_base>/inputs`)
    - campos: `ts_ms`, `ip`, `discrete_inputs`, `input_registers`
  - Subscrição de comandos de saída em JSON:
    - tópico: `<mqttSubscribeTopic>` (padrão `<topico_base>/outputs/set`)
    - campos aceitos: `coils`, `holding_registers`
  - Publicação de status:
    - tópico: `<topico_base>/status`
    - payload: `online`
- Modo **MQTT ThingsBoard** (tópicos fixos):
  - Publicação de telemetria:
    - tópico: `v1/devices/me/telemetry` (fixo)
    - campos: `ts_ms`, `ip`, `discrete_inputs`, `input_registers`
  - Subscrição de comandos RPC:
    - tópico: `v1/devices/me/rpc/request/+` (fixo)
    - aceita payload direto `{ coils, holding_registers }` ou RPC `{ method, params: { coils, holding_registers } }`
  - Autenticação: usuário = Access Token do dispositivo; **sem senha**.
  - Status: publica `{"online":true}` em `v1/devices/me/attributes`.
- Regras de faixa:
  - `holding_registers` limitados para `0..1023`
- QoS operacional atual: `0`.

### OPC UA

- Modo `opcua` disponível na seleção da interface.
- Campos de configuração persistidos: `opcUaPublisherId` e `opcUaDataSetWriterId`.
- Opera via OPC UA PubSub (IEC 62541-14) sobre MQTT — reutiliza broker/porta/user/pass.
- `GET /health` expõe `opcuaRuntimeSupported` para diagnóstico.

### Refatoração de desempenho (última mudança)

O firmware foi refatorado para melhorar o desempenho e a responsividade do loop:

- **Protocolos exclusivos**: `modbus` foi desdobrado em `modbustcp` e `modbusrtu`.
  - `modbustcp`: apenas `serviceModbusTcp()` roda; escrita vai direto ao modelo, sem sincronização.
  - `modbusrtu`: apenas `modbusRtu.task()` roda; a sincronização é direcional (entradas → banco, saídas → modelo).
  - `modbus` legado no arquivo de configuração é interpretado como `modbustcp` (compatibilidade).
- **DS18B20 não bloqueante**: `setWaitForConversion(false)` + leitura por `millis()` elimina o bloqueio de ~750 ms por ciclo (antes, o loop travava durante toda a conversão). O intervalo entre disparo e leitura é garantido ≥ **500 ms** (`DS18B20_MIN_CONVERSION_WAIT_MS`), a primeira conversão é disparada no boot, e falhas transitórias de leitura (CRC/ruído 1-Wire) mantêm o último valor válido em vez de publicar `0` no registrador 30002.
- **Saídas aplicadas sob demanda**: `applyOutputsIfChanged()` evita reescrever pinos (`analogWrite`/`digitalWrite`) a cada 60 ms quando nada mudou.
- **Boot/troca de protocolo mais rápidos**: removido o `delay(2000)` da lib `ModbusSerial::config`.
- **Configuração RTU tardia**: `configureRtu()` deixou de ser chamada no `setup()`; agora é chamada apenas ao entrar no modo `modbusrtu`, poupando tempo de inicialização nos modos TCP/MQTT/OPC UA.
- Clock mantido em 160 MHz (máximo do ESP8266), já configurado em `platformio.ini`.

Impacto:

- Loop permanece responsivo durante a conversão de temperatura (Modbus/web/MQTT atendidos sem janela de ~750 ms de travamento).
- Menos CPU gasta em sincronização e escrita repetida de pinos.

### Correção importante anterior (histórico)

Foi corrigido um defeito em que escritas Modbus TCP nas coils eram sobrescritas antes de chegar aos pinos físicos. Com os modos exclusivos, esse cenário deixou de existir (TCP e RTU não compartilham mais o mesmo runtime).

## Testes já executados

### Build

- `pio run`: sucesso.

### Validação de regressão (histórico)

### Teste funcional Modbus TCP

IP testado do NodeMCU:

- `192.168.100.204`

Teste feito escrevendo `0` e `1` nas coils base 0 `0`, `1` e `2`, com intervalo de 1 segundo.

Resultado final validado:

```text
Testando coil base0=0...
coil_base0=0 write=0 echoed=0x0000 readback=0
coil_base0=0 write=1 echoed=0xFF00 readback=1
coil_base0=0 write=0 echoed=0x0000 readback=0
coil_base0=0 write=1 echoed=0xFF00 readback=1

Testando coil base0=1...
coil_base0=1 write=0 echoed=0x0000 readback=0
coil_base0=1 write=1 echoed=0xFF00 readback=1
coil_base0=1 write=0 echoed=0x0000 readback=0
coil_base0=1 write=1 echoed=0xFF00 readback=1

Testando coil base0=2...
coil_base0=2 write=0 echoed=0x0000 readback=0
coil_base0=2 write=1 echoed=0xFF00 readback=1
coil_base0=2 write=0 echoed=0x0000 readback=0
coil_base0=2 write=1 echoed=0xFF00 readback=1
```

Conclusão:

- Coils TCP estão funcionando logicamente e fisicamente após a correção da sincronização.

### Verificações recentes de qualidade

- Build repetido após endurecimento de persistência: sucesso.
- Build e upload após a refatoração de desempenho (`pio run` / `pio run -t upload`): sucesso, sem warnings.
- Diagnósticos/linters sem erros nos arquivos atualizados (`README.md`, `test/README`, `test/mqtt-regressao-checklist.md`).
- `README.md` atualizado com a tabela "Mapa de endereçamento Modbus", espelhando o help HTML embarcado no MCU.

## Pinagem efetiva atual do firmware

### Coils

- Coil `00001` / base 0 `0`: `D0 / GPIO16`
- Coil `00002` / base 0 `1`: `TX / GPIO1`


### Entradas discretas

- Discrete input `10001` / base 0 `0`: `D8 / GPIO15`
- Discrete input `10002` / base 0 `1`: `RX / GPIO3` (também gatilho de setup AP — dual use)

### Sensores / input registers

- `30001`: `A0`
- `30002`: `D2 / GPIO4` DS18B20

### PWM / holding registers

- `40001`: `D7 / GPIO13`
- `40002`: `D3 / GPIO0`
- `40003`: `D1 / GPIO5` (dedicado à resistência de aquecimento)
- Frequência do PWM: **50 Hz** (`analogWriteFreq(PWM_FREQ_HZ)` com `PWM_FREQ_HZ = 50`), range 0..1023
- **Proteção térmica do heater (40003):** acima de 80 °C (`HEATER_MAX_TEMP_TENTHS = 800`, DS18B20 em décimos) a saída PWM da resistência é forçada a 0 via `effectiveHeaterPwm()`; `applyOutputsIfChanged()` reaplica quando a temperatura cruza o limite. Abaixo de 80 °C, vale o valor comandado (`hregs[2]`).

### RS485 MAX485

- `RO -> D5 / GPIO14`
- `DI -> D6 / GPIO12`
- `DE+RE -> D4 / GPIO2`

### Gatilho de setup AP

- `RX / GPIO3` (ED2) ativo em nível baixo apenas durante o boot.

> **Notas:** `GPIO3` é o `RX` da UART nativa — o gatilho **só funciona com `SERIAL_DEBUG_ENABLED=false`** (RX livre). Não é pino de bootstrap: com `INPUT_PULLUP` o pino fica alto por padrão e o modo AP só é forçado quando o switch puxa para GND (0 V). `GPIO15/D8` (ED1) voltou a ser somente a entrada discreta 10001.

## Pendências e riscos conhecidos

### 1. TX/RX como I/O digital

O firmware está com:

- `SERIAL_DEBUG_ENABLED = false`

Então `GPIO1/TX` e `GPIO3/RX` estão liberados como I/O do projeto.

Se um próximo agente precisar depurar por serial, pode ser necessário reativar debug e aceitar impacto sobre `00002` e `10002`.

### 2. Porta Modbus TCP não sobe instantaneamente após reboot

Após upload/reset, o IP pode responder a `ping` antes de a porta Modbus TCP configurada (padrão `502`) voltar a aceitar conexões.

Em testes automáticos, aguardar o serviço subir antes de iniciar o cliente Modbus.

### 3. OPC UA PubSub depende do broker MQTT

- O modo `opcua` reutiliza o broker MQTT como transporte; se o broker estiver indisponível, o OPC UA PubSub também para.

## Sugestões de continuidade

### Melhor próximo passo técnico

Adicionar script executável de smoke test MQTT e OPC UA PubSub (pub/sub) para rodar junto da regressão Modbus.

### Melhor próximo passo de qualidade

Transformar checklist manual de MQTT em automação parcial (bash/python) com evidências em arquivo.

### Melhor próximo passo de documentação

Manter `README.md`, `docs/requisitos.txt`, `docs/mapeamentopinosmb.txt` e `spec/spec-process-plano-desenvolvimento-firmware-modbus-nodemcu.md` sincronizados a cada mudança de protocolo/persistência.

## Artefatos novos adicionados

- `test/mqtt-regressao-checklist.md`
- `test/mqtt-payloads-exemplo.json`

## Chaves de configuração (resumo rápido)

- Globais: `protocol`, `ssid`, `psk`, `ip`, `mask`, `gateway`, `dns`
- Modbus: `deviceId` (TCP e RTU); `modbusTcpPort` (TCP, padrão 502); `baud`, `parity`, `stopBits` (somente RTU)
- MQTT: `mqttMode`, `mqttBroker`, `mqttPort`, `mqttUser`, `mqttPass`, `mqttPublishTopic`, `mqttSubscribeTopic`, `mqttTopicBase` (base/fallback e OPC UA)
- OPC UA: `opcUaPublisherId`, `opcUaDataSetWriterId`

## Comandos úteis

```bash
pio run
pio run -t upload
pio device monitor
```

## Referências internas

- `src/main.cpp`
- `README.md`
- `docs/mapeamentopinosmb.txt`
- `docs/requisitos.txt`
- `test/mqtt-regressao-checklist.md`
- `test/mqtt-payloads-exemplo.json`
- `spec/spec-process-plano-desenvolvimento-firmware-modbus-nodemcu.md`
