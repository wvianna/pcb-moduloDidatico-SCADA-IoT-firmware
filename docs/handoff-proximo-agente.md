# Handoff Técnico Para Próximo Agente

## Resumo do estado atual

Este repositório contém um firmware para NodeMCU v2 (ESP8266) com:

- Seleção de protocolo ativo via setup web: `modbus`, `mqtt` ou `opcua` (modo exclusivo).
- Modbus TCP na porta 502 e Modbus RTU via RS485 quando protocolo ativo é `modbus`.
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
- Apenas um protocolo é executado por vez (não há execução simultânea de Modbus + MQTT + OPC UA).

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
- Funções suportadas:
  - `0x01` Read Coils
  - `0x02` Read Discrete Inputs
  - `0x03` Read Holding Registers
  - `0x04` Read Input Registers
  - `0x05` Write Single Coil
  - `0x06` Write Single Register

### Modbus RTU

- Banco de registradores mantido pela lib local `ModbusSerial`.
- Sincronização entre o modelo interno e o banco RTU via:
  - `syncModelToRtu()`
  - `syncRtuToModel()`
- Fluxo de sincronização preservado para evitar regressão da correção anterior de coils TCP.

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

### Correção importante já aplicada

Foi corrigido um defeito em que escritas Modbus TCP nas coils eram sobrescritas antes de chegar aos pinos físicos.

Correção aplicada:

- No loop principal, quando `tcpWriteDirty == true`, o firmware agora executa `syncModelToRtu()` primeiro.
- Apenas quando não há escrita TCP pendente é que `syncRtuToModel()` é executado.

Impacto:

- As coils TCP passaram a refletir corretamente nos pinos físicos.

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
- Diagnósticos/linters sem erros nos arquivos atualizados (`README.md`, `test/README`, `test/mqtt-regressao-checklist.md`).

## Pinagem efetiva atual do firmware

### Coils

- Coil `00001` / base 0 `0`: `D0 / GPIO16`
- Coil `00002` / base 0 `1`: `TX / GPIO1`


### Entradas discretas

- Discrete input `10001` / base 0 `0`: `D8 / GPIO15` (também gatilho de setup AP — dual use)
- Discrete input `10002` / base 0 `1`: `RX / GPIO3`

### Sensores / input registers

- `30001`: `A0`
- `30002`: `D2 / GPIO4` DS18B20

### PWM / holding registers

- `40001`: `D7 / GPIO13`
- `40002`: `D3 / GPIO0`
- `40003`: `D1 / GPIO5` (dedicado à resistência de aquecimento)

### RS485 MAX485

- `RO -> D5 / GPIO14`
- `DI -> D6 / GPIO12`
- `DE+RE -> D4 / GPIO2`

### Gatilho de setup AP

- `D8 / GPIO15` (ED1) ativo em nível baixo apenas durante o boot.

> **Atenção:** `GPIO15` é pino de bootstrap do ESP8266. Na NodeMCU padrão ele tem pull-down (fica sempre baixo), então o dispositivo **sempre** entraria em modo AP. Para o gatilho ser opcional, a PCB precisa manter `GPIO15` baixo no instante do reset (boot normal) e elevá-lo após o boot (pull-up), com switch para GND pressionado para forçar o AP.

## Pendências e riscos conhecidos

### 1. TX/RX como I/O digital

O firmware está com:

- `SERIAL_DEBUG_ENABLED = false`

Então `GPIO1/TX` e `GPIO3/RX` estão liberados como I/O do projeto.

Se um próximo agente precisar depurar por serial, pode ser necessário reativar debug e aceitar impacto sobre `00002` e `10002`.

### 3. Porta 502 não sobe instantaneamente após reboot

Após upload/reset, o IP pode responder a `ping` antes de a porta Modbus TCP `502` voltar a aceitar conexões.

Em testes automáticos, aguardar o serviço subir antes de iniciar o cliente Modbus.

### 4. OPC UA PubSub depende do broker MQTT

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
- Modbus: `deviceId`, `baud`, `parity`, `stopBits`
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
