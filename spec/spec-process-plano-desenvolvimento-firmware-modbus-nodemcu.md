---
title: Plano de Desenvolvimento do Firmware NodeMCU Multi-Protocolo e Setup Web
version: 1.1
date_created: 2026-07-10
last_updated: 2026-07-11
owner: Equipe Firmware/Integracao
tags: [process, architecture, firmware, modbus, mqtt, opcua, nodemcu, web-ui]
---

## Plano de Desenvolvimento

Este documento define o plano tecnico para o firmware do NodeMCU com selecao de protocolo ativo por interface web.

## 1. Purpose & Scope

Objetivo: manter a base didatica atual e evoluir para operacao por modo exclusivo (`modbustcp`, `modbusrtu`, `mqtt`, `opcua`) sem regressao de I/O, sensores, setup AP e rede estatica.

Escopo:

- Firmware Arduino/PlatformIO para NodeMCU v2 (ESP8266).
- Selecao de protocolo ativo na interface web.
- Persistencia de configuracao em LittleFS.
- Mapeamento de I/O unico para todos os protocolos.
- Runtime funcional para Modbus e MQTT.
- Configuracao de endpoint OPC UA e telemetria de suporte de runtime.

Fora de escopo:

- Desenvolvimento de supervisorio externo.
- Alteracao de hardware da placa.

## 2. Definitions

- AP: Access Point.
- STA: Modo cliente Wi-Fi.
- Model: modelo interno de registradores e sinais de I/O.
- Modo exclusivo: apenas um protocolo ativo por vez.

## 3. Requirements, Constraints & Guidelines

### Requisitos funcionais

- REQ-001: Permitir selecao de protocolo ativo no setup web (`modbustcp`, `modbusrtu`, `mqtt`, `opcua`).
- REQ-002: Em modo `modbustcp`, manter Modbus TCP funcional; em modo `modbusrtu`, manter Modbus RTU funcional; sem regressao.
- REQ-003: Em modo `mqtt`, publicar entradas em JSON no broker.
- REQ-004: Em modo `mqtt`, receber comando de saidas por subscribe JSON.
- REQ-005: Em modo `opcua`, disponibilizar configuracao de endpoint no setup web.
- REQ-006: Persistir SSID, PSK, IP, mascara, gateway, DNS, protocolo ativo e parametros por protocolo.
- REQ-007: Validar IPv4 e impedir saida de AP em configuracao invalida.
- REQ-008: Preservar mapa de pinos e registradores existente.
- REQ-009: Permitir configurar a porta do servico Modbus TCP no setup web e persisti-la em `/config.txt` (padrao 502).

### Restricoes tecnicas

- CON-001: Plataforma ESP8266 NodeMCU v2.
- CON-002: Setup AP em 192.168.4.1.
- CON-003: Loop principal nao bloqueante.
- CON-004: Watchdog ativo.

### Diretrizes

- GUD-001: O modelo interno e a fonte unica de verdade para I/O.
- GUD-002: Separar inicializacao e servico por protocolo.
- GUD-003: Em runtime, executar somente o protocolo selecionado.

## 4. Interfaces & Data Contracts

### 4.1 Mapeamento de I/O

O mapeamento canonico permanece documentado em `docs/mapeamentopinosmb.txt` e reutilizado por todos os modos.

### 4.2 Contrato de configuracao persistida

Formato `key=value` em `/config.txt`.

Chaves minimas:

- Wi-Fi/rede: `ssid`, `psk`, `ip`, `mask`, `gateway`, `dns`
- Protocolo: `protocol`
- Modbus: `deviceId`, `modbusTcpPort`, `baud`, `parity`, `stopBits`
- MQTT: `mqttBroker`, `mqttPort`, `mqttUser`, `mqttPass`, `mqttTopicBase`
- OPC UA: `opcUaPublisherId`, `opcUaDataSetWriterId`

### 4.3 Setup Web

Endpoints:

- `GET /`: formulario de configuracao
- `POST /config`: persistencia e validacao
- `GET /health`: estado atual, protocolo ativo e suporte de runtime OPC UA

Campos obrigatorios globais:

- SSID, PSK, IP, Mascara, Gateway, DNS, Protocolo

Campos condicionais:

- Modbus: Device ID, Porta Modbus TCP, Baud, Paridade, Stop Bits
- MQTT: Broker, Porta, Usuario, Senha, Topico base
- OPC UA: Host, Porta

### 4.4 Contrato MQTT

Topicos padrao:

- Publish entradas: `<topico_base>/inputs`
- Subscribe saidas: `<topico_base>/outputs/set`
- Status: `<topico_base>/status`

Payload de entradas:

```json
{
  "ts_ms": 123456,
  "ip": "192.168.100.200",
  "discrete_inputs": [0, 1],
  "input_registers": [512, 255]
}
```

Payload de saidas:

```json
{
  "coils": [1, 0, 1],
  "holding_registers": [100, 450, 900]
}
```

Regras:

- `coils`: ate 2 posicoes.
- `holding_registers`: ate 3 posicoes, faixa 0..1023.
- QoS alvo: 0.

## 5. Acceptance Criteria

- AC-001: Com RX/GPIO3 baixo no boot, dispositivo entra em AP com SSID `NodeMCU_Setup_<ChipID>`.
- AC-002: Configuracao de rede invalida mantem AP.
- AC-003: Configuracao valida persiste apos reboot.
- AC-004: Em modo `modbustcp`, Modbus TCP responde conforme mapa atual; em modo `modbusrtu`, Modbus RTU responde conforme mapa atual.
- AC-005: Em modo `mqtt`, entradas publicam JSON no broker.
- AC-006: Em modo `mqtt`, JSON de comando altera coils e HREGs.
- AC-007: Troca de protocolo no setup ativa apenas um servico por vez.
- AC-008: `GET /health` informa protocolo ativo e suporte OPC UA.

## 6. Test Strategy

- Build: `pio run`
- Upload: `pio run -t upload`
- Modbus regressao: leitura/escrita de coils e registers
- MQTT publish: inspeção no topico de entradas
- MQTT subscribe: envio de JSON de comando e validacao de saida fisica
- Setup web: teste de validacao de rede e persistencia

## 7. Rationale & Context

A evolucao por modo exclusivo reduz risco de concorrencia entre stacks de comunicacao e preserva previsibilidade em ESP8266.

## 8. Dependencies

- Modbus local: `lib/ModbusSerial`
- MQTT: `knolleary/PubSubClient`
- JSON: `bblanchon/ArduinoJson`

## 9. Edge Cases

- Broker MQTT indisponivel: reconexao periodica sem bloquear loop.
- Payload MQTT invalido: mensagem ignorada.
- Perda de Wi-Fi em runtime: transicao para FAILSAFE_AP.

## 10. Validation Criteria

- Build verde e sem regressao em Modbus.
- Runtime MQTT operante com contratos JSON documentados.
- Documentacao sincronizada entre README, requisitos e spec.
