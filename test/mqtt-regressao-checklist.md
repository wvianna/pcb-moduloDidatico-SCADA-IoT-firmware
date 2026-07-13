# Regressao MQTT - Checklist

Objetivo: validar o modo MQTT do firmware apos alteracoes de codigo/configuracao.

## Pre-requisitos

- Firmware gravado e configurado no modo `MQTT` via setup web.
- Broker MQTT acessivel na rede.
- Ferramenta de cliente MQTT instalada (ex.: mosquitto_pub/mosquitto_sub ou MQTT Explorer).
- Topico base definido no setup (ex.: `nodemcu`).

## Topicos utilizados

- Entradas (publish do firmware): `<topico_base>/inputs`
- Saidas (subscribe do firmware): `<topico_base>/outputs/set`
- Status (publish do firmware): `<topico_base>/status`

## Caso 1 - Conexao e status

1. Assinar `<topico_base>/status`.
2. Reiniciar o NodeMCU.
3. Verificar mensagem `online` no topico de status.

Resultado esperado:

- Mensagem de status chega logo apos conexao MQTT.

## Caso 2 - Publicacao de entradas

1. Assinar `<topico_base>/inputs`.
2. Aguardar pelo menos 5 segundos.
3. Confirmar recepcao de payload JSON com campos:
   - `ts_ms`
   - `ip`
   - `discrete_inputs`
   - `input_registers`

Resultado esperado:

- Publicacao periodica sem travar o dispositivo.
- JSON valido e consistente com estado fisico dos sinais.

## Caso 3 - Comando de coils

1. Publicar payload no topico `<topico_base>/outputs/set`:

```json
{"coils":[1,0,1]}
```

1. Verificar refletancia nas saidas digitais correspondentes.

Resultado esperado:

- Coils aplicadas corretamente nos pinos de saida.

## Caso 4 - Comando de PWM (holding registers)

1. Publicar payload no topico `<topico_base>/outputs/set`:

```json
{"holding_registers":[100,450,900]}
```

1. Verificar saidas PWM associadas.

Resultado esperado:

- Valores aplicados na faixa 0..1023.

## Caso 5 - Payload fora de faixa

1. Publicar payload:

```json
{"holding_registers":[-10,1200,2048]}
```

Resultado esperado:

- Firmware limita valores para faixa valida 0..1023.

## Caso 6 - Payload invalido

1. Publicar mensagem invalida no topico de saida, por exemplo:

```text
{invalid-json}
```

Resultado esperado:

- Mensagem ignorada sem reset do dispositivo.
- Publicacao de entradas continua normal.

## Caso 7 - Persistencia de parametros MQTT

1. No setup web, alterar broker/porta/usuario/senha/topico base.
2. Salvar configuracao.
3. Reiniciar NodeMCU.
4. Revalidar os casos 1 e 2 com os novos parametros.

Resultado esperado:

- Configuracoes MQTT persistidas e aplicadas apos reboot.

## Evidencias recomendadas

- Captura de topicos com timestamp.
- Dump de payloads recebidos.
- Registro de versao do firmware testado.
