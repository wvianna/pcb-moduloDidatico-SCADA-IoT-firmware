#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <LittleFS.h>
#include <SoftwareSerial.h>
#include <ModbusSerial.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>

namespace {

// Note 1: Este arquivo concentra toda a aplicacao embarcada em um unico modulo para facilitar o estudo inicial.
// Note 2: Em projetos maiores, separar por arquivos (rede, modbus, sensores, io) melhora manutencao e testes.
// Note 3: O include Arduino.h expande tipos e APIs basicas como pinMode, digitalWrite, millis e String.
// Note 4: O firmware usa loop cooperativo, sem RTOS, entao tarefas devem ser curtas e nao bloqueantes.
// Note 5: Em ESP8266, atrasos longos sem yield podem disparar watchdog e reiniciar o chip.
// Note 6: A escolha por constantes constexpr reduz custo em RAM e permite otimizacao em tempo de compilacao.
// Note 7: Comentarios de pino incluem GPIO fisico para evitar confusao entre rotulo da placa e numero real.
// Note 8: D0, D1, D2 etc sao aliases da placa NodeMCU; o hardware interno usa GPIO numerico.
// Note 9: Pinos de bootstrap (GPIO0, GPIO2, GPIO15) exigem cuidado porque influenciam modo de boot.
// Note 10: GPIO1 e GPIO3 sao TX e RX da UART principal; usar como IO reduz capacidade de debug serial.
// Note 11: O codigo usa duas pilhas Modbus: TCP para rede e RTU para barramento serial RS485.
// Note 12: O modelo interno de registradores atua como ponto unico de verdade entre protocolos.
// Note 13: Essa estrategia evita divergencia entre o que TCP enxerga e o que RTU enxerga.
// Note 14: A regra de ouro: escrita de cliente deve chegar ao hardware sem ser sobrescrita por sincronizacao oposta.
// Note 15: O firmware adota periodicidade com millis para evitar bloqueio e manter latencia previsivel.
// Note 16: Cada periodo (sensor, IO, sync) representa uma responsabilidade especifica no ciclo principal.
// Note 17: Valores de timeout e periodo estao em milissegundos para manter leitura rapida e coerente.
// Note 18: Em automacao, coerencia temporal e tao importante quanto funcionalidade.
// Note 19: O objetivo didatico aparece na clareza do mapeamento de registradores e pinos.
// Note 20: A organizacao por funcoes pequenas facilita rastrear causa e efeito em bugs de comunicacao.
// Note 21: COILS representam saidas binarias; ISTS representam entradas binarias somente leitura.
// Note 22: IREGS sao entradas analogicas/sensores somente leitura no protocolo Modbus.
// Note 23: HREGS sao registradores de controle leitura/escrita, usados aqui para PWM.
// Note 24: O uso de arrays fixos simplifica indexacao base 0 alinhada ao protocolo internamente.
// Note 25: Enderecos Modbus base 1 vistos em supervisorio mapeiam para base 0 no codigo.
// Note 26: Exemplo: 40001 no SCADA corresponde a hregs[0] no firmware.
// Note 27: A configuracao de rede e persistida em arquivo para sobreviver a reboot.
// Note 28: LittleFS e adequado para pequenos dados estruturados em texto chave=valor.
// Note 29: A validacao de IPv4 evita conexoes com parametros invalidos que causariam falhas silenciosas.
// Note 30: Configuracao invalida mantém AP para recuperar dispositivo sem reflashing.
// Note 31: O modo AP e uma estrategia de comissionamento comum em IoT industrial e residencial.
// Note 32: A pagina web embutida permite setup sem ferramenta externa de desktop.
// Note 33: O SSID do AP inclui ChipID para diferenciar dispositivos em bancada com varias placas.
// Note 34: O endpoint /health ajuda diagnostico rapido sem cliente Modbus.
// Note 35: A serializacao JSON minima em /health facilita consumo por scripts e dashboards.
// Note 36: A maquina de estados explicita transicoes e reduz comportamento implicito perigoso.
// Note 37: Estados bem definidos ajudam a depurar problemas de rede intermitente.
// Note 38: A transicao para FAILSAFE_AP protege contra travamento permanente fora da rede.
// Note 39: O bootstrap por pino para forcar AP evita depender de credenciais antigas esquecidas.
// Note 40: INPUT_PULLUP no pino de setup reduz necessidade de resistor externo em prototipagem.
// Note 41: A leitura do pino de setup ocorre cedo no boot para evitar interferencia de outras rotinas.
// Note 42: O firmware aplica analogWriteRange(1023) para mapear PWM no mesmo range dos HREGS.
// Note 43: Essa escolha evita conversao adicional quando registrador ja usa 0..1023.
// Note 44: Em ESP8266, PWM e por software e pode variar com carga da CPU.
// Note 45: Clock de 160 MHz melhora folga temporal de pilhas de rede e parser Modbus.
// Note 46: Mesmo com clock maior, logica bloqueante ainda pode causar timeouts.
// Note 47: O watchdog de 4s protege contra deadlocks e loops presos.
// Note 48: Alimentar watchdog no loop principal confirma que a tarefa cooperativa ainda progride.
// Note 49: Se uma funcao travar por muito tempo, watchdog reinicia e recupera disponibilidade.
// Note 50: Sistemas industriais preferem auto-recuperacao a travamento indefinido.
// Note 51: SoftwareSerial para RS485 permite usar UART principal livre para outras funcoes.
// Note 52: Em throughput alto, SoftwareSerial pode ser limitante comparado a UART hardware.
// Note 53: Modbus RTU exige controle DE/RE para half-duplex no transceiver MAX485.
// Note 54: DE alto habilita transmissao; RE baixo habilita recepcao quando pinos estao ligados juntos.
// Note 55: Biblioteca de RTU abstrai parte da temporizacao de quadro e CRC.
// Note 56: Ainda assim, escolha de baud/paridade/stop bits deve ser consistente com mestre.
// Note 57: Device ID Modbus deve estar no intervalo 1..247 por convencao do protocolo.
// Note 58: O parser Modbus TCP implementado manualmente fornece controle didatico sobre MBAP e PDU.
// Note 59: MBAP contem Transaction ID, Protocol ID, Length e Unit ID.
// Note 60: Protocol ID deve ser zero em Modbus TCP padrao.
// Note 61: Length em MBAP inclui Unit ID + tamanho do PDU.
// Note 62: Requisicoes de leitura de bits usam FC 0x01 e 0x02.
// Note 63: Requisicoes de leitura de registradores usam FC 0x03 e 0x04.
// Note 64: Escrita simples usa FC 0x05 para coil e FC 0x06 para registrador.
// Note 65: Em excecao Modbus, bit mais significativo do function code e setado (fc | 0x80).
// Note 66: Codigo de excecao 0x01 indica funcao ilegal para este escravo.
// Note 67: Codigo de excecao 0x02 indica endereco ilegal.
// Note 68: Codigo de excecao 0x03 indica valor de dado ilegal.
// Note 69: O firmware limita HREG a 0..1023 para casar com faixa PWM.
// Note 70: Limitar faixa evita comportamento indefinido quando cliente escreve valores fora do esperado.
// Note 71: Escrita de coil aceita somente 0xFF00 ou 0x0000 conforme especificacao.
// Note 72: Qualquer outro valor na escrita de coil deve retornar erro de dado ilegal.
// Note 73: Com protocolos exclusivos, Modbus TCP escreve direto no modelo; nao ha sync TCP<->RTU.
// Note 74: No modo Modbus RTU, coils/hregs pertencem ao banco RTU (escritas do mestre).
// Note 75: Entradas (ists/iregs) fluem modelo->RTU; saidas (coils/hregs) fluem RTU->modelo.
// Note 76: Atualizacao de sensores em periodo separado evita sobrecarga no parser de rede.
// Note 77: DS18B20 pode levar tempo de conversao, por isso update periodico controlado e importante.
// Note 78: Conversao DS18B20 para x10 elimina float na interface Modbus e simplifica SCADA.
// Note 79: safeAnalogRead protege contra limites fora de faixa ao publicar no registrador.
// Note 80: Em firmware robusto, clamp de entrada e comum para prevenir propagacao de ruido.
// Note 81: updateDiscreteInputs espelha estado fisico dos pinos em registradores discretos.
// Note 82: Quando debug serial esta ativo, RX nao e usado como entrada para evitar conflito UART.
// Note 83: applyOutputs e o ponto onde registradores viram efeitos fisicos em pinos.
// Note 84: Separar leitura de entrada e aplicacao de saida melhora previsibilidade da logica.
// Note 85: O blink no modo AP oferece feedback visual sem depender de monitor serial.
// Note 86: Frequencia de 1 Hz e compreensivel para operador humano em bancada.
// Note 87: Para cargas reais, substituir LED por driver apropriado e necessario.
// Note 88: O HTML embutido adota estilo simples para minimizar footprint de firmware.
// Note 89: Reservar tamanho da String ajuda a reduzir fragmentacao de heap.
// Note 90: Mesmo assim, uso intensivo de String em ESP8266 exige cuidado em sessoes longas.
// Note 91: Campos de formulario mapeiam diretamente para estrutura DeviceConfig.
// Note 92: Validacao server-side e obrigatoria mesmo com constraints no HTML.
// Note 93: Sem validacao no firmware, cliente malformado pode gravar config invalida.
// Note 94: A persistencia imediata apos POST reduz risco de perda apos queda de energia.
// Note 95: startWifiClient aplica IP estatico configurado e inicia tentativa de associacao.
// Note 96: Timeout de conexao evita espera infinita em redes indisponiveis.
// Note 97: startApMode define IP 192.168.4.1 para setup local conhecido.
// Note 98: AP e STA em ESP8266 compartilham recursos; separar estados evita competicao.
// Note 99: configureRtu usa parametros persistidos, permitindo ajuste de campo sem recompilar.
// Note 100: Inicializar banco Modbus antes do loop evita leituras de lixo na partida.
// Note 101: A funcao initModbusRegisters registra ponteiros/valores iniciais na biblioteca RTU.
// Note 102: syncModelToRtu e idempotente; repetir nao muda resultado quando dados iguais.
// Note 103: Idempotencia e util em sistemas com sincronizacao periodica.
// Note 104: O parser de PDU assume tamanho minimo de 5 bytes para FCs suportadas.
// Note 105: FCs nao suportadas retornam excecao em vez de ignorar silenciosamente.
// Note 106: Ignorar silenciosamente dificulta diagnostico em cliente mestre.
// Note 107: Leitura de bits empacota ate 8 bits por byte no payload Modbus.
// Note 108: O byte count na resposta e calculado por (qty + 7) / 8.
// Note 109: Leitura de registradores retorna byte count igual a quantidade * 2.
// Note 110: Endianness Modbus para registrador e big-endian por byte alto primeiro.
// Note 111: readTcpExact trata fragmentacao TCP e monta frame completo antes do parse.
// Note 112: TCP e stream, nao mensagem; uma leitura pode trazer parcial ou multiplos frames.
// Note 113: Assumir read unico para frame inteiro e erro comum em implementacoes iniciantes.
// Note 114: Timeout curto no readTcpExact evita congelar loop em cliente lento ou malicioso.
// Note 115: Em timeout de frame incompleto, conexao e encerrada para limpar estado.
// Note 116: Limpar estado de conexao evita parser desalinhado em quadros seguintes.
// Note 117: O Unit ID de resposta deve refletir o Unit ID recebido no MBAP.
// Note 118: Ecoar Unit ID melhora compatibilidade com gateways e clientes estritos.
// Note 119: Enviar MBAP+PDU em write unico reduz chance de cliente mal implementado falhar.
// Note 120: Embora TCP permita segmentacao, muitos stacks de alto nivel esperam buffer coeso.
// Note 121: O tamanho maximo de frame de resposta cabe confortavelmente no buffer de 260 bytes.
// Note 122: 7 bytes MBAP + ate 253 bytes PDU totalizam 260 bytes.
// Note 123: Uso de buffer local no stack e seguro aqui por tamanho pequeno e escopo curto.
// Note 124: Evite buffers gigantes no stack em microcontroladores com RAM limitada.
// Note 125: handleStateMachine coordena transicoes sem bloquear, baseado em tempo e eventos.
// Note 126: Diferenciar estado e acao simplifica depuracao de fluxos de inicializacao.
// Note 127: Variavel stateStartedAt permite calcular tempo no estado atual.
// Note 128: Esse padrao e comum para watchdog logico e timeouts de reconexao.
// Note 129: webServer.handleClient deve ser chamado frequentemente para evitar timeout HTTP.
// Note 130: O loop atual garante atendimento web mesmo fora do estado RUN_MODBUS.
// Note 131: Isso e importante para recuperar configuracao quando conexao STA falha.
// Note 132: modbusRtu.task deve rodar com frequencia para manter timings de quadro RTU.
// Note 133: Latencia alta no loop pode degradar confiabilidade da serial em alta taxa.
// Note 134: SENSOR_PERIOD_MS de 1200ms reduz carga de sensor sem perder didatica da leitura.
// Note 135: IO_PERIOD_MS de 60ms entrega resposta visual suficientemente rapida para PWM/LED.
// Note 136: MODBUS_SYNC_PERIOD_MS de 25ms equilibra consistencia e custo de CPU.
// Note 137: Valores de periodo devem ser ajustados conforme carga e exigencia de processo.
// Note 138: Em bancada, estabilidade e clareza sao mais importantes que maxima taxa.
// Note 139: O uso de bool para coils e discretes reduz ambiguidade de estados.
// Note 140: Para telemetria historica, considerar incluir timestamp no lado supervisorio.
// Note 141: Configuracao de DNS e pouco usada no contexto atual, mas mantida por completude.
// Note 142: Mesmo com IP estatico, DNS pode ser util para recursos HTTP externos futuros.
// Note 143: A escolha por arquivo texto facilita inspeção manual durante testes.
// Note 144: Formato chave=valor simplifica parse sem dependencia de parser JSON.
// Note 145: A contrapartida e falta de estrutura para campos aninhados complexos.
// Note 146: Para escala maior, formato CBOR ou JSON com CRC pode ser considerado.
// Note 147: Hoje a simplicidade favorece aprendizado e manutencao local.
// Note 148: O codigo evita excecoes C++ por custo e suporte limitado em embedded.
// Note 149: Tratamento de erro e feito por retorno bool e codigos explicitos.
// Note 150: Esse estilo e comum em firmware por previsibilidade de recursos.
// Note 151: O parser de IPv4 rejeita caracteres invalidos e octetos fora de 0..255.
// Note 152: Validacoes explicitas ajudam a prevenir estados impossiveis.
// Note 153: Estados impossiveis em firmware causam bugs dificeis de reproduzir.
// Note 154: A funcao validateConfig centraliza regras de consistencia de configuracao.
// Note 155: Centralizar regras evita divergencia entre validacao web e carregamento de arquivo.
// Note 156: Em caso de configuracao invalida no arquivo, defaults continuam operacionais.
// Note 157: Esse fallback melhora resiliencia apos corrupcao parcial de filesystem.
// Note 158: A chamada LittleFS.format e fallback extremo; use com cautela em sistemas produtivos.
// Note 159: Em ambiente didatico, format automatico simplifica recuperacao de placa.
// Note 160: O monitor serial em 115200 e padrao comum para ESP8266.
// Note 161: Com SERIAL_DEBUG_ENABLED=false, logs continuam no codigo mas sem uso de TX/RX reservado.
// Note 162: Essa chave de compilacao permite alternar facilmente entre debug e IO extra.
// Note 163: O pino de AP trigger e RX/GPIO3, compartilhado com a entrada discreta ED2 (10002).
// Note 164: O trigger e amostrado apenas no boot; em runtime o pino volta a funcionar como entrada discreta.
// Note 165: GPIO3 nao e pino de bootstrap; com INPUT_PULLUP, o gatilho ativo em baixo funciona como modo opcional.
// Note 166: Regras de segurança eletrica devem ser seguidas ao acionar cargas reais.
// Note 167: GPIOs nao devem alimentar cargas acima da corrente suportada pelo microcontrolador.
// Note 168: Use transistor, MOSFET ou modulo relay para cargas indutivas.
// Note 169: Cargas indutivas exigem diodo flyback para proteger o transistor e o MCU.
// Note 170: A referencia comum de GND entre placa e perifericos e obrigatoria para leitura correta.
// Note 171: Em RS485, terminacao e polarizacao do barramento impactam confiabilidade.
// Note 172: Em bancada curta pode funcionar sem terminacao, mas nao e boa pratica em campo.
// Note 173: Sempre verificar inversao de linhas A/B quando nao ha comunicacao.
// Note 174: Unit ID em TCP pode ser ignorado por alguns dispositivos, mas manter correto e melhor pratica.
// Note 175: O firmware responde funcao por funcao, facilitando extensao futura para FC 0x10.
// Note 176: FC 0x10 (write multiple registers) pode reduzir trafego em atualizacoes de bloco.
// Note 177: Ao adicionar novas FCs, mantenha validacao de faixa e tamanho para seguranca.
// Note 178: Cliente malformado nao deve conseguir forcar escrita fora dos limites de array.
// Note 179: Validacao de limites protege contra corrupcao de memoria.
// Note 180: Corrupcao de memoria em firmware costuma causar resets aparentemente aleatorios.
// Note 181: A opcao por arrays estaticos elimina alocacao dinamica em runtime critico.
// Note 182: Menos alocacao dinamica significa menor risco de fragmentacao de heap.
// Note 183: O HTML inline aumenta tamanho do binario, mas evita depender de arquivos extras no FS.
// Note 184: Para UIs maiores, servir arquivos estaticos do LittleFS costuma ser melhor.
// Note 185: A resposta web em texto simples facilita debug via curl durante manutencao.
// Note 186: Endpoint /health pode ser usado em supervisao para alarme de estado FAILSAFE_AP.
// Note 187: Uma melhoria futura e incluir uptime e RSSI no payload de health.
// Note 188: Outra melhoria e expor motivo da ultima transicao de estado.
// Note 189: Em protocolos industriais, rastreabilidade reduz tempo medio de reparo.
// Note 190: O design atual privilegia clareza de fluxo para ensino de integracao Modbus.
// Note 191: A compatibilidade com Node-RED depende de framing correto e parse robusto.
// Note 192: Problemas intermitentes de leitura costumam envolver timing, fragmentacao ou estado de socket.
// Note 193: Encerrar socket em erro de frame evita lock de sessao parcialmente lida.
// Note 194: O servidor aceita novo cliente quando conexao atual cai ou invalida.
// Note 195: Em cenarios multi-cliente, seria necessario gerenciar lista de sockets.
// Note 196: O ESP8266 tem recursos limitados, entao single-client simplifica estabilidade.
// Note 197: Para producao, definir politicas de reconexao e limite de taxa pode evitar abuso.
// Note 198: Esta base pode evoluir para autenticar setup web em ambiente compartilhado.
// Note 199: Sem autenticacao, qualquer cliente na rede AP pode alterar configuracao.
// Note 200: Em ambiente didatico isso e aceitavel, mas em campo e risco de seguranca.
// Note 201: Testes automatizados com pymodbus ajudam regressao apos cada mudanca de firmware.
// Note 202: Scripts de teste devem cobrir DI, IR, HR e coil para validar mapa completo.
// Note 203: Sempre testar TCP e RTU apos ajustes de sincronizacao entre modelos.
// Note 204: Um protocolo pode passar enquanto o outro falha por ordem de sync incorreta.
// Note 205: O comportamento observado no tcpdump e ferramenta essencial para depurar framing.
// Note 206: ACK sem resposta Modbus pode indicar parse abortado no servidor.
// Note 207: Resposta em segmentos multiplos e normal em TCP, mas clientes ruins podem falhar.
// Note 208: Envio em frame unico reduz atrito com clientes menos robustos.
// Note 209: Mesmo assim, servidor continua compatível com stack TCP padrao.
// Note 210: Ajustes de timeout devem considerar latencia Wi-Fi e carga da CPU.
// Note 211: Timeout muito curto gera falsos negativos; muito longo aumenta demora de falha.
// Note 212: O valor escolhido busca equilibrio para rede local de bancada.
// Note 213: Em redes congestionadas, aumentar timeout pode melhorar estabilidade percebida.
// Note 214: Em processos criticos, heartbeat de aplicacao pode complementar watchdog de hardware.
// Note 215: Heartbeat no SCADA detecta indisponibilidade mesmo quando link IP ainda responde ping.
// Note 216: O firmware atual ja oferece base robusta para experimentos de automacao didatica.
// Note 217: Comentarios numerados facilitam referencia cruzada em aula, revisao e code review.
// Note 218: Para futuras extensoes, mantenha padrao de notas e modularidade progressiva.
// Note 219: Antes de mudar mapeamento de pinos, atualize firmware, README, spec e testes em conjunto.
// Note 220: Consistencia entre codigo e documentacao evita erros de bancada e retrabalho.
// Note 221: Se o projeto crescer, extraia interfaces para testes unitarios sem hardware real.
// Note 222: Simuladores de registradores permitem validar parser Modbus em CI sem dispositivo fisico.
// Note 223: A estrategia de evolucao ideal e incremental: pequena mudanca, build, upload e reteste.
// Note 224: Esse ciclo curto reduz risco de regressao dificil de localizar.
// Note 225: O objetivo final desta base e ensinar integracao confiavel entre firmware, rede e supervisao.
// Note 226: O pino LED_BUILTIN (GPIO2/D4) e compartilhado com PIN_RS485_DIR.
// Note 227: No modo config (AP_SETUP), o RS485 esta inativo, entao o pino pode ser usado como LED.

constexpr uint8_t PIN_CONFIG_LED = LED_BUILTIN; // GPIO2 / D4 (ativo em LOW)

constexpr uint8_t PIN_COIL_0 = D0;         // GPIO16
constexpr uint8_t PIN_COIL_1 = 1;          // TX / GPIO1
constexpr uint8_t PIN_AP_TRIGGER = 3;      // RX / GPIO3 (ED2, amostrado so no boot, ativo em nivel baixo -> AP)
constexpr uint8_t PIN_INPUT_0 = D8;        // GPIO15 (entrada discreta 10001)
constexpr uint8_t PIN_INPUT_1 = 3;         // RX / GPIO3 (entrada discreta 10002; tambem gatilho de setup no boot)
constexpr uint8_t PIN_DS18B20 = D2;        // GPIO4
constexpr uint8_t PIN_PWM_AZIMUTH = D7;    // GPIO13
constexpr uint8_t PIN_PWM_ELEVATION = D3;  // GPIO0
constexpr uint8_t PIN_PWM_HEATER = D1;     // GPIO5 (dedicado a resistencia de aquecimento)

// RS485 MAX485: RO = D5 (RX), DI = D6 (TX), DE/RE = D4 para nao conflitar com PWM em D3.
constexpr uint8_t PIN_RS485_RX = D5;       // GPIO14
constexpr uint8_t PIN_RS485_TX = D6;       // GPIO12
constexpr uint8_t PIN_RS485_DIR = D4;      // GPIO2

// Porta padrao do servico Modbus TCP (configuravel na web e persistida em /config.txt).
constexpr uint16_t MODBUS_TCP_PORT_DEFAULT = 502;
constexpr uint8_t MODBUS_UNIT_ID_DEFAULT = 1;
constexpr uint32_t WIFI_CONNECT_TIMEOUT_MS = 30000UL;
constexpr uint32_t SENSOR_PERIOD_MS = 1200UL;
// Intervalo minimo entre disparo e leitura da conversao do DS18B20.
// Evita ler o scratchpad antes da conversao terminar (0 periodico no barramento).
constexpr uint16_t DS18B20_MIN_CONVERSION_WAIT_MS = 500;
constexpr uint32_t IO_PERIOD_MS = 60UL;
constexpr uint32_t MODBUS_SYNC_PERIOD_MS = 25UL;
constexpr uint32_t MQTT_PUBLISH_PERIOD_MS = 1200UL;
constexpr uint32_t MQTT_RECONNECT_MS = 3000UL;
constexpr uint32_t AP_RETRY_DELAY_MS = 2500UL;
constexpr uint32_t AP_BLINK_HALF_PERIOD_MS = 500UL;
constexpr uint32_t MODBUS_TCP_READ_TIMEOUT_MS = 120UL;
constexpr uint32_t WATCHDOG_TIMEOUT_MS = 4000UL;
constexpr bool SERIAL_DEBUG_ENABLED = false;
constexpr uint16_t MQTT_PORT_DEFAULT = 1883;
constexpr uint16_t OPCUA_DATASET_WRITER_ID_DEFAULT = 1;

constexpr uint8_t COILS_COUNT = 2;
constexpr uint8_t ISTS_COUNT = 2;
constexpr uint8_t IREG_COUNT = 2;
constexpr uint8_t HREG_COUNT = 3;

constexpr const char *CONFIG_PATH = "/config.txt";
constexpr const char *CONFIG_TMP_PATH = "/config.tmp";

enum class DeviceState : uint8_t {
  BOOT,
  AP_SETUP,
  WIFI_CONNECTING,
  RUN_PROTOCOL,
  FAILSAFE_AP
};

enum class ActiveProtocol : uint8_t {
  MODBUS_TCP,
  MODBUS_RTU,
  MQTT,
  OPCUA
};

struct DeviceConfig {
  String ssid;
  String psk;
  String ip;
  String mask;
  String gateway;
  String dns;
  uint8_t deviceId;
  uint16_t modbusTcpPort;
  uint32_t baud;
  char parity;
  uint8_t stopBits;
  ActiveProtocol protocol;
  String mqttBroker;
  uint16_t mqttPort;
  String mqttUser;
  String mqttPass;
  String mqttTopicBase;
  String mqttMode;            // "geral" ou "thingsboard"
  String mqttPublishTopic;    // topico principal de publicacao (modo geral)
  String mqttSubscribeTopic;  // topico de subscricao (modo geral)
  String opcUaPublisherId;
  uint16_t opcUaDataSetWriterId;
};

DeviceConfig config;
DeviceState state = DeviceState::BOOT;

ESP8266WebServer webServer(80);
// Servidor Modbus TCP alocado dinamicamente para permitir porta configuravel.
WiFiServer *modbusTcpServer = nullptr;
WiFiClient modbusTcpClient;
WiFiClient mqttNetClient;
PubSubClient mqttClient(mqttNetClient);

SoftwareSerial rs485Serial(PIN_RS485_RX, PIN_RS485_TX);
ModbusSerial modbusRtu;

OneWire oneWire(PIN_DS18B20);
DallasTemperature ds18b20(&oneWire);
uint8_t ds18b20Address[8] = {0};
bool ds18b20AddressFound = false;

bool coils[COILS_COUNT] = {false, false};
bool ists[ISTS_COUNT] = {false, false};
uint16_t iregs[IREG_COUNT] = {0, 0};
uint16_t hregs[HREG_COUNT] = {0, 0, 0};

bool appliedCoils[COILS_COUNT] = {false, false};
uint16_t appliedHregs[HREG_COUNT] = {0, 0, 0};
bool outputsInitialized = false;

unsigned long stateStartedAt = 0;
unsigned long lastSensorMs = 0;
unsigned long lastIoMs = 0;
unsigned long lastModbusSyncMs = 0;
unsigned long lastMqttPublishMs = 0;
unsigned long lastMqttReconnectMs = 0;
unsigned long lastDsRequestMs = 0;
uint16_t dsConversionWaitMs = DS18B20_MIN_CONVERSION_WAIT_MS;
unsigned long apSaveAtMs = 0;
bool pendingConnectAfterSave = false;

String apSsid;
String mqttInputsTopic;
String mqttOutputsTopic;
String mqttStatusTopic;
String opcUaInputsTopic;
String opcUaOutputsTopic;
String opcUaStatusTopic;
bool opcUaRuntimeSupported = true;

unsigned long lastConfigLedBlinkMs = 0;

void blinkConfigLed() {
  if (state != DeviceState::AP_SETUP && state != DeviceState::FAILSAFE_AP) {
    digitalWrite(PIN_CONFIG_LED, HIGH); // Garante LED apagado ao sair do modo
    return;
  }
  const unsigned long now = millis();
  if (now - lastConfigLedBlinkMs >= 500UL) {
    lastConfigLedBlinkMs = now;
    digitalWrite(PIN_CONFIG_LED, !digitalRead(PIN_CONFIG_LED));
  }
}

void logStationIp() {
  Serial.print("WiFi conectado ao SSID: ");
  Serial.println(config.ssid);
  Serial.print("IP cliente obtido: ");
  Serial.println(WiFi.localIP());
}

void logSerialDebugMode() {
  if (!SERIAL_DEBUG_ENABLED) {
    return;
  }

  Serial.println();
  Serial.println("Debug serial ativo em 115200 baud.");
  Serial.println("GPIO1/TX e GPIO3/RX reservados para monitor serial.");
}

String getStateName() {
  switch (state) {
    case DeviceState::BOOT:
      return "BOOT";
    case DeviceState::AP_SETUP:
      return "AP_SETUP";
    case DeviceState::WIFI_CONNECTING:
      return "WIFI_CONNECTING";
    case DeviceState::RUN_PROTOCOL:
      return "RUN_PROTOCOL";
    case DeviceState::FAILSAFE_AP:
      return "FAILSAFE_AP";
    default:
      return "UNKNOWN";
  }
}

const char *protocolToCString(ActiveProtocol protocol) {
  switch (protocol) {
    case ActiveProtocol::MODBUS_TCP:
      return "modbustcp";
    case ActiveProtocol::MODBUS_RTU:
      return "modbusrtu";
    case ActiveProtocol::MQTT:
      return "mqtt";
    case ActiveProtocol::OPCUA:
      return "opcua";
    default:
      return "modbustcp";
  }
}

ActiveProtocol protocolFromString(const String &value) {
  if (value == "mqtt") {
    return ActiveProtocol::MQTT;
  }
  if (value == "opcua" || value == "opc ua") {
    return ActiveProtocol::OPCUA;
  }
  if (value == "modbusrtu") {
    return ActiveProtocol::MODBUS_RTU;
  }
  // "modbustcp" e o valor legado "modbus" apontam para Modbus TCP.
  return ActiveProtocol::MODBUS_TCP;
}

bool parseUInt32Strict(const String &value, uint32_t &out) {
  if (value.isEmpty()) {
    return false;
  }

  uint32_t parsed = 0;
  for (size_t i = 0; i < value.length(); i++) {
    char c = value[i];
    if (c < '0' || c > '9') {
      return false;
    }

    uint32_t next = parsed * 10UL + static_cast<uint32_t>(c - '0');
    if (next < parsed) {
      return false;
    }
    parsed = next;
  }

  out = parsed;
  return true;
}

bool parseUInt16Strict(const String &value, uint16_t &out) {
  uint32_t parsed = 0;
  if (!parseUInt32Strict(value, parsed) || parsed > 65535UL) {
    return false;
  }
  out = static_cast<uint16_t>(parsed);
  return true;
}

void sanitizeProtocolConfig(DeviceConfig &cfg) {
  cfg.ip.trim();
  cfg.mask.trim();
  cfg.gateway.trim();
  cfg.dns.trim();
  cfg.mqttBroker.trim();
  cfg.mqttTopicBase.trim();
  cfg.mqttMode.trim();
  cfg.mqttPublishTopic.trim();
  cfg.mqttSubscribeTopic.trim();
  cfg.opcUaPublisherId.trim();

  if (cfg.mqttMode != "thingsboard") {
    cfg.mqttMode = "geral";
  }

  if (cfg.modbusTcpPort == 0) {
    cfg.modbusTcpPort = MODBUS_TCP_PORT_DEFAULT;
  }
}

bool parseIPv4(const String &value, IPAddress &out) {
  uint8_t parts[4] = {0, 0, 0, 0};
  uint8_t partIdx = 0;
  uint16_t current = 0;
  bool hasDigit = false;

  for (size_t i = 0; i < value.length(); i++) {
    char c = value[i];
    if (c >= '0' && c <= '9') {
      hasDigit = true;
      current = static_cast<uint16_t>(current * 10 + (c - '0'));
      if (current > 255) {
        return false;
      }
      continue;
    }

    if (c == '.') {
      if (!hasDigit || partIdx >= 3) {
        return false;
      }
      parts[partIdx++] = static_cast<uint8_t>(current);
      current = 0;
      hasDigit = false;
      continue;
    }

    return false;
  }

  if (!hasDigit || partIdx != 3) {
    return false;
  }

  parts[3] = static_cast<uint8_t>(current);
  out = IPAddress(parts[0], parts[1], parts[2], parts[3]);
  return true;
}

bool hasRequiredConfig(const DeviceConfig &cfg) {
  if (cfg.ssid.isEmpty() || cfg.psk.isEmpty() || cfg.ip.isEmpty() ||
      cfg.mask.isEmpty() || cfg.gateway.isEmpty() || cfg.dns.isEmpty()) {
    return false;
  }

  if (cfg.protocol == ActiveProtocol::MODBUS_TCP) {
    return cfg.deviceId >= 1 && cfg.deviceId <= 247 && cfg.modbusTcpPort > 0;
  }

  if (cfg.protocol == ActiveProtocol::MODBUS_RTU) {
    return cfg.deviceId >= 1 && cfg.deviceId <= 247 && cfg.baud > 0;
  }

  if (cfg.protocol == ActiveProtocol::MQTT) {
    return !cfg.mqttBroker.isEmpty() && cfg.mqttPort > 0;
  }

  if (cfg.protocol == ActiveProtocol::OPCUA) {
    return !cfg.mqttBroker.isEmpty() && cfg.mqttPort > 0 && !cfg.opcUaPublisherId.isEmpty();
  }

  return false;
}

bool validateConfig(const DeviceConfig &cfg, String &error) {
  if (!hasRequiredConfig(cfg)) {
    error = "Campos obrigatorios ausentes.";
    return false;
  }

  IPAddress ip;
  if (!parseIPv4(cfg.ip, ip)) {
    error = "IP invalido.";
    return false;
  }
  if (!parseIPv4(cfg.mask, ip)) {
    error = "Mascara invalida.";
    return false;
  }
  if (!parseIPv4(cfg.gateway, ip)) {
    error = "Gateway invalido.";
    return false;
  }
  if (!parseIPv4(cfg.dns, ip)) {
    error = "DNS invalido.";
    return false;
  }

  if (cfg.protocol == ActiveProtocol::MODBUS_TCP && cfg.modbusTcpPort == 0) {
    error = "Porta Modbus TCP invalida.";
    return false;
  }

  if (cfg.protocol == ActiveProtocol::MODBUS_RTU) {
    if (cfg.baud == 0) {
      error = "Baud rate invalido.";
      return false;
    }

    if (!(cfg.parity == 'N' || cfg.parity == 'E' || cfg.parity == 'O')) {
      error = "Paridade invalida.";
      return false;
    }

    if (!(cfg.stopBits == 1 || cfg.stopBits == 2)) {
      error = "Stop bits invalido.";
      return false;
    }
  }

  if (cfg.protocol == ActiveProtocol::MQTT) {
    if (cfg.mqttPort == 0) {
      error = "Porta MQTT invalida.";
      return false;
    }
  }

  if (cfg.protocol == ActiveProtocol::OPCUA) {
    if (cfg.opcUaPublisherId.isEmpty()) {
      error = "OPC UA Publisher ID invalido.";
      return false;
    }
    if (cfg.opcUaDataSetWriterId == 0) {
      error = "OPC UA DataSet Writer ID invalido.";
      return false;
    }
  }

  return true;
}

void setDefaultConfig(DeviceConfig &cfg) {
  cfg.ssid = "linus";
  cfg.psk = "lapela593";
  cfg.ip = "192.168.100.200";
  cfg.mask = "255.255.255.0";
  cfg.gateway = "192.168.100.1";
  cfg.dns = "8.8.8.8";
  cfg.deviceId = MODBUS_UNIT_ID_DEFAULT;
  cfg.modbusTcpPort = MODBUS_TCP_PORT_DEFAULT;
  cfg.baud = 9600;
  cfg.parity = 'N';
  cfg.stopBits = 1;
  cfg.protocol = ActiveProtocol::MODBUS_TCP;
  cfg.mqttBroker = "192.168.100.10";
  cfg.mqttPort = MQTT_PORT_DEFAULT;
  cfg.mqttUser = "";
  cfg.mqttPass = "";
  cfg.mqttTopicBase = "nodemcu";
  cfg.mqttMode = "geral";
  cfg.mqttPublishTopic = "";
  cfg.mqttSubscribeTopic = "";
  cfg.opcUaPublisherId = "NodeMCU_" + String(ESP.getChipId(), HEX);
  cfg.opcUaDataSetWriterId = OPCUA_DATASET_WRITER_ID_DEFAULT;
}

bool saveConfig(const DeviceConfig &cfg) {
  DeviceConfig snapshot = cfg;
  sanitizeProtocolConfig(snapshot);

  File f = LittleFS.open(CONFIG_TMP_PATH, "w");
  if (!f) {
    return false;
  }

  f.printf("ssid=%s\n", snapshot.ssid.c_str());
  f.printf("psk=%s\n", snapshot.psk.c_str());
  f.printf("ip=%s\n", snapshot.ip.c_str());
  f.printf("mask=%s\n", snapshot.mask.c_str());
  f.printf("gateway=%s\n", snapshot.gateway.c_str());
  f.printf("dns=%s\n", snapshot.dns.c_str());
  f.printf("deviceId=%u\n", snapshot.deviceId);
  f.printf("modbusTcpPort=%u\n", snapshot.modbusTcpPort);
  f.printf("baud=%lu\n", static_cast<unsigned long>(snapshot.baud));
  f.printf("parity=%c\n", snapshot.parity);
  f.printf("stopBits=%u\n", snapshot.stopBits);
  f.printf("protocol=%s\n", protocolToCString(snapshot.protocol));
  f.printf("mqttBroker=%s\n", snapshot.mqttBroker.c_str());
  f.printf("mqttPort=%u\n", snapshot.mqttPort);
  f.printf("mqttUser=%s\n", snapshot.mqttUser.c_str());
  f.printf("mqttPass=%s\n", snapshot.mqttPass.c_str());
  f.printf("mqttTopicBase=%s\n", snapshot.mqttTopicBase.c_str());
  f.printf("mqttMode=%s\n", snapshot.mqttMode.c_str());
  f.printf("mqttPublishTopic=%s\n", snapshot.mqttPublishTopic.c_str());
  f.printf("mqttSubscribeTopic=%s\n", snapshot.mqttSubscribeTopic.c_str());
  f.printf("opcUaPublisherId=%s\n", snapshot.opcUaPublisherId.c_str());
  f.printf("opcUaDataSetWriterId=%u\n", snapshot.opcUaDataSetWriterId);
  f.close();

  if (LittleFS.exists(CONFIG_PATH) && !LittleFS.remove(CONFIG_PATH)) {
    LittleFS.remove(CONFIG_TMP_PATH);
    return false;
  }

  if (!LittleFS.rename(CONFIG_TMP_PATH, CONFIG_PATH)) {
    LittleFS.remove(CONFIG_TMP_PATH);
    return false;
  }

  return true;
}

bool loadConfig(DeviceConfig &cfg) {
  if (!LittleFS.exists(CONFIG_PATH)) {
    return false;
  }

  File f = LittleFS.open(CONFIG_PATH, "r");
  if (!f) {
    return false;
  }

  DeviceConfig temp;
  setDefaultConfig(temp);

  while (f.available()) {
    String line = f.readStringUntil('\n');
    line.trim();
    if (line.isEmpty()) {
      continue;
    }
    int eq = line.indexOf('=');
    if (eq <= 0) {
      continue;
    }

    String key = line.substring(0, eq);
    String value = line.substring(eq + 1);

    if (key == "ssid") temp.ssid = value;
    else if (key == "psk") temp.psk = value;
    else if (key == "ip") temp.ip = value;
    else if (key == "mask") temp.mask = value;
    else if (key == "gateway") temp.gateway = value;
    else if (key == "dns") temp.dns = value;
    else if (key == "deviceId") {
      uint32_t parsed = 0;
      if (!parseUInt32Strict(value, parsed) || parsed > 255UL) {
        f.close();
        return false;
      }
      temp.deviceId = static_cast<uint8_t>(parsed);
    }
    else if (key == "modbusTcpPort") {
      uint16_t parsed = 0;
      if (!parseUInt16Strict(value, parsed)) {
        f.close();
        return false;
      }
      temp.modbusTcpPort = parsed;
    }
    else if (key == "baud") {
      uint32_t parsed = 0;
      if (!parseUInt32Strict(value, parsed)) {
        f.close();
        return false;
      }
      temp.baud = parsed;
    }
    else if (key == "parity" && !value.isEmpty()) temp.parity = value[0];
    else if (key == "stopBits") {
      uint32_t parsed = 0;
      if (!parseUInt32Strict(value, parsed) || parsed > 255UL) {
        f.close();
        return false;
      }
      temp.stopBits = static_cast<uint8_t>(parsed);
    }
    else if (key == "protocol") temp.protocol = protocolFromString(value);
    else if (key == "mqttBroker") temp.mqttBroker = value;
    else if (key == "mqttPort") {
      uint16_t parsed = 0;
      if (!parseUInt16Strict(value, parsed)) {
        f.close();
        return false;
      }
      temp.mqttPort = parsed;
    }
    else if (key == "mqttUser") temp.mqttUser = value;
    else if (key == "mqttPass") temp.mqttPass = value;
    else if (key == "mqttTopicBase") temp.mqttTopicBase = value;
    else if (key == "mqttMode") temp.mqttMode = value;
    else if (key == "mqttPublishTopic") temp.mqttPublishTopic = value;
    else if (key == "mqttSubscribeTopic") temp.mqttSubscribeTopic = value;
    else if (key == "opcUaPublisherId") temp.opcUaPublisherId = value;
    else if (key == "opcUaDataSetWriterId") {
      uint16_t parsed = 0;
      if (!parseUInt16Strict(value, parsed)) {
        f.close();
        return false;
      }
      temp.opcUaDataSetWriterId = parsed;
    }
  }
  f.close();

  sanitizeProtocolConfig(temp);

  String error;
  if (!validateConfig(temp, error)) {
    return false;
  }

  cfg = temp;
  return true;
}

void initModbusRegisters() {
  for (uint8_t i = 0; i < COILS_COUNT; i++) {
    modbusRtu.addCoil(i, coils[i]);
  }
  for (uint8_t i = 0; i < ISTS_COUNT; i++) {
    modbusRtu.addIsts(i, ists[i]);
  }
  for (uint8_t i = 0; i < IREG_COUNT; i++) {
    modbusRtu.addIreg(i, iregs[i]);
  }
  for (uint8_t i = 0; i < HREG_COUNT; i++) {
    modbusRtu.addHreg(i, hregs[i]);
  }
}

void configureRtu() {
  rs485Serial.begin(config.baud);
  modbusRtu.setSlaveId(config.deviceId);
  modbusRtu.config(&rs485Serial, config.baud, PIN_RS485_DIR);
}

// Direcao entradas (modelo -> banco RTU): apenas ists e iregs.
// Em modo RTU exclusivo, coils/hregs pertencem ao banco RTU (escritas do mestre),
// entao nunca devem ser sobrescritos por copia modelo->banco em runtime.
void syncModelInputsToRtu() {
  for (uint8_t i = 0; i < ISTS_COUNT; i++) {
    modbusRtu.Ists(i, ists[i]);
  }
  for (uint8_t i = 0; i < IREG_COUNT; i++) {
    modbusRtu.Ireg(i, iregs[i]);
  }
}

// Direcao saidas (banco RTU -> modelo): coils e hregs escritos pelo mestre RTU.
void syncRtuOutputsToModel() {
  for (uint8_t i = 0; i < COILS_COUNT; i++) {
    coils[i] = modbusRtu.Coil(i);
  }
  for (uint8_t i = 0; i < HREG_COUNT; i++) {
    hregs[i] = modbusRtu.Hreg(i);
  }
}

// Inicializacao: popula o banco RTU com o estado atual do modelo (feito antes do
// runtime, quando nao ha mestre escrevendo; mantido idempotente por seguranca).
void syncModelToRtu() {
  for (uint8_t i = 0; i < COILS_COUNT; i++) {
    modbusRtu.Coil(i, coils[i]);
  }
  syncModelInputsToRtu();
  for (uint8_t i = 0; i < HREG_COUNT; i++) {
    modbusRtu.Hreg(i, hregs[i]);
  }
}

bool isModbusTcpActive() {
  return config.protocol == ActiveProtocol::MODBUS_TCP;
}

bool isModbusRtuActive() {
  return config.protocol == ActiveProtocol::MODBUS_RTU;
}

String defaultMqttBaseTopic() {
  String base = "nodemcu/";
  base += String(ESP.getChipId(), HEX);
  return base;
}

bool isThingsboardMqtt() {
  return config.mqttMode == "thingsboard";
}

String effectiveMqttBase() {
  String base = config.mqttTopicBase;
  base.trim();
  if (base.isEmpty()) {
    base = defaultMqttBaseTopic();
  }
  return base;
}

String effectiveMqttPublishTopic() {
  String topic = config.mqttPublishTopic;
  topic.trim();
  if (topic.isEmpty()) {
    topic = effectiveMqttBase() + "/inputs";
  }
  return topic;
}

String effectiveMqttSubscribeTopic() {
  String topic = config.mqttSubscribeTopic;
  topic.trim();
  if (topic.isEmpty()) {
    topic = effectiveMqttBase() + "/outputs/set";
  }
  return topic;
}

void buildMqttTopics() {
  if (isThingsboardMqtt()) {
    mqttInputsTopic = "v1/devices/me/telemetry";
    mqttOutputsTopic = "v1/devices/me/rpc/request/+";
    mqttStatusTopic = "v1/devices/me/attributes";
    return;
  }

  mqttInputsTopic = effectiveMqttPublishTopic();
  mqttOutputsTopic = effectiveMqttSubscribeTopic();
  mqttStatusTopic = effectiveMqttBase() + "/status";
}

void applyMqttOutputsJson(JsonVariantConst doc) {
  JsonVariantConst coilsVar = doc["coils"];
  if (!coilsVar.isNull()) {
    JsonArrayConst arr = coilsVar.as<JsonArrayConst>();
    uint8_t idx = 0;
    for (JsonVariantConst v : arr) {
      if (idx >= COILS_COUNT) {
        break;
      }
      coils[idx++] = v.as<bool>();
    }
  }

  JsonVariantConst hregsVar = doc["holding_registers"];
  if (!hregsVar.isNull()) {
    JsonArrayConst arr = hregsVar.as<JsonArrayConst>();
    uint8_t idx = 0;
    for (JsonVariantConst v : arr) {
      if (idx >= HREG_COUNT) {
        break;
      }
      int candidate = v.as<int>();
      if (candidate < 0) {
        candidate = 0;
      }
      if (candidate > 1023) {
        candidate = 1023;
      }
      hregs[idx++] = static_cast<uint16_t>(candidate);
    }
  }
}

void mqttCallback(char *topic, uint8_t *payload, unsigned int length) {
  String t = String(topic);
  if (isThingsboardMqtt()) {
    // ThingsBoard envia comandos RPC em v1/devices/me/rpc/request/<id>
    if (!t.startsWith("v1/devices/me/rpc/request/")) {
      return;
    }
  } else if (t != mqttOutputsTopic) {
    return;
  }

  StaticJsonDocument<256> doc;
  DeserializationError err = deserializeJson(doc, payload, length);
  if (err) {
    return;
  }

  JsonVariantConst params = doc["params"];
  if (!params.isNull()) {
    // ThingsBoard RPC: { "method": "...", "params": { coils, holding_registers } }
    applyMqttOutputsJson(params);
  } else {
    applyMqttOutputsJson(doc);
  }
}

bool mqttConnectIfNeeded() {
  if (mqttClient.connected()) {
    return true;
  }

  unsigned long now = millis();
  if (now - lastMqttReconnectMs < MQTT_RECONNECT_MS) {
    return false;
  }
  lastMqttReconnectMs = now;

  String clientId = "NodeMCU-";
  clientId += String(ESP.getChipId(), HEX);

  bool connected = false;
  if (isThingsboardMqtt()) {
    // ThingsBoard: usuario = access token do dispositivo; nao usa senha.
    connected = mqttClient.connect(clientId.c_str(), config.mqttUser.c_str(), "");
  } else if (!config.mqttUser.isEmpty()) {
    connected = mqttClient.connect(clientId.c_str(), config.mqttUser.c_str(), config.mqttPass.c_str());
  } else {
    connected = mqttClient.connect(clientId.c_str());
  }

  if (!connected) {
    return false;
  }

  if (config.protocol == ActiveProtocol::OPCUA) {
    mqttClient.subscribe(opcUaOutputsTopic.c_str(), 0);
    mqttClient.publish(opcUaStatusTopic.c_str(), "online", true);
  } else {
    mqttClient.subscribe(mqttOutputsTopic.c_str(), 0);
    if (isThingsboardMqtt()) {
      mqttClient.publish(mqttStatusTopic.c_str(), "{\"online\":true}", true);
    } else {
      mqttClient.publish(mqttStatusTopic.c_str(), "online", true);
    }
  }
  return true;
}

void publishMqttInputs() {
  StaticJsonDocument<256> doc;
  doc["ts_ms"] = millis();
  doc["ip"] = WiFi.localIP().toString();

  JsonArray discrete = doc.createNestedArray("discrete_inputs");
  for (uint8_t i = 0; i < ISTS_COUNT; i++) {
    discrete.add(ists[i]);
  }

  JsonArray iregArr = doc.createNestedArray("input_registers");
  for (uint8_t i = 0; i < IREG_COUNT; i++) {
    iregArr.add(iregs[i]);
  }

  String payload;
  payload.reserve(180);
  serializeJson(doc, payload);
  mqttClient.publish(mqttInputsTopic.c_str(), payload.c_str());
}

void serviceMqtt() {
  if (WiFi.status() != WL_CONNECTED) {
    return;
  }

  if (!mqttConnectIfNeeded()) {
    return;
  }

  mqttClient.loop();

  unsigned long now = millis();
  if (now - lastMqttPublishMs >= MQTT_PUBLISH_PERIOD_MS) {
    lastMqttPublishMs = now;
    publishMqttInputs();
  }
}

void buildOpcUaTopics() {
  String base = config.mqttTopicBase;
  base.trim();
  if (base.isEmpty()) {
    base = "nodemcu/" + String(ESP.getChipId(), HEX);
  }

  opcUaInputsTopic = base + "/opcua/inputs";
  opcUaOutputsTopic = base + "/opcua/outputs/set";
  opcUaStatusTopic = base + "/opcua/status";
}

void publishOpcUaInputs() {
  StaticJsonDocument<512> doc;
  doc["MessageId"] = String(millis()) + "-" + String(ESP.getChipId(), HEX);
  doc["MessageType"] = "ua-data";
  doc["PublisherId"] = config.opcUaPublisherId;
  doc["DataSetWriterId"] = config.opcUaDataSetWriterId;

  JsonObject payload = doc.createNestedObject("Payload");
  payload["Coil_0"] = coils[0];
  payload["Coil_1"] = coils[1];
  payload["DiscreteInput_0"] = ists[0];
  payload["DiscreteInput_1"] = ists[1];
  payload["ADC"] = iregs[0];
  payload["DS18B20"] = iregs[1];
  payload["PWM_Azimuth"] = hregs[0];
  payload["PWM_Elevation"] = hregs[1];
  payload["PWM_Heater"] = hregs[2];

  String payloadStr;
  payloadStr.reserve(380);
  serializeJson(doc, payloadStr);
  mqttClient.publish(opcUaInputsTopic.c_str(), payloadStr.c_str());
}

void opcUaMqttCallback(char *topic, uint8_t *msgPayload, unsigned int length) {
  if (String(topic) != opcUaOutputsTopic) {
    return;
  }

  StaticJsonDocument<256> doc;
  DeserializationError err = deserializeJson(doc, msgPayload, length);
  if (err) {
    return;
  }

  applyMqttOutputsJson(doc);
}

void serviceOpcUa() {
  if (WiFi.status() != WL_CONNECTED) {
    return;
  }

  if (!mqttConnectIfNeeded()) {
    return;
  }

  mqttClient.loop();

  unsigned long now = millis();
  if (now - lastMqttPublishMs >= MQTT_PUBLISH_PERIOD_MS) {
    lastMqttPublishMs = now;
    publishOpcUaInputs();
  }
}

uint16_t safeAnalogRead() {
  int raw = analogRead(A0);
  if (raw < 0) return 0;
  if (raw > 1023) return 1023;
  return static_cast<uint16_t>(raw);
}

uint16_t ds18b20ToTenthC(float tempC) {
  if (tempC == DEVICE_DISCONNECTED_C || isnan(tempC)) {
    return 0;
  }
  int16_t value = static_cast<int16_t>(tempC * 10.0f);
  if (value < 0) {
    value = 0;
  }
  return static_cast<uint16_t>(value);
}

String formatAddress(const uint8_t *address) {
  String out;
  out.reserve(24);
  for (uint8_t i = 0; i < 8; i++) {
    if (address[i] < 16) {
      out += '0';
    }
    out += String(address[i], HEX);
    if (i < 7) {
      out += ':';
    }
  }
  out.toUpperCase();
  return out;
}

bool initDs18b20Address() {
  const uint8_t sensorCount = ds18b20.getDeviceCount();
  if (sensorCount == 0) {
    Serial.println("DS18B20: nenhum sensor encontrado no barramento 1-Wire.");
    return false;
  }

  if (!ds18b20.getAddress(ds18b20Address, 0)) {
    Serial.println("DS18B20: falha ao ler endereco do sensor indice 0.");
    return false;
  }

  Serial.print("DS18B20 endereco detectado: ");
  Serial.println(formatAddress(ds18b20Address));
  return true;
}

void updateSensorRegisters() {
  iregs[0] = safeAnalogRead();

  // Leitura nao bloqueante: dispara a conversao e, quando o tempo minimo de
  // conversao ja foi atingido, coleta o resultado. Isso evita travar o loop
  // por ~750 ms (resolucao 12 bits) a cada ciclo, mantendo Modbus/web/MQTT
  // responsivos. O intervalo entre disparo e leitura e de pelo menos
  // DS18B20_MIN_CONVERSION_WAIT_MS (500 ms).
  unsigned long now = millis();
  if (now - lastDsRequestMs >= dsConversionWaitMs) {
    if (ds18b20AddressFound) {
      float dsTemp = ds18b20.getTempC(ds18b20Address);
      // Em falha transitória de leitura (CRC/ruido no barramento 1-Wire),
      // mantém o último valor válido em vez de publicar 0 no registrador.
      if (dsTemp != DEVICE_DISCONNECTED_C && !isnan(dsTemp)) {
        iregs[1] = ds18b20ToTenthC(dsTemp);
      }
    }

    ds18b20.requestTemperatures();  // setWaitForConversion(false): retorna imediatamente
    lastDsRequestMs = now;
  }
}

void updateDiscreteInputs() {
  ists[0] = digitalRead(PIN_INPUT_0) == HIGH;
  if (SERIAL_DEBUG_ENABLED) {
    ists[1] = false;
  } else {
    ists[1] = digitalRead(PIN_INPUT_1) == HIGH;
  }
}

void applyOutputs() {
  digitalWrite(PIN_COIL_0, coils[0] ? HIGH : LOW);
  if (!SERIAL_DEBUG_ENABLED) {
    digitalWrite(PIN_COIL_1, coils[1] ? HIGH : LOW);
  }

  analogWrite(PIN_PWM_AZIMUTH, hregs[0]);
  analogWrite(PIN_PWM_ELEVATION, hregs[1]);
  analogWrite(PIN_PWM_HEATER, hregs[2]);
}

// Aplica saidas apenas quando houve mudanca, evitando reescrever pinos
// (analogWrite/digitalWrite) a cada periodo de IO sem necessidade.
void applyOutputsIfChanged() {
  bool changed = !outputsInitialized;
  if (!changed) {
    for (uint8_t i = 0; i < COILS_COUNT; i++) {
      if (coils[i] != appliedCoils[i]) {
        changed = true;
        break;
      }
    }
  }
  if (!changed) {
    for (uint8_t i = 0; i < HREG_COUNT; i++) {
      if (hregs[i] != appliedHregs[i]) {
        changed = true;
        break;
      }
    }
  }
  if (!changed) {
    return;
  }

  applyOutputs();

  for (uint8_t i = 0; i < COILS_COUNT; i++) {
    appliedCoils[i] = coils[i];
  }
  for (uint8_t i = 0; i < HREG_COUNT; i++) {
    appliedHregs[i] = hregs[i];
  }
  outputsInitialized = true;
}

String htmlPage(const String &status, const String &message = "") {
  String checkedN = config.parity == 'N' ? "selected" : "";
  String checkedE = config.parity == 'E' ? "selected" : "";
  String checkedO = config.parity == 'O' ? "selected" : "";
  String selectedModbusTcp = config.protocol == ActiveProtocol::MODBUS_TCP ? "selected" : "";
  String selectedModbusRtu = config.protocol == ActiveProtocol::MODBUS_RTU ? "selected" : "";
  String selectedMqtt = config.protocol == ActiveProtocol::MQTT ? "selected" : "";
  String selectedOpcUa = config.protocol == ActiveProtocol::OPCUA ? "selected" : "";
  String selectedMqttGeral = config.mqttMode != "thingsboard" ? "selected" : "";
  String selectedMqttThingsboard = isThingsboardMqtt() ? "selected" : "";

  String html;
  html.reserve(12800);
  html += "<!DOCTYPE html><html><head><meta name='viewport' content='width=device-width,initial-scale=1'>";
  html += "<meta http-equiv='Cache-Control' content='no-cache, no-store, must-revalidate'>";
  html += "<meta http-equiv='Pragma' content='no-cache'>";
  html += "<meta http-equiv='Expires' content='0'>";
  html += "<title>Modulo Didatico - NodeMCU Setup</title>";
  html += "<style>";
  html += ":root{--bg:#0b1116;--panel:#101922;--accent:#49f2b8;--warn:#f4c96b;--err:#ff6d6d;--text:#eaf3ff;--muted:#8aa1b5;}";
  html += "body{margin:0;font-family:'Trebuchet MS',Verdana,sans-serif;background:radial-gradient(circle at 20% 10%,#152433,transparent 45%),linear-gradient(135deg,#060a0e,#101a24);color:var(--text);}";
  html += ".wrap{max-width:860px;margin:20px auto;padding:18px;}";
  html += ".title{font-size:1.7rem;letter-spacing:1px;text-transform:uppercase;margin:0 0 6px 0;}";
  html += ".sub{color:var(--muted);margin-bottom:18px;}";
  html += ".grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(220px,1fr));gap:12px;}";
  html += ".panel{background:linear-gradient(165deg,rgba(255,255,255,.03),rgba(255,255,255,.01));border:1px solid rgba(73,242,184,.25);padding:14px;border-radius:10px;backdrop-filter: blur(2px);} ";
  html += "label{font-size:.75rem;color:var(--muted);display:block;margin-bottom:4px;text-transform:uppercase;letter-spacing:.6px;}";
  html += "input,select{width:100%;box-sizing:border-box;background:#0b141e;color:var(--text);border:1px solid #1b3043;padding:10px;border-radius:8px;outline:none;}";
  html += "input:focus,select:focus{border-color:var(--accent);box-shadow:0 0 0 2px rgba(73,242,184,.15);} ";
  html += ".tip{display:inline-block;width:15px;height:15px;line-height:15px;text-align:center;border-radius:50%;border:1px solid var(--muted);color:var(--muted);font-size:.55rem;font-weight:700;margin-left:4px;vertical-align:top;cursor:help;} ";
  html += "button{margin-top:12px;background:linear-gradient(90deg,#2bd69f,#1ea8d8);border:none;color:#031018;font-weight:700;padding:11px 18px;border-radius:8px;cursor:pointer;}";
  html += ".status{margin:10px 0;padding:10px;border-left:4px solid var(--accent);background:rgba(73,242,184,.08);} ";
  html += ".error{border-left-color:var(--err);background:rgba(255,109,109,.12);} ";
  html += "details{margin-top:24px;background:linear-gradient(165deg,rgba(255,255,255,.03),rgba(255,255,255,.01));border:1px solid rgba(73,242,184,.25);border-radius:10px;padding:4px 14px 14px;backdrop-filter: blur(2px);}";
  html += "summary{cursor:pointer;font-weight:700;padding:12px 0;color:var(--accent);font-size:.95rem;text-transform:uppercase;letter-spacing:.6px;}";
  html += "table{width:100%;border-collapse:collapse;margin:8px 0;font-size:.82rem;}";
  html += "th,td{border:1px solid rgba(73,242,184,.15);padding:6px 10px;text-align:left;}";
  html += "th{background:rgba(73,242,184,.08);color:var(--accent);font-size:.72rem;text-transform:uppercase;letter-spacing:.5px;}";
  html += "td{color:var(--text);}";
  html += ".hl{color:var(--accent);font-family:'Courier New',monospace;font-size:.85rem;}";
  html += ".sect{margin:16px 0 6px;color:var(--accent);font-weight:700;font-size:.85rem;text-transform:uppercase;}";
  html += "@media (max-width:700px){.wrap{padding:12px}.title{font-size:1.3rem}}";
  html += "</style></head><body><div class='wrap'>";
  html += "<h1 class='title'>Modulo Didatico - NodeMCU Setup Panel</h1>";
  html += "<div class='sub'>Estado atual: " + status + " | AP: " + apSsid + " | Protocolo salvo: " + String(protocolToCString(config.protocol)) + "</div>";
  if (!message.isEmpty()) {
    html += "<div class='status'>" + message + "</div>";
  }
  html += "<form method='POST' action='/config'><div class='grid'>";
  html += "<div class='panel'><label title='Escolha o protocolo de comunicacao que o modulo usara para trocar dados com o SCADA. Modbus TCP usa Ethernet (porta 502 por padrao, configuravel), Modbus RTU usa RS485, MQTT e leve para IoT, OPC UA PubSub e o padrao moderno da Industria 4.0.'>Protocolo Ativo <span class='tip'>?</span></label><select name='protocol' required><option " + selectedModbusTcp + " value='modbustcp'>Modbus TCP</option><option " + selectedModbusRtu + " value='modbusrtu'>Modbus RTU</option><option " + selectedMqtt + " value='mqtt'>MQTT</option><option " + selectedOpcUa + " value='opcua'>OPC UA</option></select></div>";
  html += "<div class='panel'><label title='Nome da rede Wi-Fi (2.4 GHz) que o modulo deve conectar. Exatamente como aparece no roteador, respeitando maiusculas e minusculas.'>SSID <span class='tip'>?</span></label><input name='ssid' required value='" + config.ssid + "'>";
  html += "<label title='Senha da rede Wi-Fi. Minimo 8 caracteres. Fica salva no arquivo de configuracao do modulo.'>PSK <span class='tip'>?</span></label><input name='psk' required type='password' value='" + config.psk + "'></div>";
  html += "<div class='panel'><label title='Endereco IP fixo do modulo na rede. Deve ser unico na rede e pertencer a mesma faixa do roteador. Ex: 192.168.100.50'>IP Fixo <span class='tip'>?</span></label><input name='ip' required value='" + config.ip + "'>";
  html += "<label title='Mascara de rede. Quase sempre 255.255.255.0 para redes locais pequenas.'>Mascara <span class='tip'>?</span></label><input name='mask' required value='" + config.mask + "'>";
  html += "<label title='Endereco IP do roteador (gateway padrao). E por ele que o modulo acessa a internet e outros dispositivos.'>Gateway <span class='tip'>?</span></label><input name='gateway' required value='" + config.gateway + "'>";
  html += "<label title='Servidor DNS para resolver nomes como \"broker.local\". Pode ser o IP do roteador (geralmente igual ao gateway).'>DNS <span class='tip'>?</span></label><input name='dns' required value='" + config.dns + "'></div>";
  html += "<div class='panel'><label title='Identificador unico do modulo no barramento Modbus (1 a 247). Cada dispositivo na rede Modbus precisa de um ID diferente.'>Modbus ID <span class='tip'>?</span></label><input name='deviceId' type='number' min='1' max='247' value='" + String(config.deviceId) + "'>";
  html += "<label title='Porta TCP do servico Modbus TCP. A padrao e 502. Alterne apenas se o SCADA/IHM exigir outra porta.'>Modbus TCP Porta <span class='tip'>?</span></label><input name='modbusTcpPort' type='number' min='1' max='65535' value='" + String(config.modbusTcpPort) + "'>";
  html += "<label title='Velocidade de comunicacao serial em bits por segundo. Deve ser igual para todos os dispositivos no barramento RS485. 9600 e 19200 sao comuns.'>Baud <span class='tip'>?</span></label><input name='baud' type='number' value='" + String(config.baud) + "'>";
  html += "<label title='Bit de verificacao de erro na transmissao serial. N = nenhum, E = par (even), O = impar (odd). Deve ser igual em toda a rede RS485.'>Paridade <span class='tip'>?</span></label><select name='parity'><option " + checkedN + " value='N'>N</option><option " + checkedE + " value='E'>E</option><option " + checkedO + " value='O'>O</option></select>";
  html += "<label title='Bits de parada (stop bits) no final de cada byte serial. 1 e o padrao mais comum. Use 2 para maior tolerancia a ruido no barramento RS485.'>Stop Bits <span class='tip'>?</span></label><select name='stopBits'><option value='1'" + String(config.stopBits == 1 ? " selected" : "") + ">1</option><option value='2'" + String(config.stopBits == 2 ? " selected" : "") + ">2</option></select></div>";
  html += "<div class='panel'><label title='Escolha o modo MQTT. MQTT Geral usa um broker MQTT comum (Mosquitto, HiveMQ etc.) com topicos de publicacao e subscricao configuraveis. MQTT ThingsBoard conecta ao ThingsBoard: o topico de telemetria e fixo (v1/devices/me/telemetry), o usuario e o access token do dispositivo e nao ha senha.'>MQTT Modo <span class='tip'>?</span></label><select name='mqttMode' id='mqttMode' onchange='mqttModeChanged()'><option " + selectedMqttGeral + " value='geral'>MQTT Geral</option><option " + selectedMqttThingsboard + " value='thingsboard'>MQTT ThingsBoard</option></select>";
  html += "<label title='Endereco IP ou nome do servidor MQTT (broker). Ex: 192.168.100.200 ou broker.local. No ThingsBoard, use o IP ou dominio do servidor ThingsBoard.'>MQTT Broker <span class='tip'>?</span></label><input name='mqttBroker' value='" + config.mqttBroker + "'>";
  html += "<label title='Porta TCP do broker MQTT. A padrao e 1883. No ThingsBoard a porta padrao tambem e 1883 (porta 8883 e usada com TLS).'>MQTT Porta <span class='tip'>?</span></label><input name='mqttPort' type='number' min='1' max='65535' value='" + String(config.mqttPort) + "'>";
  html += "<label title='No modo Geral e o nome de usuario do broker (deixe vazio se nao exigir login). No modo ThingsBoard preencha com o ACCESS TOKEN do dispositivo.'>MQTT Usuario <span class='tip'>?</span></label><input name='mqttUser' value='" + config.mqttUser + "'>";
  html += "<div id='mqttGeralFields'>";
  html += "<label title='Senha do usuario MQTT. Usada apenas no modo Geral quando o broker exige autenticacao. No modo ThingsBoard nao ha senha.'>MQTT Senha <span class='tip'>?</span></label><input name='mqttPass' type='password' value='" + config.mqttPass + "'>";
  html += "<label title='Topico principal de publicacao do firmware (telemetria/entradas). E o topico usado no codigo para publicar as leituras. Ex: nodemcu/inputs ou sala_aula/telemetria. Se vazio, usa o prefixo configurado + /inputs.'>MQTT Topico Publicacao <span class='tip'>?</span></label><input name='mqttPublishTopic' placeholder='" + effectiveMqttPublishTopic() + "' value='" + config.mqttPublishTopic + "'>";
  html += "<label title='Topico de subscricao onde o firmware escuta comandos de saida. Ex: nodemcu/outputs/set ou sala_aula/comandos. Se vazio, usa o prefixo configurado + /outputs/set.'>MQTT Topico Subscricao <span class='tip'>?</span></label><input name='mqttSubscribeTopic' placeholder='" + effectiveMqttSubscribeTopic() + "' value='" + config.mqttSubscribeTopic + "'></div></div>";
  html += "<div class='panel'><label title='Identificador unico do publisher OPC UA no barramento. Usado pelo assinante para filtrar mensagens de origem especifica. Ex: NodeMCU_AABBCC.'>OPC UA Publisher ID <span class='tip'>?</span></label><input name='opcUaPublisherId' value='" + config.opcUaPublisherId + "'>";
  html += "<label title='Identificador do conjunto de dados (DataSet) que este modulo publica. Numeros diferentes permitem um assinante distinguir conjuntos de dados distintos do mesmo publisher.'>OPC UA DataSet Writer ID <span class='tip'>?</span></label><input name='opcUaDataSetWriterId' type='number' min='1' max='65535' value='" + String(config.opcUaDataSetWriterId) + "'>";
  html += "<label title='Prefixo usado pelos topicos OPC UA PubSub ({base}/opcua/inputs, {base}/opcua/outputs/set, {base}/opcua/status) e como fallback dos topicos MQTT quando estes ficam vazios. Ex: nodemcu.'>OPC UA Topico Base <span class='tip'>?</span></label><input name='mqttTopicBase' value='" + config.mqttTopicBase + "'>";
  html += "<div class='sub' style='margin-top:8px'>OPC UA PubSub via MQTT — reutiliza o mesmo broker.</div></div>";
  html += "</div><button type='submit'>Salvar e Conectar</button></form>";
  html += "<script>function mqttModeChanged(){var m=document.getElementById('mqttMode');var g=document.getElementById('mqttGeralFields');if(m&&g){g.style.display=(m.value==='thingsboard')?'none':'block';}}mqttModeChanged();</script>";

  // --- Instrucoes de uso ---
  html += "<details><summary>Instrucoes de Uso</summary>";

  // Mapeamento de IOs Fisicos
  html += "<div class='sect'>Mapeamento de IOs Fisicos</div>";
  html += "<table><tr><th>Funcao</th><th>Rotulo</th><th>GPIO</th><th>Descricao</th></tr>";
  html += "<tr><td>Coil 00001</td><td>D0</td><td>GPIO16</td><td>Saida binaria 0</td></tr>";
  html += "<tr><td>Coil 00002</td><td>TX</td><td>GPIO1</td><td>Saida binaria 1 (desativado se debug serial ativo)</td></tr>";
  html += "<tr><td>Entrada 10001</td><td>D8</td><td>GPIO15</td><td>Entrada binaria 0</td></tr>";
  html += "<tr><td>Entrada 10002</td><td>RX</td><td>GPIO3</td><td>Entrada binaria 1 (tambem gatilho de setup no boot, ativo em nivel baixo; desativado se debug serial ativo)</td></tr>";
  html += "<tr><td>ADC</td><td>A0</td><td>ADC</td><td>Entrada analogica (30001)</td></tr>";
  html += "<tr><td>DS18B20</td><td>D2</td><td>GPIO4</td><td>Temperatura (30002, valor x10)</td></tr>";
  html += "<tr><td>PWM Azimute</td><td>D7</td><td>GPIO13</td><td>Holding register 40001 (0-1023)</td></tr>";
  html += "<tr><td>PWM Elevacao</td><td>D3</td><td>GPIO0</td><td>Holding register 40002 (0-1023)</td></tr>";
  html += "<tr><td>PWM Resistencia de aquecimento</td><td>D1</td><td>GPIO5</td><td>Holding register 40003 (0-1023)</td></tr>";
  html += "<tr><td>RS485</td><td>D5/D6/D4</td><td>GPIO14/12/2</td><td>RX, TX, DIR (Modbus RTU)</td></tr>";
  html += "<tr><td>LED Config</td><td>D4</td><td>GPIO2</td><td>LED_BUILTIN pisca 1Hz no modo AP (compartilhado RS485 DIR)</td></tr>";
  html += "</table>";

  // Instrucoes Modbus
  html += "<div class='sect'>Acesso via Modbus TCP</div>";
  html += "<table><tr><th>Endereco</th><th>Tipo</th><th>Range</th><th>Descricao</th></tr>";
  html += "<tr><td>00001-00002</td><td>Coil (FC 01/05)</td><td>0/1</td><td>Saidas binarias</td></tr>";
  html += "<tr><td>10001-10002</td><td>Discrete Input (FC 02)</td><td>0/1</td><td>Entradas binarias (somente leitura)</td></tr>";
  html += "<tr><td>30001</td><td>Input Register (FC 04)</td><td>0-1023</td><td>ADC</td></tr>";
  html += "<tr><td>30002</td><td>Input Register (FC 04)</td><td>0-12500</td><td>DS18B20 temperatura (x10, graus Celsius)</td></tr>";
  html += "<tr><td>40001</td><td>Holding Register (FC 03/06)</td><td>0-1023</td><td>PWM Azimute</td></tr>";
  html += "<tr><td>40002</td><td>Holding Register (FC 03/06)</td><td>0-1023</td><td>PWM Elevacao</td></tr>";
  html += "<tr><td>40003</td><td>Holding Register (FC 03/06)</td><td>0-1023</td><td>PWM Resistencia de aquecimento</td></tr>";
  html += "</table>";
  html += "<p style='margin:4px 0;font-size:.82rem;color:var(--muted)'>Host: <span class='hl'>" + WiFi.localIP().toString() + "</span> | Porta: <span class='hl'>" + String(config.modbusTcpPort) + "</span> | Unit ID: <span class='hl'>" + String(config.deviceId) + "</span></p>";

  // Instrucoes MQTT
  {
    String mqttIn = effectiveMqttPublishTopic();
    String mqttOut = effectiveMqttSubscribeTopic();
    String mqttStat = effectiveMqttBase() + "/status";
    if (isThingsboardMqtt()) {
      mqttIn = "v1/devices/me/telemetry";
      mqttOut = "v1/devices/me/rpc/request/+";
      mqttStat = "v1/devices/me/attributes";
    }
    html += "<div class='sect'>Acesso via MQTT</div>";
    html += "<p style='margin:4px 0;font-size:.82rem;color:var(--muted)'>Modo: <span class='hl'>" + (isThingsboardMqtt() ? String("MQTT ThingsBoard") : String("MQTT Geral")) + "</span></p>";
    html += "<table><tr><th>Topico</th><th>Direcao</th><th>Formato</th></tr>";
    html += "<tr><td><span class='hl'>" + mqttIn + "</span></td><td>Publica (leitura)</td><td>JSON: { discrete_inputs: [...], input_registers: [...], ts_ms, ip }</td></tr>";
    html += "<tr><td><span class='hl'>" + mqttOut + "</span></td><td>Subscribe (escrita)</td><td>JSON: { coils: [0/1, 0/1], holding_registers: [0-1023, ...] }</td></tr>";
    html += "<tr><td><span class='hl'>" + mqttStat + "</span></td><td>Publica (status)</td><td>Payload: \"online\" (LWT / retained)</td></tr>";
    html += "</table>";
    html += "<p style='margin:4px 0;font-size:.82rem;color:var(--muted)'>Broker: <span class='hl'>" + config.mqttBroker + ":" + String(config.mqttPort) + "</span>";
    if (isThingsboardMqtt()) {
      html += " | Usuario = Access Token (sem senha)";
    }
    html += "</p>";
  }

  // Instrucoes OPC UA PubSub
  {
    String opcBase = config.mqttTopicBase;
    if (opcBase.isEmpty()) { opcBase = "nodemcu/" + String(ESP.getChipId(), HEX); }
    String opcIn = opcBase + "/opcua/inputs";
    String opcOut = opcBase + "/opcua/outputs/set";
    html += "<div class='sect'>Acesso via OPC UA PubSub (sobre MQTT)</div>";
    html += "<table><tr><th>Topico</th><th>Direcao</th><th>Formato</th></tr>";
    html += "<tr><td><span class='hl'>" + opcIn + "</span></td><td>Publica (leitura)</td><td>JSON: { MessageId, MessageType, PublisherId, DataSetWriterId, Payload: { Coil_0..1, DiscreteInput_0..1, ADC, DS18B20, PWM_* } }</td></tr>";
    html += "<tr><td><span class='hl'>" + opcOut + "</span></td><td>Subscribe (escrita)</td><td>JSON: { MessageId, MessageType, PublisherId, DataSetWriterId, Payload: { Coil_0..1, PWM_Azimuth, PWM_Elevation, PWM_Heater } }</td></tr>";
    html += "</table>";
    html += "<p style='margin:4px 0;font-size:.82rem;color:var(--muted)'>Broker: <span class='hl'>" + config.mqttBroker + ":" + String(config.mqttPort) + "</span> | Publisher ID: <span class='hl'>" + config.opcUaPublisherId + "</span></p>";
  }

  html += "</details>";

  html += "<footer style='margin-top:32px;padding:16px 0;border-top:1px solid rgba(73,242,184,.25);font-size:.8rem;color:var(--muted);text-align:center;'>";
  html += "Tecnologias: ESP8266, Modbus TCP/RTU, MQTT, OPC UA PubSub, Node-RED</br>";
  html += "Projetado por William Vianna</footer>";
  html += "</div></body></html>";
  return html;
}

void setNoCacheHeaders() {
  webServer.sendHeader("Cache-Control", "no-store, no-cache, must-revalidate, max-age=0");
  webServer.sendHeader("Pragma", "no-cache");
  webServer.sendHeader("Expires", "0");
}

void handleRoot() {
  setNoCacheHeaders();
  webServer.send(200, "text/html", htmlPage(getStateName()));
}

void handleHealth() {
  String payload = "{\"state\":\"" + getStateName() + "\",\"protocol\":\"" + String(protocolToCString(config.protocol)) + "\",\"ip\":\"" + WiFi.localIP().toString() + "\",\"ap\":\"" + WiFi.softAPIP().toString() + "\",\"opcuaRuntimeSupported\":" + String(opcUaRuntimeSupported ? "true" : "false") + "}";
  webServer.send(200, "application/json", payload);
}

void handleConfigPost() {
  DeviceConfig candidate = config;
  candidate.ssid = webServer.arg("ssid");
  candidate.psk = webServer.arg("psk");
  candidate.ip = webServer.arg("ip");
  candidate.mask = webServer.arg("mask");
  candidate.gateway = webServer.arg("gateway");
  candidate.dns = webServer.arg("dns");
  candidate.deviceId = static_cast<uint8_t>(webServer.arg("deviceId").toInt());
  String modbusTcpPortArg = webServer.arg("modbusTcpPort");
  if (!modbusTcpPortArg.isEmpty()) {
    uint16_t modbusTcpPortParsed = 0;
    if (!parseUInt16Strict(modbusTcpPortArg, modbusTcpPortParsed)) {
      webServer.send(400, "text/html", htmlPage(getStateName(), "Erro: Porta Modbus TCP invalida."));
      return;
    }
    candidate.modbusTcpPort = modbusTcpPortParsed;
  }
  candidate.baud = static_cast<uint32_t>(webServer.arg("baud").toInt());
  String parity = webServer.arg("parity");
  candidate.parity = parity.isEmpty() ? 'N' : parity[0];
  candidate.stopBits = static_cast<uint8_t>(webServer.arg("stopBits").toInt());
  candidate.protocol = protocolFromString(webServer.arg("protocol"));
  candidate.mqttBroker = webServer.arg("mqttBroker");
  String mqttPortArg = webServer.arg("mqttPort");
  if (!mqttPortArg.isEmpty()) {
    uint16_t mqttPortParsed = 0;
    if (!parseUInt16Strict(mqttPortArg, mqttPortParsed)) {
      webServer.send(400, "text/html", htmlPage(getStateName(), "Erro: Porta MQTT invalida."));
      return;
    }
    candidate.mqttPort = mqttPortParsed;
  }
  candidate.mqttUser = webServer.arg("mqttUser");
  candidate.mqttPass = webServer.arg("mqttPass");
  candidate.mqttTopicBase = webServer.arg("mqttTopicBase");
  candidate.mqttMode = webServer.arg("mqttMode");
  candidate.mqttPublishTopic = webServer.arg("mqttPublishTopic");
  candidate.mqttSubscribeTopic = webServer.arg("mqttSubscribeTopic");
  candidate.opcUaPublisherId = webServer.arg("opcUaPublisherId");
  String opcUaDsIdArg = webServer.arg("opcUaDataSetWriterId");
  if (!opcUaDsIdArg.isEmpty()) {
    uint16_t opcUaDsIdParsed = 0;
    if (!parseUInt16Strict(opcUaDsIdArg, opcUaDsIdParsed)) {
      webServer.send(400, "text/html", htmlPage(getStateName(), "Erro: OPC UA DataSet Writer ID invalido."));
      return;
    }
    candidate.opcUaDataSetWriterId = opcUaDsIdParsed;
  }

  sanitizeProtocolConfig(candidate);

  String error;
  if (!validateConfig(candidate, error)) {
    setNoCacheHeaders();
    webServer.send(400, "text/html", htmlPage(getStateName(), "Erro: " + error));
    return;
  }

  if (!saveConfig(candidate)) {
    setNoCacheHeaders();
    webServer.send(500, "text/html", htmlPage(getStateName(), "Erro ao persistir configuracao."));
    return;
  }

  config = candidate;
  pendingConnectAfterSave = true;
  apSaveAtMs = millis();
  setNoCacheHeaders();
  webServer.send(200, "text/html", htmlPage(getStateName(), "Configuracao salva. Iniciando conexao Wi-Fi..."));
}

void configureWebServer() {
  webServer.on("/", HTTP_GET, handleRoot);
  webServer.on("/health", HTTP_GET, handleHealth);
  webServer.on("/config", HTTP_POST, handleConfigPost);
  webServer.begin();
}

void stopProtocolServices() {
  mqttClient.disconnect();
  modbusTcpClient.stop();
  if (modbusTcpServer != nullptr) {
    modbusTcpServer->close();
    delete modbusTcpServer;
    modbusTcpServer = nullptr;
  }
}

void startProtocolServices() {
  stopProtocolServices();

  if (config.protocol == ActiveProtocol::MODBUS_TCP) {
    modbusTcpServer = new WiFiServer(config.modbusTcpPort);
    modbusTcpServer->begin();
    return;
  }

  if (config.protocol == ActiveProtocol::MODBUS_RTU) {
    configureRtu();
    syncModelToRtu();
    return;
  }

  if (config.protocol == ActiveProtocol::MQTT) {
    buildMqttTopics();
    mqttClient.setServer(config.mqttBroker.c_str(), config.mqttPort);
    mqttClient.setCallback(mqttCallback);
    lastMqttReconnectMs = 0;
    lastMqttPublishMs = 0;
    return;
  }

  if (config.protocol == ActiveProtocol::OPCUA) {
    buildOpcUaTopics();
    mqttClient.setBufferSize(512);
    mqttClient.setServer(config.mqttBroker.c_str(), config.mqttPort);
    mqttClient.setCallback(opcUaMqttCallback);
    lastMqttReconnectMs = 0;
    lastMqttPublishMs = 0;
    return;
  }
}

void startApMode(DeviceState newState) {
  stopProtocolServices();
  WiFi.disconnect(true);
  WiFi.mode(WIFI_AP);

  IPAddress apIp(192, 168, 4, 1);
  IPAddress apGateway(192, 168, 4, 1);
  IPAddress apMask(255, 255, 255, 0);
  WiFi.softAPConfig(apIp, apGateway, apMask);
  apSsid = String("NodeMCU_Setup_") + String(ESP.getChipId(), HEX);
  WiFi.softAP(apSsid.c_str());

  state = newState;
  stateStartedAt = millis();
  pendingConnectAfterSave = false;

  // Garante LED comecando apagado e timer zerado para blink 1Hz
  digitalWrite(PIN_CONFIG_LED, HIGH);
  lastConfigLedBlinkMs = 0;
}

bool startWifiClient() {
  String error;
  if (!validateConfig(config, error)) {
    return false;
  }

  IPAddress ip;
  IPAddress mask;
  IPAddress gateway;
  IPAddress dns;
  if (!parseIPv4(config.ip, ip) || !parseIPv4(config.mask, mask) ||
      !parseIPv4(config.gateway, gateway) || !parseIPv4(config.dns, dns)) {
    return false;
  }

  WiFi.mode(WIFI_STA);
  WiFi.persistent(false);
  WiFi.disconnect();
  WiFi.config(ip, gateway, mask, dns);
  WiFi.begin(config.ssid.c_str(), config.psk.c_str());

  state = DeviceState::WIFI_CONNECTING;
  stateStartedAt = millis();
  return true;
}

void handleStateMachine() {
  unsigned long now = millis();

  if ((state == DeviceState::AP_SETUP || state == DeviceState::FAILSAFE_AP) && pendingConnectAfterSave) {
    if (now - apSaveAtMs >= AP_RETRY_DELAY_MS) {
      pendingConnectAfterSave = false;
      if (!startWifiClient()) {
        startApMode(DeviceState::FAILSAFE_AP);
      }
    }
  }

  if (state == DeviceState::WIFI_CONNECTING) {
    wl_status_t status = WiFi.status();
    if (status == WL_CONNECTED) {
      startProtocolServices();
      logStationIp();
      state = DeviceState::RUN_PROTOCOL;
      stateStartedAt = now;
      return;
    }

    if (now - stateStartedAt > WIFI_CONNECT_TIMEOUT_MS) {
      startApMode(DeviceState::FAILSAFE_AP);
    }
    return;
  }

  if (state == DeviceState::RUN_PROTOCOL && WiFi.status() != WL_CONNECTED) {
    startApMode(DeviceState::FAILSAFE_AP);
  }
}

bool handleReadBits(uint8_t fc, uint16_t start, uint16_t qty, uint8_t *response, uint16_t &respLen) {
  if (qty < 1 || qty > 2000) {
    response[0] = fc | 0x80;
    response[1] = 0x03;
    respLen = 2;
    return true;
  }

  uint16_t maxCount = (fc == 0x01) ? COILS_COUNT : ISTS_COUNT;
  if (start + qty > maxCount) {
    response[0] = fc | 0x80;
    response[1] = 0x02;
    respLen = 2;
    return true;
  }

  uint8_t byteCount = (qty + 7) / 8;
  response[0] = fc;
  response[1] = byteCount;
  memset(response + 2, 0, byteCount);

  for (uint16_t i = 0; i < qty; i++) {
    bool bit = false;
    if (fc == 0x01) {
      bit = coils[start + i];
    } else {
      bit = ists[start + i];
    }
    if (bit) {
      response[2 + (i / 8)] |= (1U << (i % 8));
    }
  }

  respLen = 2 + byteCount;
  return true;
}

bool handleReadRegs(uint8_t fc, uint16_t start, uint16_t qty, uint8_t *response, uint16_t &respLen) {
  if (qty < 1 || qty > 125) {
    response[0] = fc | 0x80;
    response[1] = 0x03;
    respLen = 2;
    return true;
  }

  uint16_t maxCount = (fc == 0x03) ? HREG_COUNT : IREG_COUNT;
  if (start + qty > maxCount) {
    response[0] = fc | 0x80;
    response[1] = 0x02;
    respLen = 2;
    return true;
  }

  response[0] = fc;
  response[1] = qty * 2;
  for (uint16_t i = 0; i < qty; i++) {
    uint16_t value = (fc == 0x03) ? hregs[start + i] : iregs[start + i];
    response[2 + i * 2] = value >> 8;
    response[3 + i * 2] = value & 0xFF;
  }

  respLen = 2 + qty * 2;
  return true;
}

void handleWriteSingleCoil(uint16_t addr, uint16_t value, uint8_t *request, uint8_t *response, uint16_t &respLen) {
  if (addr >= COILS_COUNT) {
    response[0] = 0x85;
    response[1] = 0x02;
    respLen = 2;
    return;
  }

  if (!(value == 0xFF00 || value == 0x0000)) {
    response[0] = 0x85;
    response[1] = 0x03;
    respLen = 2;
    return;
  }

  coils[addr] = (value == 0xFF00);
  memcpy(response, request, 5);
  respLen = 5;
}

void handleWriteSingleReg(uint16_t addr, uint16_t value, uint8_t *request, uint8_t *response, uint16_t &respLen) {
  if (addr >= HREG_COUNT) {
    response[0] = 0x86;
    response[1] = 0x02;
    respLen = 2;
    return;
  }

  if (value > 1023) {
    response[0] = 0x86;
    response[1] = 0x03;
    respLen = 2;
    return;
  }

  hregs[addr] = value;
  memcpy(response, request, 5);
  respLen = 5;
}

bool handleModbusTcpPdu(uint8_t *request, uint16_t reqLen, uint8_t *response, uint16_t &respLen) {
  if (reqLen < 5) {
    return false;
  }

  uint8_t fc = request[0];
  uint16_t addr = (static_cast<uint16_t>(request[1]) << 8) | request[2];
  uint16_t val = (static_cast<uint16_t>(request[3]) << 8) | request[4];

  switch (fc) {
    case 0x01:
    case 0x02:
      return handleReadBits(fc, addr, val, response, respLen);
    case 0x03:
    case 0x04:
      return handleReadRegs(fc, addr, val, response, respLen);
    case 0x05:
      handleWriteSingleCoil(addr, val, request, response, respLen);
      return true;
    case 0x06:
      handleWriteSingleReg(addr, val, request, response, respLen);
      return true;
    default:
      response[0] = fc | 0x80;
      response[1] = 0x01;
      respLen = 2;
      return true;
  }
}

bool readTcpExact(uint8_t *buffer, size_t length, uint32_t timeoutMs) {
  size_t offset = 0;
  unsigned long lastProgressMs = millis();

  while (offset < length) {
    if (!modbusTcpClient || !modbusTcpClient.connected()) {
      return false;
    }

    int chunk = modbusTcpClient.read(buffer + offset, length - offset);
    if (chunk > 0) {
      offset += static_cast<size_t>(chunk);
      lastProgressMs = millis();
      continue;
    }

    if (millis() - lastProgressMs >= timeoutMs) {
      return false;
    }

    ESP.wdtFeed();
    delay(1);
  }

  return true;
}

void serviceModbusTcp() {
  if (modbusTcpServer == nullptr) {
    return;
  }

  if (!modbusTcpClient || !modbusTcpClient.connected()) {
    modbusTcpClient = modbusTcpServer->accept();
    return;
  }

  while (modbusTcpClient.available() >= 7) {
    uint8_t mbap[7];
    if (!readTcpExact(mbap, sizeof(mbap), MODBUS_TCP_READ_TIMEOUT_MS)) {
      modbusTcpClient.stop();
      return;
    }

    uint16_t protocol = (static_cast<uint16_t>(mbap[2]) << 8) | mbap[3];
    uint16_t length = (static_cast<uint16_t>(mbap[4]) << 8) | mbap[5];
    if (protocol != 0 || length < 2 || length > 254) {
      return;
    }

    uint16_t pduLen = length - 1;
    uint8_t reqPdu[253] = {0};
    if (!readTcpExact(reqPdu, pduLen, MODBUS_TCP_READ_TIMEOUT_MS)) {
      modbusTcpClient.stop();
      return;
    }

    uint8_t respPdu[253] = {0};
    uint16_t respLen = 0;
    if (!handleModbusTcpPdu(reqPdu, pduLen, respPdu, respLen)) {
      return;
    }

    uint8_t outMbap[7];
    memcpy(outMbap, mbap, 7);
    outMbap[4] = ((respLen + 1) >> 8) & 0xFF;
    outMbap[5] = (respLen + 1) & 0xFF;
    outMbap[6] = mbap[6];

    // Envia MBAP+PDU em um unico write para evitar clientes que falham
    // quando a resposta chega fragmentada em multiplos segmentos TCP.
    uint8_t outFrame[260] = {0};
    memcpy(outFrame, outMbap, 7);
    memcpy(outFrame + 7, respPdu, respLen);
    modbusTcpClient.write(outFrame, 7 + respLen);
  }
}

void setupPins() {
  pinMode(PIN_COIL_0, OUTPUT);
  if (!SERIAL_DEBUG_ENABLED) {
    pinMode(PIN_COIL_1, OUTPUT);
  }


  pinMode(PIN_INPUT_0, INPUT);
  if (!SERIAL_DEBUG_ENABLED) {
    pinMode(PIN_INPUT_1, INPUT);
  }

  pinMode(PIN_PWM_AZIMUTH, OUTPUT);
  pinMode(PIN_PWM_ELEVATION, OUTPUT);
  pinMode(PIN_PWM_HEATER, OUTPUT);

  pinMode(PIN_CONFIG_LED, OUTPUT);
  digitalWrite(PIN_CONFIG_LED, HIGH); // Apagado (ativo LOW)
}

bool shouldForceApByBootPin() {
  // RX/GPIO3 nao e bootstrap; com INPUT_PULLUP, nivel baixo no boot (switch para GND) forca AP.
  // Requer SERIAL_DEBUG_ENABLED=false (RX livre); se debug serial ativo, o pino pertence a UART.
  pinMode(PIN_AP_TRIGGER, INPUT_PULLUP);
  delay(2);
  return digitalRead(PIN_AP_TRIGGER) == LOW;
}

}  // namespace

void setup() {
  Serial.begin(115200);
  delay(80);
  logSerialDebugMode();

  // Reinicia automaticamente o MCU se o loop principal travar por mais de 4s.
  ESP.wdtEnable(WATCHDOG_TIMEOUT_MS);

  const bool forceAp = shouldForceApByBootPin();

  setupPins();
  analogWriteRange(1023);

  setDefaultConfig(config);

  if (!LittleFS.begin()) {
    LittleFS.format();
    LittleFS.begin();
  }

  loadConfig(config);

  ds18b20.begin();
  ds18b20.setWaitForConversion(false);
  ds18b20AddressFound = initDs18b20Address();
  // Garante tempo minimo de conversao do DS18B20 (>= 500 ms) antes da leitura,
  // evitando leituras prematuras/instaveis que geram 0 periodico no barramento.
  dsConversionWaitMs = ds18b20.millisToWaitForConversion();
  if (dsConversionWaitMs < DS18B20_MIN_CONVERSION_WAIT_MS) {
    dsConversionWaitMs = DS18B20_MIN_CONVERSION_WAIT_MS;
  }
  // Dispara a primeira conversao no boot: a leitura so ocorre apos
  // dsConversionWaitMs, entao o primeiro valor ja e valido.
  if (ds18b20AddressFound) {
    ds18b20.requestTemperatures();
  }
  lastDsRequestMs = millis();

  configureWebServer();
  initModbusRegisters();
  syncModelToRtu();

  if (forceAp) {
    startApMode(DeviceState::AP_SETUP);
  } else {
    if (!startWifiClient()) {
      startApMode(DeviceState::AP_SETUP);
    }
  }
}

void loop() {
  ESP.wdtFeed();
  blinkConfigLed();

  unsigned long now = millis();

  handleStateMachine();
  webServer.handleClient();

  if (now - lastSensorMs >= SENSOR_PERIOD_MS) {
    lastSensorMs = now;
    updateSensorRegisters();
    updateDiscreteInputs();
    if (isModbusRtuActive()) {
      syncModelInputsToRtu();
    }
  }

  if (state == DeviceState::RUN_PROTOCOL) {
    if (isModbusTcpActive()) {
      serviceModbusTcp();
    } else if (isModbusRtuActive()) {
      modbusRtu.task();
    } else if (config.protocol == ActiveProtocol::MQTT) {
      serviceMqtt();
    } else if (config.protocol == ActiveProtocol::OPCUA) {
      serviceOpcUa();
    }
  }

  if (isModbusRtuActive() && now - lastModbusSyncMs >= MODBUS_SYNC_PERIOD_MS) {
    lastModbusSyncMs = now;
    syncRtuOutputsToModel();
  }

  if (now - lastIoMs >= IO_PERIOD_MS) {
    lastIoMs = now;
    applyOutputsIfChanged();
  }
}