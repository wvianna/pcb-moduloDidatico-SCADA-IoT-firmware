#!/usr/bin/env python3
"""Monitora a entrada analogica (ADC / input register 30001 / pino A0) via Modbus TCP.

Le o registrador de entrada 30001 (base 0: 0) a cada 2 segundos e imprime o valor
com timestamp. Roda indefinidamente ate Ctrl+C, ou por N amostras.

Uso:
    python test/modbus-analog-monitor.py [HOST] [PORT] [UNIT_ID] [SAMPLES]

Exemplos:
    python test/modbus-analog-monitor.py
    python test/modbus-analog-monitor.py 192.168.100.204 502 1 10
    (SAMPLES=0 ou omitido -> roda ate Ctrl+C)

Dependencia:
    pip install pymodbus
"""
from __future__ import annotations

import sys
import time
from datetime import datetime

from pymodbus.client import ModbusTcpClient

DEFAULT_HOST = "192.168.100.204"
DEFAULT_PORT = 502
DEFAULT_UNIT = 1
DEFAULT_SAMPLES = 0  # 0 = indefinido (Ctrl+C para parar)
INTERVAL_S = 2.0
ADC_ADDR = 0  # base 1: 30001 -> base 0: 0


def main() -> None:
    host = sys.argv[1] if len(sys.argv) > 1 else DEFAULT_HOST
    port = int(sys.argv[2]) if len(sys.argv) > 2 else DEFAULT_PORT
    unit = int(sys.argv[3]) if len(sys.argv) > 3 else DEFAULT_UNIT
    samples = int(sys.argv[4]) if len(sys.argv) > 4 else DEFAULT_SAMPLES

    print(f"Conectando em Modbus TCP {host}:{port} (unit_id={unit}) ...")
    client = ModbusTcpClient(host, port=port, timeout=3, retries=1)
    if not client.connect():
        print(
            "ERRO: nao foi possivel conectar. Verifique IP/porta, a rede e se o "
            "firmware esta com o protocolo Modbus ativo no setup."
        )
        sys.exit(1)
    mode = f"{samples} amostras." if samples > 0 else "Ctrl+C para parar."
    print(f"Conectado! Monitorando ADC (30001) a cada {INTERVAL_S:.0f} s. {mode}\n")

    try:
        n = 0
        while samples == 0 or n < samples:
            resp = client.read_input_registers(ADC_ADDR, count=1, device_id=unit)
            if resp.isError():
                print(f"[{datetime.now().strftime('%H:%M:%S')}] ERRO: {resp}")
            else:
                value = resp.registers[0]
                print(f"[{datetime.now().strftime('%H:%M:%S')}] ADC (30001) = {value}")
            n += 1
            if samples == 0 or n < samples:
                time.sleep(INTERVAL_S)
    except KeyboardInterrupt:
        print("\nInterrompido pelo usuario.")
    finally:
        client.close()


if __name__ == "__main__":
    main()
