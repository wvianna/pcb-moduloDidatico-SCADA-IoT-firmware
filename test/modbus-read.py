#!/usr/bin/env python3
"""Leitura Modbus TCP dos registradores do modulo didatico (somente leitura).

Le os quatro tipos de dados do firmware e imprime em formato de tabela:
  - Coils             (FC 0x01) -> base 1: 00001..00002 (saidas binarias)
  - Discrete inputs   (FC 0x02) -> base 1: 10001..10002 (entradas binarias)
  - Input registers   (FC 0x04) -> base 1: 30001..30002 (ADC e DS18B20)
  - Holding registers (FC 0x03) -> base 1: 40001..40003 (PWM azimute/elevacao/resistencia)

Uso:
    python test/modbus-read.py [HOST] [PORT] [UNIT_ID]

Exemplos:
    python test/modbus-read.py
    python test/modbus-read.py 192.168.100.204 502 1

Dependencia:
    pip install pymodbus
"""
from __future__ import annotations

import sys

from pymodbus.client import ModbusTcpClient

DEFAULT_HOST = "192.168.100.204"
DEFAULT_PORT = 502
DEFAULT_UNIT = 1

# Quantidade de itens de cada tipo exposto pelo firmware.
COILS = 2        # base 1: 00001..00002
DISCRETE = 2     # base 1: 10001..10002
INPUT_REGS = 2   # base 1: 30001..30002
HOLDING_REGS = 3 # base 1: 40001..40003


def main() -> None:
    host = sys.argv[1] if len(sys.argv) > 1 else DEFAULT_HOST
    port = int(sys.argv[2]) if len(sys.argv) > 2 else DEFAULT_PORT
    unit = int(sys.argv[3]) if len(sys.argv) > 3 else DEFAULT_UNIT

    print(f"Conectando em Modbus TCP {host}:{port} (unit_id={unit}) ...")
    client = ModbusTcpClient(host, port=port, timeout=3, retries=1)
    if not client.connect():
        print(
            "ERRO: nao foi possivel conectar. Verifique IP/porta, a rede e se o "
            "firmware esta com o protocolo Modbus ativo no setup."
        )
        sys.exit(1)
    print("Conectado!\n")

    try:
        # --- Coils (FC 0x01) ---
        resp = client.read_coils(0, count=COILS, device_id=unit)
        if resp.isError():
            print(f"ERRO leitura coils: {resp}\n")
        else:
            print("Coils (FC 0x01):")
            for i in range(COILS):
                bit = resp.bits[i] if i < len(resp.bits) else None
                print(f"  {i + 1:>5} (coil {i}) -> {'1' if bit else '0'}")
            print()

        # --- Discrete inputs (FC 0x02) ---
        resp = client.read_discrete_inputs(0, count=DISCRETE, device_id=unit)
        if resp.isError():
            print(f"ERRO leitura discrete inputs: {resp}\n")
        else:
            print("Discrete inputs (FC 0x02):")
            for i in range(DISCRETE):
                bit = resp.bits[i] if i < len(resp.bits) else None
                print(f"  {i + 1:>5} (discrete {i}) -> {'1' if bit else '0'}")
            print()

        # --- Input registers (FC 0x04) ---
        resp = client.read_input_registers(0, count=INPUT_REGS, device_id=unit)
        if resp.isError():
            print(f"ERRO leitura input registers: {resp}\n")
        else:
            regs = resp.registers
            adc = regs[0] if len(regs) > 0 else None
            print("Input registers (FC 0x04):")
            print(f"  {30001:>5} (input reg 0 / ADC)     -> {adc}")
            if len(regs) > 1:
                temp = regs[1]
                print(f"  {30002:>5} (input reg 1 / DS18B20) -> {temp} ({temp / 10.0:.1f} C)")
            print()

        # --- Holding registers (FC 0x03) ---
        resp = client.read_holding_registers(0, count=HOLDING_REGS, device_id=unit)
        if resp.isError():
            print(f"ERRO leitura holding registers: {resp}\n")
        else:
            regs = resp.registers
            print("Holding registers (FC 0x03):")
            for i in range(HOLDING_REGS):
                val = regs[i] if i < len(regs) else None
                print(f"  {i + 1:>5} (holding {i}) -> {val}")
            print()
    finally:
        client.close()


if __name__ == "__main__":
    main()
