#!/usr/bin/env python3
"""Teste Modbus TCP das saidas do modulo didatico (escrita).

- Coils (FC 0x05): piscam 3 vezes, cada ciclo com intervalo de 2 s (ON 2 s -> OFF 2 s).
- Holding registers (FC 0x06): PWM varia de 0 a 100% em passos de 25% (0, 25, 50, 75, 100),
  intervalo de 2 s, e retorna a 0% ao final (estado seguro).

Uso:
    python test/modbus-outputs.py [HOST] [PORT] [UNIT_ID] [--coils-only]

Exemplos:
    python test/modbus-outputs.py
    python test/modbus-outputs.py 192.168.100.204 502 1
    python test/modbus-outputs.py 192.168.100.204 502 1 --coils-only  # somente coils

Dependencia:
    pip install pymodbus

ATENCAO: este teste ACIONA as saidas fisicas (coils -> reles/atuadores; holding
registers -> PWM de motores/resistencia). O holding 40003 controla a resistencia
de aquecimento; o teste e curto (2 s por passo) e ao final todas as saidas voltam a 0.
"""
from __future__ import annotations

import sys
import time

from pymodbus.client import ModbusTcpClient

DEFAULT_HOST = "192.168.100.204"
DEFAULT_PORT = 502
DEFAULT_UNIT = 1

COILS = 2        # 00001..00002
HOLDING_REGS = 3 # 40001..40003
PWM_MAX = 1023
INTERVAL_S = 2.0
BLINKS = 3
PERCENT_STEPS = (0, 25, 50, 75, 100)


def pwm_value(percent: int) -> int:
    """Converte percentual (0..100) para o range PWM do firmware (0..1023)."""
    return round(PWM_MAX * percent / 100)


def main() -> None:
    args = [a for a in sys.argv[1:] if a != "--coils-only"]
    coils_only = "--coils-only" in sys.argv

    host = args[0] if len(args) > 0 else DEFAULT_HOST
    port = int(args[1]) if len(args) > 1 else DEFAULT_PORT
    unit = int(args[2]) if len(args) > 2 else DEFAULT_UNIT

    scope = "coils" if coils_only else "coils + PWM"
    print(f"Conectando em Modbus TCP {host}:{port} (unit_id={unit}) | escopo: {scope} ...")
    client = ModbusTcpClient(host, port=port, timeout=3, retries=1)
    if not client.connect():
        print(
            "ERRO: nao foi possivel conectar. Verifique IP/porta, a rede e se o "
            "firmware esta com o protocolo Modbus ativo no setup."
        )
        sys.exit(1)
    print("Conectado!\n")

    try:
        # --- Coils: piscar N vezes (ON 2 s -> OFF 2 s) ---
        print(f"=== Teste coils (piscar {BLINKS}x, intervalo {INTERVAL_S:.0f} s) ===")
        for ciclo in range(1, BLINKS + 1):
            client.write_coil(0, True, device_id=unit)
            client.write_coil(1, True, device_id=unit)
            r = client.read_coils(0, count=COILS, device_id=unit)
            print(f"[{ciclo}] ON  -> coils: {r.bits[:COILS]}")
            time.sleep(INTERVAL_S)

            client.write_coil(0, False, device_id=unit)
            client.write_coil(1, False, device_id=unit)
            r = client.read_coils(0, count=COILS, device_id=unit)
            print(f"[{ciclo}] OFF -> coils: {r.bits[:COILS]}")
            if ciclo < BLINKS:
                time.sleep(INTERVAL_S)
        print("Coils finalizados (todos OFF).\n")

        # --- PWM: varrer 0..100% em passos de 25% (opcional) ---
        if not coils_only:
            print("=== Teste PWM (holding registers, 0 a 100%, step 25%, intervalo 2 s) ===")
            for pct in PERCENT_STEPS:
                value = pwm_value(pct)
                for addr in range(HOLDING_REGS):
                    client.write_register(addr, value, device_id=unit)
                r = client.read_holding_registers(0, count=HOLDING_REGS, device_id=unit)
                print(f"{pct:>3}% -> valor {value:>4} | hregs: {r.registers[:HOLDING_REGS]}")
                time.sleep(INTERVAL_S)

            # --- Retorna a 0% (estado seguro) ---
            for addr in range(HOLDING_REGS):
                client.write_register(addr, 0, device_id=unit)
            r = client.read_holding_registers(0, count=HOLDING_REGS, device_id=unit)
            print(f"Reset -> hregs: {r.registers[:HOLDING_REGS]}")

        print("\nTeste concluido. Saidas em estado seguro (coils OFF, PWM 0%).")
    finally:
        client.close()


if __name__ == "__main__":
    main()
