#!/usr/bin/env python3
"""Teste OPC UA: descobre e le os nos do servidor NodeMCU."""
import asyncio, json
from asyncua import Client, Node

ENDPOINT = "opc.tcp://192.168.100.200:4840"

async def dump_tree(node: Node, indent=0):
    """Percorre a arvore recursivamente mostrando NodeId e valor."""
    prefix = "  " * indent
    try:
        node_id = node.nodeid
        name = await node.read_browse_name()
        val = None
        has_children = False
        try:
            children = await node.get_children()
            has_children = len(children) > 0
        except:
            has_children = False

        if not has_children:
            try:
                val = await node.read_value()
            except:
                val = "(sem leitura)"

        print(f"{prefix}{name.Name}  |  NodeId={node_id}  |  Valor={val}")

        if has_children:
            for child in children:
                await dump_tree(child, indent + 2)
    except Exception as e:
        print(f"{prefix}(erro: {e})")

async def main():
    async with Client(url=ENDPOINT, timeout=10) as c:
        print(f"Conectado em: {ENDPOINT}\n")
        print("--- Arvore completa do servidor ---\n")
        for node in await c.nodes.objects.get_children():
            await dump_tree(node)

if __name__ == "__main__":
    asyncio.run(main())
