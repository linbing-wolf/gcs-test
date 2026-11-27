"""核对当前ELF32装载段和Intel HEX；只读固件，不访问硬件。"""
import argparse
import hashlib
import json
import struct
from pathlib import Path


def elf_load_bytes(path):
    data = path.read_bytes()
    if data[:6] != b"\x7fELF\x01\x01":
        raise ValueError("Expected little-endian ELF32")
    header = struct.unpack_from("<16sHHIIIIIHHHHHH", data)
    result = {}
    for index in range(header[10]):
        kind, offset, _, physical, size, _, _, _ = struct.unpack_from(
            "<8I", data, header[5] + index * header[9]
        )
        if kind == 1 and size:
            if not 0x08000000 <= physical < physical + size <= 0x08100000:
                raise ValueError("ELF load bytes outside STM32F407ZG flash")
            for address, value in enumerate(data[offset:offset + size], physical):
                if address in result and result[address] != value:
                    raise ValueError("Overlapping ELF load bytes")
                result[address] = value
    if not result:
        raise ValueError("No ELF load data")
    return result


def hex_load_bytes(path):
    result = {}
    base = 0
    ended = False
    for text in path.read_text(encoding="ascii").splitlines():
        if not text.startswith(":") or ended:
            raise ValueError("Invalid HEX record or data after EOF")
        record = bytes.fromhex(text[1:])
        if len(record) != record[0] + 5 or sum(record) & 255:
            raise ValueError("HEX record length/checksum mismatch")
        count, kind = record[0], record[3]
        address = base + (record[1] << 8) + record[2]
        payload = record[4:4 + count]
        if kind == 0:
            for target, value in enumerate(payload, address):
                if target in result and result[target] != value:
                    raise ValueError("Conflicting HEX byte")
                result[target] = value
        elif kind == 1:
            if count:
                raise ValueError("Invalid HEX EOF")
            ended = True
        elif kind in (2, 4):
            if count != 2:
                raise ValueError("Invalid extended HEX address")
            base = int.from_bytes(payload, "big") << (4 if kind == 2 else 16)
        elif kind not in (3, 5):
            raise ValueError("Unsupported HEX record")
    if not ended:
        raise ValueError("HEX EOF missing")
    return result


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("axf", type=Path)
    parser.add_argument("hex", type=Path)
    args = parser.parse_args()
    axf_bytes = elf_load_bytes(args.axf)
    hex_bytes = hex_load_bytes(args.hex)
    if axf_bytes != hex_bytes:
        raise ValueError("AXF/HEX load data mismatch")
    print(json.dumps({
        "passed": True,
        "loadBytes": len(axf_bytes),
        "flashStart": hex(min(axf_bytes)),
        "flashEndExclusive": hex(max(axf_bytes) + 1),
        "axfSha256": hashlib.sha256(args.axf.read_bytes()).hexdigest(),
        "hexSha256": hashlib.sha256(args.hex.read_bytes()).hexdigest(),
        "hardwareAccessed": False,
    }, indent=2))
