#!/usr/bin/env python3
"""
Find SequenceStabilizer offset inside libMediaSDK.so.

This utility disassembles ins::RealTimeStitcherImp::CreateStabilizer() in libMediaSDK.so
to determine the exact byte offset where the SequenceStabilizer pointer is stored
inside the internal RealTimeStitcherImp class.
"""

import argparse
import os
import re
import shutil
import subprocess
import sys


def find_default_so():
    candidates = [
        "/camera_ws/build/insta360_ros_driver/media_sdk/usr/lib/libMediaSDK.so",
        "/camera_ws/install/insta360_ros_driver/lib/insta360_ros_driver/media_sdk/libMediaSDK.so",
        os.path.join(os.path.dirname(__file__), "..", "lib", "media_sdk", "libMediaSDK.so"),
    ]
    for c in candidates:
        abs_p = os.path.abspath(c)
        if os.path.isfile(abs_p):
            return abs_p
    return None


def main():
    parser = argparse.ArgumentParser(
        description="Find internal SequenceStabilizer memory offset in libMediaSDK.so"
    )
    parser.add_argument(
        "--so",
        "-s",
        type=str,
        default=None,
        help="Path to libMediaSDK.so (defaults to workspace build/install directory)",
    )
    args = parser.parse_args()

    so_path = args.so or find_default_so()
    if not so_path or not os.path.isfile(so_path):
        print(f"[ERROR] libMediaSDK.so not found at '{so_path}'.", file=sys.stderr)
        print("Please specify path using --so /path/to/libMediaSDK.so", file=sys.stderr)
        sys.exit(1)

    print(f"[*] Analyzing: {so_path}")

    # Check required tools
    for tool in ["nm", "objdump"]:
        if not shutil.which(tool):
            print(f"[ERROR] Required tool '{tool}' not found in PATH.", file=sys.stderr)
            sys.exit(1)

    # 1. Search for CreateStabilizer symbol
    print("[*] Resolving dynamic symbols...")
    try:
        nm_out = subprocess.check_output(
            ["nm", "-D", "-C", so_path], stderr=subprocess.DEVNULL
        ).decode("utf-8", errors="ignore")
    except subprocess.CalledProcessError as e:
        print(f"[ERROR] 'nm' failed on {so_path}: {e}", file=sys.stderr)
        sys.exit(1)

    target_symbol = "ins::RealTimeStitcherImp::CreateStabilizer()"
    match = re.search(r"([0-9a-fA-F]+)\s+[TtWw]\s+" + re.escape(target_symbol), nm_out)
    if not match:
        print(f"[ERROR] Could not find symbol '{target_symbol}' in {so_path}", file=sys.stderr)
        sys.exit(1)

    func_addr = int(match.group(1), 16)
    print(f"[+] Found {target_symbol} at 0x{func_addr:x}")

    # 2. Check for GetStabilizerQuaternion symbol
    quat_sym = re.search(
        r"([0-9a-fA-F]+)\s+[TtWw]\s+arvrender::SequenceStabilizer::GetStabilizerQuaternion\(double\)",
        nm_out,
    )
    if quat_sym:
        print(f"[+] Found SequenceStabilizer::GetStabilizerQuaternion at 0x{int(quat_sym.group(1), 16):x}")
    else:
        print("[-] Warning: SequenceStabilizer::GetStabilizerQuaternion not found in exported symbols.")

    # 3. Disassemble CreateStabilizer function
    disasm_len = 0x300
    stop_addr = func_addr + disasm_len
    try:
        obj_out = subprocess.check_output(
            [
                "objdump",
                "-d",
                f"--start-address=0x{func_addr:x}",
                f"--stop-address=0x{stop_addr:x}",
                "-M",
                "intel",
                so_path,
            ],
            stderr=subprocess.DEVNULL,
        ).decode("utf-8", errors="ignore")
    except subprocess.CalledProcessError as e:
        print(f"[ERROR] 'objdump' failed: {e}", file=sys.stderr)
        sys.exit(1)

    # 4. Search for stores into [rbx + offset]
    # In x86_64 sysv ABI, 'this' pointer (RealTimeStitcherImp*) is held in rbx.
    print("[*] Analyzing disassembled instructions...")
    stores = []
    for line in obj_out.splitlines():
        # Match pattern: mov [rbx+0x...], reg
        m = re.search(r"mov\s+(?:QWORD PTR\s+)?\[rbx\+(0x[0-9a-fA-F]+)\],\s*([a-z0-9]+)", line)
        if m:
            offset_str = m.group(1)
            reg = m.group(2)
            stores.append((offset_str, reg, line.strip()))

    if not stores:
        print("[-] No '[rbx + offset]' write instructions found. Printing all memory writes:")
        for line in obj_out.splitlines():
            if "mov" in line and "[" in line:
                print("   ", line.strip())
        sys.exit(1)

    print("\n[+] Found memory write instructions into RealTimeStitcherImp:")
    for offset_str, reg, raw_line in stores:
        print(f"    {raw_line}")

    # The stabilizer pointer is the first offset stored
    raw_ptr_offset = stores[0][0]
    ctrl_block_offset = stores[1][0] if len(stores) > 1 else None

    print("\n" + "=" * 60)
    print(f"  DETECTED STABILIZER RAW POINTER OFFSET: {raw_ptr_offset}")
    if ctrl_block_offset:
        print(f"  DETECTED CONTROL BLOCK OFFSET:          {ctrl_block_offset}")
    print("=" * 60)

    print("\nWhere to update this in the codebase if changed:")
    print("  File: src/media_sdk_stitcher.cpp")
    print("  Line:")
    print(
        f"    auto* stabilizer = *reinterpret_cast<arvrender::SequenceStabilizer**>(\n"
        f"        reinterpret_cast<char*>(imp) + {raw_ptr_offset});"
    )
    print()


if __name__ == "__main__":
    main()
