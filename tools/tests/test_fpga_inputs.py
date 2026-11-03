#!/usr/bin/env python3

from __future__ import annotations

import os
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest
from typing import Optional


ROOT = Path(__file__).resolve().parents[2]


class KcuInputContractTests(unittest.TestCase):
    FIRMWARE_FILES = (
        "cyan_firmware.memh",
        *(f"cyan_firmware_b{index:02x}.memh" for index in range(16)),
    )

    def prepare_firmware(self, root: Path) -> Path:
        firmware_dir = root / "firmware"
        firmware_dir.mkdir()
        for name in self.FIRMWARE_FILES:
            (firmware_dir / name).write_text("00000000\n")
        return firmware_dir

    def run_generate(self, firmware_dir: Path, eeprom: Optional[str]):
        command = [
            "make",
            "--no-print-directory",
            "kcu-generate",
            f"FIRMWARE_DIR={firmware_dir}",
            "VIVADO=/usr/bin/true",
        ]
        if eeprom is not None:
            command.append(f"EEPROM_INIT_FILE={eeprom}")
        return subprocess.run(
            command,
            cwd=ROOT,
            text=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            check=False,
        )

    def test_missing_eeprom_refuses_before_vivado(self):
        with tempfile.TemporaryDirectory() as temp:
            firmware_dir = self.prepare_firmware(Path(temp))
            result = self.run_generate(firmware_dir, None)

        self.assertEqual(result.returncode, 2, result.stdout)
        self.assertIn("EEPROM_INIT_FILE is required", result.stdout)

    def test_nonexistent_eeprom_refuses_before_vivado(self):
        with tempfile.TemporaryDirectory() as temp:
            firmware_dir = self.prepare_firmware(Path(temp))
            missing = Path(temp) / "missing.hex"
            result = self.run_generate(firmware_dir, str(missing))

        self.assertEqual(result.returncode, 2, result.stdout)
        self.assertIn("does not name a readable file", result.stdout)

    def test_explicit_eeprom_is_forwarded_to_vivado(self):
        with tempfile.TemporaryDirectory() as temp:
            firmware_dir = self.prepare_firmware(Path(temp))
            eeprom = Path(temp) / "eeprom.hex"
            eeprom.write_text("ffff\n" * 128)
            result = self.run_generate(firmware_dir, str(eeprom))

        self.assertEqual(result.returncode, 0, result.stdout)

    @unittest.skipUnless(shutil.which("tclsh"), "tclsh is not installed")
    def test_direct_generator_rejects_missing_eeprom(self):
        with tempfile.TemporaryDirectory() as temp:
            firmware_dir = self.prepare_firmware(Path(temp))
            env = os.environ.copy()
            env["CYAN_PROM_DIR"] = str(firmware_dir)
            env.pop("CYAN_EEPROM_INIT_FILE", None)
            result = subprocess.run(
                ["tclsh", "fpga/kcu105_cyan/generate.tcl", "soc-generate"],
                cwd=ROOT,
                env=env,
                text=True,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                check=False,
            )

        self.assertNotEqual(result.returncode, 0, result.stdout)
        self.assertIn("must name an external 128-word 93CS56 image", result.stdout)

    @unittest.skipUnless(shutil.which("tclsh"), "tclsh is not installed")
    def test_direct_generator_rejects_malformed_eeprom(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            firmware_dir = self.prepare_firmware(root)
            eeprom = root / "eeprom.hex"
            eeprom.write_text("ffff\n" * 127)
            env = os.environ.copy()
            env["CYAN_PROM_DIR"] = str(firmware_dir)
            env["CYAN_EEPROM_INIT_FILE"] = str(eeprom)
            result = subprocess.run(
                ["tclsh", "fpga/kcu105_cyan/generate.tcl", "soc-generate"],
                cwd=ROOT,
                env=env,
                text=True,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                check=False,
            )

        self.assertNotEqual(result.returncode, 0, result.stdout)
        self.assertIn("expected 128 16-bit words, found 127", result.stdout)


if __name__ == "__main__":
    unittest.main()
