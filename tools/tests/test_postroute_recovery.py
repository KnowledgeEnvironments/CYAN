"""Execute Tcl preflight guards; the checkpoint-open sentinel is NOT Vivado."""

# SPDX-License-Identifier: MIT
# Copyright (c) 2026 I. Finder <src@lispm.net>
import hashlib
import os
from pathlib import Path
import re
import shutil
import subprocess
import tempfile
import unittest


ROOT = Path(__file__).resolve().parents[2]
RECIPE = ROOT / "fpga/kcu105_cyan/recover_postroute_timing.tcl"


@unittest.skipUnless(shutil.which("tclsh") and shutil.which("sha256sum"),
                     "Tcl and sha256sum are required for executable guard tests")
class PostrouteRecoveryGuardTest(unittest.TestCase):
    def setUp(self):
        parent = ROOT / "tmp/tests"
        parent.mkdir(parents=True, exist_ok=True)
        self.temp = tempfile.TemporaryDirectory(prefix="postroute-guard-", dir=parent)
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.input = self.root / "test_routed.dcp"
        self.input.write_bytes(b"Not a Vivado checkpoint: preflight sentinel fixture.\n")
        self.digest = hashlib.sha256(self.input.read_bytes()).hexdigest()
        self.output = self.root / "new-output"
        self.commit = subprocess.check_output(
            ["git", "-C", str(ROOT), "rev-parse", "HEAD"], text=True).strip()
        self.wrapper = self.root / "checkpoint-sentinel.tcl"
        self.wrapper.write_text(
            'proc open_checkpoint {path} {puts "OPEN_CHECKPOINT"; exit 99}\n'
            'source $::env(CYAN_RECOVERY_TEST_RECIPE)\n')

    def run_recipe(self, arguments=None, **changes):
        values = dict(input=self.input, output=self.output, source=ROOT,
                      source_commit=self.commit, recipe_commit=self.commit,
                      digest=self.digest)
        values.update(changes)
        if arguments is None:
            arguments = [str(values[name]) for name in (
                "input", "output", "source", "source_commit", "recipe_commit", "digest")]
        return subprocess.run(
            [shutil.which("tclsh"), str(self.wrapper), *arguments],
            env={**os.environ, "CYAN_RECOVERY_TEST_RECIPE": str(RECIPE)},
            text=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, timeout=10)

    def assert_rejected(self, result, reason):
        self.assertEqual(result.returncode, 2, result.stdout)
        self.assertIn(reason, result.stdout)
        self.assertNotIn("OPEN_CHECKPOINT", result.stdout)

    def test_legacy_unhashed_invocation_never_opens_checkpoint(self):
        result = self.run_recipe(arguments=[str(self.input), str(self.output),
                                            self.commit, self.commit])
        self.assert_rejected(result, "INPUT_SHA256")
        self.assertFalse(self.output.exists())

    def test_valid_guarded_input_reaches_open_sentinel_only(self):
        result = self.run_recipe()
        self.assertEqual(result.returncode, 99, result.stdout)
        self.assertIn("OPEN_CHECKPOINT", result.stdout)

    def test_mistagged_or_malformed_hash_is_rejected_before_open(self):
        for digest, reason in (("0" * 64, "input checkpoint hash mismatch"),
                               ("wrong", "invalid input checkpoint SHA-256")):
            with self.subTest(digest=digest):
                self.assert_rejected(self.run_recipe(digest=digest), reason)
                self.assertFalse(self.output.exists())

    def test_only_routed_checkpoint_boundary_is_accepted(self):
        other = self.root / "test_physopt.dcp"
        other.write_bytes(self.input.read_bytes())
        self.assert_rejected(self.run_recipe(input=other), "requires *_routed.dcp")

    def test_existing_output_and_dangling_link_are_rejected(self):
        self.output.mkdir()
        sentinel = self.output / "preserved.txt"
        sentinel.write_text("existing evidence\n")
        self.assert_rejected(self.run_recipe(), "output already exists")
        self.assertEqual(sentinel.read_text(), "existing evidence\n")
        dangling = self.root / "dangling-output"
        dangling.symlink_to(self.root / "absent")
        self.assert_rejected(self.run_recipe(output=dangling), "output already exists")
        self.assertFalse((self.root / "absent").exists())

    def test_source_and_recipe_identity_are_verified_before_open(self):
        for field, reason in (("source_commit", "source worktree mismatch"),
                               ("recipe_commit", "recipe worktree mismatch")):
            with self.subTest(field=field):
                self.assert_rejected(self.run_recipe(**{field: "0" * 40}), reason)
                self.assertFalse(self.output.exists())

    def test_recovery_materializes_every_post_link_include(self):
        hook = (ROOT / "fpga/kcu105_cyan/constraints/"
                "kcu105_cyan_soc_post_link.tcl").read_text()
        recovery = (ROOT / "fpga/kcu105_cyan/"
                    "recovery_constraints.tcl").read_text()
        includes = set(re.findall(r"[A-Za-z0-9_]+_constraints\.tcl", hook))
        self.assertTrue(includes, "post-link hook has no constraint includes")
        for include in sorted(includes):
            with self.subTest(include=include):
                self.assertIn(include, recovery)


if __name__ == "__main__":
    unittest.main()
