"""Quartus CLI wrapper for FPGA programming from the desktop application."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
import shutil
import subprocess
from typing import List, Optional


@dataclass
class CommandResult:
    ok: bool
    command: List[str]
    returncode: int
    stdout: str
    stderr: str


class QuartusProgrammer:
    def __init__(self, quartus_bin_dir: str | None = None) -> None:
        self.quartus_bin_dir = Path(quartus_bin_dir) if quartus_bin_dir else None

    def _candidate_paths(self, tool_name: str) -> list[str]:
        names = [tool_name, f"{tool_name}.exe"]
        paths: list[str] = []
        if self.quartus_bin_dir:
            for name in names:
                paths.append(str(self.quartus_bin_dir / name))
        paths.extend(names)
        return paths

    def _resolve_tool(self, tool_name: str) -> str:
        for candidate in self._candidate_paths(tool_name):
            if Path(candidate).exists():
                return candidate
            found = shutil.which(candidate)
            if found:
                return found
        raise FileNotFoundError(f"Cannot find {tool_name}. Set Quartus bin directory or add it to PATH.")

    def _run(self, command: list[str]) -> CommandResult:
        completed = subprocess.run(command, capture_output=True, text=True, shell=False)
        return CommandResult(
            ok=completed.returncode == 0,
            command=command,
            returncode=completed.returncode,
            stdout=completed.stdout.strip(),
            stderr=completed.stderr.strip(),
        )

    def detect_hardware(self) -> CommandResult:
        jtagconfig = self._resolve_tool("jtagconfig")
        return self._run([jtagconfig])

    def program_sof(
        self,
        sof_path: str,
        hardware_name: Optional[str] = None,
        device_index: int = 1,
        mode: str = "JTAG",
    ) -> CommandResult:
        quartus_pgm = self._resolve_tool("quartus_pgm")
        sof = str(Path(sof_path).expanduser().resolve())

        command = [quartus_pgm, "-m", mode]
        if hardware_name:
            command += ["-c", hardware_name]
        command += ["-o", f"p;{sof}@{device_index}"]
        return self._run(command)