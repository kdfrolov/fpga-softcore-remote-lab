from __future__ import annotations

import queue
import threading
from dataclasses import dataclass
from pathlib import Path
import tkinter as tk
from tkinter import filedialog, messagebox, ttk

from .agent import UartMemoryAgent
from .memory_model import MemoryAlignmentError, MemoryModel, MemoryRangeError
from .programmer import QuartusProgrammer
from .serial_port import SerialByteStream, list_serial_ports


def parse_int(value: str) -> int:
    value = value.strip()
    if not value:
        raise ValueError("Empty value")
    return int(value, 0)


@dataclass
class AgentWorker:
    memory: MemoryModel
    logger: callable

    stream: SerialByteStream | None = None
    agent: UartMemoryAgent | None = None
    thread: threading.Thread | None = None

    def start(self, port: str, baud: int) -> None:
        if self.thread and self.thread.is_alive():
            raise RuntimeError("Agent is already running")

        self.stream = SerialByteStream(port=port, baud=baud, timeout=0.2)
        self.agent = UartMemoryAgent(
            memory=self.memory,
            stream=self.stream,
            verbose=False,
            logger=lambda msg: self.logger(f"[agent] {msg}"),
        )
        self.thread = threading.Thread(target=self.agent.serve_forever, daemon=True)
        self.thread.start()

    def stop(self) -> None:
        if self.agent:
            self.agent.stop()
        if self.thread and self.thread.is_alive():
            self.thread.join(timeout=1.0)
        if self.stream:
            try:
                self.stream.close()
            except Exception:
                pass
        self.thread = None
        self.agent = None
        self.stream = None

    def running(self) -> bool:
        return bool(self.thread and self.thread.is_alive())


class App(tk.Tk):
    def __init__(self, default_mem_size: int = 4096 * 4, default_baud: int = 115200, default_hex: str | None = None) -> None:
        super().__init__()
        self.title("FPGA PC Memory Agent")
        self.geometry("1120x760")
        self.minsize(980, 680)

        self.memory = MemoryModel(size_bytes=default_mem_size)
        self.log_queue: queue.Queue[str] = queue.Queue()
        self.agent_worker = AgentWorker(memory=self.memory, logger=self.enqueue_log)

        self.port_var = tk.StringVar()
        self.baud_var = tk.StringVar(value=str(default_baud))
        self.mem_size_var = tk.StringVar(value=str(default_mem_size))

        self.quartus_bin_var = tk.StringVar()
        self.jtag_hw_var = tk.StringVar()
        self.device_index_var = tk.StringVar(value="1")
        self.sof_path_var = tk.StringVar()

        self.hex_path_var = tk.StringVar(value=default_hex or "")
        self.hex_words_var = tk.StringVar(value="0 words loaded")

        self.addr_var = tk.StringVar(value="0x00000000")
        self.value_var = tk.StringVar(value="0x00000000")
        self.mode_var = tk.StringVar(value="word")
        self.read_value_var = tk.StringVar(value="-")

        self._build()
        self.refresh_ports()
        self.after(100, self._poll_logs)

    def enqueue_log(self, text: str) -> None:
        self.log_queue.put(text)

    def _poll_logs(self) -> None:
        try:
            while True:
                msg = self.log_queue.get_nowait()
                self.log_text.configure(state="normal")
                self.log_text.insert("end", msg + "\n")
                self.log_text.see("end")
                self.log_text.configure(state="disabled")
        except queue.Empty:
            pass
        self.after(100, self._poll_logs)

    def _build(self) -> None:
        self.columnconfigure(0, weight=1)
        self.rowconfigure(2, weight=1)

        top = ttk.LabelFrame(self, text="UART agent")
        top.grid(row=0, column=0, sticky="ew", padx=10, pady=10)
        for i in range(8):
            top.columnconfigure(i, weight=1 if i in (1, 3) else 0)

        ttk.Label(top, text="Port").grid(row=0, column=0, sticky="w", padx=6, pady=6)
        self.port_combo = ttk.Combobox(top, textvariable=self.port_var, state="readonly")
        self.port_combo.grid(row=0, column=1, sticky="ew", padx=6, pady=6)

        ttk.Button(top, text="Refresh", command=self.refresh_ports).grid(row=0, column=2, padx=6, pady=6)

        ttk.Label(top, text="Baud").grid(row=0, column=3, sticky="w", padx=6, pady=6)
        ttk.Entry(top, textvariable=self.baud_var, width=12).grid(row=0, column=4, sticky="w", padx=6, pady=6)

        ttk.Button(top, text="Start agent", command=self.start_agent).grid(row=0, column=5, padx=6, pady=6)
        ttk.Button(top, text="Stop agent", command=self.stop_agent).grid(row=0, column=6, padx=6, pady=6)

        self.status_label = ttk.Label(top, text="Stopped")
        self.status_label.grid(row=0, column=7, sticky="e", padx=6, pady=6)

        notebook = ttk.Notebook(self)
        notebook.grid(row=1, column=0, sticky="nsew", padx=10, pady=(0, 10))
        self.rowconfigure(1, weight=1)

        fpga_tab = ttk.Frame(notebook)
        prog_tab = ttk.Frame(notebook)
        mem_tab = ttk.Frame(notebook)

        notebook.add(fpga_tab, text="FPGA")
        notebook.add(prog_tab, text="Program")
        notebook.add(mem_tab, text="PC Memory")

        self._build_fpga_tab(fpga_tab)
        self._build_program_tab(prog_tab)
        self._build_memory_tab(mem_tab)

        log_frame = ttk.LabelFrame(self, text="Log")
        log_frame.grid(row=2, column=0, sticky="nsew", padx=10, pady=(0, 10))
        log_frame.rowconfigure(0, weight=1)
        log_frame.columnconfigure(0, weight=1)

        self.log_text = tk.Text(log_frame, height=12, state="disabled")
        self.log_text.grid(row=0, column=0, sticky="nsew")
        scrollbar = ttk.Scrollbar(log_frame, orient="vertical", command=self.log_text.yview)
        scrollbar.grid(row=0, column=1, sticky="ns")
        self.log_text.configure(yscrollcommand=scrollbar.set)
        self.log_text.bind("<Control-c>", self.copy_selected_log)
        self.log_text.bind("<Control-C>", self.copy_selected_log)

    def _build_fpga_tab(self, parent: ttk.Frame) -> None:
        parent.columnconfigure(1, weight=1)

        ttk.Label(parent, text="Quartus bin dir").grid(row=0, column=0, sticky="w", padx=8, pady=8)
        ttk.Entry(parent, textvariable=self.quartus_bin_var).grid(row=0, column=1, sticky="ew", padx=8, pady=8)
        ttk.Button(parent, text="Browse", command=self.choose_quartus_bin).grid(row=0, column=2, padx=8, pady=8)

        ttk.Label(parent, text="Hardware").grid(row=1, column=0, sticky="w", padx=8, pady=8)
        ttk.Entry(parent, textvariable=self.jtag_hw_var).grid(row=1, column=1, sticky="ew", padx=8, pady=8)
        ttk.Button(parent, text="Detect JTAG", command=self.detect_jtag).grid(row=1, column=2, padx=8, pady=8)

        ttk.Label(parent, text="Device index").grid(row=2, column=0, sticky="w", padx=8, pady=8)
        ttk.Entry(parent, textvariable=self.device_index_var, width=10).grid(row=2, column=1, sticky="w", padx=8, pady=8)

        ttk.Label(parent, text="SOF file").grid(row=3, column=0, sticky="w", padx=8, pady=8)
        ttk.Entry(parent, textvariable=self.sof_path_var).grid(row=3, column=1, sticky="ew", padx=8, pady=8)
        ttk.Button(parent, text="Browse", command=self.choose_sof).grid(row=3, column=2, padx=8, pady=8)

        ttk.Button(parent, text="Program FPGA", command=self.program_fpga).grid(row=4, column=1, sticky="w", padx=8, pady=12)

    def _build_program_tab(self, parent: ttk.Frame) -> None:
        parent.columnconfigure(1, weight=1)

        ttk.Label(parent, text="program.hex").grid(row=0, column=0, sticky="w", padx=8, pady=8)
        ttk.Entry(parent, textvariable=self.hex_path_var).grid(row=0, column=1, sticky="ew", padx=8, pady=8)
        ttk.Button(parent, text="Browse", command=self.choose_hex).grid(row=0, column=2, padx=8, pady=8)

        ttk.Button(parent, text="Load into PC memory model", command=self.load_program_hex).grid(
            row=1, column=1, sticky="w", padx=8, pady=8
        )
        ttk.Label(parent, textvariable=self.hex_words_var).grid(row=2, column=1, sticky="w", padx=8, pady=8)

    def _build_memory_tab(self, parent: ttk.Frame) -> None:
        parent.columnconfigure(1, weight=1)

        ttk.Label(parent, text="Address").grid(row=0, column=0, sticky="w", padx=8, pady=8)
        ttk.Entry(parent, textvariable=self.addr_var).grid(row=0, column=1, sticky="ew", padx=8, pady=8)

        ttk.Label(parent, text="Mode").grid(row=1, column=0, sticky="w", padx=8, pady=8)
        mode_combo = ttk.Combobox(parent, textvariable=self.mode_var, values=["word", "half", "byte"], state="readonly")
        mode_combo.grid(row=1, column=1, sticky="w", padx=8, pady=8)

        ttk.Label(parent, text="Write value").grid(row=2, column=0, sticky="w", padx=8, pady=8)
        ttk.Entry(parent, textvariable=self.value_var).grid(row=2, column=1, sticky="ew", padx=8, pady=8)

        button_row = ttk.Frame(parent)
        button_row.grid(row=3, column=1, sticky="w", padx=8, pady=8)
        ttk.Button(button_row, text="Read", command=self.read_memory).pack(side="left", padx=(0, 8))
        ttk.Button(button_row, text="Write", command=self.write_memory).pack(side="left")

        ttk.Label(parent, text="Read result").grid(row=4, column=0, sticky="w", padx=8, pady=8)
        ttk.Label(parent, textvariable=self.read_value_var).grid(row=4, column=1, sticky="w", padx=8, pady=8)

        ttk.Button(parent, text="Dump 8 words from address", command=self.dump_memory).grid(
            row=5, column=1, sticky="w", padx=8, pady=8
        )

    def refresh_ports(self) -> None:
        ports = list_serial_ports()
        self.port_combo["values"] = ports
        if ports and self.port_var.get() not in ports:
            self.port_var.set(ports[0])
        self.enqueue_log(f"[ui] Serial ports: {ports or 'none'}")

    def start_agent(self) -> None:
        try:
            port = self.port_var.get().strip()
            baud = parse_int(self.baud_var.get())
            if not port:
                raise ValueError("Select serial port")
            self.agent_worker.start(port=port, baud=baud)
            self.status_label.config(text=f"Running on {port} @ {baud}")
            self.enqueue_log(f"[ui] Agent started on {port} @ {baud}")
        except Exception as exc:
            messagebox.showerror("Start agent", str(exc))
            self.enqueue_log(f"[ui] ERROR start agent: {exc}")

    def stop_agent(self) -> None:
        try:
            self.agent_worker.stop()
            self.status_label.config(text="Stopped")
            self.enqueue_log("[ui] Agent stopped")
        except Exception as exc:
            messagebox.showerror("Stop agent", str(exc))
            self.enqueue_log(f"[ui] ERROR stop agent: {exc}")

    def choose_quartus_bin(self) -> None:
        path = filedialog.askdirectory(title="Select Quartus bin directory")
        if path:
            self.quartus_bin_var.set(path)

    def choose_sof(self) -> None:
        path = filedialog.askopenfilename(
            title="Select .sof file",
            filetypes=[("SOF files", "*.sof"), ("All files", "*.*")],
        )
        if path:
            self.sof_path_var.set(path)

    def choose_hex(self) -> None:
        path = filedialog.askopenfilename(
            title="Select program.hex",
            filetypes=[("HEX files", "*.hex"), ("All files", "*.*")],
        )
        if path:
            self.hex_path_var.set(path)

    def detect_jtag(self) -> None:
        try:
            programmer = QuartusProgrammer(self.quartus_bin_var.get().strip() or None)
            result = programmer.detect_hardware()
            self.enqueue_log(f"[quartus] {' '.join(result.command)}")
            if result.stdout:
                self.enqueue_log(result.stdout)
            if result.stderr:
                self.enqueue_log(result.stderr)
            if not result.ok:
                raise RuntimeError(f"jtagconfig failed with code {result.returncode}")
        except Exception as exc:
            messagebox.showerror("Detect JTAG", str(exc))
            self.enqueue_log(f"[ui] ERROR detect JTAG: {exc}")

    def program_fpga(self) -> None:
        try:
            sof_path = self.sof_path_var.get().strip()
            if not sof_path:
                raise ValueError("Select .sof file")
            if not Path(sof_path).exists():
                raise FileNotFoundError(sof_path)

            hardware = self.jtag_hw_var.get().strip() or None
            device_index = parse_int(self.device_index_var.get())
            programmer = QuartusProgrammer(self.quartus_bin_var.get().strip() or None)

            result = programmer.program_sof(
                sof_path=sof_path,
                hardware_name=hardware,
                device_index=device_index,
            )
            self.enqueue_log(f"[quartus] {' '.join(result.command)}")
            if result.stdout:
                self.enqueue_log(result.stdout)
            if result.stderr:
                self.enqueue_log(result.stderr)

            if not result.ok:
                raise RuntimeError(f"quartus_pgm failed with code {result.returncode}")

            self.enqueue_log("[ui] FPGA programmed successfully")
            messagebox.showinfo("Program FPGA", "FPGA programmed successfully")
        except Exception as exc:
            messagebox.showerror("Program FPGA", str(exc))
            self.enqueue_log(f"[ui] ERROR program FPGA: {exc}")

    def load_program_hex(self) -> None:
        try:
            path = self.hex_path_var.get().strip()
            if not path:
                raise ValueError("Select program.hex")
            if not Path(path).exists():
                raise FileNotFoundError(path)

            self.memory.reset()
            words = self.memory.load_hex_words(path)
            self.hex_words_var.set(f"{words} words loaded")
            self.enqueue_log(f"[ui] Loaded {words} words from {path}")
        except Exception as exc:
            messagebox.showerror("Load program.hex", str(exc))
            self.enqueue_log(f"[ui] ERROR load hex: {exc}")

    def read_memory(self) -> None:
        try:
            addr = parse_int(self.addr_var.get())
            mode = self.mode_var.get()
            value = self.memory.read_typed(addr, mode)
            width = {"word": 8, "half": 4, "byte": 2}[mode]
            self.read_value_var.set(f"0x{value:0{width}X}")
            self.enqueue_log(f"[ui] READ {mode} @ 0x{addr:08X} -> 0x{value:0{width}X}")
        except Exception as exc:
            messagebox.showerror("Read memory", str(exc))
            self.enqueue_log(f"[ui] ERROR read: {exc}")

    def write_memory(self) -> None:
        try:
            addr = parse_int(self.addr_var.get())
            value = parse_int(self.value_var.get())
            mode = self.mode_var.get()
            self.memory.write_typed(addr, value, mode)

            width = {"word": 8, "half": 4, "byte": 2}[mode]
            self.enqueue_log(f"[ui] WRITE {mode} @ 0x{addr:08X} <- 0x{value:0{width}X}")
        except Exception as exc:
            messagebox.showerror("Write memory", str(exc))
            self.enqueue_log(f"[ui] ERROR write: {exc}")

    def dump_memory(self) -> None:
        try:
            addr = parse_int(self.addr_var.get())
            base = addr & ~0x3
            dump = self.memory.dump_hex(base, 8)
            self.enqueue_log("[ui] Memory dump:")
            for line in dump.splitlines():
                self.enqueue_log("  " + line)
        except Exception as exc:
            messagebox.showerror("Dump memory", str(exc))
            self.enqueue_log(f"[ui] ERROR dump: {exc}")

    def destroy(self) -> None:
        try:
            self.agent_worker.stop()
        finally:
            super().destroy()
            
    def copy_selected_log(self, event=None):
        try:
            text = self.log_text.get("sel.first", "sel.last")
        except tk.TclError:
            return "break"
        self.clipboard_clear()
        self.clipboard_append(text)
        self.update()
        return "break"


def launch_ui(default_mem_size: int = 4096 * 4, default_baud: int = 115200, default_hex: str | None = None) -> None:
    app = App(default_mem_size=default_mem_size, default_baud=default_baud, default_hex=default_hex)
    app.mainloop()