"""
SIH26169: Virtual Camera Tracking System for Coarse Alignment of Mobile FSOC Terminals
Module: ui.app

Desktop Application Entry Point & Standalone Launcher:
- Starts local simulation backend thread
- Spawns native desktop UI window
- Supports CLI headless mode and automated benchmark execution
"""

import os
import sys
import time
import socket
import threading
import webbrowser
from http.server import HTTPServer

from ui.server import FSOCRequestHandler, run_server
from simulation.scenarios import ScenarioCatalog
from simulation.benchmark import BenchmarkSuite
from analytics.report_generator import PerformanceReportGenerator


def find_free_port(default_port: int = 8765) -> int:
    """Finds an available local TCP port starting from default."""
    for port in range(default_port, default_port + 100):
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            if s.connect_ex(("127.0.0.1", port)) != 0:
                return port
    return default_port


def start_desktop_app(port: int = 8765, open_window: bool = True):
    """
    Launches the embedded HTTP server and native desktop application window.
    """
    actual_port = find_free_port(port)
    url = f"http://127.0.0.1:{actual_port}/index.html"

    # Start simulation server in daemon thread
    server_address = ("127.0.0.1", actual_port)
    httpd = HTTPServer(server_address, FSOCRequestHandler)
    server_thread = threading.Thread(target=httpd.serve_forever, daemon=True)
    server_thread.start()

    print("=" * 70)
    print("  SIH26169: VIRTUAL FSOC COARSE ALIGNMENT & TRACKING LABORATORY")
    print("  Indian Space Research Organisation (ISRO) - Smart Automation")
    print(f"  Local Mission Control Dashboard: {url}")
    print("=" * 70)

    if open_window:
        # Give server 200ms to bind
        time.sleep(0.3)
        # Attempt pywebview if installed, otherwise launch system browser in app mode
        try:
            import webview
            webview.create_window(
                "ISRO SIH26169: Virtual Camera Tracking System for FSOC Coarse Alignment",
                url,
                width=1600,
                height=980,
                resizable=True,
                background_color="#0a0d14"
            )
            webview.start()
            return
        except Exception:
            # Fall back to launching default browser window
            print(f"[Launcher] Opening dashboard in system desktop browser at: {url}")
            webbrowser.open(url)

    # Keep main thread alive if browser launched
    try:
        while True:
            time.sleep(1.0)
    except KeyboardInterrupt:
        print("\n[FSOC System] Shutdown signal received. Exiting.")
        httpd.shutdown()


def run_cli_benchmark():
    """Executes automated benchmark across all 14 scenarios and displays summary."""
    print("=" * 70)
    print("  RUNNING COMPLETE 14-SCENARIO SIH26169 BENCHMARK SUITE")
    print("  Comparing Baseline Classical CV vs AI/ML Enhanced Detector")
    print("=" * 70)

    suite = BenchmarkSuite()
    reports = suite.run_all_benchmarks()

    print("\n" + "-" * 88)
    print(f"{'Scen ID':<8}{'Scenario Name':<32}{'Baseline RMS':<16}{'Enhanced RMS':<16}{'RMS Imprv':<12}{'Lock % (B/E)'}")
    print("-" * 88)

    for r in reports:
        b_rms = f"{r.baseline.rms_error_mrad:.2f} mrad"
        e_rms = f"{r.enhanced.rms_error_mrad:.2f} mrad"
        imprv = f"{r.rms_error_reduction_pct:+.1f}%"
        locks = f"{r.baseline.lock_retention_rate_pct:.0f}% / {r.enhanced.lock_retention_rate_pct:.0f}%"
        print(f"{r.scenario_id:<8}{r.scenario_name[:30]:<32}{b_rms:<16}{e_rms:<16}{imprv:<12}{locks}")

    print("-" * 88)
    print(f"Benchmark execution completed for {len(reports)} scenarios.\n")
