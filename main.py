"""
SIH26169: Virtual Camera Tracking System for Coarse Alignment of Mobile FSOC Terminals
Top-Level Application Entry Point

Usage:
  python main.py             # Launches interactive desktop application
  python main.py --benchmark # Runs batch benchmark suite on all 14 scenarios
  python main.py --headless  # Runs server without popping browser
  python main.py --report    # Generates official markdown report
"""

import sys
import argparse
from ui.app import start_desktop_app, run_cli_benchmark
from simulation.scenarios import ScenarioCatalog
from simulation.benchmark import BenchmarkSuite
from analytics.report_generator import PerformanceReportGenerator


def main():
    parser = argparse.ArgumentParser(
        description="ISRO SIH26169: AI-Based Virtual Camera Tracking System for FSOC Coarse Alignment"
    )
    parser.add_argument("--benchmark", action="store_true", help="Run automated benchmark across all 14 scenarios")
    parser.add_argument("--headless", action="store_true", help="Run simulation server in headless mode (no window)")
    parser.add_argument("--port", type=int, default=8765, help="Port to bind simulation server (default: 8765)")
    parser.add_argument("--report", action="store_true", help="Generate performance report file and exit")

    args = parser.parse_args()

    if args.benchmark:
        run_cli_benchmark()
        return

    if args.report:
        suite = BenchmarkSuite()
        scen = ScenarioCatalog.scenario_1_static_target()
        res = suite.run_single_scenario(scen, use_ai=True)
        md = PerformanceReportGenerator.generate_single_report_markdown(res)
        out_file = "FSOC_Performance_Report.md"
        with open(out_file, "w", encoding="utf-8") as f:
            f.write(md)
        print(f"[Report] Generated: {out_file}")
        return

    # Default: launch interactive desktop application
    start_desktop_app(port=args.port, open_window=(not args.headless))


if __name__ == "__main__":
    main()
