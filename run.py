"""
Entry point for the CellTracker application.

Run with:
    python run.py

This is a thin wrapper around CellTracker.__main__, which performs the
TensorFlow threading configuration that must happen before any other
imports.
"""
from CellTracker.__main__ import main


if __name__ == "__main__":
    main()
