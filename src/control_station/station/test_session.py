from __future__ import annotations

from datetime import datetime, timezone
import json
import os
from pathlib import Path
import re
import time
from typing import IO, Any


SCHEMA_VERSION = 2


def _safe_label(value: str) -> str:
    label = re.sub(r"[^a-z0-9]+", "-", (value or "test").strip().lower()).strip("-")
    return label or "test"


def _iso_utc(timestamp: float) -> str:
    return datetime.fromtimestamp(timestamp, tz=timezone.utc).isoformat(timespec="milliseconds")


class PersistentTestSession:
    """Append-only, immediately flushed telemetry session."""

    def __init__(self, directory: Path, environment: str, metadata: dict[str, Any]) -> None:
        directory.mkdir(parents=True, exist_ok=True)
        self.started_at = time.time()
        stamp = datetime.fromtimestamp(self.started_at).strftime("%Y%m%d-%H%M%S-%f")[:-3]
        self.path = directory / f"{_safe_label(environment)}-{stamp}.jsonl"
        self.environment = environment
        self.sample_count = 0
        self._handle: IO[str] | None = self.path.open("x", encoding="utf-8", newline="\n")
        self._last_sync_monotonic = time.monotonic()
        self._write(
            {
                "record_type": "metadata",
                "schema_version": SCHEMA_VERSION,
                "timestamp_unix_s": self.started_at,
                "timestamp_utc": _iso_utc(self.started_at),
                "environment": environment,
                **metadata,
            }
        )

    @property
    def active(self) -> bool:
        return self._handle is not None

    def _write(self, record: dict[str, Any]) -> None:
        if self._handle is None:
            raise RuntimeError("test session is closed")
        self._handle.write(json.dumps(record, separators=(",", ":"), allow_nan=False) + "\n")
        self._handle.flush()
        now = time.monotonic()
        if now - self._last_sync_monotonic >= 1.0:
            os.fsync(self._handle.fileno())
            self._last_sync_monotonic = now

    def write_sample(self, sample: dict[str, Any], timestamp: float | None = None) -> None:
        captured_at = time.time() if timestamp is None else timestamp
        self.sample_count += 1
        self._write(
            {
                "record_type": "sample",
                "schema_version": SCHEMA_VERSION,
                "sample_index": self.sample_count,
                "timestamp_unix_s": captured_at,
                "timestamp_utc": _iso_utc(captured_at),
                "elapsed_s": round(captured_at - self.started_at, 6),
                **sample,
            }
        )

    def close(self, reason: str = "operator_stop", summary: dict[str, Any] | None = None) -> None:
        if self._handle is None:
            return
        stopped_at = time.time()
        self._write(
            {
                "record_type": "end",
                "schema_version": SCHEMA_VERSION,
                "timestamp_unix_s": stopped_at,
                "timestamp_utc": _iso_utc(stopped_at),
                "elapsed_s": round(stopped_at - self.started_at, 6),
                "reason": reason,
                "sample_count": self.sample_count,
                "summary": summary or {},
            }
        )
        os.fsync(self._handle.fileno())
        self._handle.close()
        self._handle = None
