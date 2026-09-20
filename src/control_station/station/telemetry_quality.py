from __future__ import annotations

from dataclasses import dataclass


UINT32_MODULUS = 1 << 32
UINT32_HALF_RANGE = 1 << 31


@dataclass
class ImuStreamQuality:
    expected_interval_ms: int = 50
    last_sequence: int | None = None
    last_timestamp_ms: int | None = None
    received_count: int = 0
    missing_count: int = 0
    duplicate_count: int = 0
    out_of_order_count: int = 0
    restart_count: int = 0
    elapsed_firmware_ms: int = 0
    last_interval_ms: int | None = None

    def observe(self, sequence: int, timestamp_ms: int) -> bool:
        sequence &= 0xFFFF
        timestamp_ms &= 0xFFFFFFFF

        if self.last_timestamp_ms is None:
            self.last_sequence = sequence
            self.last_timestamp_ms = timestamp_ms
            self.received_count = 1
            return True

        if sequence == self.last_sequence and timestamp_ms == self.last_timestamp_ms:
            self.duplicate_count += 1
            return False

        delta_ms = (timestamp_ms - self.last_timestamp_ms) & 0xFFFFFFFF
        if delta_ms == 0:
            self.duplicate_count += 1
            return False

        if delta_ms >= UINT32_HALF_RANGE:
            # A small timestamp following a much larger non-wrapped timestamp
            # indicates an ESP32 reboot rather than an old packet.
            backward_ms = self.last_timestamp_ms - timestamp_ms
            if (
                timestamp_ms < self.last_timestamp_ms
                and self.last_timestamp_ms < 0xF0000000
                and (timestamp_ms < 1000 or backward_ms >= 5000)
            ):
                self.restart_count += 1
                self.last_sequence = sequence
                self.last_timestamp_ms = timestamp_ms
                self.last_interval_ms = None
                self.received_count += 1
                return True
            self.out_of_order_count += 1
            return False

        expected_samples = max(1, int((delta_ms + (self.expected_interval_ms / 2)) // self.expected_interval_ms))
        self.missing_count += max(0, expected_samples - 1)
        self.elapsed_firmware_ms += delta_ms
        self.last_interval_ms = delta_ms
        self.last_sequence = sequence
        self.last_timestamp_ms = timestamp_ms
        self.received_count += 1
        return True

    @property
    def expected_count(self) -> int:
        return self.received_count + self.missing_count

    @property
    def loss_percent(self) -> float:
        return (self.missing_count / self.expected_count * 100.0) if self.expected_count else 0.0

    @property
    def effective_hz(self) -> float:
        completed_intervals = self.received_count - 1 - self.restart_count
        if completed_intervals <= 0 or self.elapsed_firmware_ms <= 0:
            return 0.0
        return completed_intervals / (self.elapsed_firmware_ms / 1000.0)

    def snapshot(self) -> dict[str, int | float | None]:
        return {
            "received": self.received_count,
            "missing": self.missing_count,
            "duplicates": self.duplicate_count,
            "out_of_order": self.out_of_order_count,
            "restarts": self.restart_count,
            "loss_percent": round(self.loss_percent, 3),
            "effective_hz": round(self.effective_hz, 3),
            "last_firmware_interval_ms": self.last_interval_ms,
        }
