import sys
from argparse import ArgumentParser, Namespace
from pathlib import Path

from station.serial_link import (
    HM10_DEFAULT_CHARACTERISTIC_UUID,
    HM10_DEFAULT_SERVICE_UUID,
    WIFI_DEFAULT_AUDIO_PORT,
    WIFI_DEFAULT_HOST,
    WIFI_DEFAULT_TCP_PORT,
    WIFI_DEFAULT_UDP_PORT,
)
from station.settings import DEFAULT_BAUDRATE, DEFAULT_SEND_RATE


SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))


def parse_args() -> Namespace:
    parser = ArgumentParser(description="Buoy telemetry station with TX12 radio control and optional Wi-Fi keyboard drive.")
    parser.add_argument(
        "--port",
        help="Serial port for the Bluetooth link, for example /dev/tty.HC-05-DevB",
    )
    parser.add_argument(
        "--transport",
        choices=("serial", "ble", "wifi"),
        default="wifi",
        help="Link transport to use for the buoy connection",
    )
    parser.add_argument(
        "--device-name",
        default="ESP32-BUOY",
        help="Device name(s) to auto-discover when --port is not supplied or when using BLE/serial (comma-separated)",
    )
    parser.add_argument("--wifi-host", default=WIFI_DEFAULT_HOST, help="ESP32 Wi-Fi host or IP address")
    parser.add_argument(
        "--wifi-local-ip",
        "--source-ip",
        dest="wifi_local_ip",
        default="auto",
        help="Local IPv4 address to use as the source interface (auto selects all interfaces)",
    )
    parser.add_argument(
        "--tcp-port",
        type=int,
        default=WIFI_DEFAULT_TCP_PORT,
        help="TCP port used for reliable command/control",
    )
    parser.add_argument(
        "--udp-port",
        type=int,
        default=WIFI_DEFAULT_UDP_PORT,
        help="Local UDP port used for fast telemetry",
    )
    parser.add_argument(
        "--audio-port",
        type=int,
        default=WIFI_DEFAULT_AUDIO_PORT,
        help="Local UDP port used for streamed audio",
    )
    parser.add_argument(
        "--ble-service-uuid",
        default=HM10_DEFAULT_SERVICE_UUID,
        help="HM-10 BLE service UUID",
    )
    parser.add_argument(
        "--ble-characteristic-uuid",
        default=HM10_DEFAULT_CHARACTERISTIC_UUID,
        help="HM-10 BLE characteristic UUID used for write and notify",
    )
    parser.add_argument(
        "--baudrate",
        type=int,
        default=DEFAULT_BAUDRATE,
        help="Serial baudrate used by the serial link",
    )
    parser.add_argument(
        "--send-rate",
        type=float,
        default=DEFAULT_SEND_RATE,
        help="How many command updates to send per second",
    )
    parser.add_argument(
        "--hello-ping",
        action="store_true",
        help="Send hello world text messages over the active transport instead of CTRL commands",
    )
    parser.add_argument(
        "--send-text",
        help="Send this text over the active transport instead of CTRL commands",
    )
    parser.add_argument(
        "--send-interval",
        type=float,
        default=1.0,
        help="Seconds between repeated text messages when --send-text or --hello-ping is enabled",
    )
    parser.add_argument(
        "--comm-log-file",
        help="Write raw transport communication logs to this file",
    )
    return parser.parse_args()


if __name__ == "__main__":
    args = parse_args()
    try:
        from station.qt_app import run as qt_run
    except ModuleNotFoundError as exc:
        if exc.name == "PySide6":
            print("PySide6 is required for the control station. Run setup_env.bat to install dependencies.")
            raise SystemExit(1)
        raise
    qt_run(args)
