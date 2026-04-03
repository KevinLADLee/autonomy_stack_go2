#!/bin/bash

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SERVICES_DIR="$SCRIPT_DIR/services"

usage() {
	echo "Usage: bash autostart/install_service.sh <service-name|service-file>"
	echo "Available services:"
	find "$SERVICES_DIR" -maxdepth 1 -type f -name '*.service' -printf '  %f\n' | sort
}

if [[ $# -ne 1 ]]; then
	usage
	exit 1
fi

service_name="$1"
if [[ "$service_name" != *.service ]]; then
	service_name="${service_name}.service"
fi

service_source="$SERVICES_DIR/$service_name"
service_target="/etc/systemd/system/$service_name"

if [[ ! -f "$service_source" ]]; then
	echo "Service file not found: $service_source" >&2
	usage
	exit 1
fi

sudo install -m 644 "$service_source" "$service_target"
sudo systemctl daemon-reload
sudo systemctl enable "$service_name"

echo "Autostart enabled for $service_name"
echo "Start now: sudo systemctl start $service_name"
echo "Check status: sudo systemctl status $service_name"