#!/usr/bin/env bash
# Host-only prerequisite. Run in a local terminal with sudo authentication.
# Official procedure: https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html
set -euo pipefail
version=1.20.1-1
sudo -v
if [[ -n $(docker ps -q) ]]; then
  echo 'Docker containers are running; stop them before configuring/restarting Docker.' >&2
  exit 1
fi
task_tmp=$(mktemp -d)
trap 'rm -rf "$task_tmp"' EXIT
curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey -o "$task_tmp/key"
gpg --dearmor --output "$task_tmp/key.gpg" "$task_tmp/key"
curl -fsSL https://nvidia.github.io/libnvidia-container/stable/deb/nvidia-container-toolkit.list \
  | sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g' \
  > "$task_tmp/toolkit.list"
sudo install -m644 "$task_tmp/key.gpg" /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg
sudo install -m644 "$task_tmp/toolkit.list" /etc/apt/sources.list.d/nvidia-container-toolkit.list
sudo apt-get update
sudo apt-get install -y "nvidia-container-toolkit=$version" "nvidia-container-toolkit-base=$version" \
  "libnvidia-container-tools=$version" "libnvidia-container1=$version"
if [[ -f /etc/docker/daemon.json ]]; then
  sudo cp -a /etc/docker/daemon.json "/etc/docker/daemon.json.before-rmt-$(date +%Y%m%d%H%M%S)"
fi
sudo nvidia-ctk runtime configure --runtime=docker
sudo systemctl restart docker
docker info --format '{{json .Runtimes}}'
