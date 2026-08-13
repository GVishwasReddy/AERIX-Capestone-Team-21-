# Installation guide

## 1. Target system

- Raspberry Pi 5, Raspberry Pi OS 64-bit (Debian 12 "bookworm")
- Python 3.11+ (ships with the OS)

The stack also runs on any Linux/macOS machine for development — simulation mode
needs no hardware.

## 2. System packages (Raspberry Pi)

```bash
sudo apt update
sudo apt install -y python3-venv python3-pip
```

Add your user to the `dialout` group so it can open serial ports **without
sudo** (needed for the Pixhawk and RPLIDAR). Log out/in afterwards:

```bash
sudo usermod -aG dialout "$USER"
```

## 3. Get the code and create a virtual environment

```bash
cd ~
git clone <this-repo> drone_stack   # or copy the folder over
cd drone_stack
python3 -m venv .venv
source .venv/bin/activate
```

## 4. Install dependencies

Simulation / development only:

```bash
pip install -r requirements-dev.txt
pip install -e .
```

Real hardware (adds the RPLIDAR driver):

```bash
pip install -r requirements-hardware.txt
pip install -e .
```

## 5. Verify with the simulator

```bash
scripts/run_sim.sh
```

You should see log lines from every node and be able to open the dashboard at
`http://<pi-ip>:8090`. Press `Ctrl-C` for a clean shutdown.

Run the test suite:

```bash
pytest
```

## 6. Switch to real hardware

See [CONFIGURATION.md](CONFIGURATION.md). In short: edit the two serial ports in
`config/real.yaml`, then run `scripts/run_real.sh`.

### Finding your serial ports

```bash
ls -l /dev/serial/by-id/      # stable names (recommended)
dmesg | grep -Ei 'ttyACM|ttyUSB'
```

- Pixhawk 2.4.8 over USB → usually `/dev/ttyACM0`
- RPLIDAR C1 over USB → usually `/dev/ttyUSB0`

Using a `/dev/serial/by-id/...` path is strongly recommended because it does not
change when devices are re-plugged in a different order.

## 7. (Optional) run as a service

To start the stack on boot, create a systemd unit that runs
`scripts/run_real.sh` from the project directory with the virtualenv activated.
A template is documented in [ARCHITECTURE.md](ARCHITECTURE.md#deployment).
