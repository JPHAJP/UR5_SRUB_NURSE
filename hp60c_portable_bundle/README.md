# HP60C Portable Bundle

This folder contains everything needed to deploy and run the YAHBOOM Nuwa-HP60C depth camera stack on Ubuntu 24.04 with ROS 2 Jazzy.

## What is included

- Bundled driver source: `drivers/ascamera`
- Udev rule: `angstrong-camera.rules`
- Web viewer script: `depth_web_ui.py`
- One-command installer: `install_all.sh`
- One-command runner: `run_all.sh`
- Stop script: `stop_all.sh`
- Dependency list: `dependencies_apt.txt`

## Install (one time)

```bash
cd hp60c_portable_bundle
chmod +x install_all.sh run_all.sh stop_all.sh
./install_all.sh
```

On Ubuntu 24.04, the installer will automatically install ROS 2 Jazzy `ros-base` if `/opt/ros/jazzy/setup.bash` is missing.

Log out and back in (or reboot) after installation so the `video` group permission takes effect.

## Run

```bash
cd hp60c_portable_bundle
./run_all.sh
```

`run_all.sh` now launches the web viewer from a local `.venv` created with `--system-site-packages`, which keeps ROS Jazzy packages available while avoiding incompatible `numpy` overrides from `~/.local`.

Open `http://127.0.0.1:5000`.

## Stop

```bash
cd hp60c_portable_bundle
./stop_all.sh
```
