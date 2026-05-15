# Lite6 + GELLO (Zhonglin) + OpenParallelGripper -- Setup Guide

## Summary of All Changes

### gello_software -- New Files

| File | Purpose |
|------|---------|
| `gello/zhonglin/__init__.py` | Package init |
| `gello/zhonglin/driver.py` | `ZhonglinDriver` + `FakeZhonglinDriver` -- serial ASCII driver implementing `DynamixelDriverProtocol` |
| `gello/robots/zhonglin.py` | `ZhonglinRobot` -- implements `Robot` protocol, wraps `ZhonglinDriver` with offsets/signs/gripper normalization |
| `scripts/zhonglin_get_offset.py` | Calibration script to determine joint offsets and signs |
| `configs/lite6_sim_test.yaml` | YAML config for GELLO + simulated Lite6 in MuJoCo |

### gello_software -- Modified Files

| File | Change |
|------|--------|
| `gello/agents/gello_agent.py` | Added `ZhonglinRobotConfig` dataclass; updated `GelloAgent` to accept `zhonglin_config` parameter |
| `experiments/quick_run.py` | Added `sim_lite6` robot option |
| `experiments/launch_nodes.py` | Added `sim_lite6` robot option |
| `requirements.txt` | Added `pyserial` |
| `third_party/mujoco_menagerie` | Initialized git submodule (contains Lite6 MuJoCo model) |

### lerobot -- New Files

| File | Purpose |
|------|---------|
| `teleoperators/gello_lite6/__init__.py` | Package init |
| `teleoperators/gello_lite6/config_gello_lite6.py` | `GelloLite6Config` -- teleoperator config for Zhonglin GELLO |
| `teleoperators/gello_lite6/gello_lite6.py` | `GelloLite6` -- teleoperator that reads Zhonglin servos via gello_software |
| `ufactory_usage/config/lite6_gello_record_config.yaml` | Recording config for Lite6 + GELLO + OpenParallelGripper |

### lerobot -- Modified Files

| File | Change |
|------|--------|
| `teleoperators/utils.py` | Registered `gello_lite6` in teleoperator factory |
| `ufactory_usage/uf_robot_teleop_test.py` | Added `gello_lite6` import |
| `ufactory_usage/uf_robot_record.py` | Added `gello_lite6` import |
| `robots/ufactory_robot/config_uf_robot.py` | Added `modbus_gripper_baudrate`, `modbus_gripper_open`, `modbus_gripper_close` fields for `gripper_type: 20` |
| `robots/ufactory_robot/uf_robot.py` | Added `gripper_type: 20` (OpenParallelGripper) support -- Modbus init, read, write, normalization |

---

## How to Run Each Test

All commands assume you've already done `pip install -e .` in both `gello_software/` and `lerobot/`.

### Test 1: Zhonglin servo raw communication

Verifies the Zhonglin servos respond over serial. No robot needed, just plug in the GELLO USB.

```bash
cd /home/jake.tan/P_PAI/gello_software
python -m gello.zhonglin.driver
```

Move GELLO joints by hand. You should see angles in radians and degrees updating in the terminal. Ctrl+C to stop.

---

### Test 2: Calibration (determine offsets and signs)

Move the Lite6 to home position (all zeros). Manually align the GELLO to match. Then:

```bash
cd /home/jake.tan/P_PAI/gello_software
python scripts/zhonglin_get_offset.py \
    --port /dev/ttyUSB0 \
    --start-joints 0 0 0 0 0 0 \
    --joint-signs 1 1 1 1 1 1
```

If joints move in the wrong direction, flip the corresponding sign to `-1` and re-run. Save the printed `joint_offsets`, `joint_signs`, and gripper values.

---

### Test 3: GELLO agent end-to-end (no robot, just leader arm)

Verifies the full stack: driver -> robot -> agent, with your calibrated values.

```bash
cd /home/jake.tan/P_PAI/gello_software
python -c "
import numpy as np
from gello.agents.gello_agent import GelloAgent, ZhonglinRobotConfig

config = ZhonglinRobotConfig(
    joint_ids=(0, 1, 2, 3, 4, 5),
    joint_offsets=(0, 0, 0, 0, 0, 0),      # paste calibrated values
    joint_signs=(1, 1, 1, 1, 1, 1),        # paste calibrated values
    gripper_config=(6, 135.0, 93.0),        # paste calibrated values
)
agent = GelloAgent(port='/dev/ttyUSB0', zhonglin_config=config)
while True:
    action = agent.act({'joint_state': np.zeros(7)})
    print(f'joints: {np.round(action[:6], 3)}, gripper: {action[6]:.3f}')
"
```

At home position, joints should read near zero. Gripper should go 0 (open) to 1 (closed).

---

### Test 4: GELLO to simulated Lite6 (MuJoCo)

No real robot needed. Opens a MuJoCo viewer with Lite6 that follows the GELLO.

**Option A -- quick_run (simplest):**

```bash
cd /home/jake.tan/P_PAI/gello_software
python experiments/quick_run.py \
    --robot sim_lite6 \
    --agent gello \
    --gello-port /dev/ttyUSB0
```

**Option B -- without GELLO hardware (just test the sim loads):**

```bash
python experiments/quick_run.py --robot sim_lite6 --agent dummy
```

**Option C -- YAML config (after editing offsets/signs in the YAML):**

```bash
python experiments/launch_yaml.py \
    --left-config-path configs/lite6_sim_test.yaml
```

---

### Test 5: OpenParallelGripper standalone test

Verifies the gripper hardware via Modbus. No GELLO needed, just the Lite6 + gripper.

```bash
cd /home/jake.tan/P_PAI/OpenParallelGripper/XL330_version/software/examples
python control_openRB150_with_modbus_rtu.py 192.168.1.85 ocs
```

`o` = open, `c` = close, `s` = sleep 1s. Watch the gripper move.

---

### Test 6: GELLO to Lite6 teleop (lerobot, real robot)

First update `lite6_gello_record_config.yaml` with your calibrated values:
- `robot_ip` -- your Lite6's IP
- `port` -- your Zhonglin USB serial path
- `joint_signs` and `start_joints` -- from calibration

Then run the teleop test:

```bash
cd /home/jake.tan/P_PAI/lerobot
python src/lerobot/ufactory_usage/uf_robot_teleop_test.py \
    -c src/lerobot/ufactory_usage/config/lite6_gello_record_config.yaml
```

Move the GELLO -- Lite6 follows. Squeeze GELLO gripper -- OpenParallelGripper closes. Press Escape to stop.

---

### Test 7: Record training episodes (lerobot, full pipeline)

```bash
cd /home/jake.tan/P_PAI/lerobot
python src/lerobot/ufactory_usage/uf_robot_record.py \
    -c src/lerobot/ufactory_usage/config/lite6_gello_record_config.yaml
```

Press Enter between episodes. Data saves to the path in the YAML config.

---

**Recommended order: 1 -> 2 -> 3 -> 4 -> 5 -> 6 -> 7** (each step validates the next layer).
