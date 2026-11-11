---
name: uv-data-collector-conda-migrate
description: >-
  Expert playbook for migrating fiveages-sim/data-collector-platform from Conda
  to UV for G1D / ROS2 Jazzy pose-mode collection and LeRobot v2.1 conversion.
  Use when replacing conda platform-data-collect/convert, fixing CONVERT_PYTHON,
  bootstrap_uv_envs, init.sh platform-data, health_check UV backend, or Agent
  collect_cmd that still says conda activate.
---

# UV migrate: data-collector-platform

Convert **conda** `platform-data-collect` / `platform-data-convert` (and related
docs/scripts) to **UV** so collection + ACT demos no longer require Miniconda.

Validated pattern: UV collect env + UV convert env (`lerobot==0.3.3`) + ROS
sourced at runtime. Dataset schema is unchanged → old conda pose demos remain
train-compatible with new UV demos.

## When to apply

- Company bans conda/anaconda/miniconda
- `./init.sh platform-data` still creates conda envs
- Collector logs `Spawning converter in conda env` instead of `via UV/python`
- `agent_config.json` `collect_cmd` still has `conda activate platform-data-collect`
- Health check fails looking for conda envs after UV install

## Non-goals

- Do **not** pip-install `rclpy` / ROS message packages into UV envs (source
  `/opt/ros/jazzy` + workspace instead).
- Do **not** change LeRobot feature layout / `STATE_ACTION_SOURCE` as part of
  the migrate unless the user asks.
- Agent `install.sh` conda detection may remain for `agent_client`; prefer
  documenting UV `collect_cmd` over rewriting the whole Agent installer unless
  requested.

## Target layout

| Role | UV path |
|------|---------|
| Collect | `~/uv-envs/envs/platform-data-collect/.venv` |
| Convert | `~/uv-envs/envs/platform-data-convert/.venv` |
| Activate helper | `source ~/uv-envs/activate <name>` |
| Req files | `unified_data_platform/requirements_{collect,convert}.txt` |
| Bootstrap | `unified_data_platform/scripts/bootstrap_uv_envs.sh` |

`UV_ENVS_ROOT` may override `~/uv-envs`.

## Migration checklist (agent steps)

1. **Confirm `uv` on PATH**; install if missing:
   `curl -LsSf https://astral.sh/uv/install.sh | sh`
2. **Bootstrap** from repo root (preferred):
   - `./init.sh platform-data` (default `PLATFORM_ENV_BACKEND=uv`)
   - or `bash unified_data_platform/scripts/bootstrap_uv_envs.sh`
3. **Wire convert interpreter** in `unified_data_platform/config.py`:
   - `CONVERT_PYTHON` → `Path.home() / "uv-envs/envs/platform-data-convert/.venv/bin/python"`
   - Keep `CONVERT_CONDA_ENV` only as fallback when `CONVERT_PYTHON` missing
4. **Update spawn path** in `unified_collector_converter.py`:
   - Prefer `CONVERT_PYTHON` subprocess; log `Spawning converter via UV/python`
   - Fall back to `conda run -n CONVERT_CONDA_ENV` only if needed
5. **Update Agent** `agent_client_installer/Agent/agent_config.json` `collect_cmd`:
   ```bash
   source "$HOME/uv-envs/activate" platform-data-collect
   source /opt/ros/jazzy/setup.bash
   source "$HOME/ros2_ws/install/setup.bash"   # machine-specific
   export RMW_IMPLEMENTATION=rmw_zenoh_cpp
   export ROS_DOMAIN_ID=72
   cd .../unified_data_platform
   python unified_collector_converter.py ...
   ```
   Restart Agent service after edit (`./run.sh` → restart).
6. **Health check**: `PLATFORM_ENV_BACKEND=uv bash unified_data_platform/scripts/health_check.sh`
   - Expect collect numpy **1.x**, convert **lerobot 0.3.3**, system `ffmpeg`
7. **Docs**: README / README.EN / script headers — UV primary, conda marked legacy
   (`environment_*.yml` kept for `PLATFORM_ENV_BACKEND=conda` only).
8. **Smoke collect**:
   ```bash
   source ~/uv-envs/activate platform-data-collect
   source /opt/ros/jazzy/setup.bash && source ~/ros2_ws/install/setup.bash
   export RMW_IMPLEMENTATION=rmw_zenoh_cpp ROS_DOMAIN_ID=72
   cd unified_data_platform && python unified_collector_converter.py
   ```
   After stop(51): `Conversion OK` + `data_collected/<ts>/{data,videos,meta}`.

## Env pins (do not drift)

| Env | Python | Critical pins |
|-----|--------|----------------|
| collect | 3.12 | `numpy>=1.26,<2`, opencv-headless, pandas, pillow, pyyaml |
| convert | 3.12 | `lerobot==0.3.3`, `av==17.0.1`, torch/torchvision |

Convert may use local wheelhouse `~/uv-envs/wheelhouse/platform-data-convert/` for
offline/reproducible installs (`bootstrap_uv_envs.sh` checks it first).

Torch index: `https://download.pytorch.org/whl/cu128` with
`--index-strategy unsafe-best-match` when mixing PyPI + PyTorch indexes.

## Collect command (operator)

Same ROS Int32 **50/51** gates as before. Only activation changes:

```bash
source ~/uv-envs/activate platform-data-collect
# ROS underlays + RMW/DOMAIN …
python unified_collector_converter.py
```

Motion-generation stays on `lerobot_ros2` project `.venv` (already UV-managed);
not part of this conda→UV migrate unless the user asks.

## ACT / dataset compatibility

UV vs conda collectors write the **same** raw CSV/image layout and LeRobot
**v2.1** pose features when `STATE_ACTION_SOURCE=pose`. Mixing UV-new and
conda-old pose episodes for ACT is OK. Do **not** mix joint-mode episodes.

## Common failures

| Symptom | Fix |
|---------|-----|
| `CONVERT_PYTHON not found` → conda fallback | Bootstrap convert; fix path in `config.py`; restart collector |
| Index conflict (`requests` pinned on torch index) | `--index-strategy unsafe-best-match` + PyPI as default index |
| `numpy 2.x` in collect | Reinstall from `requirements_collect.txt` (`numpy<2`) |
| `lerobot` not 0.3.3 in convert | Recreate convert env; do not use lerobot_ros2 0.5.x for convert |
| Agent still conda | Edit `collect_cmd` + restart service |
| Parallel `uv pip` lock timeout | Serialize installs; raise `UV_LOCK_TIMEOUT`; set `UV_CACHE_DIR=$HOME/.cache/uv` |

## Repo files to touch (typical PR)

- `init.sh` — UV default for `platform-data`
- `unified_data_platform/scripts/bootstrap_uv_envs.sh`
- `unified_data_platform/scripts/health_check.sh` / `check_collect_env.sh`
- `unified_data_platform/requirements_{collect,convert}.txt`
- `unified_data_platform/config.py` (`CONVERT_PYTHON`)
- `unified_data_platform/unified_collector_converter.py`
- `agent_client_installer/Agent/agent_config.json`
- `README.md` / `README.EN.md`
- Mark `environment_collect.yml` / `environment_v21.yml` as legacy

## Verify done

- [ ] `source ~/uv-envs/activate platform-data-collect` works (Py 3.12)
- [ ] Convert python imports `lerobot==0.3.3`
- [ ] One episode: UV spawn log + `Conversion OK` + parquet/videos
- [ ] No required operator step says `conda activate` for collect/convert
