# Hardware-bot scripts

These scripts are for the **hardware bot only** (Raspberry Pi 5, ROS 2 Jazzy).
They are not for AMR4.

## `setup_amr_bridge.sh`

Run it on the Pi as your normal user (not root, and not with `sudo`):

```bash
bash scripts/setup_amr_bridge.sh
```

The script expects the repo at `~/Desktop/ros2_ws/src/Ros_lidar_bot` and ROS 2
Jazzy at `/opt/ros/jazzy`. It calls `sudo` only for package install and for
creating `/etc/amr-bridge.env`.

`git fetch` / `git pull` in step 2 talk to a private GitHub repo, so the Pi
user needs credentials that can read it (an SSH key or a credential helper).
A fresh Pi with no login will stop there.

It does eight steps:

1. Checks that the Pi can reach Supabase over HTTPS (one request; any HTTP response counts as reachable).
2. If `~/amr_bridge_rollback_commit.txt` does not already exist, saves the current commit there and the current branch in `~/amr_bridge_rollback_branch.txt`. Re-runs leave those files alone. Then fast-forwards the repo onto the `hardware` branch.
3. Installs system packages (`python3-venv`, `python3-pip`, `python3-aiohttp`, `git`, `curl`, `python3-colcon-common-extensions`, `python3-rosdep`).
4. Updates rosdep and installs ROS dependencies for this repo.
5. Creates `amr-bridge/.venv` and installs `amr-bridge/requirements.txt`.
6. Creates `/etc/amr-bridge.env` (mode `600`, owned by you) if it does not already exist. An existing file is left as-is.
7. Builds `Ros_lidar_bot` with colcon.
8. Writes the launcher `~/run_robot_with_bridge.sh`. The launcher prompts `Type 'yes' to continue`, so it needs a terminal. Do not start it from systemd.

After it finishes, the only values you type by hand are in `/etc/amr-bridge.env`:

- `AMR_SUPABASE_KEY` — the Supabase **publishable** key (`sb_publishable_...`). Never the `service_role` / secret key, and never `AMR_SUPABASE_SERVICE_KEY`.
- `AMR_BRIDGE_TOKEN` — the bridge token from the website owner's private `amr-bridge/.env.pi`.

Do not commit either value. This repository must not contain real secrets.

### Rollback

Run the robot without the bridge:

```bash
~/run_robot_with_bridge.sh enable_amr_bridge:=false
```

Return the repo to the branch and commit saved on the first run:

```bash
git -C ~/Desktop/ros2_ws/src/Ros_lidar_bot checkout "$(cat ~/amr_bridge_rollback_branch.txt)"
git -C ~/Desktop/ros2_ws/src/Ros_lidar_bot checkout "$(cat ~/amr_bridge_rollback_commit.txt)"
```
