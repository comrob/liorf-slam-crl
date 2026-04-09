import shlex
import math
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration


def _build_actions(context, *args, **kwargs):
    share_dir = get_package_share_directory('liorf')
    save_map_script = os.path.join(share_dir, 'scripts', 'save_map.sh')

    service_name = LaunchConfiguration('service_name').perform(context).strip()
    service_name_norm = service_name if service_name.startswith('/') else f'/{service_name}'
    resolution_str = LaunchConfiguration('resolution').perform(context).strip()
    destination = LaunchConfiguration('destination').perform(context)
    wait_timeout_str = LaunchConfiguration('wait_timeout_sec').perform(context).strip()

    if not service_name:
        raise RuntimeError("service_name must not be empty")

    try:
        resolution = float(resolution_str)
    except ValueError as exc:
        raise RuntimeError(f"Invalid resolution '{resolution_str}': must be a float") from exc
    if resolution < 0.0:
        raise RuntimeError("resolution must be >= 0.0")

    try:
        wait_timeout_sec = float(wait_timeout_str)
    except ValueError as exc:
        raise RuntimeError(
            f"Invalid wait_timeout_sec '{wait_timeout_str}': must be a float"
        ) from exc
    if wait_timeout_sec <= 0.0:
        raise RuntimeError("wait_timeout_sec must be > 0.0")
    wait_timeout_sec_int = max(1, int(math.ceil(wait_timeout_sec)))

    if not os.path.isfile(save_map_script):
        raise RuntimeError(
            f"save_map.sh not found at '{save_map_script}'. "
            "Install scripts into share/ or run the helper directly from source."
        )

    destination_for_log = destination if destination else '<default from map node>'

    script_cmd = f"{shlex.quote(save_map_script)} -r {resolution:g}"
    if destination:
        script_cmd += f" -d {shlex.quote(destination)}"

    # Use ros2 CLI from launch so map export can be triggered with launch arguments.
    cmd = (
        f"deadline=$((SECONDS + {wait_timeout_sec_int})) && "
        f"until services=\"$(ros2 service list 2>/dev/null || true)\"; "
        f"grep -Fx {shlex.quote(service_name)} <<<\"$services\" >/dev/null "
        f"|| grep -Fx {shlex.quote(service_name_norm)} <<<\"$services\" >/dev/null; do "
        f"if (( SECONDS >= deadline )); then "
        f"echo 'Timed out waiting for service {service_name} (or {service_name_norm}) after {wait_timeout_sec_int}s' >&2; exit 1; "
        f"fi; sleep 0.5; done && "
        f"{script_cmd}"
    )

    return [
        LogInfo(
            msg=(
                "Triggering SaveMap service call with "
                f"resolution={resolution:g}, destination='{destination_for_log}'"
            )
        ),
        ExecuteProcess(
            cmd=['bash', '-lc', cmd],
            output='screen',
            shell=False,
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'service_name',
            default_value='liorf/save_map',
            description='SaveMap service name.',
        ),
        DeclareLaunchArgument(
            'resolution',
            default_value='0.0',
            description='Map voxel resolution. 0 keeps original density.',
        ),
        DeclareLaunchArgument(
            'destination',
            default_value='',
            description=(
                'Save destination path passed to SaveMap.destination. '
                'Supports absolute paths, HOME-relative paths, and ~/ expansion in the node.'
            ),
        ),
        DeclareLaunchArgument(
            'wait_timeout_sec',
            default_value='30.0',
            description='How long to wait for the SaveMap service to appear.',
        ),
        OpaqueFunction(function=_build_actions),
    ])
