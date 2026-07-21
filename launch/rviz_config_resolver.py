import os


def default_rviz_config_path(share_dir: str) -> str:
    installed_path = os.path.join(share_dir, 'rviz', 'mapping.rviz')
    candidates = []

    source_dir_from_env = os.environ.get('LIORF_SOURCE_DIR', '').strip()
    if source_dir_from_env:
        candidates.append(os.path.join(source_dir_from_env, 'rviz', 'mapping.rviz'))

    # If invoked from inside the source tree, prefer that tree's RViz config.
    current = os.getcwd()
    while True:
        candidates.append(os.path.join(current, 'rviz', 'mapping.rviz'))
        parent = os.path.dirname(current)
        if parent == current:
            break
        current = parent

    launch_dir_real = os.path.dirname(os.path.realpath(__file__))
    candidates.append(os.path.join(os.path.dirname(launch_dir_real), 'rviz', 'mapping.rviz'))

    for candidate in candidates:
        if os.path.isfile(candidate):
            return candidate
    return installed_path