#!/usr/bin/env python3

import argparse
import math
import sys
from pathlib import Path

import numpy as np

try:
    import folium
    from folium.plugins import HeatMap
except Exception as exc:
    print(
        "Error: folium is required. Install with: pip install folium "
        "or from scripts/ run: poetry install",
        file=sys.stderr,
    )
    raise


WGS84_A = 6378137.0
WGS84_F = 1.0 / 298.257223563
WGS84_E2 = WGS84_F * (2.0 - WGS84_F)
WGS84_B = WGS84_A * (1.0 - WGS84_F)
WGS84_EP2 = (WGS84_A * WGS84_A - WGS84_B * WGS84_B) / (WGS84_B * WGS84_B)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Visualize saved LILI map over satellite imagery (HTML map)."
    )
    parser.add_argument(
        "--map-dir",
        type=Path,
        required=False,
        default=None,
        help="Directory produced by SaveMap service. Optional: auto-resolved if omitted.",
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=None,
        help="Output HTML path (default: <map-dir>/satellite_overlay.html).",
    )
    parser.add_argument(
        "--max-surf-points",
        type=int,
        default=12000,
        help="Max surf-map points sampled for heatmap overlay.",
    )
    parser.add_argument(
        "--max-traj-points",
        type=int,
        default=5000,
        help="Max trajectory points sampled for polyline.",
    )
    parser.add_argument(
        "--zoom-start",
        type=int,
        default=18,
        help="Initial map zoom level.",
    )
    return parser.parse_args()


def resolve_map_directory(cli_map_dir: Path | None):
    if cli_map_dir is not None:
        path = cli_map_dir.expanduser().resolve()
        if path.exists() and path.is_dir():
            return path, "user input (--map-dir)", str(path)
        raise FileNotFoundError(f"Provided --map-dir does not exist or is not a directory: {path}")

    home = Path.home()
    last_saved_file = home / ".lili_last_saved_map_path"
    if last_saved_file.exists():
        content = last_saved_file.read_text(encoding="utf-8").strip()
        if content:
            candidate = Path(content).expanduser().resolve()
            if candidate.exists() and candidate.is_dir():
                return candidate, "last-saved file", f"{last_saved_file} -> {candidate}"

    default_dir = (home / "Downloads" / "LOAM").resolve()
    if default_dir.exists() and default_dir.is_dir():
        return default_dir, "default path", str(default_dir)

    raise FileNotFoundError(
        "Could not resolve map directory. Tried: "
        f"{last_saved_file} and default {default_dir}. "
        "Provide --map-dir explicitly or save a map first."
    )


def parse_map_metadata(metadata_path: Path):
    if not metadata_path.exists():
        raise FileNotFoundError(f"Missing metadata file: {metadata_path}")

    state = None
    datum = {"latitude": None, "longitude": None, "altitude": None}
    t_enu_local = {
        "x": 0.0,
        "y": 0.0,
        "z": 0.0,
        "qx": 0.0,
        "qy": 0.0,
        "qz": 0.0,
        "qw": 0.0,
        "roll": 0.0,
        "pitch": 0.0,
        "yaw": 0.0,
    }
    has_quaternion = False

    with metadata_path.open("r", encoding="utf-8") as f:
        for raw in f:
            line = raw.strip()
            if not line or line.startswith("#"):
                continue
            if line == "gps_origin_enu:" or line == "global_datum:":
                state = "datum"
                continue
            if line == "T_enu_local:" or line == "T_global_local:":
                state = "tgl"
                continue
            if ":" not in line:
                continue

            key, val = [x.strip() for x in line.split(":", 1)]
            if state == "datum" and key in datum:
                datum[key] = None if val == "null" else float(val)
            elif state == "tgl" and key in t_enu_local:
                t_enu_local[key] = float(val)
                if key in {"qx", "qy", "qz", "qw"}:
                    has_quaternion = True

    if datum["latitude"] is None or datum["longitude"] is None or datum["altitude"] is None:
        raise ValueError(f"{metadata_path.name} has null gps_origin_enu/global_datum; GPS origin is required for satellite overlay")

    return datum, t_enu_local, has_quaternion


def read_pcd_xyz(path: Path) -> np.ndarray:
    if not path.exists():
        raise FileNotFoundError(f"PCD file not found: {path}")

    with path.open("rb") as f:
        header_lines = []
        while True:
            line = f.readline()
            if not line:
                raise ValueError(f"Invalid PCD (missing DATA line): {path}")
            decoded = line.decode("ascii", errors="ignore").strip()
            header_lines.append(decoded)
            if decoded.startswith("DATA"):
                break

        header = {}
        for line in header_lines:
            if not line or line.startswith("#"):
                continue
            parts = line.split()
            key = parts[0]
            vals = parts[1:]
            header[key] = vals

        fields = header.get("FIELDS")
        sizes = [int(x) for x in header.get("SIZE", [])]
        types = header.get("TYPE", [])
        counts = [int(x) for x in header.get("COUNT", ["1"] * len(fields))]
        points = int(header.get("POINTS", ["0"])[0])
        data_mode = header.get("DATA", [""])[0].lower()

        if not fields or not sizes or not types:
            raise ValueError(f"Incomplete PCD header: {path}")
        if "x" not in fields or "y" not in fields or "z" not in fields:
            raise ValueError(f"PCD missing x/y/z fields: {path}")

        def np_code(t: str, s: int):
            if t == "F" and s == 4:
                return "<f4"
            if t == "F" and s == 8:
                return "<f8"
            if t == "U" and s == 1:
                return "u1"
            if t == "U" and s == 2:
                return "<u2"
            if t == "U" and s == 4:
                return "<u4"
            if t == "I" and s == 1:
                return "i1"
            if t == "I" and s == 2:
                return "<i2"
            if t == "I" and s == 4:
                return "<i4"
            raise ValueError(f"Unsupported PCD field type/size: TYPE={t} SIZE={s}")

        dtype_fields = []
        for name, size, typ, cnt in zip(fields, sizes, types, counts):
            code = np_code(typ, size)
            if cnt == 1:
                dtype_fields.append((name, code))
            else:
                for i in range(cnt):
                    dtype_fields.append((f"{name}_{i}", code))

        if data_mode == "binary":
            payload = f.read()
            arr = np.frombuffer(payload, dtype=np.dtype(dtype_fields), count=points)
            xyz = np.stack([arr["x"], arr["y"], arr["z"]], axis=1).astype(np.float64)
            return xyz

        if data_mode == "ascii":
            usecols = [fields.index("x"), fields.index("y"), fields.index("z")]
            txt = np.loadtxt(path, comments="#", skiprows=len(header_lines), usecols=usecols)
            txt = np.atleast_2d(txt)
            return txt.astype(np.float64)

        raise ValueError(f"Unsupported DATA mode '{data_mode}' in {path}")


def read_csv_xyz(path: Path, x_col: str, y_col: str, z_col: str) -> np.ndarray:
    if not path.exists():
        raise FileNotFoundError(f"CSV file not found: {path}")

    data = np.genfromtxt(path, delimiter=",", names=True, dtype=np.float64, encoding=None)
    if data.size == 0:
        return np.empty((0, 3), dtype=np.float64)

    if data.shape == ():
        data = np.array([data], dtype=data.dtype)

    field_names = data.dtype.names or ()
    for field in (x_col, y_col, z_col):
        if field not in field_names:
            raise ValueError(f"CSV missing required column '{field}': {path}")

    return np.column_stack((data[x_col], data[y_col], data[z_col])).astype(np.float64)


def rpy_to_rotmat(roll: float, pitch: float, yaw: float) -> np.ndarray:
    cr = math.cos(roll)
    sr = math.sin(roll)
    cp = math.cos(pitch)
    sp = math.sin(pitch)
    cy = math.cos(yaw)
    sy = math.sin(yaw)

    rx = np.array([[1.0, 0.0, 0.0], [0.0, cr, -sr], [0.0, sr, cr]], dtype=np.float64)
    ry = np.array([[cp, 0.0, sp], [0.0, 1.0, 0.0], [-sp, 0.0, cp]], dtype=np.float64)
    rz = np.array([[cy, -sy, 0.0], [sy, cy, 0.0], [0.0, 0.0, 1.0]], dtype=np.float64)
    return rz @ ry @ rx


def quat_to_rotmat(qx: float, qy: float, qz: float, qw: float) -> np.ndarray:
    n = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    if n <= 1e-12:
        return np.eye(3, dtype=np.float64)

    x = qx / n
    y = qy / n
    z = qz / n
    w = qw / n

    return np.array(
        [
            [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)],
            [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)],
            [2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)],
        ],
        dtype=np.float64,
    )


def local_to_enu(local_xyz: np.ndarray, t_enu_local: dict, has_quaternion: bool) -> np.ndarray:
    if has_quaternion:
        r = quat_to_rotmat(
            t_enu_local.get("qx", 0.0),
            t_enu_local.get("qy", 0.0),
            t_enu_local.get("qz", 0.0),
            t_enu_local.get("qw", 1.0),
        )
    else:
        r = rpy_to_rotmat(t_enu_local["roll"], t_enu_local["pitch"], t_enu_local["yaw"])

    t = np.array([t_enu_local["x"], t_enu_local["y"], t_enu_local["z"]], dtype=np.float64)
    return (local_xyz @ r.T) + t


def geodetic_to_ecef(lat_deg: float, lon_deg: float, alt_m: float):
    lat = math.radians(lat_deg)
    lon = math.radians(lon_deg)
    sin_lat = math.sin(lat)
    cos_lat = math.cos(lat)
    sin_lon = math.sin(lon)
    cos_lon = math.cos(lon)
    n = WGS84_A / math.sqrt(1.0 - WGS84_E2 * sin_lat * sin_lat)
    x = (n + alt_m) * cos_lat * cos_lon
    y = (n + alt_m) * cos_lat * sin_lon
    z = (n * (1.0 - WGS84_E2) + alt_m) * sin_lat
    return x, y, z


def ecef_to_geodetic(x: np.ndarray, y: np.ndarray, z: np.ndarray):
    p = np.sqrt(x * x + y * y)
    theta = np.arctan2(z * WGS84_A, p * WGS84_B)
    sin_theta = np.sin(theta)
    cos_theta = np.cos(theta)

    lon = np.arctan2(y, x)
    lat = np.arctan2(
        z + WGS84_EP2 * WGS84_B * sin_theta * sin_theta * sin_theta,
        p - WGS84_E2 * WGS84_A * cos_theta * cos_theta * cos_theta,
    )

    sin_lat = np.sin(lat)
    n = WGS84_A / np.sqrt(1.0 - WGS84_E2 * sin_lat * sin_lat)
    alt = p / np.cos(lat) - n

    return np.degrees(lat), np.degrees(lon), alt


def enu_to_geodetic(enu_xyz: np.ndarray, datum: dict) -> np.ndarray:
    lat0 = math.radians(datum["latitude"])
    lon0 = math.radians(datum["longitude"])

    sin_lat = math.sin(lat0)
    cos_lat = math.cos(lat0)
    sin_lon = math.sin(lon0)
    cos_lon = math.cos(lon0)

    r_ecef_enu = np.array(
        [
            [-sin_lon, -sin_lat * cos_lon, cos_lat * cos_lon],
            [cos_lon, -sin_lat * sin_lon, cos_lat * sin_lon],
            [0.0, cos_lat, sin_lat],
        ],
        dtype=np.float64,
    )

    x0, y0, z0 = geodetic_to_ecef(datum["latitude"], datum["longitude"], datum["altitude"])
    ecef = (enu_xyz @ r_ecef_enu.T) + np.array([x0, y0, z0], dtype=np.float64)
    lat, lon, alt = ecef_to_geodetic(ecef[:, 0], ecef[:, 1], ecef[:, 2])
    return np.stack([lat, lon, alt], axis=1)


def sample_points(points: np.ndarray, max_points: int) -> np.ndarray:
    if points.shape[0] <= max_points:
        return points
    step = max(1, points.shape[0] // max_points)
    return points[::step][:max_points]


def resolve_enu_trajectory(map_dir: Path, t_enu_local: dict, has_quaternion: bool) -> np.ndarray:
    dense_local_csv = map_dir / "trajectories" / "trajectory_dense_local.csv"
    if dense_local_csv.exists():
        return local_to_enu(
            read_csv_xyz(dense_local_csv, "x_local", "y_local", "z_local"),
            t_enu_local,
            has_quaternion,
        )

    keyframe_local_csv = map_dir / "trajectories" / "trajectory_keyframes_local.csv"
    if keyframe_local_csv.exists():
        return local_to_enu(
            read_csv_xyz(keyframe_local_csv, "x_local", "y_local", "z_local"),
            t_enu_local,
            has_quaternion,
        )

    traj_enu = map_dir / "trajectories" / "trajectory_ENU.pcd"
    if traj_enu.exists():
        return read_pcd_xyz(traj_enu)

    traj_local = map_dir / "trajectories" / "trajectory_local.pcd"
    if traj_local.exists():
        return local_to_enu(read_pcd_xyz(traj_local), t_enu_local, has_quaternion)

    # Legacy fallback (flat export layout)
    legacy_traj_enu = map_dir / "trajectory_ENU.pcd"
    if legacy_traj_enu.exists():
        return read_pcd_xyz(legacy_traj_enu)

    legacy_traj_local = map_dir / "trajectory_local.pcd"
    if legacy_traj_local.exists():
        return local_to_enu(read_pcd_xyz(legacy_traj_local), t_enu_local, has_quaternion)

    raise FileNotFoundError(
        "No supported trajectory export found in trajectories/ or legacy root layout"
    )


def resolve_enu_surf(map_dir: Path, t_enu_local: dict, has_quaternion: bool) -> np.ndarray:
    surf_enu = map_dir / "maps" / "SurfaceMap_ENU.pcd"
    if surf_enu.exists():
        return read_pcd_xyz(surf_enu)

    surf_local = map_dir / "maps" / "SurfaceMap_local.pcd"
    if surf_local.exists():
        return local_to_enu(read_pcd_xyz(surf_local), t_enu_local, has_quaternion)

    # Legacy fallback (flat export layout)
    legacy_surf_enu = map_dir / "SurfMap_ENU.pcd"
    if legacy_surf_enu.exists():
        return read_pcd_xyz(legacy_surf_enu)

    legacy_surf_local = map_dir / "SurfMap_local.pcd"
    if legacy_surf_local.exists():
        return local_to_enu(read_pcd_xyz(legacy_surf_local), t_enu_local, has_quaternion)

    raise FileNotFoundError(
        "Neither maps/SurfaceMap_ENU.pcd nor maps/SurfaceMap_local.pcd found"
    )


def main() -> int:
    args = parse_args()
    map_dir, map_dir_source, map_dir_trace = resolve_map_directory(args.map_dir)
    if args.output is None:
        output_html = map_dir / "satellite_overlay.html"
    else:
        output_html = args.output.resolve()

    print(f"Map directory source: {map_dir_source}")
    print(f"Map directory resolved to: {map_dir}")
    print(f"Resolution trace: {map_dir_trace}")

    georef_path = map_dir / "goereference.yaml"
    if not georef_path.exists():
        legacy_georef = map_dir / "map_georeference.yaml"
        georef_path = legacy_georef if legacy_georef.exists() else map_dir / "map_metadata.yaml"

    datum, t_enu_local, has_quaternion = parse_map_metadata(georef_path)

    traj_enu = resolve_enu_trajectory(map_dir, t_enu_local, has_quaternion)
    surf_enu = resolve_enu_surf(map_dir, t_enu_local, has_quaternion)

    traj_enu = sample_points(traj_enu, args.max_traj_points)
    surf_enu = sample_points(surf_enu, args.max_surf_points)

    traj_geo = enu_to_geodetic(traj_enu, datum)
    surf_geo = enu_to_geodetic(surf_enu, datum)

    center_lat = float(np.mean(traj_geo[:, 0]))
    center_lon = float(np.mean(traj_geo[:, 1]))

    fmap = folium.Map(location=[center_lat, center_lon], zoom_start=args.zoom_start, control_scale=True)

    folium.TileLayer(
        tiles="https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}",
        attr="Esri",
        name="Esri World Imagery",
        overlay=False,
        control=True,
    ).add_to(fmap)

    folium.TileLayer("OpenStreetMap", name="OpenStreetMap", overlay=False, control=True).add_to(fmap)

    traj_latlon = [[float(p[0]), float(p[1])] for p in traj_geo]
    folium.PolyLine(traj_latlon, color="#00D4FF", weight=3, opacity=0.95, tooltip="Trajectory").add_to(fmap)

    folium.Marker(traj_latlon[0], tooltip="Trajectory start", icon=folium.Icon(color="green", icon="play")).add_to(fmap)
    folium.Marker(traj_latlon[-1], tooltip="Trajectory end", icon=folium.Icon(color="red", icon="stop")).add_to(fmap)

    heat_pts = [[float(p[0]), float(p[1]), 0.7] for p in surf_geo]
    HeatMap(heat_pts, name="SurfMap density", radius=7, blur=5, min_opacity=0.25).add_to(fmap)

    folium.LayerControl(collapsed=False).add_to(fmap)

    output_html.parent.mkdir(parents=True, exist_ok=True)
    fmap.save(str(output_html))

    print(f"Saved satellite overlay: {output_html}")
    print(f"Open in browser: {output_html.as_uri()}")
    print(f"Trajectory points rendered: {len(traj_latlon)}")
    print(f"Surf points rendered: {len(heat_pts)}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
