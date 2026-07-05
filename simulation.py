"""
Jovi AI Smart Tourism - Simulasi Perjalanan (OSRM + Kinematic Bicycle Model)
===============================================================================
Alur:
  1. Ambil geometry rute jalan asli antar tiap pasangan waypoint berurutan
     dari OSRM demo API (routing di atas data OpenStreetMap, tanpa perlu
     hitung graph/A* sendiri - geometry dari response OSRM dipakai langsung)
  2. Kinematic Bicycle Model + pure-pursuit controller untuk gerakan kendaraan
     mengikuti rute tsb
  3. Return dict JSON-serializable (route + posisi kendaraan per timestep) -
     TIDAK render file GIF/video, biar dirender live di frontend lewat peta
     Leaflet.
"""

import math
from dataclasses import dataclass
from typing import List, Tuple

import requests

WHEELBASE_M = 2.8
MAX_STEER_RAD = math.radians(35)
MAX_SPEED = 12.0            # m/s (~43 km/h)
MAX_ACCEL = 2.5             # m/s^2
LOOKAHEAD_M = 8.0
DT = 0.1
MAX_SIM_STEPS = 5000
MAX_TRAJECTORY_POINTS = 300  # subsample supaya payload JSON & animasi frontend tetap ringan

EARTH_RADIUS_M = 6371000.0

# Server demo publik OSRM - HANYA untuk development/testing, BUKAN untuk production
# (ada rate limit & tanpa SLA, lihat https://github.com/Project-OSRM/osrm-backend/wiki/Api-usage-policy).
# Untuk production: host OSRM sendiri, atau pakai provider berbayar (Mapbox, Google, dll).
OSRM_BASE_URL = "http://router.project-osrm.org/route/v1/driving"


# ---------------------------------------------------------------------------
# 1. Rute jalan asli antar 2 titik lewat OSRM demo API
# ---------------------------------------------------------------------------
def _fetch_osrm_route(orig: Tuple[float, float], dest: Tuple[float, float]) -> List[Tuple[float, float]]:
    """Ambil geometry rute (lat, lng) antara `orig` dan `dest` dari OSRM. Raise ValueError kalau gagal."""
    lat1, lng1 = orig
    lat2, lng2 = dest
    url = f"{OSRM_BASE_URL}/{lng1},{lat1};{lng2},{lat2}"

    try:
        resp = requests.get(url, params={"overview": "full", "geometries": "geojson"}, timeout=15)
        resp.raise_for_status()
        data = resp.json()
    except requests.RequestException as e:
        raise ValueError(f"OSRM tidak bisa dihubungi: {e}") from e

    if data.get("code") != "Ok" or not data.get("routes"):
        raise ValueError(f"OSRM tidak menemukan rute: {data.get('code', 'unknown error')}")

    coordinates = data["routes"][0]["geometry"]["coordinates"]  # [[lng, lat], ...]
    return [(lat, lng) for lng, lat in coordinates]


def plan_route(
    waypoints: List[Tuple[float, float]], names: List[str], days: List[int]
) -> Tuple[List[Tuple[float, float]], List[dict], List[str]]:
    """
    Ambil rute OSRM antar tiap pasangan waypoint berurutan, digabung jadi satu rute penuh.

    Kalau OSRM gagal menemukan rute ke satu waypoint (mis. lokasi tidak terjangkau kendaraan,
    atau API down/rate-limited), lewati waypoint itu dari RUTE (tetap muncul sebagai marker
    di peta) daripada menggagalkan seluruh simulasi.

    Return:
      - full_route: rute lengkap digabung (dipakai Kinematic Bicycle Model, tidak peduli hari)
      - route_segments: list {"day": int, "points": [(lat,lng), ...]} per leg - "day" diambil dari
        hari waypoint TUJUAN leg tsb - dipakai frontend untuk warnai rute per hari
      - skipped_waypoints
    """
    full_route: List[Tuple[float, float]] = [waypoints[0]]
    route_segments: List[dict] = []
    current_point = waypoints[0]
    skipped_waypoints: List[str] = []

    for idx in range(1, len(waypoints)):
        target_point = waypoints[idx]
        try:
            segment = _fetch_osrm_route(current_point, target_point)
        except ValueError:
            skipped_waypoints.append(names[idx])
            continue

        route_segments.append({"day": days[idx], "points": segment})

        if full_route and segment:
            segment = segment[1:]  # hindari duplikasi titik sambungan
        full_route.extend(segment)
        current_point = target_point

    return full_route, route_segments, skipped_waypoints


# ---------------------------------------------------------------------------
# Proyeksi lat/lng <-> meter lokal (Kinematic Bicycle Model bekerja di meter)
# ---------------------------------------------------------------------------
def latlng_to_local(points: List[Tuple[float, float]], origin: Tuple[float, float]) -> List[Tuple[float, float]]:
    lat0, lng0 = origin
    lat0_rad = math.radians(lat0)
    local = []
    for lat, lng in points:
        dx = math.radians(lng - lng0) * EARTH_RADIUS_M * math.cos(lat0_rad)
        dy = math.radians(lat - lat0) * EARTH_RADIUS_M
        local.append((dx, dy))
    return local


def local_to_latlng(points: List[Tuple[float, float]], origin: Tuple[float, float]) -> List[Tuple[float, float]]:
    lat0, lng0 = origin
    lat0_rad = math.radians(lat0)
    result = []
    for x, y in points:
        lat = lat0 + math.degrees(y / EARTH_RADIUS_M)
        lng = lng0 + math.degrees(x / (EARTH_RADIUS_M * math.cos(lat0_rad)))
        result.append((lat, lng))
    return result


# ---------------------------------------------------------------------------
# 3. Kinematic Bicycle Model + pure-pursuit controller
# ---------------------------------------------------------------------------
@dataclass
class VehicleState:
    x: float
    y: float
    yaw: float  # radian
    v: float    # m/s


def _normalize_angle(angle: float) -> float:
    return (angle + math.pi) % (2 * math.pi) - math.pi


def _pure_pursuit_steer(state: VehicleState, target: Tuple[float, float]) -> float:
    """
    Kontrol steering berbasis heading-error ke target.

    Catatan: rumus pure-pursuit klasik (curvature = 2*y/Ld^2, dengan Ld konstan)
    hanya valid kalau target kira-kira di depan kendaraan. Titik jalan asli (OSM)
    berjarak puluhan-ratusan meter dan persimpangan bisa memaksa belokan >90 derajat
    - rumus curvature pecah di kasus itu (target "di belakang" -> steering nol ->
    kendaraan lurus terus tanpa pernah berbelok). Heading-error jauh lebih robust
    untuk sudut berapa pun.
    """
    dx = target[0] - state.x
    dy = target[1] - state.y
    if math.hypot(dx, dy) < 0.5:
        return 0.0
    desired_heading = math.atan2(dy, dx)
    heading_error = _normalize_angle(desired_heading - state.yaw)
    return max(-MAX_STEER_RAD, min(MAX_STEER_RAD, heading_error))


def simulate_bicycle_model(path: List[Tuple[float, float]]) -> List[VehicleState]:
    """Jalankan kendaraan mengikuti `path` (koordinat meter) pakai Kinematic Bicycle Model + pure-pursuit."""
    if len(path) < 2:
        path = path * 2

    heading0 = math.atan2(path[1][1] - path[0][1], path[1][0] - path[0][0])
    state = VehicleState(x=path[0][0], y=path[0][1], yaw=heading0, v=0.0)

    path_length = sum(math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(path[:-1], path[1:]))
    # perkiraan langkah dibutuhkan pada kecepatan rata-rata separuh MAX_SPEED, plus buffer 50%
    est_steps = int(path_length / (MAX_SPEED * 0.5 * DT) * 1.5) + 200
    max_steps = min(max(est_steps, MAX_SIM_STEPS), 30000)

    trajectory = [state]
    path_idx = 0

    for _ in range(max_steps):
        while (path_idx < len(path) - 1 and
               math.hypot(path[path_idx][0] - state.x, path[path_idx][1] - state.y) < LOOKAHEAD_M):
            path_idx += 1
        target = path[path_idx]

        dist_to_goal = math.hypot(path[-1][0] - state.x, path[-1][1] - state.y)
        if dist_to_goal < 1.5 and path_idx >= len(path) - 1:
            break

        delta = _pure_pursuit_steer(state, target)
        accel = MAX_ACCEL if dist_to_goal > LOOKAHEAD_M * 2 else -MAX_ACCEL * 0.6

        new_v = max(0.0, min(MAX_SPEED, state.v + accel * DT))
        new_x = state.x + new_v * math.cos(state.yaw) * DT
        new_y = state.y + new_v * math.sin(state.yaw) * DT
        new_yaw = state.yaw + (new_v / WHEELBASE_M) * math.tan(delta) * DT

        state = VehicleState(x=new_x, y=new_y, yaw=new_yaw, v=new_v)
        trajectory.append(state)

    return trajectory


# ---------------------------------------------------------------------------
# Entry point yang dipanggil dari main.py
# ---------------------------------------------------------------------------
def run_simulation(coordinates) -> dict:
    """
    coordinates: list objek dengan atribut .lat, .lng, .name (Coordinate dari main.py)

    Return dict JSON-serializable:
      - route: list {"lat", "lng"} - rute jalan asli dari OSRM (digabung, tanpa info hari)
      - route_segments: list {"day", "points": [{"lat","lng"}, ...]} - rute dipecah per leg,
        dipakai frontend untuk warnai rute per hari
      - vehicle_trajectory: list {"lat", "lng", "yaw", "v"} - posisi kendaraan
        per timestep dari Kinematic Bicycle Model (sudah disubsample)
      - skipped_route_waypoints: nama waypoint yang berhasil di-geocode tapi
        OSRM tidak bisa menemukan rute ke sana (dilewati dari rute)
    """
    waypoints = [(c.lat, c.lng) for c in coordinates]
    names = [c.name for c in coordinates]
    days = [c.day for c in coordinates]

    route_latlng, route_segments, skipped_route_waypoints = plan_route(waypoints, names, days)

    origin = route_latlng[0]
    route_local = latlng_to_local(route_latlng, origin)
    trajectory_local = simulate_bicycle_model(route_local)

    step = max(1, len(trajectory_local) // MAX_TRAJECTORY_POINTS)
    indices = list(range(0, len(trajectory_local), step))
    if indices[-1] != len(trajectory_local) - 1:
        indices.append(len(trajectory_local) - 1)  # pastikan titik kedatangan asli selalu ikut terkirim
    sampled = [trajectory_local[i] for i in indices]
    sampled_latlng = local_to_latlng([(s.x, s.y) for s in sampled], origin)

    vehicle_trajectory = [
        {"lat": lat, "lng": lng, "yaw": s.yaw, "v": s.v}
        for (lat, lng), s in zip(sampled_latlng, sampled)
    ]
    route = [{"lat": lat, "lng": lng} for lat, lng in route_latlng]
    route_segments_json = [
        {"day": seg["day"], "points": [{"lat": lat, "lng": lng} for lat, lng in seg["points"]]}
        for seg in route_segments
    ]

    return {
        "route": route,
        "route_segments": route_segments_json,
        "vehicle_trajectory": vehicle_trajectory,
        "skipped_route_waypoints": skipped_route_waypoints,
    }
