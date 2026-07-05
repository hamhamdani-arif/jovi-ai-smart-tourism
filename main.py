"""
Jovi AI Smart Tourism - Backend Bridge
========================================
Jembatan antara Dify (chatbot Jovi) dan simulasi rute (OSRM + Kinematic Bicycle Model).

Alur:
1. Dify mengirim itinerary (hasil chat) ke POST /webhook/itinerary
2. Backend melakukan geocoding tiap lokasi -> koordinat GPS
3. Backend menjalankan simulasi rute berdasarkan koordinat tsb (lihat simulation.py)
4. Frontend mengambil hasil simulasi (JSON: route + vehicle_trajectory + waypoints)
   via GET /simulate/{simulation_id} dan merender di peta Leaflet

Catatan pembagian sistem:
- Dify = satu-satunya tempat chatflow/LLM (API key LLM dipasang di Dify workspace,
  BUKAN di sini). Backend ini murni middleware (geocoding, routing OSRM, kinematic
  model), tidak pernah menyimpan/memanggil API key LLM apa pun.

TODO untuk dikembangkan di Claude Code:
- Tambahkan penyimpanan hasil simulasi (saat ini masih in-memory dict, ganti ke DB/file storage kalau perlu)
- Tambahkan validasi/error handling yang lebih lengkap
"""

from fastapi import FastAPI, HTTPException, BackgroundTasks
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel
from typing import List, Optional
import time
import uuid
import requests

import simulation

app = FastAPI(title="Jovi AI Smart Tourism - Backend Bridge")

# CORS: izinkan frontend (ganti "*" dengan domain frontend kamu saat production)
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_methods=["*"],
    allow_headers=["*"],
)

# ---------------------------------------------------------------------------
# In-memory storage sementara (ganti ke DB/file kalau proyek makin besar)
# ---------------------------------------------------------------------------
simulations = {}  # simulation_id -> {"status", "waypoints", "route", "vehicle_trajectory"}
latest_simulation_id: Optional[str] = None  # dipoll frontend biar otomatis, tanpa paste manual di console


# ---------------------------------------------------------------------------
# Schema data
# ---------------------------------------------------------------------------
class LocationItem(BaseModel):
    day: int
    name: str  # nama lokasi dari itinerary Jovi, misal "Malioboro, Yogyakarta"
    notes: Optional[str] = None  # catatan tambahan (misal "kuliner", "sejarah")


class ItineraryPayload(BaseModel):
    conversation_id: Optional[str] = None
    locations: List[LocationItem]


class Coordinate(BaseModel):
    day: int
    name: str
    lat: float
    lng: float


# ---------------------------------------------------------------------------
# Endpoint utama: menerima itinerary dari Dify
# ---------------------------------------------------------------------------
@app.post("/webhook/itinerary")
async def receive_itinerary(payload: ItineraryPayload, background_tasks: BackgroundTasks):
    """
    Dipanggil oleh Dify (HTTP Request node) setelah Jovi selesai generate itinerary.
    """
    if not payload.locations:
        raise HTTPException(status_code=400, detail="Itinerary kosong, tidak ada lokasi.")

    simulation_id = str(uuid.uuid4())
    simulations[simulation_id] = {
        "status": "processing",
        "waypoints": [],
        "route": None,
        "route_segments": None,
        "vehicle_trajectory": None,
    }

    global latest_simulation_id
    latest_simulation_id = simulation_id

    # Jalankan proses geocoding + simulasi di background agar Dify tidak menunggu lama
    background_tasks.add_task(process_itinerary, simulation_id, payload.locations)

    return {"simulation_id": simulation_id, "status": "processing"}


# ---------------------------------------------------------------------------
# Proses background: geocode -> simulasi
# ---------------------------------------------------------------------------
def process_itinerary(simulation_id: str, locations: List[LocationItem]):
    try:
        coordinates: List[Coordinate] = []
        skipped_locations: List[str] = []
        for loc in locations:
            try:
                lat, lng = geocode_location(loc.name)
                coordinates.append(Coordinate(day=loc.day, name=loc.name, lat=lat, lng=lng))
            except ValueError:
                # nama lokasi dari LLM kadang tidak persis cocok dengan basis data
                # Nominatim (mis. "Mausoleum Dr. Sun Yat-sen" vs "Sun Yat-sen Mausoleum") -
                # lewati lokasi ini saja, jangan gagalkan seluruh itinerary.
                skipped_locations.append(loc.name)

        if not coordinates:
            raise ValueError(f"Semua lokasi gagal di-geocode: {', '.join(skipped_locations)}")

        simulations[simulation_id]["waypoints"] = [c.dict() for c in coordinates]
        if skipped_locations:
            simulations[simulation_id]["skipped_locations"] = skipped_locations

        sim_result = run_metadrive_simulation(coordinates)

        simulations[simulation_id]["route"] = sim_result["route"]
        simulations[simulation_id]["route_segments"] = sim_result["route_segments"]
        simulations[simulation_id]["vehicle_trajectory"] = sim_result["vehicle_trajectory"]
        if sim_result.get("skipped_route_waypoints"):
            simulations[simulation_id]["skipped_route_waypoints"] = sim_result["skipped_route_waypoints"]
        simulations[simulation_id]["status"] = "done"

    except Exception as e:
        simulations[simulation_id]["status"] = "error"
        simulations[simulation_id]["error"] = str(e)


# ---------------------------------------------------------------------------
# TODO: Ganti dengan geocoding API asli
# ---------------------------------------------------------------------------
def geocode_location(location_name: str) -> tuple[float, float]:
    """
    Ubah nama lokasi (misal "Malioboro, Yogyakarta") jadi (lat, lng) via Nominatim (OpenStreetMap).

    Nominatim usage policy membatasi 1 request/detik dan wajib User-Agent -
    process_itinerary memanggil fungsi ini secara sekuensial per lokasi sehingga aman.
    """
    resp = requests.get(
        "https://nominatim.openstreetmap.org/search",
        params={"q": location_name, "format": "json", "limit": 1},
        headers={"User-Agent": "jovi-smart-tourism"},
        timeout=10,
    )
    resp.raise_for_status()
    data = resp.json()
    if not data:
        raise ValueError(f"Lokasi tidak ditemukan: {location_name}")

    time.sleep(1)  # jaga rate limit Nominatim (max 1 req/detik)
    return float(data[0]["lat"]), float(data[0]["lon"])


# ---------------------------------------------------------------------------
# Simulasi kendaraan otonom (OSRM + Kinematic Bicycle Model)
# Implementasi lengkap ada di simulation.py.
# ---------------------------------------------------------------------------
def run_metadrive_simulation(coordinates: List[Coordinate]) -> dict:
    """Jalankan simulasi kendaraan melewati waypoint, return dict {route, vehicle_trajectory}."""
    return simulation.run_simulation(coordinates)


# ---------------------------------------------------------------------------
# Endpoint untuk frontend mengecek status/hasil simulasi
# ---------------------------------------------------------------------------
@app.get("/simulate/{simulation_id}")
async def get_simulation_result(simulation_id: str):
    if simulation_id not in simulations:
        raise HTTPException(status_code=404, detail="Simulation ID tidak ditemukan.")
    return simulations[simulation_id]


# ---------------------------------------------------------------------------
# Endpoint untuk frontend polling otomatis: simulation_id terakhir yang dibuat
# ---------------------------------------------------------------------------
@app.get("/latest_simulation")
async def get_latest_simulation():
    return {"simulation_id": latest_simulation_id}


@app.get("/")
async def root():
    return {"message": "Jovi AI Smart Tourism backend aktif."}
