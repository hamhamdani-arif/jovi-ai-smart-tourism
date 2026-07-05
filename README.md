# Jovi AI Smart Tourism — Starter Kit

Kerangka awal untuk proyek final: **Jovi (Dify chatbot) → Geocoding → MetaDrive Simulation**, ditampilkan dalam satu web (2 panel: chat kiri, simulasi kanan).

## Struktur Folder

```
jovi-project/
├── frontend/
│   └── index.html          # UI 2-panel: embed chat Jovi + panel simulasi
├── backend/
│   ├── main.py              # FastAPI: jembatan Dify <-> MetaDrive
│   └── requirements.txt     # dependency Python
└── README.md
```

## Alur Data (sesuai desain yang sudah dibahas)

```
User chat di Jovi (Dify, di-embed di frontend)
   -> Dify generate itinerary (list nama lokasi per hari)
   -> Dify HTTP Request node panggil -> POST /webhook/itinerary (backend ini)
   -> Backend geocode nama lokasi -> koordinat (lat/lng)
   -> Backend panggil fungsi simulasi (MetaDrive: A* + Kinematic Bicycle Model)
   -> Hasil simulasi (path/video/log) disimpan & diambil frontend via GET /simulate/{id}
```

## Yang PERLU kamu isi/kembangkan di Claude Code

1. **`backend/main.py`**
   - Ganti `TODO: geocode_location()` dengan API geocoding asli (Google Maps API atau Nominatim/OpenStreetMap — gratis, tinggal daftar/tanpa key tergantung provider)
   - Ganti `TODO: run_metadrive_simulation()` dengan pemanggilan script MetaDrive kamu yang sebenarnya (A* + Kinematic Bicycle Model)
   - Simpan API key (geocoding, dll) di file `.env`, JANGAN hardcode di kode

2. **`frontend/index.html`**
   - Ganti `DIFY_EMBED_SRC` dengan URL embed asli dari Dify (Studio -> app kamu -> Embed on website -> ambil src iframe)
   - Sesuaikan endpoint `BACKEND_URL` ke alamat backend kamu (localhost saat dev, URL deploy saat production)

3. **Integrasi Dify -> Backend**
   - Di Dify, tambahkan node **HTTP Request** di akhir workflow Jovi, arahkan ke `POST http://<backend-kamu>/webhook/itinerary`, kirim hasil itinerary sebagai JSON

## Cara Jalanin (development)

```bash
# Backend
cd backend
pip install -r requirements.txt
uvicorn main:app --reload --port 8000

# Frontend
cd ../frontend
# buka index.html langsung di browser, atau pakai live server
```

## Deployment (opsional, untuk demo)

- Frontend: Vercel / Netlify (gratis)
- Backend: Render / Railway (gratis tier, cukup untuk demo)
