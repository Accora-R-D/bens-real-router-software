# Ben's Router Software

The wood router CAM: drop STEP files on the page, get a nested sheet and
Syntec G-code for the router, with a material-removal simulation and a
safety gate (`app/cam/engine/verify.py`) between the program and the machine.

Split out of the fab exporter (Accora-R-D/real-accora-fab-exporter) on
2026-10-27 so the two run on separate instances and no longer compete for one
instance's memory. The code is the exporter's router, unchanged apart from:

- `app/backend/main.py` — its own small app: the router page and API only.
- `app/backend/routes/cam_routes.py` — keeps its own job table instead of
  sharing the exporter's.
- `app/backend/services/memory_service.py` — the memory log reports router jobs.
- `app/scripts/introspect_bodies_batch.py` — only the bought-in hardware name
  check (`is_hardware_name`), extracted from the exporter's Onshape script.

## Run locally

    cd app
    pip install -r requirements.txt
    python -m uvicorn backend.main:app --port 8020

then open http://localhost:8020/cam.

## Tests

    cd app
    python -m pytest -q

## Deploy

The Render service runs this repo's `Dockerfile` (Docker runtime). The build
imports OpenCascade and the whole router, so a missing system library fails
the build with its name in the log. No environment variables are needed;
Render's `PORT` is honoured.

`render.yaml` describes the same service on Render's native Python runtime,
for a Blueprint deploy instead of Docker.

Runs single-worker on purpose: the engine applies per-request settings to
module globals under a lock.
