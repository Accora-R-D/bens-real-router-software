"""
Ben's wood router: STEP files in, nested and posted Syntec G-code out.

Split out of the fab exporter (Accora-R-D/real-accora-fab-exporter) on
2026-10-27 so the two no longer share one instance's memory: a router nest and
an exporter pack at the same time had nowhere to go but over the limit.

Serves:
  /cam              -- the router page (frontend/cam.html)
  /cam/*            -- its API: upload, generate, viewer, nc, defaults, memory
  /                 -- redirects to /cam

Run from the app/ directory:
  uvicorn backend.main:app --reload --port 8020
"""
import os

from fastapi import FastAPI
from fastapi.responses import RedirectResponse

from backend.services.memory_service import (InflightMiddleware, limit_arenas,
                                             start_watchdog)

# Before any worker thread exists: glibc arenas are per thread (see
# memory_service). Must come before the CAM routes, which start threads.
limit_arenas(2)

from backend.routes.cam_routes import router as cam_router  # noqa: E402

app = FastAPI(
    title="Ben's Router Software",
    description="Wood router CAM: STEP to nested Syntec G-code",
    version="1.0.0",
    docs_url="/docs",
    redoc_url=None,
)
app.add_middleware(InflightMiddleware)
if os.environ.get("MEM_WATCHDOG", "1") != "0":
    start_watchdog()

app.include_router(cam_router)


@app.get("/")
async def root() -> RedirectResponse:
    return RedirectResponse("/cam")
