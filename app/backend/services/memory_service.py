"""Keep the process inside the instance's memory.

Two Linux (glibc) allocator settings matter once heavy work runs on worker
threads, as the router's does:

* glibc gives each thread that allocates its own malloc arena and keeps freed
  memory in it. With a thread pool that is up to 8 x cores arenas of retained
  memory. Two arenas is plenty for this app.
* Memory freed after a job stays mapped to the process until something asks for
  it back. malloc_trim(0) returns it, so the next job starts from the floor
  instead of from the last job's high-water mark.

Both are no-ops anywhere but Linux/glibc, so local Windows runs are unchanged.
"""
from __future__ import annotations

import ctypes
import ctypes.util
import gc
import os
import sys

_M_ARENA_MAX = -8          # mallopt parameter number in glibc's malloc.h

# OpenCascade's optimised allocator (MMGT_OPT=1) keeps freed blocks in its own
# pools for reuse, where neither Python's gc nor malloc_trim can reach them, so
# a big STEP's geometry stays resident after the job is released. MMGT_OPT=0
# makes it use plain malloc, which malloc_trim can return to the OS. Must be
# set before OpenCascade is first loaded — this module is imported first thing
# by backend.main for that reason.
os.environ.setdefault("MMGT_OPT", "0")
_libc = None


def _glibc():
    global _libc
    if _libc is None:
        _libc = False
        if sys.platform.startswith("linux"):
            try:
                lib = ctypes.CDLL(ctypes.util.find_library("c") or "libc.so.6")
                lib.malloc_trim  # noqa: B018 - glibc only; musl lacks it
                _libc = lib
            except (OSError, AttributeError):
                _libc = False
    return _libc or None


def limit_arenas(n: int = 2) -> bool:
    """Cap glibc malloc arenas. Call once, early, before worker threads start."""
    lib = _glibc()
    if lib is None:
        return False
    try:
        return bool(lib.mallopt(_M_ARENA_MAX, int(n)))
    except Exception:                                   # noqa: BLE001
        return False


def give_back() -> None:
    """Collect garbage and hand freed heap back to the OS (Linux only)."""
    gc.collect()
    lib = _glibc()
    if lib is not None:
        try:
            lib.malloc_trim(0)
        except Exception:                               # noqa: BLE001
            pass


def limit_mb() -> float:
    """The container's real memory limit, MB (cgroup v2 or v1); -1 if unknown."""
    for path in ("/sys/fs/cgroup/memory.max", "/sys/fs/cgroup/memory/memory.limit_in_bytes"):
        try:
            raw = open(path).read().strip()
        except OSError:
            continue
        if raw.isdigit() and int(raw) < 1 << 50:
            return round(int(raw) / 2**20, 1)
    return -1.0


def container_mb() -> dict:
    """The whole container's memory as the platform counts it — the web server
    AND any pack-building subprocess — in MB: {"current", "peak"}; -1 where the
    cgroup does not say. This is the number Render enforces its limit on; the
    web process's own RSS (rss_mb) leaves out the export subprocess entirely."""
    def read(*paths):
        for path in paths:
            try:
                raw = open(path).read().strip()
            except OSError:
                continue
            if raw.isdigit():
                return round(int(raw) / 2**20, 1)
        return -1.0
    return {"current": read("/sys/fs/cgroup/memory.current",
                            "/sys/fs/cgroup/memory/memory.usage_in_bytes"),
            "peak": read("/sys/fs/cgroup/memory.peak",
                         "/sys/fs/cgroup/memory/memory.max_usage_in_bytes")}


def rss_mb() -> float:
    """Resident memory of this process, MB; -1 if it cannot be read."""
    try:
        with open("/proc/self/statm") as fh:
            pages = int(fh.read().split()[1])
        return round(pages * os.sysconf("SC_PAGE_SIZE") / 2**20, 1)
    except (OSError, ValueError, AttributeError):
        pass
    try:
        import psutil                                   # local dev, if present
        return round(psutil.Process().memory_info().rss / 2**20, 1)
    except Exception:                                   # noqa: BLE001
        return -1.0


# ------------------------------------------------------------------ watchdog
# When the instance is killed for memory, everything that would explain it dies
# with it, and the exports themselves never come near the limit on their own
# (2026-10-21: whole operator flows measured at <0.8 GB locally, ~1.1 GB live,
# yet the 2 GB instance restarted). So when memory climbs, say in the log —
# which Render keeps across the restart — what was running: every request in
# flight with its age, and every export with its stage.
import threading as _threading
import time as _time

_INFLIGHT: dict = {}
_INFLIGHT_LOCK = _threading.Lock()


class InflightMiddleware:
    """Pure-ASGI: records each HTTP request while it runs (streaming-safe)."""

    def __init__(self, app):
        self.app = app

    async def __call__(self, scope, receive, send):
        if scope.get("type") != "http":
            return await self.app(scope, receive, send)
        key = object()
        with _INFLIGHT_LOCK:
            _INFLIGHT[key] = (f"{scope.get('method', '?')} {scope.get('path', '?')}", _time.time())
        try:
            await self.app(scope, receive, send)
        finally:
            with _INFLIGHT_LOCK:
                _INFLIGHT.pop(key, None)


def inflight_summary() -> list:
    now = _time.time()
    with _INFLIGHT_LOCK:
        items = sorted(_INFLIGHT.values(), key=lambda v: v[1])
    return [f"{label} {now - t0:.0f}s" for label, t0 in items
            if not label.endswith("/cam/memory")]


def _jobs_summary() -> list:
    try:
        from backend.routes.cam_routes import _JOBS
    except Exception:                                   # noqa: BLE001
        return []
    held = sum(1 for j in list(_JOBS.values()) if j.get("_step_parts") is not None)
    out = [f"{len(_JOBS)} router job(s)"]
    if held:
        out.append(f"{held} holding STEP geometry")
    return out


def memory_line() -> str:
    c = container_mb()
    return (f"[memory] container {c['current']:.0f} MB (peak {c['peak']:.0f}) of "
            f"{limit_mb():.0f} | web {rss_mb():.0f} MB | in flight: "
            f"{'; '.join(inflight_summary()) or 'nothing'} | jobs: "
            f"{'; '.join(_jobs_summary()) or 'none'}")


def start_watchdog(warn_mb: float = None, jump_mb: float = 250.0, every_s: float = 5.0):
    """Background thread: log a memory_line() when the container is above
    warn_mb (at most every 30 s while it stays there) or has jumped by jump_mb
    since the last sample. Uses web RSS where the cgroup is unreadable."""
    if warn_mb is None:
        warn_mb = float(os.environ.get("MEM_LOG_MB", "1200") or 1200)

    def run():
        last = None
        last_logged = 0.0
        while True:
            try:
                cur = container_mb()["current"]
                if cur < 0:
                    cur = rss_mb()
                now = _time.time()
                jumped = last is not None and cur - last >= jump_mb
                high = cur >= warn_mb and now - last_logged >= 30
                if jumped or high:
                    print(memory_line(), flush=True)
                    last_logged = now
                last = cur
            except Exception:                           # noqa: BLE001 - never kill the app
                pass
            _time.sleep(every_s)

    t = _threading.Thread(target=run, name="memory-watchdog", daemon=True)
    t.start()
    return t
