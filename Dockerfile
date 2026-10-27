# Ben's router software on Render (Docker runtime).
FROM python:3.12-slim-bookworm

# OpenCascade (cadquery-ocp) and the vtk wheel it pulls in link against the
# system's OpenGL / X11 client libraries even though nothing here draws to a
# screen. The slim image has none of them.
RUN apt-get update && apt-get install -y --no-install-recommends \
        libgl1 libglu1-mesa libegl1 libgomp1 \
        libx11-6 libxext6 libxrender1 libxt6 libsm6 libice6 \
        libxcursor1 libxi6 libxinerama1 libxrandr2 libxfixes3 \
        libfontconfig1 libfreetype6 libglib2.0-0 \
    && rm -rf /var/lib/apt/lists/*

ENV PYTHONUNBUFFERED=1 \
    PIP_NO_CACHE_DIR=1 \
    MPLBACKEND=Agg

WORKDIR /srv/app
COPY app/requirements.txt .
RUN pip install -r requirements.txt

COPY app/ .

# Fail the BUILD, with the reason in the build log, if OpenCascade or the router
# cannot load — rather than a service that starts and dies on the first upload.
RUN MEM_WATCHDOG=0 python -c "from OCP.TDF import TDF_LabelSequence; from OCP.STEPCAFControl import STEPCAFControl_Reader; import backend.main; print('router imports OK')"

# Render sets PORT (10000 by default). One worker on purpose: the CAM engine
# applies per-request settings to module globals under a lock.
CMD ["sh", "-c", "exec python -m uvicorn backend.main:app --host 0.0.0.0 --port ${PORT:-10000}"]
