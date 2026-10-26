FROM python:3.12-slim-bookworm

RUN pip install --no-cache-dir \
    numpy==2.1.3 \
    scipy==1.14.1 \
    trimesh==4.9.0 \
    shapely==2.1.1 \
    ezdxf==1.4.3 \
    mapbox-earcut==1.0.3

WORKDIR /app
COPY data /app/data
