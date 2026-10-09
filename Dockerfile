FROM python:3.12-slim-bookworm

ENV PYTHONDONTWRITEBYTECODE=1 \
    PYTHONUNBUFFERED=1 \
    GNSS_DEVICE=/dev/gnss \
    GNSS_BAUD=38400

RUN pip install --no-cache-dir 'pyserial==3.5' \
    && groupadd --system gnss \
    && useradd --system --gid gnss --create-home gnss

WORKDIR /app
COPY gnss_base.py /app/gnss_base.py
USER gnss

EXPOSE 2102 8080
ENTRYPOINT ["python", "/app/gnss_base.py"]
CMD ["serve"]
