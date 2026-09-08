FROM python:3.13-slim

WORKDIR /opt/app

COPY requirements.txt .

RUN pip install --no-cache-dir -r requirements.txt


CMD ["python", "server.py"]
