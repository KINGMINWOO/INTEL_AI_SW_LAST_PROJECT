"""Farm data ingestion service for CCTV and sensor clients.

Run with:
    $ FARM_DB_URL='mysql+pymysql://user:pass@host:3306/farm' \
      python3 farm_data_service.py

The service accepts JSON payloads (POST /api/farm-data) with a raw string such as
"farm@10.2@75.3@25.8" and stores the parsed metrics in MariaDB. Metric names are
loaded from ``farm_metrics.json`` (create or edit to change the order)."""

from __future__ import annotations

import json
import logging
import os
import signal
import sys
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Dict, List, Optional, Sequence

from flask import Flask, jsonify, request

try:
    from sqlalchemy import Column, DateTime, Integer, String, Text, create_engine, func
    from sqlalchemy.exc import SQLAlchemyError
    from sqlalchemy.orm import Session, declarative_base, sessionmaker

    SQLALCHEMY_AVAILABLE = True
except ImportError:  # pragma: no cover - sqlalchemy optional
    SQLALCHEMY_AVAILABLE = False


LOG = logging.getLogger("farm_data_service")
DEFAULT_METRIC_NAMES: List[str] = [
    "temperature_c",
    "humidity_pct",
    "air_quality",
    "soil_ph",
    "illuminance_lux",
]
CONFIG_FILE = Path("farm_metrics.json")


@dataclass
class ParsedSample:
    device_id: str
    metrics: Dict[str, Optional[float]]
    captured_at: datetime
    raw_payload: str
    extra_tokens: Sequence[str]


class FarmMetricParser:
    """Parse delimited sensor strings into metric dictionaries."""

    def __init__(self, config_path: Path = CONFIG_FILE) -> None:
        self.config_path = config_path
        self.metric_names: List[str] = DEFAULT_METRIC_NAMES.copy()
        self.reload()

    def reload(self) -> None:
        if not self.config_path.exists():
            LOG.warning("Metric config %s not found, using defaults: %s", self.config_path, self.metric_names)
            return
        try:
            with self.config_path.open("r", encoding="utf-8") as fh:
                data = json.load(fh)
            if not isinstance(data, list) or not all(isinstance(it, str) for it in data):
                raise ValueError("Metric config must be a JSON array of strings")
            self.metric_names = [name.strip() or f"metric_{idx}" for idx, name in enumerate(data)]
            LOG.info("Loaded %d metric names from %s", len(self.metric_names), self.config_path)
        except Exception as exc:  # pragma: no cover - configuration errors
            LOG.error("Failed to load metric config %s: %s", self.config_path, exc)
            LOG.warning("Fallback to defaults: %s", self.metric_names)

    def parse(self, payload: str, device_hint: Optional[str] = None) -> ParsedSample:
        payload = payload.strip()
        if not payload:
            raise ValueError("Empty payload")
        tokens = [token.strip() for token in payload.split("@") if token.strip()]
        if not tokens:
            raise ValueError("No tokens found in payload")

        if device_hint:
            device_id = device_hint
            values = tokens[1:] if tokens[0].lower() == device_hint.lower() else tokens
        else:
            device_id = tokens[0]
            values = tokens[1:]

        metrics: Dict[str, Optional[float]] = {}
        extras: List[str] = []
        for idx, token in enumerate(values):
            name = self.metric_names[idx] if idx < len(self.metric_names) else f"metric_{idx+1}"
            if token in {"-", "", "nan"}:
                metrics[name] = None
                continue
            try:
                metrics[name] = float(token)
            except ValueError:
                metrics[name] = None
                extras.append(token)

        return ParsedSample(
            device_id=device_id,
            metrics=metrics,
            captured_at=datetime.now(timezone.utc),
            raw_payload=payload,
            extra_tokens=extras,
        )


if SQLALCHEMY_AVAILABLE:
    Base = declarative_base()

    class FarmSample(Base):
        __tablename__ = "farm_samples"

        id = Column(Integer, primary_key=True)
        device_id = Column(String(64), index=True, nullable=False)
        captured_at = Column(DateTime(timezone=True), nullable=False)
        created_at = Column(DateTime(timezone=True), server_default=func.now(), nullable=False)
        metrics_json = Column(Text, nullable=False)
        raw_payload = Column(Text, nullable=False)
        extra_tokens = Column(Text, nullable=True)

else:  # pragma: no cover - fallback without SQLAlchemy
    Base = None
    FarmSample = object  # type: ignore


class FarmDataRepository:
    """Store samples into MariaDB; degrade gracefully if SQLAlchemy is unavailable."""

    def __init__(self, db_url: Optional[str]) -> None:
        self.db_url = db_url
        self.engine = None
        if db_url and SQLALCHEMY_AVAILABLE:
            self.engine = create_engine(db_url, future=True)
            Base.metadata.create_all(self.engine)  # type: ignore[union-attr]
            self._session_factory = sessionmaker(bind=self.engine, expire_on_commit=False, future=True)
            LOG.info("FarmDataRepository connected to %s", db_url)
        elif db_url and not SQLALCHEMY_AVAILABLE:
            LOG.warning("SQLAlchemy is not installed; running in logging-only mode.")
        else:
            LOG.info("No DB URL provided; running in logging-only mode.")

    def add_sample(self, sample: ParsedSample) -> None:
        metrics_json = json.dumps(sample.metrics, ensure_ascii=False)
        extra_json = json.dumps(list(sample.extra_tokens), ensure_ascii=False) if sample.extra_tokens else None

        if not self.engine:
            LOG.info("Sample (no DB): device=%s metrics=%s", sample.device_id, sample.metrics)
            return

        session: Session = self._session_factory()  # type: ignore[attr-defined]
        try:
            row = FarmSample(  # type: ignore[call-arg]
                device_id=sample.device_id,
                captured_at=sample.captured_at,
                metrics_json=metrics_json,
                raw_payload=sample.raw_payload,
                extra_tokens=extra_json,
            )
            session.add(row)
            session.commit()
        except SQLAlchemyError as exc:  # pragma: no cover - DB runtime issues
            session.rollback()
            LOG.error("Failed to insert sample: %s", exc)
            raise
        finally:
            session.close()


class FarmDataService:
    """HTTP service that ingests farm telemetry and stores it in MariaDB."""

    def __init__(self, parser: FarmMetricParser, repository: FarmDataRepository) -> None:
        self.parser = parser
        self.repository = repository
        self.app = Flask(__name__)
        self._register_routes()

    def _register_routes(self) -> None:
        app = self.app

        @app.route("/api/farm-data", methods=["POST"])
        def ingest():
            payload = request.get_json(silent=True) or {}
            raw_payload = payload.get("payload") or request.data.decode("utf-8").strip()
            device_id = payload.get("device_id")
            if not raw_payload:
                return jsonify({"error": "Missing payload"}), 400
            try:
                sample = self.parser.parse(raw_payload, device_hint=device_id)
            except ValueError as exc:
                return jsonify({"error": str(exc)}), 400
            self.repository.add_sample(sample)
            return jsonify(
                {
                    "device_id": sample.device_id,
                    "captured_at": sample.captured_at.isoformat(),
                    "metrics": sample.metrics,
                    "extra_tokens": list(sample.extra_tokens),
                }
            )

        @app.route("/api/farm-metrics", methods=["GET"])
        def list_metrics():
            return jsonify({"metrics": self.parser.metric_names})

        @app.route("/api/farm-metrics/reload", methods=["POST"])
        def reload_metrics():
            self.parser.reload()
            return jsonify({"metrics": self.parser.metric_names})

    def run(self, host: str = "0.0.0.0", port: int = 8081) -> None:
        LOG.info("Starting FarmDataService on %s:%d", host, port)
        self.app.run(host=host, port=port)


def setup_logging() -> None:
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
        datefmt="%Y-%m-%d %H:%M:%S",
    )


def create_service() -> FarmDataService:
    parser = FarmMetricParser()
    repository = FarmDataRepository(os.getenv("FARM_DB_URL"))
    return FarmDataService(parser, repository)


def main() -> None:
    setup_logging()
    service = create_service()

    def handle_signal(signum, frame):  # type: ignore[assignment]
        LOG.info("Received signal %s, shutting down.", signum)
        sys.exit(0)

    signal.signal(signal.SIGINT, handle_signal)
    signal.signal(signal.SIGTERM, handle_signal)

    host = os.getenv("FARM_SERVICE_HOST", "0.0.0.0")
    port = int(os.getenv("FARM_SERVICE_PORT", "8081"))
    service.run(host=host, port=port)


if __name__ == "__main__":
    main()
