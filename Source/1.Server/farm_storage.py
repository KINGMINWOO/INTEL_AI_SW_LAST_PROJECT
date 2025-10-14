"""MariaDB helper for storing smart-farm sensor samples."""

from __future__ import annotations

import logging
from datetime import datetime, timezone
from typing import Optional

LOG = logging.getLogger("farm_storage")

try:
    from sqlalchemy import (
        Column,
        DateTime,
        Float,
        Integer,
        String,
        Text,
        create_engine,
        func,
    )
    from sqlalchemy.exc import SQLAlchemyError
    from sqlalchemy.orm import Session, declarative_base, sessionmaker

    SQLALCHEMY_AVAILABLE = True
except ImportError:  # pragma: no cover - optional dependency
    SQLALCHEMY_AVAILABLE = False
    Column = DateTime = Float = Integer = String = Text = create_engine = func = None  # type: ignore
    Session = declarative_base = sessionmaker = None  # type: ignore


if SQLALCHEMY_AVAILABLE:
    Base = declarative_base()

    class FarmAirSample(Base):
        __tablename__ = "farm_air_samples"

        id = Column(Integer, primary_key=True)
        device_id = Column(String(64), nullable=False, index=True)
        captured_at = Column(DateTime(timezone=True), nullable=False)
        temperature_c = Column(Float, nullable=True)
        humidity_pct = Column(Float, nullable=True)
        air_quality = Column(Integer, nullable=True)
        illuminance_lux = Column(Integer, nullable=True)
        created_at = Column(DateTime(timezone=True), server_default=func.now(), nullable=False)

    class FarmLandSample(Base):
        __tablename__ = "farm_land_samples"

        id = Column(Integer, primary_key=True)
        device_id = Column(String(64), nullable=False, index=True)
        captured_at = Column(DateTime(timezone=True), nullable=False)
        soil_temperature_c = Column(Float, nullable=True)
        soil_humidity_pct = Column(Float, nullable=True)
        soil_ec = Column(Integer, nullable=True)
        soil_ph = Column(Float, nullable=True)
        created_at = Column(DateTime(timezone=True), server_default=func.now(), nullable=False)

    class FarmTomatoSnapshot(Base):
        __tablename__ = "farm_tomato_snapshots"

        id = Column(Integer, primary_key=True)
        device_id = Column(String(64), nullable=False, index=True)
        captured_at = Column(DateTime(timezone=True), nullable=False)
        ripe_count = Column(Integer, nullable=False)
        rotten_count = Column(Integer, nullable=False)
        created_at = Column(DateTime(timezone=True), server_default=func.now(), nullable=False)
else:  # pragma: no cover - fallback without SQLAlchemy
    Base = None
    FarmAirSample = FarmLandSample = FarmTomatoSnapshot = object  # type: ignore


class FarmDataLogger:
    """Persist smart-farm samples if SQLAlchemy and DB URL are available."""

    def __init__(self, db_url: Optional[str]) -> None:
        self.db_url = db_url
        self.engine = None
        self._session_factory = None

        if not db_url:
            LOG.info("FARM_DB_URL가 설정되지 않았습니다. 센서 데이터는 로그로만 남습니다.")
            return

        if not SQLALCHEMY_AVAILABLE:
            LOG.warning("SQLAlchemy를 찾을 수 없습니다. FARM_DB_URL=%s", db_url)
            return

        try:
            self.engine = create_engine(db_url, future=True, pool_pre_ping=True)
            Base.metadata.create_all(self.engine)  # type: ignore[union-attr]
            self._session_factory = sessionmaker(bind=self.engine, expire_on_commit=False, future=True)
            LOG.info("FarmDataLogger가 %s에 연결되었습니다.", db_url)
        except Exception as exc:  # noqa: BLE001
            LOG.error("FarmDataLogger 초기화 실패: %s", exc)
            self.engine = None
            self._session_factory = None

    def _store(self, row: object) -> bool:
        if not self.engine or not self._session_factory:
            LOG.debug("DB 연결이 없어 샘플을 저장하지 않았습니다: %s", row)
            return False

        session: Session = self._session_factory()
        try:
            session.add(row)  # type: ignore[arg-type]
            session.commit()
            return True
        except SQLAlchemyError as exc:  # pragma: no cover - DB 오류
            session.rollback()
            LOG.error("센서 샘플 저장 실패: %s", exc)
            return False
        finally:
            session.close()

    @staticmethod
    def _ts(captured_at: Optional[datetime]) -> datetime:
        return captured_at.astimezone(timezone.utc) if captured_at else datetime.now(timezone.utc)

    def log_air_sample(
        self,
        device_id: str,
        temperature_c: Optional[float],
        humidity_pct: Optional[float],
        air_quality: Optional[int],
        illuminance_lux: Optional[int],
        captured_at: Optional[datetime] = None,
    ) -> bool:
        if not SQLALCHEMY_AVAILABLE or not self.engine:
            LOG.info(
                "[농가-공기] device=%s temp=%s hum=%s air=%s lux=%s payload=%s",
                device_id,
                temperature_c,
                humidity_pct,
                air_quality,
                illuminance_lux,
                "(저장 안 함)",
            )
            return False

        row = FarmAirSample(
            device_id=device_id,
            captured_at=self._ts(captured_at),
            temperature_c=temperature_c,
            humidity_pct=humidity_pct,
            air_quality=air_quality,
            illuminance_lux=illuminance_lux,
        )
        return self._store(row)

    def log_land_sample(
        self,
        device_id: str,
        temperature_c: Optional[float],
        humidity_pct: Optional[float],
        ec: Optional[int],
        ph: Optional[float],
        captured_at: Optional[datetime] = None,
    ) -> bool:
        if not SQLALCHEMY_AVAILABLE or not self.engine:
            LOG.info(
                "[농가-토양] device=%s temp=%s hum=%s ec=%s ph=%s payload=%s",
                device_id,
                temperature_c,
                humidity_pct,
                ec,
                ph,
                "(저장 안 함)",
            )
            return False

        row = FarmLandSample(
            device_id=device_id,
            captured_at=self._ts(captured_at),
            soil_temperature_c=temperature_c,
            soil_humidity_pct=humidity_pct,
            soil_ec=ec,
            soil_ph=ph,
        )
        return self._store(row)

    def log_tomato_snapshot(
        self,
        device_id: str,
        ripe_count: int,
        rotten_count: int,
        captured_at: Optional[datetime] = None,
    ) -> bool:
        if not SQLALCHEMY_AVAILABLE or not self.engine:
            LOG.info(
                "[농가-이미지] device=%s ripe=%s rotten=%s",
                device_id,
                ripe_count,
                rotten_count,
            )
            return False

        row = FarmTomatoSnapshot(
            device_id=device_id,
            captured_at=self._ts(captured_at),
            ripe_count=int(ripe_count),
            rotten_count=int(rotten_count),
        )
        return self._store(row)
