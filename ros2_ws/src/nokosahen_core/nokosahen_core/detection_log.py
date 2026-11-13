"""本番環境でしか収集できない、LiDAR/カメラによる人物検知の生ログをSQLiteへ
永続化する(2026-07-24、明電舎でのPoC現地検証当日にユーザー要望で追加)。

`person_target_detector`(書き込み)と、ダッシュボードノード(読み取り専用)は
それぞれ別プロセスなので、同じSQLiteファイルを個別に開くことで連携する
(consumption_log.py/utterance_log.py/engagement_log.pyと同じ考え方)。

1件 = 「新しく人物を検知した瞬間」(同一人物を追跡し続けている間は重複して
記録しない。person_target_detector.py側のID追跡ロジックを使って、追跡が
途切れて新規に選び直された時だけ記録する)。
"""
from __future__ import annotations

import contextlib
import os
import sqlite3
import threading
from typing import Any, Dict, Iterator, List, Optional


class DetectionLog:
    def __init__(self, db_path: str) -> None:
        self.db_path = os.path.expanduser(db_path)
        parent = os.path.dirname(self.db_path)
        if parent:
            os.makedirs(parent, exist_ok=True)
        self._lock = threading.Lock()
        self._init_schema()

    def _connect(self) -> sqlite3.Connection:
        conn = sqlite3.connect(self.db_path, timeout=5.0)
        conn.execute("PRAGMA journal_mode=WAL")
        return conn

    @contextlib.contextmanager
    def _conn(self) -> Iterator[sqlite3.Connection]:
        """接続を開き、コミット/ロールバック後に必ずcloseする(2026-08-06修正、
        consumption_log.py/engagement_log.pyと同じ理由)。
        """
        conn = self._connect()
        try:
            yield conn
        except Exception:
            conn.rollback()
            raise
        else:
            conn.commit()
        finally:
            conn.close()

    def _init_schema(self) -> None:
        with self._lock, self._conn() as conn:
            conn.execute(
                """
                CREATE TABLE IF NOT EXISTS detections (
                    id INTEGER PRIMARY KEY AUTOINCREMENT,
                    timestamp TEXT NOT NULL,
                    distance_m REAL,
                    camera_confirmed INTEGER NOT NULL,
                    cluster_count INTEGER NOT NULL
                )
                """
            )
            conn.execute(
                "CREATE INDEX IF NOT EXISTS idx_detections_timestamp ON detections(timestamp)"
            )

    def record(
        self,
        timestamp: str,
        distance_m: Optional[float],
        camera_confirmed: bool,
        cluster_count: int,
    ) -> None:
        with self._lock, self._conn() as conn:
            conn.execute(
                "INSERT INTO detections (timestamp, distance_m, camera_confirmed, cluster_count) "
                "VALUES (?, ?, ?, ?)",
                (timestamp, distance_m, 1 if camera_confirmed else 0, cluster_count),
            )

    def fetch_detections(self, since: Optional[str] = None) -> List[Dict[str, Any]]:
        query = (
            "SELECT timestamp, distance_m, camera_confirmed, cluster_count FROM detections"
        )
        params: tuple = ()
        if since:
            query += " WHERE timestamp >= ?"
            params = (since,)
        query += " ORDER BY timestamp ASC"
        with self._lock, self._conn() as conn:
            rows = conn.execute(query, params).fetchall()
        return [
            {
                "timestamp": row[0],
                "distance_m": row[1],
                "camera_confirmed": bool(row[2]),
                "cluster_count": row[3],
            }
            for row in rows
        ]
