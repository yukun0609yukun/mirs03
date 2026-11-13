"""接客(人物への接近・案内)の履歴と、A/Bテスト(推奨機能ON/OFF)の
切り替え履歴をSQLiteへ永続化する。

`behavior_director`(書き込み)と、ダッシュボードノード(読み取り専用、
`full_dashboard_node.py`)はそれぞれ別プロセスなので、同じSQLiteファイルを
個別に開くことで連携する(consumption_log.py/utterance_log.pyと同じ考え方)。

- `engagements`テーブル: 1件 = 「人物に接近して発話し終えた」1回分。
  `promoted_menu`はその時おすすめしていたメニュー名(induction phaseでなければNULL)。
  推奨効果分析(いつ・誰に・何を勧めて・それが売上に繋がったか)と、
  来客数(接客回数)・反応率の算出に使う。
- `recommend_mode_changes`テーブル: A/Bテストモード(推奨発話のON/OFF)を
  切り替えた履歴。ある時刻に推奨がON/OFFどちらだったかは、その時刻以前で
  一番新しい変更を見れば分かる。
"""
from __future__ import annotations

import contextlib
import os
import sqlite3
import threading
from typing import Any, Dict, Iterator, List, Optional


class EngagementLog:
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
        """接続を開き、コミット/ロールバック後に必ずcloseする。

        【2026-08-06修正】以前は`with self._connect() as conn:`だけを
        使っていたが、sqlite3.Connectionのcontext managerはトランザクション
        管理(コミット/ロールバック)のみを行い、接続自体はcloseしない
        (utterance_log.pyだけがtry/finally+close()で正しく実装されていた)。
        呼び出しの都度、短命な接続を開いて閉じる設計なのに実際は閉じられて
        おらず、fdリークの原因になっていた。
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
                CREATE TABLE IF NOT EXISTS engagements (
                    id INTEGER PRIMARY KEY AUTOINCREMENT,
                    timestamp TEXT NOT NULL,
                    promoted_menu TEXT,
                    recommend_enabled INTEGER NOT NULL
                )
                """
            )
            conn.execute(
                "CREATE INDEX IF NOT EXISTS idx_engagements_timestamp ON engagements(timestamp)"
            )
            conn.execute(
                """
                CREATE TABLE IF NOT EXISTS recommend_mode_changes (
                    id INTEGER PRIMARY KEY AUTOINCREMENT,
                    timestamp TEXT NOT NULL,
                    enabled INTEGER NOT NULL
                )
                """
            )
            conn.execute(
                "CREATE INDEX IF NOT EXISTS idx_mode_changes_timestamp "
                "ON recommend_mode_changes(timestamp)"
            )

    def record_engagement(
        self, timestamp: str, promoted_menu: Optional[str], recommend_enabled: bool
    ) -> None:
        with self._lock, self._conn() as conn:
            conn.execute(
                "INSERT INTO engagements (timestamp, promoted_menu, recommend_enabled) "
                "VALUES (?, ?, ?)",
                (timestamp, promoted_menu, 1 if recommend_enabled else 0),
            )

    def record_mode_change(self, timestamp: str, enabled: bool) -> None:
        with self._lock, self._conn() as conn:
            conn.execute(
                "INSERT INTO recommend_mode_changes (timestamp, enabled) VALUES (?, ?)",
                (timestamp, 1 if enabled else 0),
            )

    def fetch_engagements(self, since: Optional[str] = None) -> List[Dict[str, Any]]:
        query = "SELECT timestamp, promoted_menu, recommend_enabled FROM engagements"
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
                "promoted_menu": row[1],
                "recommend_enabled": bool(row[2]),
            }
            for row in rows
        ]

    def fetch_mode_changes(self, since: Optional[str] = None) -> List[Dict[str, Any]]:
        query = "SELECT timestamp, enabled FROM recommend_mode_changes"
        params: tuple = ()
        if since:
            query += " WHERE timestamp >= ?"
            params = (since,)
        query += " ORDER BY timestamp ASC"
        with self._lock, self._conn() as conn:
            rows = conn.execute(query, params).fetchall()
        return [{"timestamp": row[0], "enabled": bool(row[1])} for row in rows]
