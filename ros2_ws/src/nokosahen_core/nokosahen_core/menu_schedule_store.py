"""メニュー事前登録(2週間分程度)をSQLiteで管理する共有ストア。

`menu_schedule_node.py`(専用の登録サイト、管理者向け)と`web_ui_node.py`
(日次の食数入力画面)の両方から同じDBファイルを読み書きする。ROS2の
トピック経由ではなく直接ファイルを共有する設計にしているのは、
どちらのノードも同じマシン(本体)上で動く前提のため、単純なSQLiteの方が
無駄なプロセス間通信を増やさずに済むから。

日付は常に"YYYY-MM-DD"のISO形式文字列で扱う。
"""
from __future__ import annotations

import sqlite3
import threading
from typing import Dict, List, Optional


class MenuScheduleStore:
    def __init__(self, db_path: str) -> None:
        self._db_path = db_path
        self._lock = threading.Lock()
        with self._connect() as conn:
            conn.execute(
                """
                CREATE TABLE IF NOT EXISTS menu_schedule (
                    id INTEGER PRIMARY KEY AUTOINCREMENT,
                    date TEXT NOT NULL,
                    name TEXT NOT NULL,
                    initial INTEGER NOT NULL DEFAULT 0
                )
                """
            )
            conn.execute(
                "CREATE INDEX IF NOT EXISTS idx_menu_schedule_date ON menu_schedule(date)"
            )
            conn.commit()

    def _connect(self) -> sqlite3.Connection:
        # 複数ノード(専用サイト+日次入力画面)から同じファイルへ同時アクセス
        # されうるため、ロック待ちのタイムアウトを長めに取っておく。
        return sqlite3.connect(self._db_path, timeout=10.0)

    def get_menu_for_date(self, date: str) -> List[Dict[str, object]]:
        with self._lock, self._connect() as conn:
            rows = conn.execute(
                "SELECT name, initial FROM menu_schedule WHERE date = ? ORDER BY id",
                (date,),
            ).fetchall()
        return [{"name": name, "initial": initial} for name, initial in rows]

    def set_menu_for_date(self, date: str, items: List[Dict[str, object]]) -> None:
        """指定日のメニューを丸ごと置き換える(既存分は削除してから入れ直す)。"""
        with self._lock, self._connect() as conn:
            conn.execute("DELETE FROM menu_schedule WHERE date = ?", (date,))
            for item in items:
                name = str(item.get("name", "")).strip()
                if not name:
                    continue
                try:
                    initial = max(0, int(item.get("initial", 0)))
                except (TypeError, ValueError):
                    initial = 0
                conn.execute(
                    "INSERT INTO menu_schedule (date, name, initial) VALUES (?, ?, ?)",
                    (date, name, initial),
                )
            conn.commit()

    def list_range(self, start_date: str, end_date: str) -> Dict[str, List[Dict[str, object]]]:
        with self._lock, self._connect() as conn:
            rows = conn.execute(
                "SELECT date, name, initial FROM menu_schedule "
                "WHERE date >= ? AND date <= ? ORDER BY date, id",
                (start_date, end_date),
            ).fetchall()
        out: Dict[str, List[Dict[str, object]]] = {}
        for date, name, initial in rows:
            out.setdefault(date, []).append({"name": name, "initial": initial})
        return out

    def delete_date(self, date: str) -> None:
        with self._lock, self._connect() as conn:
            conn.execute("DELETE FROM menu_schedule WHERE date = ?", (date,))
            conn.commit()
