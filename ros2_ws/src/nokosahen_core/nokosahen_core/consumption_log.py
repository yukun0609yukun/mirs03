"""カメラ/手動によるメニュー消費(count_event)の履歴をSQLiteへ永続化する。

いつ・どのメニューが・(カメラ/手動どちらの検知で)何食減った/戻ったかを
時系列で追跡し、ダッシュボードのグラフ表示から参照できるようにする。
既存の`nokosahen_events.jsonl`(全種別を含む簡易監査ログ、追記のみで検索・
集計には不向き)とは別に、count_eventだけを対象にした検索・集計しやすい
専用ストアとして追加した(2026-07-22)。

`inventory_manager`ノード(書き込み)と、ダッシュボードノード
(`full_dashboard_node`、読み取り専用)はそれぞれ別プロセスなので、
Pythonオブジェクトを共有できない。同じSQLiteファイルパスをそれぞれが
個別に開くことで連携する(SQLiteはファイルロックにより複数プロセスからの
同時アクセスに対応できるため、この用途には十分)。
"""
from __future__ import annotations

import contextlib
import os
import sqlite3
import threading
from typing import Any, Dict, Iterator, List, Optional


class ConsumptionLog:
    """`consumption_events`テーブルへの書き込み・検索を行う薄いラッパー。

    呼び出しの都度、短命な接続を開いて閉じる(常駐接続を持たない)。書き込み
    頻度(数十食分の会計イベント程度)・読み取り頻度(グラフページを開いた
    ときだけ)ともに低いため、接続の使い回しによる複雑さは不要と判断した。
    """

    def __init__(self, db_path: str) -> None:
        self.db_path = os.path.expanduser(db_path)
        parent = os.path.dirname(self.db_path)
        if parent:
            os.makedirs(parent, exist_ok=True)
        self._lock = threading.Lock()
        self._init_schema()

    def _connect(self) -> sqlite3.Connection:
        conn = sqlite3.connect(self.db_path, timeout=5.0)
        # WAL: 書き込み中でも読み取り側(ダッシュボード)がブロックされないようにする。
        conn.execute("PRAGMA journal_mode=WAL")
        return conn

    @contextlib.contextmanager
    def _conn(self) -> Iterator[sqlite3.Connection]:
        """接続を開き、コミット/ロールバック後に必ずcloseする。

        【2026-08-06修正】`with self._connect() as conn:`だけではsqlite3の
        トランザクション管理(コミット/ロールバック)しか行われず、接続は
        closeされない。docstringの「短命な接続を開いて閉じる」を実際に
        保証するための修正(engagement_log.py/detection_log.pyと同じ)。
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
                CREATE TABLE IF NOT EXISTS consumption_events (
                    id INTEGER PRIMARY KEY AUTOINCREMENT,
                    timestamp TEXT NOT NULL,
                    menu TEXT NOT NULL,
                    delta INTEGER NOT NULL,
                    source TEXT NOT NULL,
                    remaining_after INTEGER NOT NULL,
                    sold_after INTEGER NOT NULL
                )
                """
            )
            conn.execute(
                "CREATE INDEX IF NOT EXISTS idx_consumption_events_timestamp "
                "ON consumption_events(timestamp)"
            )
            # 【2026-09-25追加】tablet_ocr_reader.pyがOCR連続失敗時にTrendPredictor
            # の外挿予測値を送ってくることがあるため、実測値と区別して記録する。
            # 既存DBに対する後方互換のマイグレーション(ALTER TABLE)として追加。
            existing_columns = {
                row[1] for row in conn.execute("PRAGMA table_info(consumption_events)").fetchall()
            }
            if "estimated" not in existing_columns:
                conn.execute(
                    "ALTER TABLE consumption_events ADD COLUMN estimated INTEGER NOT NULL DEFAULT 0"
                )

    def record(
        self,
        timestamp: str,
        menu: str,
        delta: int,
        source: str,
        remaining_after: int,
        sold_after: int,
        estimated: bool = False,
    ) -> None:
        """1件のcount_event(カメラ検知・手動+1/-1どちらも)を記録する。

        deltaは正(消費)・負(訂正による戻し)どちらもそのまま記録し、
        カメラ由来かどうかは`source`(例: "checkout_cam_xxx" / "web_ui")で
        判別できるようにする(グラフ側でフィルタする)。estimatedは実測では
        なく予測値による記録であることを示す(2026-09-25追加)。
        """
        with self._lock, self._conn() as conn:
            conn.execute(
                "INSERT INTO consumption_events "
                "(timestamp, menu, delta, source, remaining_after, sold_after, estimated) "
                "VALUES (?, ?, ?, ?, ?, ?, ?)",
                (timestamp, menu, delta, source, remaining_after, sold_after, int(estimated)),
            )

    def fetch_events(self, since: Optional[str] = None) -> List[Dict[str, Any]]:
        """timestamp昇順で記録を返す。sinceを渡すとそれ以降(ISO8601文字列比較)だけに絞る。"""
        query = (
            "SELECT timestamp, menu, delta, source, remaining_after, sold_after, estimated "
            "FROM consumption_events"
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
                "menu": row[1],
                "delta": row[2],
                "source": row[3],
                "remaining_after": row[4],
                "sold_after": row[5],
                "estimated": bool(row[6]),
            }
            for row in rows
        ]
