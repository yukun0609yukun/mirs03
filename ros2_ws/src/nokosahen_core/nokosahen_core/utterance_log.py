"""発話履歴(いつ・何と発話したか)をダッシュボードから参照するための読み取り専用リーダー。

書き込み側は`voice_bridge/voice_bridge.py`(ROS2パッケージの外、Jetsonの
ホストOS上に常駐する別プロセス)が担当する。同じSQLiteファイルパスを
それぞれのプロセスが個別に開くことで連携する(consumption_log.pyと同じ考え方)。
"""
from __future__ import annotations

import json
import logging
import os
import sqlite3
import threading
import urllib.parse
import urllib.request
from typing import Any, Dict, List, Optional

logger = logging.getLogger(__name__)


class UtteranceLogReader:
    """`utterances`テーブル(voice_bridge.py側が作成・書き込み)を読み取るだけの薄いラッパー。"""

    def __init__(
        self,
        db_path: str,
        remote_url: str = "",
        remote_timeout: float = 3.0,
        remote_limit: int = 1000,
    ) -> None:
        self.db_path = os.path.expanduser(db_path)
        # 【2026-10-02追加】voice_bridgeが別Jetsonで動く構成向け。空でなければ
        # voice_bridgeの GET /utterances から取得する(例 "http://203.0.113.9:8100")。
        # 空ならこれまで通りローカルのSQLiteファイルを読む。
        self.remote_url = (remote_url or "").strip().rstrip("/")
        self.remote_timeout = float(remote_timeout)
        self.remote_limit = int(remote_limit)
        self._lock = threading.Lock()

    def _fetch_remote(self, since: Optional[str]) -> List[Dict[str, Any]]:
        params = {"limit": str(self.remote_limit)}
        if since:
            params["since"] = since
        url = f"{self.remote_url}/utterances?{urllib.parse.urlencode(params)}"
        with urllib.request.urlopen(url, timeout=self.remote_timeout) as res:
            data = json.loads(res.read().decode("utf-8"))
        return [
            {"timestamp": u["timestamp"], "text": u["text"]}
            for u in data.get("utterances", [])
        ]

    def fetch_utterances_ex(self, since: Optional[str] = None) -> Dict[str, Any]:
        """発話履歴と、その取得元(source)・エラー情報をまとめて返す。

        【2026-10-02追加】リモート取得に失敗してローカルDB(旧データ)へフォールバック
        した場合に、画面側がそれと分かるようにするため。
          source: "remote"(voice_bridgeから取得) / "local-fallback"(リモート失敗で
                  ローカルDBへフォールバック) / "local"(リモート未設定でローカルDB)
          error : フォールバック時のみ、リモート取得失敗の理由。それ以外はNone
          remote_limit: リモート取得時の最大件数(超えた分の古い履歴は取得できない)
        """
        if self.remote_url:
            try:
                return {
                    "utterances": self._fetch_remote(since),
                    "source": "remote",
                    "error": None,
                    "remote_limit": self.remote_limit,
                }
            except Exception as exc:  # noqa: BLE001 - 失敗してもダッシュボードは落とさない
                logger.warning("発話履歴のリモート取得に失敗(%s): %s", self.remote_url, exc)
                # ローカルDBが存在すれば(旧データ等)そちらにフォールバックする。
                return {
                    "utterances": self._fetch_local(since),
                    "source": "local-fallback",
                    "error": f"{type(exc).__name__}: {exc}",
                    "remote_limit": self.remote_limit,
                }
        return {
            "utterances": self._fetch_local(since),
            "source": "local",
            "error": None,
            "remote_limit": None,
        }

    def fetch_utterances(self, since: Optional[str] = None) -> List[Dict[str, Any]]:
        return self.fetch_utterances_ex(since)["utterances"]

    def _fetch_local(self, since: Optional[str] = None) -> List[Dict[str, Any]]:
        if not os.path.exists(self.db_path):
            # voice_bridgeがまだ一度も発話していない(DBファイル未作成)場合は空を返す。
            return []
        query = "SELECT timestamp, text FROM utterances"
        params: tuple = ()
        if since:
            query += " WHERE timestamp >= ?"
            params = (since,)
        query += " ORDER BY timestamp ASC"
        with self._lock:
            conn = sqlite3.connect(self.db_path, timeout=5.0)
            try:
                rows = conn.execute(query, params).fetchall()
            except sqlite3.OperationalError:
                # テーブル未作成(起動直後等)なら空を返す。
                return []
            finally:
                conn.close()
        return [{"timestamp": row[0], "text": row[1]} for row in rows]
