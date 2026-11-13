import json
from typing import Any


def dumps(data: Any) -> str:
    return json.dumps(data, ensure_ascii=False, separators=(",", ":"))


def loads(text: str, fallback: Any = None) -> Any:
    try:
        return json.loads(text)
    except (TypeError, ValueError):
        return fallback

