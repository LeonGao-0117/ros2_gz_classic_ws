#!/usr/bin/env python3
import json
import os
import tempfile
from typing import Dict


def _default_table() -> Dict[str, dict]:
    return {'waypoints': {}}


def _normalize_table(data) -> Dict[str, dict]:
    if not isinstance(data, dict):
        return _default_table()

    waypoints = data.get('waypoints')
    if not isinstance(waypoints, dict):
        return _default_table()

    return {'waypoints': waypoints}


def load_waypoints(file_path: str) -> Dict[str, dict]:
    resolved = os.path.abspath(file_path)
    if not os.path.exists(resolved):
        return _default_table()

    try:
        with open(resolved, 'r', encoding='utf-8') as f:
            data = json.load(f)
    except (OSError, json.JSONDecodeError):
        return _default_table()

    return _normalize_table(data)


def write_waypoints(file_path: str, data: Dict[str, dict]) -> None:
    resolved = os.path.abspath(file_path)
    directory = os.path.dirname(resolved)
    if directory:
        os.makedirs(directory, exist_ok=True)

    normalized = _normalize_table(data)

    fd, tmp_path = tempfile.mkstemp(prefix='.waypoints.', suffix='.json.tmp', dir=directory or None)
    try:
        with os.fdopen(fd, 'w', encoding='utf-8') as f:
            json.dump(normalized, f, ensure_ascii=False, indent=2)
            f.flush()
            os.fsync(f.fileno())

        os.replace(tmp_path, resolved)
    finally:
        if os.path.exists(tmp_path):
            try:
                os.remove(tmp_path)
            except OSError:
                pass
