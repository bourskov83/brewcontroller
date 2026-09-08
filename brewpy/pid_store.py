# pid_store.py
from __future__ import annotations

import os
import math
import tempfile
import threading
from dataclasses import dataclass, asdict
from pathlib import Path
from typing import Dict, Optional

import yaml  # PyYAML

# --- Simple PID params container ---
@dataclass
class PidParams:
    p: float
    i: float
    d: float

    def validate(self) -> None:
        for name, val in [("p", self.p), ("i", self.i), ("d", self.d)]:
            if not isinstance(val, (int, float)):
                raise ValueError(f"{name} must be a number, got {type(val)}")
            if not math.isfinite(val):
                raise ValueError(f"{name} must be finite, got {val!r}")

    @classmethod
    def from_mapping(cls, m: dict) -> "PidParams":
        # Treat missing fields as errors to avoid silent surprises
        try:
            p = float(m["p"])
            i = float(m["i"])
            d = float(m["d"])
        except KeyError as e:
            raise ValueError(f"Missing PID field: {e}") from e
        pp = cls(p=p, i=i, d=d)
        pp.validate()
        return pp

    def to_mapping(self) -> dict:
        # Optionally normalize precision here if you want stable diffs
        return {"p": float(self.p), "i": float(self.i), "d": float(self.d)}


class PidParamsStore:
    """
    Thread-safe PID parameter store with YAML load/save and dirty tracking.
    YAML layout: top-level keys are loop names -> {p,i,d}.
    """

    def __init__(self, yaml_path: Path):
        self._path = Path(yaml_path)
        self._lock = threading.RLock()
        self._params: Dict[str, PidParams] = {}
        self._dirty = False

    # ---------- Public API ----------

    @property
    def dirty(self) -> bool:
        with self._lock:
            return self._dirty

    def load_from_disk(self, create_if_missing: Optional[Dict[str, PidParams]] = None) -> None:
        """
        Load YAML file. If missing and create_if_missing provided, initializes with those defaults
        and saves immediately.
        """
        with self._lock:
            if not self._path.exists():
                if create_if_missing is not None:
                    self._params = {k: v for k, v in create_if_missing.items()}
                    # Save the default file atomically
                    self._save_locked_atomic()
                    self._dirty = False
                    return
                else:
                    raise FileNotFoundError(f"{self._path} does not exist")

            try:
                data = yaml.safe_load(self._path.read_text(encoding="utf-8"))
            except Exception as e:
                raise RuntimeError(f"Failed to read YAML {self._path}: {e}") from e

            if data is None:
                data = {}

            if not isinstance(data, dict):
                raise ValueError(f"Top-level YAML must be a mapping, got {type(data)}")

            parsed: Dict[str, PidParams] = {}
            for loop_name, mapping in data.items():
                if not isinstance(mapping, dict):
                    raise ValueError(f"Loop '{loop_name}' must be a mapping, got {type(mapping)}")
                parsed[loop_name] = PidParams.from_mapping(mapping)

            self._params = parsed
            self._dirty = False

    def get(self, loop_name: str) -> PidParams:
        with self._lock:
            if loop_name not in self._params:
                raise KeyError(f"Unknown PID loop '{loop_name}'")
            # Return a copy to avoid external mutation
            p = self._params[loop_name]
            return PidParams(p=p.p, i=p.i, d=p.d)

    def set(self, loop_name: str, p: Optional[float] = None, i: Optional[float] = None,
            d: Optional[float] = None) -> PidParams:
        """
        Update one or more fields for the loop. Creates the loop if missing.
        Marks store dirty if values change.
        """
        with self._lock:
            existing = self._params.get(loop_name, PidParams(p=0.0, i=0.0, d=0.0))
            new = PidParams(
                p=float(p if p is not None else existing.p),
                i=float(i if i is not None else existing.i),
                d=float(d if d is not None else existing.d),
            )
            new.validate()

            if (new.p != existing.p) or (new.i != existing.i) or (new.d != existing.d):
                self._params[loop_name] = new
                self._dirty = True

            return PidParams(p=new.p, i=new.i, d=new.d)

    def set_bulk(self, updates: Dict[str, Dict[str, float]]) -> None:
        """
        Apply multiple updates at once: updates = {"mlt":{"p":1.0}, "hlt":{"i":22.0, "d":0.7}}
        """
        with self._lock:
            changed = False
            for loop_name, fields in updates.items():
                existing = self._params.get(loop_name, PidParams(p=0.0, i=0.0, d=0.0))
                new = PidParams(
                    p=float(fields.get("p", existing.p)),
                    i=float(fields.get("i", existing.i)),
                    d=float(fields.get("d", existing.d)),
                )
                new.validate()
                if (new.p != existing.p) or (new.i != existing.i) or (new.d != existing.d):
                    self._params[loop_name] = new
                    changed = True
            if changed:
                self._dirty = True

    def to_dict(self) -> Dict[str, Dict[str, float]]:
        with self._lock:
            return {k: v.to_mapping() for k, v in self._params.items()}

    def save_to_disk_atomic(self, only_if_dirty: bool = True) -> bool:
        """
        Atomically write YAML (tmp file + os.replace). Returns True if saved, False if skipped.
        """
        with self._lock:
            if only_if_dirty and not self._dirty:
                return False
            self._save_locked_atomic()
            self._dirty = False
            return True

    # ---------- Internal helpers ----------

    def _save_locked_atomic(self) -> None:
        data = self.to_dict()
        yaml_text = yaml.safe_dump(
            data,
            default_flow_style=False,
            sort_keys=True,
            allow_unicode=True
        )

        tmp_dir = self._path.parent
        tmp_fd, tmp_name = tempfile.mkstemp(prefix=self._path.name + ".", dir=tmp_dir)
        try:
            with os.fdopen(tmp_fd, "w", encoding="utf-8") as f:
                f.write(yaml_text)
                f.flush()
                os.fsync(f.fileno())  # ensure contents hit disk
            # Replace atomically
            os.replace(tmp_name, self._path)
        finally:
            # If something went wrong before replace, ensure tmp is gone
            try:
                if os.path.exists(tmp_name):
                    os.remove(tmp_name)
            except Exception:
                pass