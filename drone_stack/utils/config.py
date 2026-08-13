"""YAML configuration loading with layered overrides.

Resolution order (later wins):
    1. config/default.yaml           (all defaults, fully documented)
    2. the profile passed in         (config/sim.yaml or config/real.yaml)
    3. environment variables         (DRONE_<SECTION>_<KEY>=value)

No serial ports or tunables are hardcoded anywhere else in the codebase.
"""
from __future__ import annotations

import copy
import os
from pathlib import Path
from typing import Any

import yaml


def _repo_root() -> Path:
    # utils/config.py -> utils -> drone_stack -> <repo root>
    return Path(__file__).resolve().parents[2]


def default_config_path() -> Path:
    return _repo_root() / "config" / "default.yaml"


def _deep_merge(base: dict, override: dict) -> dict:
    """Recursively merge *override* into *base*, returning a new dict."""
    result = copy.deepcopy(base)
    for key, value in override.items():
        if (
            key in result
            and isinstance(result[key], dict)
            and isinstance(value, dict)
        ):
            result[key] = _deep_merge(result[key], value)
        else:
            result[key] = copy.deepcopy(value)
    return result


def _coerce(value: str, template: Any) -> Any:
    """Coerce an environment string to the type of the existing *template*."""
    if isinstance(template, bool):
        return value.strip().lower() in ("1", "true", "yes", "on")
    if isinstance(template, int) and not isinstance(template, bool):
        try:
            return int(value)
        except ValueError:
            return template
    if isinstance(template, float):
        try:
            return float(value)
        except ValueError:
            return template
    return value


class Config:
    """Read-only view over the merged configuration dictionary."""

    def __init__(self, data: dict[str, Any]) -> None:
        self._data = data

    # -- construction --------------------------------------------------------
    @classmethod
    def load(
        cls,
        profile_path: str | os.PathLike | None = None,
        default_path: str | os.PathLike | None = None,
        env_prefix: str = "DRONE_",
    ) -> "Config":
        default_path = Path(default_path) if default_path else default_config_path()
        data = _load_yaml(default_path)

        if profile_path is not None:
            profile = _load_yaml(Path(profile_path))
            data = _deep_merge(data, profile)

        data = cls._apply_env(data, env_prefix)
        return cls(data)

    @staticmethod
    def _apply_env(data: dict, prefix: str) -> dict:
        result = copy.deepcopy(data)
        for env_key, env_val in os.environ.items():
            if not env_key.startswith(prefix):
                continue
            remainder = env_key[len(prefix):].lower()
            section, _, key = remainder.partition("_")
            if not key:
                continue
            if section in result and isinstance(result[section], dict) and key in result[section]:
                result[section][key] = _coerce(env_val, result[section][key])
        return result

    # -- access --------------------------------------------------------------
    def section(self, name: str) -> dict[str, Any]:
        """Return a section dict (never ``None``)."""
        value = self._data.get(name, {})
        return value if isinstance(value, dict) else {}

    def get(self, dotted: str, default: Any = None) -> Any:
        """Look up a value by dotted path, e.g. ``"mavlink.connection"``."""
        node: Any = self._data
        for part in dotted.split("."):
            if isinstance(node, dict) and part in node:
                node = node[part]
            else:
                return default
        return node

    def __getitem__(self, key: str) -> Any:
        return self._data[key]

    def __contains__(self, key: str) -> bool:
        return key in self._data

    @property
    def mode(self) -> str:
        return str(self._data.get("mode", "sim")).lower()

    @property
    def raw(self) -> dict[str, Any]:
        return copy.deepcopy(self._data)

    def with_overrides(self, **top_level: Any) -> "Config":
        """Return a copy with top-level keys replaced (e.g. ``mode='real'``)."""
        data = copy.deepcopy(self._data)
        data.update(top_level)
        return Config(data)

    def __repr__(self) -> str:
        return f"Config(mode={self.mode!r}, sections={sorted(self._data)})"


def _load_yaml(path: Path) -> dict[str, Any]:
    if not path.exists():
        raise FileNotFoundError(f"config file not found: {path}")
    with path.open("r", encoding="utf-8") as handle:
        loaded = yaml.safe_load(handle) or {}
    if not isinstance(loaded, dict):
        raise ValueError(f"config file {path} must contain a mapping at the top level")
    return loaded
