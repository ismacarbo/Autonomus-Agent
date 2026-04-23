from __future__ import annotations

import json
from dataclasses import dataclass
from pathlib import Path
from typing import Any


DEFAULT_PRESET = "unstructured"
PRESET_CONFIGS = {
    "unstructured": Path("data_analisys/unstructured_validation_config.json"),
    "structured_validation_road": Path("data_analisys/structured_validation_road_config.json"),
    "structured_figure_eight": Path("data_analisys/structured_figure_eight_config.json"),
}


@dataclass(frozen=True)
class ValidationPreset:
    name: str
    label: str
    description: str
    config_path: Path
    config: dict[str, Any]


def preset_names() -> list[str]:
    return list(PRESET_CONFIGS.keys())


def preset_config_path(name: str) -> Path:
    try:
        return PRESET_CONFIGS[name]
    except KeyError as exc:
        raise ValueError(f"Unknown validation preset: {name}") from exc


def load_preset_config(name: str) -> dict[str, Any]:
    path = preset_config_path(name)
    if not path.exists():
        raise FileNotFoundError(f"Validation preset config not found: {path}")
    return json.loads(path.read_text(encoding="utf-8"))


def list_presets() -> list[ValidationPreset]:
    presets: list[ValidationPreset] = []
    for name in preset_names():
        config_path = preset_config_path(name)
        config = load_preset_config(name)
        presets.append(
            ValidationPreset(
                name=name,
                label=str(config.get("label") or name),
                description=str(config.get("description") or ""),
                config_path=config_path,
                config=config,
            )
        )
    return presets
