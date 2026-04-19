"""FastAPI app factory.

PR-A 阶段只挂一个 health check 路由,D2 起补齐 RunIndex / Panels.
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

from fastapi import FastAPI
from fastapi.responses import JSONResponse


@dataclass(frozen=True)
class WebConfig:
    telemetry_dir: Path
    artifacts_dir: Path


def create_app(config: WebConfig) -> FastAPI:
    app = FastAPI(title="Crazyflie AFC Analysis", version="0.1.0")

    @app.get("/healthz")
    def healthz() -> JSONResponse:
        return JSONResponse(
            {
                "status": "ok",
                "telemetry_dir": str(config.telemetry_dir),
                "artifacts_dir": str(config.artifacts_dir),
            }
        )

    @app.get("/")
    def root() -> JSONResponse:
        return JSONResponse(
            {
                "message": "Crazyflie AFC Analysis Web",
                "stage": "PR-A D1 skeleton",
                "next": "D2 will add RunIndex + run_list page",
            }
        )

    return app
