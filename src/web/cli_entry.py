"""`afc web` 子命令入口.

被 [src/app/cli.py](../app/cli.py) 在 dispatcher 里调用.
启动 uvicorn 单 worker 服务 FastAPI app.
"""

from __future__ import annotations

import argparse
from pathlib import Path

import uvicorn

from .app import WebConfig, create_app


def run(args: argparse.Namespace) -> int:
    config = WebConfig(
        telemetry_dir=Path(args.telemetry_dir).resolve(),
        artifacts_dir=Path(args.artifacts_dir).resolve(),
    )
    app = create_app(config)
    print(f"Serving on http://{args.host}:{args.port}")
    print(f"  telemetry_dir = {config.telemetry_dir}")
    print(f"  artifacts_dir = {config.artifacts_dir}")
    uvicorn.run(app, host=args.host, port=args.port, log_level="info")
    return 0
