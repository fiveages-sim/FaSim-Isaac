#!/usr/bin/env python3
"""下载原版 ArtVIP 到 environment/ArtVIP。

场景组件在 environment/artvip-usd，本脚本不再做转换。
镜像在导入 huggingface_hub 之前设置。默认 https://hf-mirror.com。
环境变量 HF_ENDPOINT 或参数 --endpoint 已有值时不覆盖。

    python tools/artvip_kitchen.py download
"""

from __future__ import annotations

import argparse
import os
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_HF_ENDPOINT = "https://hf-mirror.com"
DATASET_ID = "X-Humanoid/ArtVIP"
OFFICIAL_DIR = REPO_ROOT / "environment" / "ArtVIP"


def configure_mirror(endpoint: str | None) -> str:
    if endpoint:
        os.environ["HF_ENDPOINT"] = endpoint.rstrip("/")
    elif not os.environ.get("HF_ENDPOINT"):
        os.environ["HF_ENDPOINT"] = DEFAULT_HF_ENDPOINT
    os.environ.setdefault("HF_HUB_DISABLE_XET", "1")
    os.environ.setdefault("HF_HUB_ENABLE_HF_TRANSFER", "0")
    return os.environ["HF_ENDPOINT"]


def _snapshot_download(snapshot_download, **kwargs) -> None:
    import time

    from huggingface_hub.errors import HfHubHTTPError

    for attempt in range(6):
        try:
            snapshot_download(
                repo_id=DATASET_ID,
                repo_type="dataset",
                local_dir=str(OFFICIAL_DIR),
                max_workers=2,
                **kwargs,
            )
            return
        except HfHubHTTPError as exc:
            if attempt == 5:
                raise
            wait = 8 * (attempt + 1)
            print(f"下载被镜像限流或拒绝 ({exc.__class__.__name__})，{wait}s 后重试 {attempt + 2}/6")
            time.sleep(wait)


def download() -> None:
    try:
        from huggingface_hub import snapshot_download
    except ImportError:
        import subprocess

        print("正在安装 huggingface_hub ...")
        subprocess.check_call([sys.executable, "-m", "pip", "install", "huggingface_hub"])
        from huggingface_hub import snapshot_download

    OFFICIAL_DIR.mkdir(parents=True, exist_ok=True)
    print(f"HF_ENDPOINT={os.environ.get('HF_ENDPOINT')}")
    print(f"下载 {DATASET_ID} -> {OFFICIAL_DIR}")
    ignored = ["**/.DS_Store", "**/.thumbs/**", "**/__MACOSX/**", "**/._*"]
    _snapshot_download(
        snapshot_download,
        allow_patterns=["Interactive_scene/**", "Articulated_objects/**", "README.md", ".gitattributes"],
        ignore_patterns=ignored,
    )
    _snapshot_download(snapshot_download, ignore_patterns=ignored)
    entry = OFFICIAL_DIR / "Interactive_scene" / "kitchen" / "Interactive_kitchen.usd"
    if not entry.is_file():
        raise SystemExit(f"下载结束但没有找到 {entry}")
    print(f"原版厨房入口: {entry}")


def main() -> None:
    parser = argparse.ArgumentParser(description="用 Hugging Face 镜像下载官方 ArtVIP")
    parser.add_argument("--endpoint", help="Hugging Face 端点。默认在未设置 HF_ENDPOINT 时用 https://hf-mirror.com")
    sub = parser.add_subparsers(dest="cmd", required=True)
    sub.add_parser("download", help="下载原版数据集到 environment/ArtVIP")
    args = parser.parse_args()
    endpoint = configure_mirror(args.endpoint)
    print(f"使用 Hugging Face 端点 {endpoint}")
    if args.cmd == "download":
        download()


if __name__ == "__main__":
    main()
