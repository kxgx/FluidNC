#!/usr/bin/env python3
"""Package CAN firmware build artifacts into one structured zip per board.

Release layout (one zip per variant):

    <label>/
        README.txt
        OTA/
            firmware.bin
            firmware.addrinfo
            manifest.json
            README.txt
            SHA256SUMS
        USB/
            firmware.factory.bin
            bootloader.bin
            partitions.bin
            firmware.addrinfo
            manifest.json
            README.txt
            SHA256SUMS

The zip file itself is named fluidcan-<tag>-<label>.zip.
No loose firmware binaries are published as release assets.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import shutil
import tempfile
import zipfile
from pathlib import Path

VARIANTS = [
    {
        "label": "quad-oled",
        "env": "wifi_s3_oled",
        "flash": "8 MB",
        "psram": "Quad",
        "oled": "Yes",
        "usb": "No",
        "default": True,
    },
    {
        "label": "octal-oled",
        "env": "wifi_s3_octalPSRAM_oled",
        "flash": "8 MB",
        "psram": "Octal",
        "oled": "Yes",
        "usb": "No",
    },
    {
        "label": "quad",
        "env": "wifi_s3",
        "flash": "8 MB",
        "psram": "Quad",
        "oled": "No",
        "usb": "No",
    },
    {
        "label": "octal",
        "env": "wifi_s3_octalPSRAM",
        "flash": "8 MB",
        "psram": "Octal",
        "oled": "No",
        "usb": "No",
    },
    {
        "label": "quad-oled-usb",
        "env": "wifi_s3_oled_usb",
        "flash": "8 MB",
        "psram": "Quad",
        "oled": "Yes",
        "usb": "Yes",
    },
    {
        "label": "octal-oled-usb",
        "env": "wifi_s3_octalPSRAM_oled_usb",
        "flash": "8 MB",
        "psram": "Octal",
        "oled": "Yes",
        "usb": "Yes",
    },
    {
        "label": "quad-oled-usb-4mb",
        "env": "wifi_s3_oled_usb_4mb",
        "flash": "4 MB",
        "psram": "Quad",
        "oled": "Yes",
        "usb": "Yes",
    },
    {
        "label": "octal-oled-usb-4mb",
        "env": "wifi_s3_octalPSRAM_oled_usb_4mb",
        "flash": "4 MB",
        "psram": "Octal",
        "oled": "Yes",
        "usb": "Yes",
    },
    {
        "label": "nopsram-oled-usb-4mb",
        "env": "wifi_s3_oled_usb_4mb_nopsram",
        "flash": "4 MB",
        "psram": "None",
        "oled": "Yes",
        "usb": "Yes",
    },
]

BUILD_FILES = (
    "firmware.bin",
    "firmware.factory.bin",
    "bootloader.bin",
    "partitions.bin",
    "firmware.addrinfo",
)


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def image_info(path: Path, offset: str, manifest_dir: Path) -> dict:
    return {
        "size": path.stat().st_size,
        "offset": offset,
        "path": path.relative_to(manifest_dir).as_posix(),
        "signature": {"algorithm": "SHA2-256", "value": sha256(path)},
    }


def manifest(
    variant: dict,
    tag: str,
    repo: str,
    commit: str,
    image_name: str,
    image_path: Path,
    offset: str,
    install_name: str,
    install_description: str,
    erase: bool,
) -> dict:
    return {
        "name": f"FluidCAN {variant['label']}",
        "version": tag,
        "variant": variant["label"],
        "env": variant["env"],
        "board": {
            "flash": variant["flash"],
            "psram": variant["psram"],
            "oled": variant["oled"],
            "usb": variant["usb"],
        },
        "source_url": f"https://github.com/{repo}/tree/{commit}",
        "release_url": f"https://github.com/{repo}/releases/tag/{tag}",
        "images": {image_name: image_info(image_path, offset, image_path.parent)},
        "installable": {
            "name": "installable",
            "description": f"FluidCAN {tag} for {variant['label']}",
            "choice-name": "Processor type",
            "choices": [
                {
                    "name": "esp32s3",
                    "description": (
                        f"ESP32-S3 {variant['flash']} flash, "
                        f"{variant['psram']} PSRAM, OLED {variant['oled']}"
                    ),
                    "choice-name": "安装方式",
                    "choices": [
                        {
                            "name": install_name,
                            "description": install_description,
                            "erase": erase,
                            "images": [image_name],
                        }
                    ],
                }
            ],
        },
    }


def write_text(path: Path, text: str) -> None:
    path.write_text(text, encoding="utf-8")


def write_checksums(directory: Path, names: list[str]) -> None:
    lines = []
    for name in names:
        path = directory / name
        lines.append(f"{sha256(path)}  {name}")
    write_text(directory / "SHA256SUMS", "\n".join(lines) + "\n")


def stage_variant(
    variant: dict,
    source: Path,
    staging: Path,
    tag: str,
    repo: str,
    commit: str,
) -> None:
    label = variant["label"]
    root = staging / label
    ota = root / "OTA"
    usb = root / "USB"
    ota.mkdir(parents=True)
    usb.mkdir(parents=True)

    for name in BUILD_FILES:
        source_file = source / name
        if not source_file.is_file():
            raise FileNotFoundError(f"{label}: missing artifact {source_file}")

    shutil.copy2(source / "firmware.bin", ota / "firmware.bin")
    shutil.copy2(source / "firmware.addrinfo", ota / "firmware.addrinfo")
    for name in ("firmware.factory.bin", "bootloader.bin", "partitions.bin", "firmware.addrinfo"):
        shutil.copy2(source / name, usb / name)

    ota_manifest = manifest(
        variant,
        tag,
        repo,
        commit,
        image_name="fluidcan-app",
        image_path=ota / "firmware.bin",
        offset="0x10000",
        install_name="can-ota",
        install_description="升级应用区（0x10000），保留 bootloader/partitions",
        erase=False,
    )
    usb_manifest = manifest(
        variant,
        tag,
        repo,
        commit,
        image_name="fluidcan-factory",
        image_path=usb / "firmware.factory.bin",
        offset="0x0",
        install_name="can-full",
        install_description="完整写入 CAN 固件（factory image @ 0x0）",
        erase=True,
    )
    write_text(ota / "manifest.json", json.dumps(ota_manifest, ensure_ascii=False, indent=2) + "\n")
    write_text(usb / "manifest.json", json.dumps(usb_manifest, ensure_ascii=False, indent=2) + "\n")

    no_ota = variant["flash"] == "4 MB"
    ota_note = (
        "此 4 MB 变体只有单个 app 分区，不能做网络 OTA；请使用 USB/factory 刷写。"
        if no_ota
        else "使用 firmware.bin（0x10000）做 OTA；不要用 factory image 覆盖分区。"
    )
    write_text(
        ota / "README.txt",
        f"""FluidCAN {tag} - {label}

{ota_note}

OTA 文件：
  firmware.bin      应用镜像，写入偏移 0x10000
  manifest.json     安装器/OTA 元数据
  firmware.addrinfo 调试符号地址信息
  SHA256SUMS        文件校验

不要在此目录使用 firmware.factory.bin；那是完整 USB 刷写镜像。
""",
    )
    write_text(
        usb / "README.txt",
        f"""FluidCAN {tag} - {label}

USB 完整刷写文件：
  firmware.factory.bin  完整镜像，偏移 0x0
  bootloader.bin        偏移 0x1000
  partitions.bin        偏移 0x8000
  firmware.addrinfo     调试符号地址信息
  manifest.json         完整安装元数据
  SHA256SUMS            文件校验

ESP32-S3 参数：Flash {variant['flash']}, PSRAM {variant['psram']}, OLED {variant['oled']}, USB {variant['usb']}
""",
    )
    write_text(
        root / "README.txt",
        f"""FluidCAN {tag}

型号：{label}
构建环境：{variant['env']}
Flash：{variant['flash']}
PSRAM：{variant['psram']}
OLED：{variant['oled']}
USB：{variant['usb']}
默认变体：{'yes' if variant.get('default') else 'no'}

目录：
  OTA/  应用区升级
  USB/  完整 USB 刷写

完整说明见两个子目录中的 README.txt。
""",
    )
    write_checksums(ota, ["firmware.bin", "firmware.addrinfo", "manifest.json"])
    write_checksums(
        usb,
        ["firmware.factory.bin", "bootloader.bin", "partitions.bin", "firmware.addrinfo", "manifest.json"],
    )


def make_zip(staging: Path, label: str, output: Path) -> None:
    with zipfile.ZipFile(output, "w", compression=zipfile.ZIP_DEFLATED, compresslevel=9) as archive:
        root = staging / label
        for path in sorted(root.rglob("*")):
            if path.is_file():
                archive.write(path, path.relative_to(staging).as_posix())


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--artifacts", required=True, type=Path)
    parser.add_argument("--out", required=True, type=Path)
    parser.add_argument("--tag", required=True)
    parser.add_argument("--repo", required=True)
    parser.add_argument("--commit", required=True)
    parser.add_argument("--only", action="append", default=None, help="optional variant label")
    args = parser.parse_args()

    args.out.mkdir(parents=True, exist_ok=True)
    selected = [v for v in VARIANTS if not args.only or v["label"] in args.only]
    if not selected:
        raise SystemExit("no matching variants")

    with tempfile.TemporaryDirectory(prefix="fluidcan-release-") as temp:
        staging = Path(temp)
        for variant in selected:
            source = args.artifacts / f"firmware-{variant['label']}"
            stage_variant(variant, source, staging, args.tag, args.repo, args.commit)
            zip_path = args.out / f"fluidcan-{args.tag}-{variant['label']}.zip"
            make_zip(staging, variant["label"], zip_path)
            print(f"created {zip_path} ({zip_path.stat().st_size} bytes)")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
