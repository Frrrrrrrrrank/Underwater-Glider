#!/usr/bin/env python3
"""Open a latitude/longitude position in an online map."""

import argparse
import re
import sys
import webbrowser
from urllib.parse import urlencode


PAYLOAD_PATTERN = re.compile(
    r"LAT\s*=\s*([+-]?\d+(?:\.\d+)?)\s*[,; ]+\s*LON\s*=\s*([+-]?\d+(?:\.\d+)?)",
    re.IGNORECASE,
)


def parse_coordinates(text: str) -> tuple[float, float]:
    match = PAYLOAD_PATTERN.search(text)
    if match:
        latitude, longitude = map(float, match.groups())
    else:
        values = re.findall(r"[+-]?\d+(?:\.\d+)?", text)
        if len(values) != 2:
            raise ValueError(
                "请输入两个十进制度数，例如 22.12 114.12，"
                "或 LAT=+22.12,LON=+114.12"
            )
        latitude, longitude = map(float, values)

    if not -90.0 <= latitude <= 90.0:
        raise ValueError("纬度必须在 -90 到 90 之间")
    if not -180.0 <= longitude <= 180.0:
        raise ValueError("经度必须在 -180 到 180 之间")
    return latitude, longitude


def make_map_url(
    latitude: float, longitude: float, zoom: int, provider: str = "baidu"
) -> str:
    if provider == "baidu":
        query = urlencode(
            {
                "location": f"{latitude:.7f},{longitude:.7f}",
                "title": "北斗定位",
                "content": f"WGS84: {latitude:.7f}, {longitude:.7f}",
                "coord_type": "wgs84",
                "zoom": zoom,
                "output": "html",
                "src": "webapp.underwaterglider.location",
            }
        )
        return f"https://api.map.baidu.com/marker?{query}"

    query = urlencode({"mlat": latitude, "mlon": longitude})
    return (
        f"https://www.openstreetmap.org/?{query}"
        f"#map={zoom}/{latitude:.6f}/{longitude:.6f}"
    )


def open_location(
    latitude: float, longitude: float, zoom: int = 16, provider: str = "baidu"
) -> bool:
    url = make_map_url(latitude, longitude, zoom, provider)
    print(f"纬度: {latitude:.7f}")
    print(f"经度: {longitude:.7f}")
    print(f"地图: {url}")
    return webbrowser.open(url)


def main() -> int:
    parser = argparse.ArgumentParser(description="在在线地图中显示经纬度位置")
    parser.add_argument(
        "coordinates",
        nargs="*",
        help='经纬度，或固件文本 "LAT=+22.12,LON=+114.12"',
    )
    parser.add_argument("--zoom", type=int, default=16, help="地图缩放级别，默认 16")
    parser.add_argument(
        "--provider",
        choices=("baidu", "osm"),
        default="baidu",
        help="地图服务，默认 baidu",
    )
    args = parser.parse_args()

    if not 1 <= args.zoom <= 19:
        parser.error("--zoom 必须在 1 到 19 之间")

    text = " ".join(args.coordinates).strip()
    if not text:
        try:
            text = input("输入经纬度: ").strip()
        except (EOFError, KeyboardInterrupt):
            print()
            return 1

    try:
        latitude, longitude = parse_coordinates(text)
    except ValueError as error:
        print(f"输入错误: {error}", file=sys.stderr)
        return 2

    if not open_location(latitude, longitude, args.zoom, args.provider):
        print("未能自动打开浏览器，请手动打开上面的地址。", file=sys.stderr)
        return 3
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
