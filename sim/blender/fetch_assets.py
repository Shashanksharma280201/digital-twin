"""Download optional CC0 textures (Poly Haven) into sim/assets/ (design §5, fetch_assets).

python3 sim/blender/fetch_assets.py [--res 1k]

common.material() picks up sim/assets/<id>_diff_<res>.jpg automatically when present;
the world generator works without them (procedural shaders). All assets are CC0.
"""
import argparse
import sys
import urllib.request
from pathlib import Path

ASSETS = Path(__file__).resolve().parent.parent / "assets"
TEXTURES = ["concrete_floor", "plywood", "rusty_metal_02", "gravel", "raked_dirt", "asphalt_02",
            "concrete_block_wall"]
URL = "https://dl.polyhaven.org/file/ph-assets/Textures/jpg/{res}/{id}/{id}_diff_{res}.jpg"
HEADERS = {"User-Agent": "dt-sim/1.0 (digital-twin construction sim)"}


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--res", default="1k", choices=["1k", "2k"])
    args = parser.parse_args()
    ASSETS.mkdir(parents=True, exist_ok=True)
    failed = 0
    for tex in TEXTURES:
        out = ASSETS / f"{tex}_diff_1k.jpg"  # common.py looks for the _1k name
        if out.exists():
            print(f"[fetch_assets] have {out.name}")
            continue
        url = URL.format(res=args.res, id=tex)
        try:
            with urllib.request.urlopen(urllib.request.Request(url, headers=HEADERS), timeout=60) as r:
                out.write_bytes(r.read())
            print(f"[fetch_assets] {url} -> {out}")
        except OSError as exc:
            failed += 1
            print(f"[fetch_assets] FAILED {url}: {exc}", file=sys.stderr)
    (ASSETS / "CREDITS.md").write_text(
        "# Asset credits\n\nTextures from Poly Haven (https://polyhaven.com), CC0. "
        "IDs: " + ", ".join(TEXTURES) + ".\n")
    sys.exit(1 if failed else 0)


if __name__ == "__main__":
    main()
