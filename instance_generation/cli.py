"""Command-line entry point for generating one benchmark instance."""

import argparse
from pathlib import Path

from .instanceGenerate import generate_instance
from .io import instance_save, parse_string


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Generate one HMDVRRP benchmark instance as JSON."
    )
    parser.add_argument(
        "instance_name",
        help="Instance filename, for example M-d2-n4-k1-p2.json.",
    )
    parser.add_argument(
        "--output",
        default="instances",
        help="Output directory (default: instances).",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    info = parse_string(args.instance_name)
    required = {
        "depot_num",
        "customer_number_each_depot",
        "vehicle_number_each_depot",
        "parking_point_num",
    }
    missing = sorted(required.difference(info))
    if missing:
        raise SystemExit(
            "Invalid instance name. Expected M-dD-nN-kK-pP.json; "
            f"missing fields: {', '.join(missing)}"
        )

    output_dir = Path(args.output)
    output_dir.mkdir(parents=True, exist_ok=True)
    generated = generate_instance(info)
    instance_save(str(output_dir), generated)
    print(output_dir / f"{generated.name}.json")


if __name__ == "__main__":
    main()
