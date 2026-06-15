"""Instance construction support for the HMDVRRP benchmarks."""

from .instance import instance


def generate_instance(info):
    from .instanceGenerate import generate_instance as _generate_instance

    return _generate_instance(info)


def plot_points(*args, **kwargs):
    from .instanceGenerate import plot_points as _plot_points

    return _plot_points(*args, **kwargs)


__all__ = ["generate_instance", "instance", "plot_points"]
