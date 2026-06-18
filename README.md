# Paper Appendices and Instance Generation

This repository contains the computational code, benchmark instances, and paper appendices for the HMDVRRP study.

## Paper information

- **Title:** Bidirectional collaborative routing: synchronizing en-route transfers and robot-assisted delivery in heterogeneous multi-depot systems
- **Authors:** Tianyu Mao and Mingrui Yang (corresponding author)

The accompanying appendix is available as [Appendices.pdf](Appendices.pdf).

## Appendix contents

`Appendices.pdf` is a five-page supplement organized as follows:

- **Appendix A - Benchmark instances and parameter settings**
  - A.1 defines the benchmark construction and summarizes the instances in Table A1.
  - A.2 reports the default generation parameters in Table A2.
  - A.3 reports the ALNS configuration in Table A3.
- **Appendix B - Detailed computational results**
  - B.1 gives instance-level results for standard ALNS versus diversity-aware reheating in Table B4.
  - B.2 gives results for the no-horizontal-collaboration variant in Table B5.

The appendix labels an instance as `ED-N-K`, where `D` is the number of depots, `N` is the number of customers assigned to each depot, and `K` is the number of trucks at each depot. Each truck is paired with one robot. The JSON files use the corresponding filename format:

```text
M-dD-nN-kK-pP.json
```

For example, `E2-4-1` corresponds to `M-d2-n4-k1-p2.json` when two parking nodes are generated for each robot-only customer.

## Repository layout

```text
Appendices.pdf              Paper appendices
instance_generation/        Instance model, generator, redistribution utility, and CLI
instances/                  Generated benchmark JSON files
README.md                   Appendix and instance-generation guide
misc/                       Other solver code, scripts, docs, figures, and reference outputs
```

## Generate an instance

Run commands from the repository root. The full solving workflow depends on Python, Matplotlib, and Gurobi. Instance generation itself uses Python and Matplotlib.

Generate one JSON instance with the command-line interface:

```powershell
python -m instance_generation.cli M-d2-n4-k1-p2.json
```

Choose another output directory with `--output`:

```powershell
python -m instance_generation.cli M-d3-n10-k1-p2.json --output generated_instances
```

The generator uses the following filename fields:

| Field | Meaning |
| --- | --- |
| `dD` | Number of depots |
| `nN` | Customers assigned to each depot |
| `kK` | Trucks at each depot |
| `pP` | Parking nodes associated with each robot-only customer |

The same function can be called from Python:

```python
from instance_generation import generate_instance
from instance_generation.io import instance_save, parse_string

name = "M-d2-n4-k1-p2.json"
generated = generate_instance(parse_string(name))
instance_save("instances", generated)
```

Generation is deterministic because `instance_generation/instanceGenerate.py` sets the random seed to `1`. Default values in the code include a `100 x 100` region, parking-node distance range `[1, 5]`, customer demand range `[10, 30]`, truck capacity `300`, robot capacity `30`, robot battery capacity `3`, and robot energy consumption `0.1`.

## Important consistency note

The revised appendix defines the number of robot-only customers as `ceil(0.3 * D * N)`. The current generator code uses `floor(D * N / 3)`. These expressions differ for some instance sizes, such as `D=2, N=4`. The code was reorganized without changing this experimental behavior; align the formula before regenerating the appendix benchmark set if exact reproduction of Table A1 is required.

## Other bundled files

The previous solver runner, solver implementations, legacy scripts, model notes, figures, and reference outputs are bundled under `misc/`. They are kept for traceability, but the root-level documented entry point is the instance generator above.
