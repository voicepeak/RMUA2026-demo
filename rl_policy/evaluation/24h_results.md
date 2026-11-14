# 24-hour training and evaluation results

All numbers below are from the deterministic evaluation protocol used by the
pipeline: mean action, 1024 parallel environments, 600 steps per checkpoint
unless the row says otherwise. `survival` means reaching the 150-step episode
limit without collision or out-of-bound. It is not an official simulator score.

## Baseline

| checkpoint | protocol | survival | path error |
| --- | --- | --- | --- |
| `model_11100` | 1024 envs | 69.2% | 0.699m |
| `model_11100` | 1024 envs, later re-eval | 70.3% | 0.699m |
| `model_11100` | 64 envs | 67.0% | 0.698m |

The 70.3% figure is the same original `model_11100`, re-evaluated; it is not a
new fine-tuned checkpoint. The global-best record was updated to this copy.

## Source repository baseline

The source `v12.onnx` scored **76.6%** at obstacle 2 with the 64-env protocol
(282 episodes). Its 1024-env evaluation was not completed because the laptop GPU
was occupied by training. The source `v12.onnx` has no training-step or accuracy
metadata.

## Strong-penalty continuation (14100 -> 16099)

The continuation used the original strong settings: `p_center=1.5`, `lat_v=0.5`,
no clearance term, `entropy_coef=0.01`, obstacle 2.

| checkpoint | survival | pos_xy_err | min_clearance |
| --- | --- | --- | --- |
| 14100 | 59.5% | 0.438 | 0.898 |
| 14400 | 59.6% | 0.434 | 0.982 |
| 14650 | 60.7% | 0.410 | 0.863 |
| 14950 | 56.2% | 0.396 | 0.810 |
| 15250 | 60.0% | 0.429 | 0.893 |
| **15550** | **61.8%** | 0.431 | 0.923 |
| 15800 | 59.7% | 0.392 | 0.868 |
| 16099 | 57.9% | 0.492 | 0.912 |

At obstacle 3, `15550` reached 45.3%. The 13000 -> 14000 rise continued briefly
to 15550, then plateaued and fell. It never approached the `model_11100` baseline.

## Supervisor cycles after the continuation

| cycle | variant | best survival | note |
| --- | --- | --- | --- |
| 0 | 1 (moderate, obstacle 3) | 69.1% | did not beat global best |
| 1 | 2 (moderate, obstacle 2) | 70.3% | best checkpoint was the initial `model_11100` |
| 2 | 0 (moderate, obstacle 2) | stopped mid-Phase B | A best 67.4%, initial `model_11100` |

Cycle 1 fine-tuned checkpoints (`11550` - `14099`) had `56-66%` survival but
better path error (`0.47-0.56m`). The trade-off is consistent across cycles:
more offset/lateral pressure improves path precision and reduces safety margin.

## Conclusion

- No fine-tuned checkpoint beat `model_11100` on survival.
- Path precision improved, but survival and minimum clearance decreased.
- The strong continuation did not break the plateau; `15550` is the best strong
  checkpoint at 61.8%, and it is kept only as a reference.
- The source `v12.onnx` remains the strongest reference at 76.6% (small-batch
  protocol).
- The 24h run used batch 1024, while the original config uses 4096: about 4x
  fewer samples per iteration. This is the first thing to restore on a
  high-compute machine.

## Files

- `cycle0.csv`, `cycle1.csv`: sanitized per-cycle evaluation rows
- `strong_resume.csv`: all strong-continuation evaluation rows
- `global_best.json`: global-best metadata (same weights as `model_11100`)
- `results.csv`: the earlier 11100/14999 comparison
- `../policy/model_15550.pt`: best strong-continuation checkpoint (61.8%)
