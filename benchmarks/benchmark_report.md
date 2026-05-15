# Comprehensive Optimizer Benchmark Report
Generated on: 2026-05-14 22:50:42

## Summary Statistics
| Metric | Simple | Multi-Verse | Delta / Speedup |
| :--- | :--- | :--- | :--- |
| Avg Solve Time | 0.287s | 7.618s | 0.04x |
| Avg Traj Duration | 12.744s | 12.744s | 0.0% improvement |
| Success Rate | 1/1 | 1/1 | |

## Heuristic Effectiveness
| Window | Winning Heuristic | Improvement |
| :--- | :--- | :--- |
| run_000_complex:W0-W2 | TEB_Base | 61.7% |
| run_000_complex:W1-W3 | Point_Turn_Override | 56.7% |
| run_000_complex:W2-W4 | STOMP_Noise_0 | 32.4% |
| run_000_complex:W3-W5 | Point_Turn_Override | 57.7% |
| run_000_complex:W4-W6 | STOMP_Noise_0 | 57.7% |

## Physical Validation (Multi-Verse)
| Run | Max Pos Error | Max Slip | Pass/Fail |
| :--- | :--- | :--- | :--- |
| run_000_complex | 0.023173m | 0.000000N | ❌ FAIL |