"""
Test suite for FLL Trajectory Optimizer with Multi-Verse refinement.

This test suite follows test-driven development (TDD) principles to:
1. Isolate and test each heuristic individually
2. Measure accuracy improvements from each refinement
3. Validate kinematic constraints and dynamics
4. Benchmark performance characteristics
"""

from .test_teb_topologies import TestTEBTopologies
from .test_stomp_variants import TestSTOMPVariants
from .test_refinement_accuracy import TestRefinementAccuracy
from .test_critic_metrics import TestCriticMetrics
