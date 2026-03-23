#!/usr/bin/env python3
"""Benchmark: mesh vs capsule collision detection performance.

Run: python3 src/collision_manager/test_mesh_performance.py
"""
import time
import numpy as np

import fcl
import trimesh


MESH_DIR = "src/franka_description/meshes/robots/fr3/collision"


def load_mesh_fcl(stl_path):
    """Load STL mesh into FCL collision object."""
    mesh = trimesh.load(stl_path)
    verts = mesh.vertices
    tris = mesh.faces
    fcl_mesh = fcl.BVHModel()
    fcl_mesh.beginModel(len(tris), len(verts))
    fcl_mesh.addSubModel(verts, tris)
    fcl_mesh.endModel()
    return fcl.CollisionObject(fcl_mesh, fcl.Transform())


def make_capsule_fcl(radius, length):
    """Create capsule (cylinder) FCL collision object."""
    geom = fcl.Cylinder(radius, length)
    return fcl.CollisionObject(geom, fcl.Transform())


def benchmark_distance(obj_a, obj_b, n_iters=1000):
    """Benchmark distance query."""
    req = fcl.DistanceRequest(enable_nearest_points=True)
    res = fcl.DistanceResult()

    # Set some offset so they're not overlapping
    obj_b.setTransform(fcl.Transform(np.eye(3), np.array([0.5, 0, 0])))

    # Warmup
    for _ in range(10):
        fcl.distance(obj_a, obj_b, req, res)

    # Benchmark
    start = time.perf_counter()
    for _ in range(n_iters):
        fcl.distance(obj_a, obj_b, req, res)
    elapsed = time.perf_counter() - start

    return elapsed / n_iters * 1000  # ms per query


def main():
    print("=== Collision Detection Performance Benchmark ===\n")

    # Load meshes
    print("Loading FR3 collision meshes...")
    meshes = {}
    for i in range(8):
        path = f"{MESH_DIR}/link{i}.stl"
        try:
            meshes[f"link{i}"] = load_mesh_fcl(path)
            m = trimesh.load(path)
            print(f"  link{i}: {len(m.faces)} faces, {len(m.vertices)} vertices")
        except Exception as e:
            print(f"  link{i}: FAILED ({e})")

    # Create capsules (approximate)
    capsules = {}
    capsule_params = [
        ("link0", 0.06, 0.03),
        ("link1", 0.06, 0.283),
        ("link2", 0.06, 0.12),
        ("link3", 0.06, 0.15),
        ("link4", 0.06, 0.12),
        ("link5", 0.06, 0.10),
        ("link6", 0.05, 0.08),
        ("link7", 0.04, 0.14),
    ]
    for name, r, l in capsule_params:
        capsules[name] = make_capsule_fcl(r, l)

    n_iters = 1000
    print(f"\nBenchmark: {n_iters} iterations per pair\n")

    # --- Single pair benchmark ---
    print("--- Single pair (link2 vs link6) ---")
    if "link2" in meshes and "link6" in meshes:
        t_mesh = benchmark_distance(meshes["link2"], meshes["link6"], n_iters)
        print(f"  Mesh vs Mesh:       {t_mesh:.4f} ms/query")

    t_cap = benchmark_distance(capsules["link2"], capsules["link6"], n_iters)
    print(f"  Capsule vs Capsule: {t_cap:.4f} ms/query")

    if "link2" in meshes:
        t_mix = benchmark_distance(meshes["link2"], capsules["link6"], n_iters)
        print(f"  Mesh vs Capsule:    {t_mix:.4f} ms/query")

    # --- Full dual-arm benchmark (simulate 20 collision pairs) ---
    print("\n--- Dual-arm simulation (20 pairs) ---")

    # Mesh pairs
    mesh_pairs = []
    cap_pairs = []
    link_keys = [k for k in meshes.keys()]
    for i in range(min(4, len(link_keys))):
        for j in range(i + 2, min(8, len(link_keys))):
            mesh_pairs.append((meshes[link_keys[i]], meshes[link_keys[j]]))
            cap_pairs.append((capsules[link_keys[i]], capsules[link_keys[j]]))
    n_pairs = len(mesh_pairs)
    print(f"  Testing {n_pairs} pairs")

    # Benchmark all pairs together
    req = fcl.DistanceRequest(enable_nearest_points=True)
    res = fcl.DistanceResult()

    # Mesh
    start = time.perf_counter()
    for _ in range(n_iters):
        for a, b in mesh_pairs:
            fcl.distance(a, b, req, res)
    t_mesh_all = (time.perf_counter() - start) / n_iters * 1000
    print(f"  Mesh:    {t_mesh_all:.3f} ms / frame ({n_pairs} pairs)")

    # Capsule
    start = time.perf_counter()
    for _ in range(n_iters):
        for a, b in cap_pairs:
            fcl.distance(a, b, req, res)
    t_cap_all = (time.perf_counter() - start) / n_iters * 1000
    print(f"  Capsule: {t_cap_all:.3f} ms / frame ({n_pairs} pairs)")

    print(f"\n  Speedup: {t_mesh_all / t_cap_all:.1f}x")
    print(f"\n  @ 100Hz (10ms budget):")
    print(f"    Mesh:    {'OK' if t_mesh_all < 10 else 'TOO SLOW'} ({t_mesh_all:.1f}ms)")
    print(f"    Capsule: {'OK' if t_cap_all < 10 else 'TOO SLOW'} ({t_cap_all:.1f}ms)")


if __name__ == "__main__":
    main()
