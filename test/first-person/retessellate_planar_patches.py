#!/usr/bin/env python3
"""Conservative connected coplanar triangle retessellation.

This research tool changes TRIANGULATION, never vertex positions. It
replaces a patch only if the exact original polygon (including boundary
vertices and any holes) can be retriangulated with fewer triangles.
The complete output is rejected if mesh watertightness, Euler topology or
enclosed signed volume changes. Native palette materials are NOT preserved
or reconstructed; export only a geometry validation candidate.

Requires numpy, trimesh and Shapely >= 2.1. Private DAT-derived meshes must
remain local; no proprietary artwork is included.
"""
from __future__ import annotations

import argparse
from collections import Counter, defaultdict
import json
from pathlib import Path

import numpy as np
import trimesh
from shapely import Polygon, constrained_delaunay_triangles


def retessellate(mesh: trimesh.Trimesh, min_patch_faces: int = 6):
    if min_patch_faces < 2:
        raise ValueError("minimum patch size must be at least two")
    if len(mesh.faces) == 0:
        return mesh.copy(), {"unchanged_empty": True}
    normals = mesh.face_normals
    plane_offsets = (normals * mesh.triangles_center).sum(axis=1)
    parent = list(range(len(mesh.faces)))

    def root(index):
        while parent[index] != index:
            parent[index] = parent[parent[index]]
            index = parent[index]
        return index

    # Connectivity AND oriented coplanarity: touching at a vertex is not
    # enough, and opposite-facing surfaces must never be glued together.
    for a, b in mesh.face_adjacency:
        if ((normals[a] * normals[b]).sum() > 0.999999
                and abs(plane_offsets[a] - plane_offsets[b]) < 1e-5):
            ra, rb = root(int(a)), root(int(b))
            if ra != rb:
                parent[rb] = ra

    groups = defaultdict(list)
    for i in range(len(mesh.faces)):
        groups[root(i)].append(i)

    vertices = mesh.vertices
    retained = []
    stats = Counter()
    for group in groups.values():
        if len(group) < min_patch_faces:
            retained.extend(mesh.faces[group])
            stats["small_patch"] += 1
            continue

        triangles = mesh.faces[group]
        outward = normals[group[0]]
        projected_axes = [i for i in range(3) if i != np.argmax(np.abs(outward))]
        projected = vertices[:, projected_axes]
        edges = defaultdict(list)
        for triangle in triangles:
            for a, b in zip(triangle, np.roll(triangle, -1)):
                edges[tuple(sorted((int(a), int(b))))].append((int(a), int(b)))
        if any(len(v) > 2 for v in edges.values()):
            retained.extend(triangles)
            stats["nonmanifold_patch"] += 1
            continue

        boundary = [v[0] for v in edges.values() if len(v) == 1]
        successor = defaultdict(list)
        indegree = Counter()
        for a, b in boundary:
            successor[a].append(b)
            indegree[b] += 1
        if (len(boundary) < 3
                or any(len(v) != 1 or indegree[k] != 1
                       for k, v in successor.items())):
            retained.extend(triangles)
            stats["branching_boundary"] += 1
            continue

        loops, visited, bad = [], set(), False
        for start in successor:
            if start in visited:
                continue
            here, loop = start, []
            while here not in visited:
                visited.add(here)
                loop.append(here)
                here = successor[here][0]
            if here != start or len(loop) < 3:
                bad = True
                break
            loops.append(loop)
        if bad:
            retained.extend(triangles)
            stats["invalid_boundary"] += 1
            continue

        def signed_area(loop):
            pts = projected[loop]
            return np.sum(pts[:, 0] * np.roll(pts[:, 1], -1)
                          - pts[:, 1] * np.roll(pts[:, 0], -1)) / 2

        loops.sort(key=lambda loop: -abs(signed_area(loop)))
        polygon = Polygon(projected[loops[0]],
                          [projected[loop] for loop in loops[1:]])
        if not polygon.is_valid or polygon.area <= 0:
            retained.extend(triangles)
            stats["invalid_polygon"] += 1
            continue

        original_area = sum(
            abs(np.linalg.det(np.stack(
                [projected[t[1]] - projected[t[0]],
                 projected[t[2]] - projected[t[0]]], axis=1))) / 2
            for t in triangles)
        if abs(original_area - polygon.area) > 1e-5:
            retained.extend(triangles)
            stats["inexact_polygon"] += 1
            continue

        try:
            replacement_shapes = list(constrained_delaunay_triangles(polygon).geoms)
        except Exception:
            retained.extend(triangles)
            stats["triangulation_failed"] += 1
            continue

        boundary_lookup = {
            tuple(np.round(projected[v], 8)): v
            for loop in loops for v in loop}
        replacement = []
        for shape in replacement_shapes:
            pts = np.array(shape.exterior.coords)[:-1]
            if len(pts) != 3:
                bad = True
                break
            ids = [boundary_lookup.get(tuple(np.round(p, 8))) for p in pts]
            if None in ids:
                bad = True
                break
            tri = np.asarray(ids, dtype=int)
            n = np.cross(vertices[tri[1]] - vertices[tri[0]],
                         vertices[tri[2]] - vertices[tri[0]])
            if np.dot(n, outward) < 0:
                tri = tri[[0, 2, 1]]
            replacement.append(tri)

        if (bad or len(replacement) >= len(triangles)
                or abs(sum(shape.area for shape in replacement_shapes)
                       - polygon.area) > 1e-5):
            retained.extend(triangles)
            stats["not_smaller_or_inexact"] += 1
            continue

        new_edges = Counter(
            tuple(sorted((int(a), int(b))))
            for tri in replacement for a, b in zip(tri, np.roll(tri, -1)))
        original_boundary = Counter(tuple(sorted(pair)) for pair in boundary)
        new_boundary = Counter(e for e, count in new_edges.items() if count == 1)
        if new_boundary != original_boundary:
            retained.extend(triangles)
            stats["changed_boundary_edges"] += 1
            continue

        retained.extend(replacement)
        stats["merged_patches"] += 1
        stats["removed_triangles"] += len(triangles) - len(replacement)

    result = trimesh.Trimesh(
        vertices=vertices.copy(), faces=np.asarray(retained, dtype=int),
        process=True)
    # An individual planar match is necessary but insufficient. Reject
    # unexpected changes in topology and exact enclosed geometry globally.
    if mesh.is_watertight:
        volume_tolerance = max(1e-6, abs(mesh.volume) * 1e-9)
        if (not result.is_watertight
                or not result.is_winding_consistent
                or result.euler_number != mesh.euler_number
                or abs(result.volume - mesh.volume) > volume_tolerance):
            return mesh.copy(), {"global_geometry_rejected": True}
    stats["input_triangles"] = len(mesh.faces)
    stats["output_triangles"] = len(result.faces)
    return result, dict(stats)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("input_mesh", type=Path)
    parser.add_argument("output_mesh", type=Path)
    parser.add_argument("--report", type=Path)
    parser.add_argument("--min-patch-faces", type=int, default=6)
    args = parser.parse_args(argv)
    source = trimesh.load(args.input_mesh, force="mesh")
    candidate, stats = retessellate(source, args.min_patch_faces)
    args.output_mesh.parent.mkdir(parents=True, exist_ok=True)
    candidate.export(args.output_mesh)
    report = {
        "input": str(args.input_mesh), "output": str(args.output_mesh),
        "input_faces": len(source.faces), "output_faces": len(candidate.faces),
        "topology_preserved": (
            source.euler_number == candidate.euler_number
            and source.is_watertight == candidate.is_watertight),
        "signed_volume_delta": float(candidate.volume - source.volume),
        "material_certified": False,
        "production_renderer_executed": False,
        "patches": stats,
    }
    output = json.dumps(report, indent=2) + "\n"
    if args.report:
        args.report.parent.mkdir(parents=True, exist_ok=True)
        args.report.write_text(output)
    print(output, end="")
    return report


if __name__ == "__main__":
    main()
