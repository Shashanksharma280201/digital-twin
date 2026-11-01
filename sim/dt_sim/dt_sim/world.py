"""Ground-truth world mesh (mesh_<variant>.npz) with Embree ray casting via Open3D."""

from __future__ import annotations

import json
from dataclasses import dataclass
from pathlib import Path

import numpy as np
import open3d as o3d


@dataclass
class RayHits:
    distance: np.ndarray      # (N,) inf where no hit
    normal: np.ndarray        # (N, 3) world-frame unit normals
    face: np.ndarray          # (N,) triangle id, -1 where no hit
    reflectivity: np.ndarray  # (N,) 0..1
    face_class: np.ndarray    # (N,) uint8, 0 where no hit


class World:
    def __init__(self, vertices, triangles, face_class=None, face_reflectivity=None, classes=None):
        self.vertices = np.asarray(vertices, dtype=np.float32)
        self.triangles = np.asarray(triangles, dtype=np.int32)
        n = len(self.triangles)
        self.face_class = (np.zeros(n, np.uint8) if face_class is None
                           else np.asarray(face_class, dtype=np.uint8))
        self.face_reflectivity = (np.full(n, 0.5, np.float32) if face_reflectivity is None
                                  else np.asarray(face_reflectivity, dtype=np.float32))
        self.classes = classes or {}
        mesh = o3d.t.geometry.TriangleMesh()
        mesh.vertex.positions = o3d.core.Tensor(self.vertices)
        mesh.triangle.indices = o3d.core.Tensor(self.triangles)
        self.scene = o3d.t.geometry.RaycastingScene()
        self.scene.add_triangles(mesh)

    @classmethod
    def load(cls, path: str | Path) -> "World":
        data = np.load(path, allow_pickle=False)
        classes = {}
        if "classes" in data:
            raw = data["classes"]
            classes = json.loads(str(raw.item() if raw.shape == () else raw))
        return cls(data["vertices"], data["triangles"],
                   data["face_class"] if "face_class" in data else None,
                   data["face_reflectivity"] if "face_reflectivity" in data else None,
                   classes)

    def save(self, path: str | Path) -> None:
        np.savez_compressed(path, vertices=self.vertices, triangles=self.triangles,
                            face_class=self.face_class, face_reflectivity=self.face_reflectivity,
                            classes=np.array(json.dumps(self.classes)))

    def cast(self, origins: np.ndarray, directions: np.ndarray) -> RayHits:
        rays = np.hstack([origins, directions]).astype(np.float32)
        ans = self.scene.cast_rays(o3d.core.Tensor(rays))
        dist = ans["t_hit"].numpy().astype(np.float64)
        face = ans["primitive_ids"].numpy().astype(np.int64)
        hit = np.isfinite(dist)
        face = np.where(hit, face, -1)
        normal = ans["primitive_normals"].numpy().astype(np.float64)
        refl = np.where(hit, self.face_reflectivity[np.clip(face, 0, None)], 0.0)
        cls = np.where(hit, self.face_class[np.clip(face, 0, None)], 0).astype(np.uint8)
        return RayHits(dist, normal, face, refl.astype(np.float32), cls)

    def distance(self, points: np.ndarray) -> np.ndarray:
        """Unsigned distance from each point to the mesh surface."""
        q = o3d.core.Tensor(np.asarray(points, dtype=np.float32))
        return self.scene.compute_distance(q).numpy().astype(np.float64)

    def closest(self, points: np.ndarray) -> np.ndarray:
        """Nearest point on the mesh surface for each point."""
        q = o3d.core.Tensor(np.asarray(points, dtype=np.float32))
        return self.scene.compute_closest_points(q)["points"].numpy().astype(np.float64)

    def sample_surface(self, n: int, rng: np.random.Generator) -> np.ndarray:
        v = self.vertices.astype(np.float64)
        tri = v[self.triangles]
        area = 0.5 * np.linalg.norm(np.cross(tri[:, 1] - tri[:, 0], tri[:, 2] - tri[:, 0]), axis=1)
        idx = rng.choice(len(tri), size=n, p=area / area.sum())
        r1, r2 = rng.random(n), rng.random(n)
        s = np.sqrt(r1)
        a, b, c = tri[idx, 0], tri[idx, 1], tri[idx, 2]
        return (1 - s)[:, None] * a + (s * (1 - r2))[:, None] * b + (s * r2)[:, None] * c
