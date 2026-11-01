"""Image-textured mesh: colour stored per texel instead of per mesh vertex.

A vertex every 2-3 cm cannot hold detail finer than that. Here each triangle gets a small square
cell of a texture atlas, so one triangle carries many colour samples. Texels are coloured
like points (``GpuColourer``: occlusion test, exposure compensation, best views win) with a steep
view weight, so the sharpest view dominates instead of a blur of all of them.

Cells need no unwrapping and every texel keeps a one-to-one world position, at the cost of a
texture that compresses less well than a chart-based atlas.
"""

from __future__ import annotations

import math

import numpy as np


def atlas_layout(triangles: int, cell: int = 7) -> tuple[int, np.ndarray]:
    """Square atlas side in texels and the cell (row, col) of each triangle."""
    per_side = max(1, math.ceil(math.sqrt(triangles)))
    index = np.arange(triangles)
    return per_side * cell, np.column_stack([index // per_side, index % per_side])


def texel_barycentrics(cell: int = 7) -> tuple[np.ndarray, np.ndarray]:
    """Texel ``(row, col)`` offsets inside a cell and their barycentrics.

    A triangle fills the lower-left half of the cell's inner square. Texels just outside it are
    kept as padding with clamped barycentrics, so bilinear sampling at an edge reads this triangle.
    Texels further out are unused.
    """
    inner = cell - 2
    rc = np.stack(np.meshgrid(np.arange(cell), np.arange(cell), indexing="ij"), -1).reshape(-1, 2)
    s = (rc[:, 1] - 0.5) / inner  # 0 and 1 are the inner square's edges
    t = (rc[:, 0] - 0.5) / inner
    used = s + t <= 1 + 1.5 / inner
    rc, s, t = rc[used], np.clip(s[used], 0, 1), np.clip(t[used], 0, 1)
    over = np.maximum(s + t, 1.0)
    s, t = s / over, t / over
    return rc, np.column_stack([1 - s - t, s, t])


def triangle_uvs(triangles: int, side: int, rc: np.ndarray, cell: int = 7) -> np.ndarray:
    """UVs of each triangle's three corners, (triangles * 3, 2), OpenGL style (v up)."""
    v0, u0 = ((rc * cell + 1) / side).T
    size = (cell - 2) / side
    corners = np.stack([np.column_stack([u0, v0]), np.column_stack([u0 + size, v0]),
                        np.column_stack([u0, v0 + size])], 1)
    corners[:, :, 1] = 1.0 - corners[:, :, 1]
    return corners.reshape(-1, 2)


def texel_positions(verts: np.ndarray, faces: np.ndarray, side: int, rc: np.ndarray,
                    cell: int = 7) -> tuple[np.ndarray, np.ndarray]:
    """World position of every used texel and its index into the flat ``side * side`` atlas."""
    off, bary = texel_barycentrics(cell)
    xyz = np.einsum("tvi,bv->tbi", verts[faces].astype(np.float32), bary.astype(np.float32)).reshape(-1, 3)
    origin = rc * cell
    rows = origin[:, 0, None] + off[:, 0]
    cols = origin[:, 1, None] + off[:, 1]
    return xyz, (rows * side + cols).ravel()


def viewer_uvs(uvs: np.ndarray) -> np.ndarray:
    """OBJ UVs (v up) -> what Open3D's viewer samples (v down, row 0 at v = 0).

    Open3D passes UVs through its OBJ reader unchanged, so a textured OBJ needs this before it is
    drawn, or every triangle reads another triangle's cell. Other tools read the OBJ as it is.
    """
    out = np.array(uvs, dtype=float)
    out[:, 1] = 1.0 - out[:, 1]
    return out


def fill_atlas(side: int, flat: np.ndarray, rgb: np.ndarray, seen: np.ndarray, unseen=(128, 128, 128)) -> np.ndarray:
    """RGB atlas (side, side, 3) from per-texel colours."""
    atlas = np.tile(np.asarray(unseen, np.uint8), (side * side, 1))
    atlas[flat[seen]] = rgb[seen]
    return atlas.reshape(side, side, 3)
