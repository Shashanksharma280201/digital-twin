import numpy as np

from dt_pipeline import texture


def two_triangles():
    verts = np.array([[0, 0, 0], [1, 0, 0], [0, 1, 0], [2, 2, 2], [3, 2, 2], [2, 3, 2]], float)
    return verts, np.array([[0, 1, 2], [3, 4, 5]])


def test_layout_gives_each_triangle_a_cell():
    side, rc = texture.atlas_layout(5, cell=8)
    assert side == 24  # 5 cells need a 3x3 grid
    assert len({tuple(c) for c in rc}) == 5


def test_texels_stay_on_their_triangle():
    verts, faces = two_triangles()
    side, rc = texture.atlas_layout(len(faces))
    xyz, flat = texture.texel_positions(verts, faces, side, rc)
    assert len(np.unique(flat)) == len(flat)
    first = np.abs(xyz[:, 2]) < 1e-5
    second = np.abs(xyz[:, 2] - 2) < 1e-5
    assert (first | second).all()  # every texel lies on one of the two triangles, none between
    assert first.sum() == second.sum() > 15
    assert (xyz[first, :2].sum(1) <= 1 + 1e-5).all() and (xyz[first, :2] >= -1e-5).all()
    assert (xyz[second, :2].sum(1) <= 5 + 1e-5).all() and (xyz[second, :2] >= 2 - 1e-5).all()


def test_uvs_address_the_same_texels_as_the_positions():
    """A point on the triangle sampled through its UVs must land on a texel of that place."""
    verts, faces = two_triangles()
    side, rc = texture.atlas_layout(len(faces))
    xyz, flat = texture.texel_positions(verts, faces, side, rc)
    atlas = texture.fill_atlas(side, flat, np.clip(xyz * 60, 0, 255).astype(np.uint8), np.ones(len(flat), bool))
    uv = texture.triangle_uvs(len(faces), side, rc).reshape(-1, 3, 2)
    for t in range(2):
        for c in range(3):
            col = min(int(uv[t, c, 0] * side), side - 1)
            row = min(int((1 - uv[t, c, 1]) * side), side - 1)
            expected = np.clip(verts[faces[t, c]] * 60, 0, 255)
            assert np.abs(atlas[row, col].astype(float) - expected).max() <= 12


def test_unseen_texels_are_grey():
    side, rc = texture.atlas_layout(2)
    atlas = texture.fill_atlas(side, np.arange(4), np.full((4, 3), 200, np.uint8), np.array([True, False, True, False]))
    assert atlas[0, 0, 0] == 200 and atlas[0, 1, 0] == 128


def test_viewer_uvs_land_on_the_atlas_row():
    """Open3D's viewer reads row = v * side; through viewer_uvs that must be the triangle's own cell."""
    side, rc = texture.atlas_layout(9)
    uv = texture.viewer_uvs(texture.triangle_uvs(9, side, rc)).reshape(-1, 3, 2)
    for t in range(9):
        rows = (uv[t, :, 1] * side).astype(int)
        assert (rows // 7 == rc[t, 0]).all()
