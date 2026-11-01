import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).parent))

from synth import Scene, make_session, world_offset  # noqa: E402

from dt_pipeline import steps  # noqa: E402
from dt_pipeline.session import Session  # noqa: E402


@pytest.fixture(scope="session")
def sessions(tmp_path_factory):
    root = tmp_path_factory.mktemp("sessions")
    a = Session(make_session(root / "a", Scene()))
    b = Session(make_session(root / "b", Scene(added_box=True, T_frame_world=world_offset())))
    for s in (a, b):
        steps.traj(s, "fastlio")
        steps.build(s, voxel=0.05)
    return a, b


@pytest.fixture(scope="session")
def person_session(tmp_path_factory):
    s = Session(make_session(tmp_path_factory.mktemp("person") / "c", Scene(moving_person=True)))
    steps.traj(s, "fastlio")
    steps.build(s, voxel=0.05)
    return s
