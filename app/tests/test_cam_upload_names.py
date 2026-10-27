"""Dropping a STEP into the router: where each part's name and quantity come from.

An assembly exported from Onshape names every part and places every occurrence,
and the operator wants those names on the parts list and engraved on the parts.
A single-part file from the pack's "Wood Files" folder carries no product name,
so its filename ("18mm_2off_Part_61") stays the name and quantity, exactly as
it always did.
"""

import os
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
os.environ.setdefault("ONSHAPE_ACCESS_KEY", "x")
os.environ.setdefault("ONSHAPE_SECRET_KEY", "x")

pytest.importorskip("OCP", reason="OpenCascade not installed")
pytest.importorskip("fastapi", reason="FastAPI not installed")

from backend.routes.cam_routes import (ingest_uploads, _parse_upload_name,   # noqa: E402
                                       upload_title, safe_prefix)
from backend.services import step_ingest_service as S                     # noqa: E402


def _board(x, y, z, dx, dy, dz):
    from OCP.BRepPrimAPI import BRepPrimAPI_MakeBox
    from OCP.gp import gp_Pnt
    return BRepPrimAPI_MakeBox(gp_Pnt(x, y, z), dx, dy, dz).Shape()


def _assembly_step(path: Path, boards: list) -> None:
    """Write an XCAF STEP assembly the way Onshape does: one product per named
    part, one component per placement. `boards` = [(name, shape, [offsets])]."""
    from OCP.STEPCAFControl import STEPCAFControl_Writer
    from OCP.TDocStd import TDocStd_Document
    from OCP.TCollection import TCollection_ExtendedString
    from OCP.XCAFDoc import XCAFDoc_DocumentTool
    from OCP.TDataStd import TDataStd_Name
    from OCP.TopLoc import TopLoc_Location
    from OCP.gp import gp_Trsf, gp_Vec
    from OCP.TopoDS import TopoDS_Compound
    from OCP.BRep import BRep_Builder

    doc = TDocStd_Document(TCollection_ExtendedString("step"))
    tool = XCAFDoc_DocumentTool.ShapeTool_s(doc.Main())
    # the assembly root: an empty compound, components added underneath
    comp = TopoDS_Compound()
    BRep_Builder().MakeCompound(comp)
    root = tool.NewShape()
    TDataStd_Name.Set_s(root, TCollection_ExtendedString("Seat Assembly"))
    for name, shape, offsets in boards:
        prod = tool.AddShape(shape, False)
        TDataStd_Name.Set_s(prod, TCollection_ExtendedString(name))
        for (ox, oy, oz) in offsets:
            t = gp_Trsf()
            t.SetTranslation(gp_Vec(ox, oy, oz))
            tool.AddComponent(root, prod, TopLoc_Location(t))
    tool.UpdateAssemblies()
    writer = STEPCAFControl_Writer()
    writer.SetNameMode(True)
    writer.Transfer(doc)
    assert writer.Write(str(path)) == 1


def test_an_assembly_keeps_its_own_part_names_and_counts(tmp_path):
    """Two "Seat Board"s and one "Back Board" arrive as exactly that — not as
    "Seat Assembly (1)", "(2)", "(3)" with a quantity of one each."""
    step = tmp_path / "Seat Assembly.step"
    _assembly_step(step, [
        ("Seat Board", _board(0, 0, 0, 400, 300, 18), [(0, 0, 0), (0, 0, 500)]),
        ("Back Board", _board(0, 0, 0, 400, 200, 18), [(600, 0, 0)]),
    ])
    job = ingest_uploads([(step.name, step.read_bytes())])
    by_name = {p.name: p.qty for p in job["_step_parts"]}
    assert by_name == {"Seat Board": 2, "Back Board": 1}, by_name
    assert job["wood_names"] == ["Back Board", "Seat Board"]


def test_a_quantity_on_an_assembly_filename_multiplies_the_set(tmp_path):
    step = tmp_path / "2off Seat Assembly.step"
    _assembly_step(step, [
        ("Seat Board", _board(0, 0, 0, 400, 300, 18), [(0, 0, 0), (0, 0, 500)]),
        ("Back Board", _board(0, 0, 0, 400, 200, 18), [(600, 0, 0)]),
    ])
    job = ingest_uploads([(step.name, step.read_bytes())])
    by_name = {p.name: p.qty for p in job["_step_parts"]}
    assert by_name == {"Seat Board": 4, "Back Board": 2}, by_name


def test_the_engraving_label_is_the_step_name(tmp_path):
    """What the V-bit writes on the board is the name Onshape gave it."""
    from backend.services import wood_cam_service as wood
    step = tmp_path / "Seat Assembly.step"
    _assembly_step(step, [
        ("Seat Board", _board(0, 0, 0, 400, 300, 18), [(0, 0, 0)]),
        ("Back Board", _board(0, 0, 0, 400, 200, 18), [(600, 0, 0)]),
    ])
    job = ingest_uploads([(step.name, step.read_bytes())])
    parts, _diag = wood.cam_parts_and_diag(job["_step_parts"],
                                           names=set(job["wood_names"]))
    labels = {name: meta["label"] for name, _loops, meta in parts}
    assert labels == {"Seat Board": "Seat Board", "Back Board": "Back Board"}


def test_a_single_part_file_is_still_named_from_its_filename(tmp_path):
    """The pack's Wood Files carry no product name, so nothing changes there:
    '18mm_2off_Part_61.step' is Part_61, 2 off."""
    step = tmp_path / "18mm_2off_Part_61.step"
    S.write_solid_step(_board(0, 0, 0, 400, 300, 18), str(step))
    job = ingest_uploads([(step.name, step.read_bytes())])
    (part,) = job["_step_parts"]
    assert (part.name, part.qty) == ("Part_61", 2)


def test_filename_parsing_is_unchanged():
    assert _parse_upload_name("18mm_2off_Part_61") == ("Part_61", 2)
    assert _parse_upload_name("Part 7") == ("Part 7", 1)
    assert _parse_upload_name("3off Part 13") == ("Part 13", 3)


def test_programs_are_named_after_the_assembly(tmp_path):
    """An assembly dropped in names the job; the job names every .NC file, so a
    folder of programs sorts by job instead of forty 'WOOD_18mm's."""
    step = tmp_path / "Seat Assembly.step"
    _assembly_step(step, [("Seat Board", _board(0, 0, 0, 400, 300, 18), [(0, 0, 0)])])
    job = ingest_uploads([(step.name, step.read_bytes())])
    assert job["title"] == "Seat Assembly"
    assert safe_prefix(job["title"]) == "Seat_Assembly"


def test_a_folder_of_pack_files_shares_what_their_names_share():
    # a pack's Wood Files share only their thickness: that is not a name
    assert upload_title(["18mm_2off_Part_61.step", "18mm_1off_Part_62.step"]) == ""
    assert upload_title(["Seat Assembly - Part 1.step", "Seat Assembly - Part 2.step"]) == "Seat Assembly - Part"
    # nothing in common worth having -> no title, and the prefix falls back
    assert upload_title(["Alpha.step", "Bravo.step"]) == ""
    assert safe_prefix(upload_title(["Alpha.step", "Bravo.step"])) == "WOOD"


def test_the_prefix_is_a_clean_filename():
    assert safe_prefix("Folding Half Siderail - Foot end - SDRFLD-HFF-CB2-000 TEST")         == "Folding_Half_Siderail_Foot_end_SDRFLD"
    assert safe_prefix("  ") == "WOOD"
    assert safe_prefix("Wooden Jig - Backrest 18mm") == "Wooden_Jig_Backrest_18mm"


def test_the_downloaded_program_is_named_after_the_assembly(tmp_path):
    """Drop 'Seat Assembly.step' in, nest it, download: the file the operator
    puts on the USB stick is Seat_Assembly_18mm_....NC, not WOOD_18mm_....NC."""
    from fastapi.testclient import TestClient
    from backend.main import app
    step = tmp_path / "Seat Assembly.step"
    _assembly_step(step, [("Seat Board", _board(0, 0, 0, 400, 300, 18), [(0, 0, 0)])])
    with TestClient(app) as c:
        up = c.post("/cam/upload", files=[("files", (step.name, step.read_bytes(), "application/octet-stream"))])
        assert up.status_code == 200, up.text
        job_id = up.json()["job_id"]
        assert c.get(f"/cam/wood/{job_id}").json()["title"] == "Seat Assembly"
        defaults = c.get("/cam/defaults").json()
        gen = c.post(f"/cam/generate/{job_id}", json={
            "material": defaults["materials"][0], "tools": defaults["seats"],
            "sheet_w": 1220, "sheet_h": 2440})
        assert gen.status_code == 200, gen.text
        key = gen.json()["programs"][0]["key"]
        dl = c.get(f"/cam/nc/{job_id}", params={"thickness": key})
        assert dl.status_code == 200, dl.text
        disp = dl.headers["content-disposition"]
        assert 'filename="Seat_Assembly_18mm_' in disp, disp
