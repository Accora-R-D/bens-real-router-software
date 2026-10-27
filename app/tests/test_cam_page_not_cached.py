"""The router page is never cached: after a deploy a cached copy hid the new
options until the operator knew to press Ctrl+F5."""
from starlette.testclient import TestClient

from backend.main import app


def test_the_router_page_is_not_cached():
    r = TestClient(app).get("/cam")
    assert r.status_code == 200
    assert "no-store" in r.headers.get("cache-control", "")
