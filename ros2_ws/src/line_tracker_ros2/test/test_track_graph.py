"""Unit test untuk validasi integritas struktur peta graf dan route manager (Tahap 4A)."""

import os
import pytest
from line_tracker_ros2.route_manager import RouteManager


@pytest.fixture
def map_path():
    base_dir = os.path.dirname(__file__)
    return os.path.abspath(os.path.join(base_dir, "..", "config", "track_graph.yaml"))


def test_load_valid_map(map_path):
    """Pastikan file peta graf dapat dimuat tanpa error."""
    rm = RouteManager(map_path)
    assert len(rm.nodes) >= 6
    assert len(rm.edges) >= 6


def test_unique_aruco_ids(map_path):
    """Pastikan setiap node memiliki ID ArUco yang unik."""
    rm = RouteManager(map_path)
    aruco_ids = [n["aruco_id"] for n in rm.nodes.values() if "aruco_id" in n]
    assert len(aruco_ids) == len(set(aruco_ids)), "Terdapat ID ArUco duplikat di peta!"


def test_no_orphan_nodes(map_path):
    """Pastikan tidak ada node yatim (semua terhubung oleh edge)."""
    rm = RouteManager(map_path)
    # validate_integrity dipanggil otomatis di load_map
    rm.validate_integrity()


def test_plan_route_cp1_to_cp2(map_path):
    """Uji perencanaan rute CP1 -> CP2 (melewati INT1 belok kanan)."""
    rm = RouteManager(map_path)
    route = rm.plan_route("CP1", "CP2")
    assert len(route) == 2

    # Langkah 1: CP1 -> INT1
    assert route[0].from_node == "CP1"
    assert route[0].to_node == "INT1"
    assert route[0].target_aruco_id == 1

    # Langkah 2: INT1 -> CP2
    assert route[1].from_node == "INT1"
    assert route[1].to_node == "CP2"
    assert route[1].target_aruco_id == 20
    assert route[1].branch_action == "RIGHT"


def test_plan_route_cp2_to_cp3(map_path):
    """Uji perencanaan rute CP2 -> CP3 (melewati INT1 dan INT2)."""
    rm = RouteManager(map_path)
    route = rm.plan_route("CP2", "CP3")
    assert len(route) >= 2
    assert route[-1].to_node == "CP3"
    assert route[-1].target_aruco_id == 30


def test_aruco_lookup(map_path):
    """Uji pemetaan ID ArUco ke nama node."""
    rm = RouteManager(map_path)
    assert rm.get_node_by_aruco_id(10) == "CP1"
    assert rm.get_node_by_aruco_id(1) == "INT1"
    assert rm.get_node_by_aruco_id(20) == "CP2"
    assert rm.get_node_by_aruco_id(999) is None
