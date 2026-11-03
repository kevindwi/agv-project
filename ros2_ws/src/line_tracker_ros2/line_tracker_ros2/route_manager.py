"""Waypoint & Route Manager berbasis Graf Lintasan (Tahap 4A).

Mengelola pemuatan peta graf lintasan, validasi integritas struktur peta,
dan pencarian rute sekuensial antar checkpoint dengan instruksi percabangan.
"""

from collections import deque
from dataclasses import dataclass
from typing import Dict, List, Optional
import yaml


@dataclass
class RouteStep:
    """Satu langkah segmen perjalanan di graf lintasan."""
    from_node: str
    to_node: str
    target_aruco_id: int
    target_type: str  # "checkpoint" atau "intersection"
    branch_action: str  # "STRAIGHT", "LEFT", "RIGHT", "STOP"
    speed_mps: float
    distance_m: float


class RouteManager:
    """Manajer rute dan graf lintasan AGV."""

    def __init__(self, map_file_path: Optional[str] = None):
        self.nodes: Dict[str, dict] = {}
        self.edges: List[dict] = []
        self.adjacency: Dict[str, List[dict]] = {}
        self.aruco_to_node: Dict[int, str] = {}

        if map_file_path:
            self.load_map(map_file_path)

    def load_map(self, map_file_path: str) -> None:
        """Muat dan validasi file peta YAML graf."""
        with open(map_file_path, "r", encoding="utf-8") as f:
            data = yaml.safe_load(f)

        if not data or "nodes" not in data or "edges" not in data:
            raise ValueError(f"File peta tidak valid: harus memiliki 'nodes' dan 'edges' ({map_file_path})")

        self.nodes = data["nodes"]
        self.edges = data["edges"]
        self.adjacency = {node_id: [] for node_id in self.nodes}
        self.aruco_to_node = {}

        # Petakan ArUco ID ke node
        for node_id, node_data in self.nodes.items():
            aruco_id = node_data.get("aruco_id")
            if aruco_id is not None:
                if aruco_id in self.aruco_to_node:
                    raise ValueError(f"Duplikat aruco_id {aruco_id} pada node '{node_id}' dan '{self.aruco_to_node[aruco_id]}'")
                self.aruco_to_node[aruco_id] = node_id

        # Bangun graf keterhubungan
        for edge in self.edges:
            src = edge.get("from")
            dst = edge.get("to")
            if src not in self.nodes:
                raise ValueError(f"Edge merujuk node asal yang tidak ada: '{src}'")
            if dst not in self.nodes:
                raise ValueError(f"Edge merujuk node tujuan yang tidak ada: '{dst}'")

            self.adjacency[src].append(edge)

        self.validate_integrity()

    def validate_integrity(self) -> None:
        """Validasi integritas graf (tidak ada node yatim atau buntu tak terhubung)."""
        if not self.nodes:
            raise ValueError("Graf tidak memiliki node")

        # Cek apakah setiap node memiliki minimal 1 relasi (masuk atau keluar)
        referenced_nodes = set()
        for edge in self.edges:
            referenced_nodes.add(edge["from"])
            referenced_nodes.add(edge["to"])

        orphan_nodes = set(self.nodes.keys()) - referenced_nodes
        if orphan_nodes:
            raise ValueError(f"Ditemukan node yatim tanpa koneksi: {orphan_nodes}")

    def get_node_by_aruco_id(self, aruco_id: int) -> Optional[str]:
        """Dapatkan nama node berdasarkan ID marker ArUco."""
        return self.aruco_to_node.get(aruco_id)

    def plan_route(self, start_node: str, target_node: str) -> List[RouteStep]:
        """Rencanakan rute sekuensial dari start_node ke target_node (BFS jalur terpendek)."""
        if start_node not in self.nodes:
            raise ValueError(f"Start node tidak dikenal: '{start_node}'")
        if target_node not in self.nodes:
            raise ValueError(f"Target node tidak dikenal: '{target_node}'")

        if start_node == target_node:
            return []

        # BFS shortest path pada directed graph
        queue = deque([[start_node]])
        visited = {start_node}
        found_path: Optional[List[str]] = None

        while queue:
            current_path = queue.popleft()
            current_node = current_path[-1]

            if current_node == target_node:
                found_path = current_path
                break

            for edge in self.adjacency.get(current_node, []):
                neighbor = edge["to"]
                if neighbor not in visited:
                    visited.add(neighbor)
                    new_path = list(current_path)
                    new_path.append(neighbor)
                    queue.append(new_path)

        if not found_path:
            raise ValueError(f"Tidak ada jalur yang menghubungkan '{start_node}' ke '{target_node}'")

        # Konversi jalur node menjadi daftar RouteStep
        steps: List[RouteStep] = []
        for i in range(len(found_path) - 1):
            u = found_path[i]
            v = found_path[i + 1]

            # Cari edge spesifik u -> v
            matching_edge = None
            for e in self.adjacency[u]:
                if e["to"] == v:
                    matching_edge = e
                    break

            if not matching_edge:
                raise RuntimeError(f"Edge hilang antara {u} dan {v}")

            v_data = self.nodes[v]
            steps.append(
                RouteStep(
                    from_node=u,
                    to_node=v,
                    target_aruco_id=v_data.get("aruco_id", -1),
                    target_type=v_data.get("type", "unknown"),
                    branch_action=matching_edge.get("branch_action", "STRAIGHT"),
                    speed_mps=float(matching_edge.get("speed_mps", 0.20)),
                    distance_m=float(matching_edge.get("distance_m", 1.0)),
                )
            )

        return steps
