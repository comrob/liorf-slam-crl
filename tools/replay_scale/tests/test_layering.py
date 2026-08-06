"""Enforce the import direction declared in replay_scale/__init__.py.

The whole point of the split is that a second frontend (a GUI) can be added
without the core growing a dependency on it. That only holds while the arrows
point one way, and nothing else checks it.
"""

import ast
import pathlib

import pytest

PACKAGE = pathlib.Path(__file__).resolve().parents[1] / "src" / "replay_scale"

# layer -> the layers it is allowed to import from.
ALLOWED = {
    "core": set(),
    "io": {"core"},
    "settings": {"core"},
    "pipeline": {"core", "io", "settings"},
    "plotting": {"core", "io", "settings", "pipeline"},
    "cli": {"core", "io", "settings", "pipeline", "plotting"},
    "gui": {"core", "io", "settings", "pipeline", "plotting"},
}


def _layer_of(path):
    rel = path.relative_to(PACKAGE)
    return rel.parts[0] if len(rel.parts) > 1 else rel.stem


def _imported_layers(path):
    """Layer names this module imports from within the package."""
    tree = ast.parse(path.read_text(encoding="utf-8"), filename=str(path))
    own_depth = len(path.relative_to(PACKAGE).parts) - 1  # 0 for top-level modules
    layers = set()
    for node in ast.walk(tree):
        if not isinstance(node, ast.ImportFrom):
            continue
        # level 1 from a subpackage module is a sibling, i.e. the same layer.
        if node.level == 0 or (node.level == 1 and own_depth == 1):
            continue
        head = (node.module or "").split(".")[0]
        if head:
            layers.add(head)
    return layers


def _modules():
    return [p for p in PACKAGE.rglob("*.py")
            if p.name != "__init__.py" and "__pycache__" not in p.parts]


@pytest.mark.parametrize("path", _modules(), ids=lambda p: str(p.relative_to(PACKAGE)))
def test_module_only_imports_allowed_layers(path):
    layer = _layer_of(path)
    assert layer in ALLOWED, f"{path} is in an undeclared layer {layer!r}"
    violations = _imported_layers(path) - ALLOWED[layer] - {layer}
    assert not violations, (
        f"{path.relative_to(PACKAGE)} (layer {layer!r}) imports from {sorted(violations)}, "
        f"but may only import from {sorted(ALLOWED[layer]) or ['nothing']}")


def test_core_does_no_file_io():
    """core/ computes; it must not open files."""
    for path in _modules():
        if _layer_of(path) != "core":
            continue
        tree = ast.parse(path.read_text(encoding="utf-8"), filename=str(path))
        called = {node.func.id for node in ast.walk(tree)
                  if isinstance(node, ast.Call) and isinstance(node.func, ast.Name)}
        assert "open" not in called, f"{path.relative_to(PACKAGE)} opens files"
