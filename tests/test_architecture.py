"""Static layer and context dependency rules for src/roboter_arm; nothing is imported."""
import ast
from pathlib import Path
import sys
import unittest

SRC = Path(__file__).resolve().parents[1] / 'src'
LAYERS = ('domain', 'application', 'infrastructure', 'presentation')
HARDWARE = ('board', 'adafruit_')


def modules():
    for path in sorted(SRC.rglob('*.py')):
        parts = path.relative_to(SRC).with_suffix('').parts
        yield path, '.'.join(parts[:-1] if parts[-1] == '__init__' else parts)


def imports(path):
    """Yield (imported module, at module level) for every import in a file."""
    def visit(node, top):
        for child in ast.iter_child_nodes(node):
            if isinstance(child, ast.Import):
                yield from ((alias.name, top) for alias in child.names)
            elif isinstance(child, ast.ImportFrom):
                if child.level:
                    raise AssertionError(f'{path}: use absolute imports so these rules see them')
                yield child.module, top
            yield from visit(child, top and not isinstance(child, (ast.FunctionDef, ast.AsyncFunctionDef, ast.ClassDef)))
    yield from visit(ast.parse(path.read_text()), True)


def place(module):
    """(context, layer) of a roboter_arm module; the shared kernel has no layer."""
    parts = module.split('.')
    return parts[1], None if parts[1] == 'shared' else parts[2]


def allowed(source, target):
    (context, layer), (other, other_layer) = place(source), place(target)
    if other == 'shared':
        return True
    if context == 'shared':
        return False
    if context != other:
        return layer == 'infrastructure' and other_layer == 'domain'
    return LAYERS.index(other_layer) <= LAYERS.index(layer)


class ArchitectureTests(unittest.TestCase):
    def test_layers_and_contexts_depend_inward_only(self):
        checked = 0
        for path, module in modules():
            for target, _ in imports(path):
                if target.split('.')[0] == 'roboter_arm':
                    checked += 1
                    with self.subTest(module=module, target=target):
                        self.assertTrue(allowed(module, target))
        self.assertGreater(checked, 0)

    def test_domain_uses_only_stdlib_and_project_modules(self):
        for path, module in modules():
            if module.count('.') >= 2 and place(module)[1] == 'domain':
                for target, _ in imports(path):
                    with self.subTest(module=module, target=target):
                        self.assertIn(target.split('.')[0], {'roboter_arm', *sys.stdlib_module_names})

    def test_imports_alone_never_load_hardware_libraries(self):
        for path, module in modules():
            for target, top in imports(path):
                with self.subTest(module=module, target=target):
                    self.assertFalse(top and target.startswith(HARDWARE))


if __name__ == '__main__':
    unittest.main()
