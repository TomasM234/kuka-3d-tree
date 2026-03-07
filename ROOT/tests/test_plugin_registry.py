import os
import sys
import tempfile
import textwrap
import unittest


ROOT_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if ROOT_DIR not in sys.path:
    sys.path.insert(0, ROOT_DIR)

from plugin_registry import discover_python_plugins


class PluginRegistryTests(unittest.TestCase):
    def test_discovery_filters_hidden_and_missing_entrypoints(self):
        with tempfile.TemporaryDirectory() as tmpdir:
            importer_dir = os.path.join(tmpdir, "Importer")
            os.makedirs(importer_dir, exist_ok=True)

            files = {
                "__init__.py": "",
                "valid.py": """
                    def get_label():
                        return "Valid Importer"

                    def run_import(input_path, output_csv):
                        return None
                """,
                "hidden.py": """
                    HIDE_FROM_UI = True

                    def run_import(input_path, output_csv):
                        return None
                """,
                "invalid.py": """
                    def get_label():
                        return "Missing run_import"
                """,
            }

            for name, payload in files.items():
                with open(os.path.join(importer_dir, name), "w", encoding="utf-8") as f:
                    f.write(textwrap.dedent(payload).strip() + "\n")

            specs = discover_python_plugins(
                importer_dir,
                root_dir=tmpdir,
                required_attrs=("run_import",),
            )

        self.assertEqual([spec.file_name for spec in specs], ["valid.py"])
        self.assertEqual(specs[0].module_name, "Importer.valid")
        self.assertIn("Valid Importer", specs[0].display_name)


if __name__ == "__main__":
    unittest.main()
