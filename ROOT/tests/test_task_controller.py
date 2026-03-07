import os
import sys
import unittest


ROOT_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if ROOT_DIR not in sys.path:
    sys.path.insert(0, ROOT_DIR)

from task_controller import TaskController


class TaskControllerTests(unittest.TestCase):
    def test_latest_ticket_wins_and_stale_finish_is_rejected(self):
        busy_events = []
        status_events = []
        controller = TaskController(
            on_busy_changed=busy_events.append,
            on_status=status_events.append,
        )

        first = controller.begin("load", "Loading A")
        second = controller.begin("load", "Loading B")

        self.assertTrue(controller.is_busy())
        self.assertFalse(controller.finish(first))
        self.assertTrue(controller.is_current(second))
        self.assertTrue(controller.finish(second))
        self.assertFalse(controller.is_busy())
        self.assertEqual(status_events, ["Loading A", "Loading B"])
        self.assertEqual(busy_events, [True, True, False])


if __name__ == "__main__":
    unittest.main()
