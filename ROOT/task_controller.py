from dataclasses import dataclass


@dataclass(frozen=True)
class TaskTicket:
    name: str
    request_id: int


class TaskController:
    """Tracks latest request ids and active async jobs."""

    def __init__(self, on_busy_changed=None, on_status=None):
        self._active = {}
        self._latest_ids = {}
        self._on_busy_changed = on_busy_changed
        self._on_status = on_status

    def begin(self, name: str, status_text: str | None = None) -> TaskTicket:
        request_id = self._latest_ids.get(name, 0) + 1
        ticket = TaskTicket(name=name, request_id=request_id)
        self._latest_ids[name] = request_id
        self._active[name] = ticket
        if status_text and self._on_status is not None:
            self._on_status(status_text)
        self._notify_busy()
        return ticket

    def finish(self, ticket: TaskTicket) -> bool:
        active = self._active.get(ticket.name)
        if active != ticket:
            return False
        del self._active[ticket.name]
        self._notify_busy()
        return True

    def is_current(self, ticket: TaskTicket) -> bool:
        return self._latest_ids.get(ticket.name) == ticket.request_id

    def is_busy(self) -> bool:
        return bool(self._active)

    def _notify_busy(self):
        if self._on_busy_changed is not None:
            self._on_busy_changed(self.is_busy())
