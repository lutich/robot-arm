"""Optional finite test caps for one armed session: duration and accepted targets."""


class Budget:
    """Targets are accepted commands, not measured motion; callers pass the current time."""
    def __init__(self, seconds=None, max_moves=None):
        if (seconds is not None and (type(seconds) is not int or not 1 <= seconds <= 600)
                or max_moves is not None and (type(max_moves) is not int or not 1 <= max_moves <= 60)):
            raise ValueError('Optional test caps must be 1–600 seconds and 1–60 moves')
        self.seconds, self.max_moves = seconds, max_moves
        self.deadline = None
        self.moves = self.accepted_targets = 0

    def start(self, now):
        """A freshly armed session: counters reset and the optional deadline starts."""
        self.moves = self.accepted_targets = 0
        self.deadline = now + self.seconds if self.seconds is not None else None

    def reached(self, time):
        """The deadline has arrived by time; also refuses work that ends exactly on it."""
        return self.deadline is not None and time >= self.deadline

    def passed(self, time):
        """Time lies strictly after the deadline."""
        return self.deadline is not None and time > self.deadline

    def allows(self, count):
        return self.max_moves is None or self.accepted_targets + count <= self.max_moves

    def accept(self, *, manual=False):
        self.accepted_targets += 1
        if manual:
            self.moves += 1

    def remaining_moves(self):
        return max(0, self.max_moves-self.accepted_targets) if self.max_moves is not None else None

    def remaining_seconds(self, now):
        return max(0, round(self.deadline-now)) if self.deadline is not None else None
