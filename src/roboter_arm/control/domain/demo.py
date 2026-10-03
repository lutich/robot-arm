"""Demo draft: named recorded positions and playback progress; no IO, threads or motion."""


class Demo:
    """Commands only, never position feedback. Callers serialize access and validate counts."""
    def __init__(self):
        self.title, self.positions = 'My demo', []
        self.reset()

    def reset(self):
        """Playback restarts from the first position; edits never keep progress."""
        self.cursor, self.status = 0, 'idle'
        self.pause_requested = False
        self.mode = None

    def rename(self, name):
        if not isinstance(name, str) or not name.strip():
            raise ValueError('A demo needs a name')
        self.title = name.strip()
        self.reset()

    def add(self, counts, name=None):
        if name is None:
            name = f'Position {len(self.positions)+1}'
        if not isinstance(name, str) or not name.strip():
            raise ValueError('A position needs a name')
        self.positions.append(dict(name=name.strip(), counts=counts))
        self.reset()

    def position(self, index):
        if type(index) is not int or not 0 <= index < len(self.positions):
            raise ValueError('Select a recorded position')
        return self.positions[index]

    def replace(self, index, counts):
        self.position(index)['counts'] = counts
        self.reset()

    def rename_position(self, index, name):
        position = self.position(index)
        if not isinstance(name, str) or not name.strip():
            raise ValueError('A position needs a name')
        position['name'] = name.strip()
        self.reset()

    def reorder(self, index, direction):
        self.position(index)
        if type(direction) is not int or direction not in (-1,1) or not 0 <= index+direction < len(self.positions):
            raise ValueError('Select an adjacent position to reorder')
        other = index+direction
        self.positions[index], self.positions[other] = self.positions[other], self.positions[index]
        self.reset()

    def remove(self, index):
        self.position(index)
        self.positions.pop(index)
        self.reset()

    def load(self, title, positions):
        self.title, self.positions = title, positions
        self.reset()

    def targets(self, start, all_positions, valid):
        """Commands from start: the rest of the draft, or one position when single stepping."""
        if not self.positions or start >= len(self.positions):
            raise ValueError('No next recorded position')
        # Check the entire draft even when single stepping: invalid rows remain visible.
        for position in self.positions:
            ok, error = valid(position['counts'])
            if not ok:
                raise ValueError(error)
        return ([dict(p['counts']) for p in self.positions[start:]] if all_positions
                else [dict(self.positions[start]['counts'])])

    def begin(self, start, all_positions):
        self.cursor, self.status = start, 'running'
        self.pause_requested = False
        self.mode = 'all' if all_positions else 'single'

    def request_pause(self, busy):
        if not busy or self.mode != 'all' or self.status not in ('running','pausing'):
            raise ValueError('Run all before requesting a pause after the step')
        self.pause_requested, self.status = True, 'pausing'

    def reached(self, index, all_positions):
        """Record arrival at position index; True when playback stops there."""
        self.cursor = index+1
        if self.cursor == len(self.positions):
            self.status = 'complete'
            self.pause_requested = False
            return True
        if all_positions and self.pause_requested:
            self.status = 'paused'
            self.pause_requested = False
            return True
        if not all_positions:
            self.status = 'idle'
            return True
        return False

    def finish(self):
        """The playback worker ended; cursor and status remain for next or resume."""
        self.mode = None
