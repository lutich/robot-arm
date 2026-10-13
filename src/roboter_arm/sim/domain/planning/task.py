"""Task text of the trajectory planner (from px-robotic-arm-sim planner.js and agent/task-plan.js).

  "blue on orange", "blau auf orange"   pick blue and put it on top of orange's stack
  "green to 5 15", "gelb nach -8 12"    pick green and put it at board x, y (cm)
  "grab green", "pick up red"           pick it and hold it above its place
  "... home"                            end at HOME, still holding a picked cube; "home" alone goes there
"""
from dataclasses import dataclass
import math
import re

COLOUR_WORDS = {'orange': 'orange', 'blue': 'blue', 'blau': 'blue', 'yellow': 'yellow', 'gelb': 'yellow',
                'green': 'green', 'grün': 'green', 'gruen': 'green', 'red': 'red', 'rot': 'red'}
PICK = re.compile(r'\b(grab|grip|pick|hold)\b')
HINT = 'say "<colour> on <colour>", "<colour> to <x> <y>" (cm, board frame), "grab <colour>" or "home"'


class PlanError(ValueError):
    """The task cannot be planned; the message says why."""


@dataclass(frozen=True)
class Task:
    pick: str | None            # cube id; None: only go HOME
    on: str | None = None       # put it on this cube's stack ...
    at: tuple | None = None     # ... or at board (x, y)
    home: bool = False          # end at HOME

    @property
    def place(self):
        return self.on is not None or self.at is not None


def number(word):
    try:
        value = float(word)
    except ValueError:
        return None
    return value if math.isfinite(value) else None


def parse_task(text, world):
    lower = text.lower()
    words = [w for w in re.split(r'[\s,;()]+', lower) if w]
    colours = [COLOUR_WORDS[w] for w in words if w in COLOUR_WORDS]
    numbers = [n for n in map(number, words) if n is not None]
    home = 'home' in words

    def known(colour):
        if world.cube(colour) is None:
            raise PlanError(f'no {colour} cube in the scene')
        return colour

    if PICK.search(lower) and len(colours) == 1 and not numbers:
        return Task(known(colours[0]), home=home)
    if len(colours) >= 2:
        return Task(known(colours[0]), on=known(colours[1]), home=home)
    if len(colours) == 1 and len(numbers) >= 2:
        return Task(known(colours[0]), at=(numbers[0], numbers[1]), home=home)
    if home and not colours:
        return Task(None, home=True)
    raise PlanError(HINT)
