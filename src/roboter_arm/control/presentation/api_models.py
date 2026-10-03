"""Strict JSON bodies for the HTTP actions; Session stays the authority on allowed values."""
from typing import Any

from pydantic import BaseModel, ConfigDict


class Body(BaseModel):
    # Strict: "yes" or 1 never becomes true and "410" never 410, because Session's readiness and
    # type checks rely on exact values. Unknown fields are refused as before.
    model_config = ConfigDict(strict=True, extra='forbid')


class Empty(Body):
    pass


class PowerOn(Body):
    park_confirmed: bool


class Prepare(Body):
    supported: bool
    power_off: bool


class Arm(Body):
    ready: bool
    supported: bool
    switch_ready: bool


class Move(Body):
    channel: int
    target: int
    first_clear: bool


class SavePose(Body):
    name: str
    observed: bool
    provenance: str


class Stop(Body):
    reason: str | None = None


class GoPose(Body):
    name: str


class Limits(Body):
    channel: int
    low: int
    high: int


class Settings(Body):
    step_size: int
    demo_speed: int


class Jog(Body):
    channel: int
    direction: int


class Name(Body):
    name: str


class OptionalName(Body):
    name: str | None = None


class Index(Body):
    index: int


class Rename(Body):
    index: int
    name: str


class Reorder(Body):
    index: int
    direction: int


class Filename(Body):
    filename: str


class Accepted(BaseModel):
    ok: bool
    result: Any


class Refused(BaseModel):
    error: str
