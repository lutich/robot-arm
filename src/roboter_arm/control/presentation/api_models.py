"""Strict JSON bodies for the HTTP actions; Session stays the authority on allowed values."""
from typing import Any, Literal

from pydantic import BaseModel, ConfigDict, Field, field_validator, model_validator


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


class CameraExposure(Body):
    mode: Literal['auto', 'manual']
    time_us: int | None = None
    iso: int | None = None

    @model_validator(mode='after')
    def mode_values(self):
        if self.mode == 'manual' and (self.time_us is None or self.iso is None):
            raise ValueError('Manual exposure needs time_us and iso')
        if self.mode == 'auto' and self.model_fields_set != {'mode'}:
            raise ValueError('Auto exposure does not accept manual values')
        return self


class CameraFocus(Body):
    mode: Literal['once', 'manual']
    lens_position: int | None = None

    @model_validator(mode='after')
    def mode_values(self):
        if self.mode == 'manual' and self.lens_position is None:
            raise ValueError('Manual focus needs lens_position')
        if self.mode == 'once' and self.model_fields_set != {'mode'}:
            raise ValueError('Focus once does not accept lens_position')
        return self


class CameraWhiteBalance(Body):
    mode: Literal['auto', 'manual']
    temperature_k: int | None = None

    @model_validator(mode='after')
    def mode_values(self):
        if self.mode == 'manual' and self.temperature_k is None:
            raise ValueError('Manual white balance needs temperature_k')
        if self.mode == 'auto' and self.model_fields_set != {'mode'}:
            raise ValueError('Auto white balance does not accept temperature_k')
        return self


class CameraSettings(Body):
    expected_run_id: str = Field(min_length=1)
    expected_revision: int = Field(ge=0)
    exposure: CameraExposure | None = None
    focus: CameraFocus | None = None
    white_balance: CameraWhiteBalance | None = None
    anti_banding: Literal['off', '50hz', '60hz', 'auto'] | None = None

    @field_validator('expected_run_id')
    @classmethod
    def nonempty_run(cls, value):
        if not value.strip():
            raise ValueError('Camera run ID must not be blank')
        return value

    @field_validator('exposure', 'focus', 'white_balance', 'anti_banding')
    @classmethod
    def nonnull_setting(cls, value):
        if value is None:
            raise ValueError('Camera settings must not be null')
        return value


class Accepted(BaseModel):
    ok: bool
    result: Any


class Refused(BaseModel):
    error: str
