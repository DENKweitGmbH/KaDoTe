import numpy as np
from typing import TypeAlias, TypedDict
from enum import IntEnum
from typing import List


class EvalParameters(TypedDict, total=True):
    probability_threshold: float


class ObjectType(IntEnum):
    """Classification object type."""

    Unknown = -1


class ImageObject(TypedDict, total=True):
    x1: int
    y1: int
    x2: int
    y2: int
    confidence: float
    type: ObjectType


class AnalysisResult(TypedDict, total=True):
    """Python representation of results json object.

    See `schemas/result.schema.json.
    """

    x: float
    y: float
    z: float
    r: float
    p: float
    yaw: float
    t: ObjectType
    c: float

EvalResult2D: TypeAlias = tuple[np.ndarray, List[ImageObject]]

EvalResult3D: TypeAlias = tuple[np.ndarray, List[AnalysisResult]]