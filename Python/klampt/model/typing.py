"""Helpers for Klampt type hints using the Python typing module.

.. versionadded:: 0.9
"""

#from __future__ import annotations
from typing import Sequence,Tuple,List

Vector = Sequence[float]
Vector2 = Sequence[float]
Vector3 = Sequence[float]
Point = Sequence[float]
Matrix3 = Sequence[Sequence[float]]
Rotation = Sequence[float]
RigidTransform = Tuple[Sequence[float],Sequence[float]]
IntArray = Sequence[int]
StringArray = Sequence[str]
Config = Vector
Configs = List[Vector]