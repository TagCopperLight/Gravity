from math import sqrt, cos, sin
from typing import Generator


class Vector2:
    def __init__(self, x: int | float=0, y: int | float=0)-> None:
        self.x, self.y = x, y
    
    def __add__(self, other: "Vector2") -> "Vector2":
        return self.__class__(self.x + other.x, self.y + other.y)
        
    def __sub__(self, other: "Vector2") -> "Vector2":
        return self.__class__(self.x - other.x, self.y - other.y)

    def __mul__(self, other: int | float) -> "Vector2":
        return self.__class__(self.x * other, self.y * other)

    def __rmul__(self, other: int | float) -> "Vector2":
        return self.__mul__(other)
    
    def __truediv__(self, other: int | float) -> "Vector2":
        return self.__class__(self.x / other, self.y / other)
    
    def __neg__(self) -> "Vector2":
        return self.__class__(-self.x, -self.y)

    def __eq__(self, other: object) -> bool:
        if not isinstance(other, Vector2):
            return NotImplemented
        return self.x == other.x and self.y == other.y

    def __len__(self) -> int:
        return 2

    def __getitem__(self, index: int) -> int | float:
        return (self.x, self.y)[index]

    def __iter__(self) -> Generator[int | float, None, None]:
        yield self.x
        yield self.y
    
    def __repr__(self) -> str:
        return f"Vector2({self.x}, {self.y})"

    def length_squared(self) -> int | float:
        return self.x**2 + self.y**2

    def length(self) -> int | float:
        return sqrt(self.length_squared())

    def normalize(self) -> "Vector2":
        length = self.length()
        if length == 0:
            return self.__class__()
        return self / length
    
    def rotate(self, angle: int | float) -> "Vector2":
        cos_angle, sin_angle = cos(angle), sin(angle)
        return self.__class__(self.x * cos_angle - self.y * sin_angle, self.x * sin_angle + self.y * cos_angle)

    def dot(self, other: "Vector2") -> int | float:
        return self.x * other.x + self.y * other.y

    def cross(self, other: "Vector2") -> int | float:
        return self.x * other.y - self.y * other.x