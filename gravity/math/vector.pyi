from typing import Generator


class Vector2:
    def __init__(self, x: int | float=0, y: int | float=0) -> None:
        self.x: int | float
        self.y: int | float
    
    def __add__(self, other: "Vector2") -> "Vector2":
        ...
        
    def __sub__(self, other: "Vector2") -> "Vector2":
        ...

    def __mul__(self, other: int | float) -> "Vector2":
        ...

    def __rmul__(self, other: int | float) -> "Vector2":
        ...

    def __truediv__(self, other: int | float) -> "Vector2":
        ...

    def __neg__(self) -> "Vector2":
        ...

    def __eq__(self, other: object) -> bool:
        ...

    def __len__(self) -> int:
        ...

    def __getitem__(self, index: int) -> int | float:
        ...
    
    def __iter__(self) -> Generator[int | float, None, None]:
        ...

    def __repr__(self) -> str:
        ...

    def length_squared(self) -> int | float:
        ...
        
    def length(self) -> int | float:
        ...

    def normalize(self) -> "Vector2":
        ...

    def rotate(self, angle: int | float) -> "Vector2":
        ...

    def dot(self, other: "Vector2") -> int | float:
        ...

    def cross(self, other: "Vector2") -> int | float:
        ...