from dataclasses import dataclass


@dataclass
class PlateModel:
    x_cord: float = 0.0
    y_cord: float = 0.0
    z_cord: float = 0.0

    def update(self, key: str, value: float) -> None:
        match key:
            case 'x':
                self.x_cord = value

            case 'y':
                self.y_cord = value

            case 'z':
                self.z_cord = value

            case _:
                raise ValueError(
                    f"Invalid coordinate key: '{key}'. Expected 'x', 'y', or 'z'.")


@dataclass
class SliderModel:
    left_pos: float = 0.0
    right_pos: float = 0.0

    def update(self, key: str, value: float) -> None:
        match key:
            case 'left':
                self.left_pos = value

            case 'right':
                self.right_pos = value

            case _:
                raise ValueError(
                    f"Invalid coordinate key: '{key}'. Expected 'left' or 'right'.")
