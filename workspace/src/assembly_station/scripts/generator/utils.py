import math

from yaml import SafeDumper


class NoAliasDumper(SafeDumper):
    def ignore_aliases(self, data):
        return True


LAYOUT_ORIGIN = (-1.5, 2.0, 0.0)
ROBOT_MOUNT_Z = 0.595


def apply_base_offset(support_xyz: str, support_rpy: str) -> str:
    """World position of the robot base link, same as MuJoCo's layout."""
    x, y, z = (float(v) for v in support_xyz.split())
    r, p, yaw = (float(v) for v in support_rpy.split())

    # third column of Rz(yaw) @ Ry(p) @ Rx(r) = local +Z axis in world
    zx = math.cos(yaw) * math.sin(p) * math.cos(r) + \
        math.sin(yaw) * math.sin(r)
    zy = math.sin(yaw) * math.sin(p) * math.cos(r) - \
        math.cos(yaw) * math.sin(r)
    zz = math.cos(p) * math.cos(r)

    ox, oy, oz = LAYOUT_ORIGIN
    bx = ox + x + ROBOT_MOUNT_Z * zx
    by = oy + y + ROBOT_MOUNT_Z * zy
    bz = oz + z + ROBOT_MOUNT_Z * zz
    return f'{bx:.6f} {by:.6f} {bz:.6f}'
