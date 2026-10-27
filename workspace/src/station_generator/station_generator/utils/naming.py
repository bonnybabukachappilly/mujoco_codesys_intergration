ALL_ARMS = "all_arms"


def link(rid: str, name: str) -> str:
    return f"{rid}_{name}"


def joint(rid: str, name: str) -> str:
    return f"{rid}_{name}"


def arm_group(rid: str) -> str:
    return f"{rid}_arm"


def xacro_prefix(rid: str) -> str:
    return f"{rid}_"
