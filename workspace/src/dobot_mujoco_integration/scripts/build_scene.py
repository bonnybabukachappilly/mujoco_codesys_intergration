import mujoco


def build(scene_path, gripper_path, out_path, prefix="grip_"):
    # robot already attached as left_*
    scene = mujoco.MjSpec.from_file(scene_path)
    grip = mujoco.MjSpec.from_file(gripper_path)   # includes resolved here

    # Mount on the flange. euler cancels the gripper's built-in -90° X rotation.
    mount = scene.body("left_link_6").add_frame(
        pos=[0, 0, 0], euler=[1.5708, 0, 0])
    mount.attach_body(grip.body("gripper_base"), prefix, "")

    base = f"{prefix}gripper_base"
    for link in ("left_link_5", "left_link_6"):
        scene.add_exclude(bodyname1=link, bodyname2=base)

    # Check whether the gripper's equality/actuator/contact came along
    probe = scene.copy()
    m = probe.compile()
    print(f"after attach: nq={m.nq} nu={m.nu} neq={m.neq}")

    if m.neq < 1:
        scene.add_equality(
            type=mujoco.mjtEq.mjEQ_JOINT,
            name1=f"{prefix}slider_2", name2=f"{prefix}slider_1",
            data=[0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0],
            solref=[0.005, 1], solimp=[0.95, 0.99, 0.001, 0.5, 2])

    if m.nu < 7:
        scene.add_actuator(
            name=f"{prefix}slider_1", target=f"{prefix}slider_1",
            trntype=mujoco.mjtTrn.mjTRN_JOINT,
            ctrlrange=[0, 0.0187], ctrllimited=1,
            gaintype=mujoco.mjtGain.mjGAIN_FIXED,
            biastype=mujoco.mjtBias.mjBIAS_AFFINE,
            gainprm=[1000, 0, 0, 0, 0, 0, 0, 0, 0, 0],
            biasprm=[0, -1000, -10, 0, 0, 0, 0, 0, 0, 0],
            forcerange=[-50, 50], forcelimited=1)

    # Finger/base excludes: add only if the gripper's <contact> didn't come through
    if m.nexclude < 5:   # 2 from our script + robot's base/link_1 + 3 gripper ones
        pairs = [(base, f"{prefix}finger_1"), (base, f"{prefix}finger_2"),
                 (f"{prefix}finger_1", f"{prefix}finger_2")]
        for a, b in pairs:
            try:
                scene.add_exclude(bodyname1=a, bodyname2=b)
            except Exception:
                pass

    scene.add_key(name="home", qpos=[0] * 8, ctrl=[0] * 7)

    final = scene.compile()
    print(
        f"final: nq={final.nq} nu={final.nu} neq={final.neq}  (expect 8, 7, 1)")

    with open(out_path, "w") as f:
        f.write(scene.to_xml())
    return out_path


if __name__ == "__main__":
    build("scenes/main.xml",
          "grippers/pgc_50_35/gripper.xml",
          "scenes/main_with_gripper.xml")
