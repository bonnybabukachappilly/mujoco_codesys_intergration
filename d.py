import mujoco as mj
m = mj.MjModel.from_xml_path('/home/bonnybk/Projects/mujoco_codesys_intergration/workspace/install/mujoco_system/share/mujoco_system/scenes/station.xml')
d = mj.MjData(m)
aid = mj.mj_name2id(m, mj.mjtObj.mjOBJ_ACTUATOR, 'left_lift_slide')
jid = mj.mj_name2id(m, mj.mjtObj.mjOBJ_JOINT, 'left_lift_slide')
d.ctrl[aid] = 20.0                      # constant 20 N
for _ in range(500):                    # 1 s at 2 ms
    mj.mj_step(m, d)
print('pos', d.qpos[m.jnt_qposadr[jid]], 'vel', d.qvel[m.jnt_dofadr[jid]])
for c in d.contact[:d.ncon]:
    print(mj.mj_id2name(m, mj.mjtObj.mjOBJ_GEOM, c.geom1),
          mj.mj_id2name(m, mj.mjtObj.mjOBJ_GEOM, c.geom2), round(c.dist, 4))