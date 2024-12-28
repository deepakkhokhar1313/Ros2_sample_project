import numpy as np
import genesis as gs
import time


class GenesisInterface:
    def __init__(self):
        gs.init(backend=gs.gpu)
        self.scene = None
        self.franka = None
        self.dofs_idx = None

    def init_genesis(self):
        self.scene = gs.Scene(
            viewer_options=gs.options.ViewerOptions(
                camera_options=gs.options.CameraOptions(
                    pos=(0, -3.5, 2.5),
                    lookat=(0, 0, 0.5),
                    fov=30,
                    res=(960, 640)
                ),
                max_FPS=60
            ),
            sim_options=gs.options.SimOptions(
                dt=0.01
            )
        )
        plane = self.scene.add_entity(gs.morphs.Plane())
        self.franka = self.scene.add_entity(
            gs.morphs.MJCF(
                file="xml/franka_emika_panda/panda.xml",
            )
        )

        # Set default gains and forces
        jnt_names = [
            "joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "joint7",
            "finger_joint1", "finger_joint2"
        ]
        self.dofs_idx = self.get_joint_dof_indices(jnt_names)

        self.franka.set_dofs_kp(
            kp=np.array([4500, 4500, 3500, 3500, 2000, 2000, 2000, 100, 100]),
            dofs_idx_local=self.dofs_idx
        )

        self.franka.set_dofs_kv(
            kv=np.array([450, 450, 350, 350, 200, 200, 200, 10, 10]),
            dofs_idx_local=self.dofs_idx
        )

        self.franka.set_dofs_force_range(
            lower=np.array([-87, -87, -87, -87, -12, -12, -12, -100, -100]),
            upper=np.array([87, 87, 87, 87, 12, 12, 12, 100, 100]),
            dofs_idx_local=self.dofs_idx
        )

    def build_scene(self):
        self.scene.build()

    def step_simulation(self):
        self.scene.step()

    def reset_simulation(self):
        for i in range(150):
            if i < 50:
                self.set_joint_position([1, 1, 0, 0, 0, 0, 0, 0.04, 0.04])
            elif i < 100:
                self.set_joint_position([-1, 0.8, 1, -2, 1, 0.5, -0.5, 0.04, 0.04])
            else:
                self.set_joint_position([0, 0, 0, 0, 0, 0, 0, 0, 0])
            self.scene.step()

    def set_joint_position(self, positions):
        self.franka.set_dofs_position(np.array(positions), self.dofs_idx)

    def control_joint_position(self, positions):
        self.franka.control_dofs_position(np.array(positions), self.dofs_idx)

    def control_joint_velocity(self, velocities):
        self.franka.control_dofs_velocity(np.array(velocities), self.dofs_idx)

    def control_joint_force(self, forces):
        self.franka.control_dofs_force(np.array(forces), self.dofs_idx)

    def get_joint_positions(self):
        return self.franka.get_dofs_position(self.dofs_idx).tolist()

    def get_joint_velocities(self):
        return self.franka.get_dofs_velocity(self.dofs_idx).tolist()

    def get_joint_forces(self):
        return self.franka.get_dofs_force(self.dofs_idx).tolist()

    def get_joint_control_forces(self):
        return self.franka.get_dofs_control_force(self.dofs_idx).tolist()

    def get_joint_dof_indices(self, joint_names):
        dof_indices = []
        for name in joint_names:
             dof_indices.append(self.franka.get_joint(name).dof_idx_local)
        return dof_indices

    def get_current_time(self):
        return time.time()