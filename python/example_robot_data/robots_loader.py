import sys
import typing
from os.path import dirname, exists, join

import hppfcl
import numpy as np
import pinocchio as pin
from pinocchio.robot_wrapper import RobotWrapper

try:
    from .path import EXAMPLE_ROBOT_DATA_MODEL_DIR, EXAMPLE_ROBOT_DATA_SOURCE_DIR
except ImportError:
    pass


def getModelPath(subpath, verbose=False):
    source = dirname(dirname(dirname(__file__)))  # top level source directory
    paths = [
        # function called from "make release" in build/ dir
        join(dirname(dirname(dirname(source))), "robots"),
        # function called from a build/ dir inside top level source
        join(dirname(source), "robots"),
        # function called from top level source dir
        join(source, "robots"),
    ]
    try:
        EXAMPLE_ROBOT_DATA_MODEL_DIR

        # function called from installed project
        paths.append(EXAMPLE_ROBOT_DATA_MODEL_DIR)
        # function called from off-tree build dir
        paths.append(EXAMPLE_ROBOT_DATA_SOURCE_DIR)
    except NameError:
        pass
    paths += [join(p, "../../../share/example-robot-data/robots") for p in sys.path]
    for path in paths:
        if exists(join(path, subpath.strip("/"))):
            if verbose:
                print(f"using {path} as modelPath")
            return path
    raise OSError(f"{subpath} not found")


def readParamsFromSrdf(
    model,
    SRDF_PATH,
    verbose=False,
    has_rotor_parameters=True,
    referencePose="half_sitting",
):
    if has_rotor_parameters:
        pin.loadRotorParameters(model, SRDF_PATH, verbose)
    model.armature = np.multiply(
        model.rotorInertia.flat, np.square(model.rotorGearRatio.flat)
    )
    pin.loadReferenceConfigurations(model, SRDF_PATH, verbose)
    q0 = pin.neutral(model)
    if referencePose is not None:
        q0 = model.referenceConfigurations[referencePose].copy()
    q0 = pin.normalize(model, q0)
    return q0


class RobotLoader:
    path = ""
    urdf_filename = ""
    srdf_filename = ""
    sdf_filename = ""
    sdf_root_link_name = ""
    sdf_parent_guidance: typing.ClassVar = []
    urdf_subpath = "robots"
    srdf_subpath = "srdf"
    sdf_subpath = ""
    ref_posture = "half_sitting"
    has_rotor_parameters = False
    free_flyer = False
    model_path = None

    def __init__(self, verbose=False):
        self.verbose = verbose
        if self.urdf_filename:
            if self.sdf_filename:
                raise AttributeError("Please choose between URDF *or* SDF")
            df_path = join(self.path, self.urdf_subpath, self.urdf_filename)
            builder = RobotWrapper.BuildFromURDF
            if self.model_path is None:
                self.model_path = getModelPath(df_path, self.verbose)
            self.df_path = join(self.model_path, df_path)
            self.robot = builder(
                self.df_path,
                [join(self.model_path, "../..")],
                pin.JointModelFreeFlyer() if self.free_flyer else None,
            )
        else:
            df_path = join(self.path, self.sdf_subpath, self.sdf_filename)
            try:
                builder = RobotWrapper.BuildFromSDF
                if self.model_path is None:
                    self.model_path = getModelPath(df_path, self.verbose)
                self.df_path = join(self.model_path, df_path)
                if tuple(int(i) for i in pin.__version__.split(".")) > (2, 9, 1):
                    self.robot = builder(
                        self.df_path,
                        package_dirs=[join(self.model_path, "../..")],
                        root_joint=(
                            pin.JointModelFreeFlyer() if self.free_flyer else None
                        ),
                        root_link_name=self.sdf_root_link_name,
                        parent_guidance=self.sdf_parent_guidance,
                    )
                else:
                    self.robot = builder(
                        self.df_path,
                        package_dirs=[join(self.model_path, "../..")],
                        root_joint=(
                            pin.JointModelFreeFlyer() if self.free_flyer else None
                        ),
                    )
            except AttributeError:
                raise ImportError("Building SDF models require pinocchio >= 3.0.0")

        if self.srdf_filename:
            self.srdf_path = join(
                self.model_path, self.path, self.srdf_subpath, self.srdf_filename
            )
            self.robot.q0 = readParamsFromSrdf(
                self.robot.model,
                self.srdf_path,
                self.verbose,
                self.has_rotor_parameters,
                self.ref_posture,
            )

            if pin.WITH_HPP_FCL and pin.WITH_HPP_FCL_BINDINGS:
                # Add all collision pairs
                self.robot.collision_model.addAllCollisionPairs()

                # Remove collision pairs per SRDF
                pin.removeCollisionPairs(
                    self.robot.model, self.robot.collision_model, self.srdf_path, False
                )

                # Recreate collision data since the collision pairs changed
                self.robot.collision_data = self.robot.collision_model.createData()
        else:
            self.srdf_path = None
            self.robot.q0 = pin.neutral(self.robot.model)
        root = getModelPath(self.path)
        self.robot.urdf = join(root, self.path, self.urdf_subpath, self.urdf_filename)

        if self.free_flyer:
            self.addFreeFlyerJointLimits()

    def addFreeFlyerJointLimits(self):
        ub = self.robot.model.upperPositionLimit
        ub[:7] = 1
        self.robot.model.upperPositionLimit = ub
        lb = self.robot.model.lowerPositionLimit
        lb[:7] = -1
        self.robot.model.lowerPositionLimit = lb

    def generate_capsule_name(self, base_name: str, existing_names: list) -> str:
        """Generates a unique capsule name for a geometry object.

        Args:
            base_name (str): The base name of the geometry object.
            existing_names (list): List of names already assigned to capsules.

        Returns:
            str: Unique capsule name.
        """
        i = 0
        while f"{base_name}_capsule_{i}" in existing_names:
            i += 1
        return f"{base_name}_capsule_{i}"


class B1Loader(RobotLoader):
    path = "b1_description"
    urdf_filename = "b1.urdf"
    urdf_subpath = "urdf"
    srdf_filename = "b1.srdf"
    ref_posture = "standing"
    free_flyer = True


class Go1Loader(RobotLoader):
    path = "go1_description"
    urdf_filename = "go1.urdf"
    urdf_subpath = "urdf"
    srdf_filename = "go1.srdf"
    ref_posture = "standing"
    free_flyer = True


class FalconBravo7NoEndEffectorLoader(RobotLoader):
    path = "falcon_description"
    urdf_filename = "falcon_bravo7_no_ee.urdf"
    urdf_subpath = "urdf"
    srdf_filename = "falcon_bravo7_no_ee.srdf"
    ref_posture = "standing"
    free_flyer = True


class BluevoltaBravo7NoEndEffectorLoader(RobotLoader):
    path = "bluevolta_description"
    urdf_filename = "bluevolta_bravo7_no_ee.urdf"
    urdf_subpath = "urdf"
    srdf_filename = "bluevolta_bravo7_no_ee.srdf"
    ref_posture = "standing"
    free_flyer = True


class FalconBravo7GripperLoader(RobotLoader):
    path = "falcon_description"
    urdf_filename = "falcon_bravo7_gripper.urdf"
    urdf_subpath = "urdf"
    srdf_filename = "falcon_bravo7_gripper.srdf"
    ref_posture = "standing"
    free_flyer = True


class BluevoltaBravo7GripperLoader(RobotLoader):
    path = "bluevolta_description"
    urdf_filename = "bluevolta_bravo7_gripper.urdf"
    urdf_subpath = "urdf"
    srdf_filename = "bluevolta_bravo7_gripper.srdf"
    ref_posture = "standing"
    free_flyer = True


class Bravo7NoEndEffectorLoader(RobotLoader):
    path = "bravo7_description"
    urdf_filename = "bravo7_no_ee.urdf"
    urdf_subpath = "urdf"
    srdf_filename = "bravo7_no_ee.srdf"
    ref_posture = "standing"
    free_flyer = False


class Bravo7GripperLoader(RobotLoader):
    path = "bravo7_description"
    urdf_filename = "bravo7_gripper.urdf"
    urdf_subpath = "urdf"
    srdf_filename = "bravo7_gripper.srdf"
    ref_posture = "standing"
    free_flyer = False


class Go2Loader(RobotLoader):
    path = "go2_description"
    urdf_filename = "go2.urdf"
    urdf_subpath = "urdf"
    srdf_filename = "go2.srdf"
    ref_posture = "standing"
    free_flyer = True


class A1Loader(RobotLoader):
    path = "a1_description"
    urdf_filename = "a1.urdf"
    urdf_subpath = "urdf"
    srdf_filename = "a1.srdf"
    ref_posture = "standing"
    free_flyer = True


class Z1Loader(RobotLoader):
    path = "z1_description"
    urdf_filename = "z1.urdf"
    urdf_subpath = "urdf"
    srdf_filename = "z1.srdf"
    ref_posture = "arm_up"


class B1Z1Loader(B1Loader):
    urdf_filename = "b1-z1.urdf"
    srdf_filename = "b1-z1.srdf"
    ref_posture = "standing_with_arm_home"


class ANYmalLoader(RobotLoader):
    path = "anymal_b_simple_description"
    urdf_filename = "anymal.urdf"
    srdf_filename = "anymal.srdf"
    ref_posture = "standing"
    free_flyer = True


class ANYmalCLoader(RobotLoader):
    path = "anymal_c_simple_description"
    urdf_subpath = "urdf"
    urdf_filename = "anymal.urdf"
    srdf_filename = "anymal.srdf"
    ref_posture = "standing"
    free_flyer = True


class LaikagoLoader(RobotLoader):
    path = "laikago_description"
    urdf_subpath = "urdf"
    urdf_filename = "laikago.urdf"
    free_flyer = True


class ANYmalKinovaLoader(ANYmalLoader):
    urdf_filename = "anymal-kinova.urdf"
    srdf_filename = "anymal-kinova.srdf"
    ref_posture = "standing_with_arm_up"


class BaxterLoader(RobotLoader):
    path = "baxter_description"
    urdf_filename = "baxter.urdf"
    srdf_filename = "baxter_manipulation.srdf"
    urdf_subpath = "urdf"
    srdf_subpath = "srdf"
    ref_posture = "neutral"


class CassieLoader(RobotLoader):
    path = "cassie_description"
    if tuple(int(i) for i in pin.__version__.split(".")) > (2, 9, 1):
        sdf_filename = "cassie.sdf"
    else:
        sdf_filename = "cassie_v2.sdf"
    sdf_subpath = "robots"
    srdf_filename = "cassie_v2.srdf"
    ref_posture = "standing"
    free_flyer = True
    sdf_root_link_name = "pelvis"
    sdf_parent_guidance: typing.ClassVar = [
        "left-roll-op",
        "left-yaw-op",
        "left-pitch-op",
        "left-knee-op",
        "left-tarsus-spring-joint",
        "left-foot-op",
        "right-roll-op",
        "right-yaw-op",
        "right-pitch-op",
        "right-knee-op",
        "right-tarsus-spring-joint",
        "right-foot-op",
    ]


class TalosLoader(RobotLoader):
    path = "talos_data"
    urdf_filename = "talos_reduced.urdf"
    srdf_filename = "talos.srdf"
    free_flyer = True
    has_rotor_parameters = True


class AsrTwoDofLoader(RobotLoader):
    path = "asr_twodof_description"
    urdf_filename = "TwoDofs.urdf"
    urdf_subpath = "urdf"


class TalosBoxLoader(TalosLoader):
    urdf_filename = "talos_reduced_box.urdf"


class TalosFullLoader(TalosLoader):
    urdf_filename = "talos_full_v2.urdf"


class TalosFullBoxLoader(TalosLoader):
    urdf_filename = "talos_full_v2_box.urdf"


class TalosArmLoader(TalosLoader):
    urdf_filename = "talos_left_arm.urdf"
    free_flyer = False


class TalosLegsLoader(TalosLoader):
    def __init__(self, verbose=False):
        super().__init__(verbose=verbose)
        legMaxId = 14
        m1 = self.robot.model
        m2 = pin.Model()
        for j, M, name, parent, Y in zip(
            m1.joints, m1.jointPlacements, m1.names, m1.parents, m1.inertias
        ):
            if j.id < legMaxId:
                jid = m2.addJoint(parent, getattr(pin, j.shortname())(), M, name)
                idx_q, idx_v = m2.joints[jid].idx_q, m2.joints[jid].idx_v
                m2.upperPositionLimit[idx_q : idx_q + j.nq] = m1.upperPositionLimit[
                    j.idx_q : j.idx_q + j.nq
                ]
                m2.lowerPositionLimit[idx_q : idx_q + j.nq] = m1.lowerPositionLimit[
                    j.idx_q : j.idx_q + j.nq
                ]
                m2.velocityLimit[idx_v : idx_v + j.nv] = m1.velocityLimit[
                    j.idx_v : j.idx_v + j.nv
                ]
                m2.effortLimit[idx_v : idx_v + j.nv] = m1.effortLimit[
                    j.idx_v : j.idx_v + j.nv
                ]
                assert jid == j.id
                m2.appendBodyToJoint(jid, Y, pin.SE3.Identity())

        upperPos = m2.upperPositionLimit
        upperPos[:7] = 1
        m2.upperPositionLimit = upperPos
        lowerPos = m2.lowerPositionLimit
        lowerPos[:7] = -1
        m2.lowerPositionLimit = lowerPos
        effort = m2.effortLimit
        effort[:6] = np.inf
        m2.effortLimit = effort

        # q2 = self.robot.q0[:19]
        for f in m1.frames:
            if tuple(int(i) for i in pin.__version__.split(".")) >= (3, 0, 0):
                if f.parentJoint < legMaxId:
                    m2.addFrame(f)
            elif f.parent < legMaxId:
                m2.addFrame(f)

        g2 = pin.GeometryModel()
        for g in self.robot.visual_model.geometryObjects:
            if g.parentJoint < 14:
                g2.addGeometryObject(g)

        self.robot.model = m2
        self.robot.data = m2.createData()
        self.robot.visual_model = g2
        # self.robot.q0=q2
        self.robot.visual_data = pin.GeometryData(g2)

        # Load SRDF file
        self.robot.q0 = readParamsFromSrdf(
            self.robot.model,
            self.srdf_path,
            self.verbose,
            self.has_rotor_parameters,
            self.ref_posture,
        )

        assert (m2.armature[:6] == 0.0).all()
        # Add the free-flyer joint limits to the new model
        self.addFreeFlyerJointLimits()


class HyQLoader(RobotLoader):
    path = "hyq_description"
    urdf_filename = "hyq_no_sensors.urdf"
    srdf_filename = "hyq.srdf"
    ref_posture = "standing"
    free_flyer = True


class BoltLoader(RobotLoader):
    path = "bolt_description"
    urdf_filename = "bolt.urdf"
    srdf_filename = "bolt.srdf"
    ref_posture = "standing"
    free_flyer = True


class BorinotLoader(RobotLoader):
    path = "borinot_description"
    urdf_subpath = "urdf"
    srdf_subpath = "srdf"
    urdf_filename = "borinot_flying_arm_2.urdf"
    srdf_filename = "borinot_flying_arm_2.srdf"
    ref_posture = "home"
    free_flyer = True


class Solo8Loader(RobotLoader):
    path = "solo_description"
    urdf_filename = "solo.urdf"
    srdf_filename = "solo.srdf"
    ref_posture = "standing"
    free_flyer = True


class Solo12Loader(Solo8Loader):
    urdf_filename = "solo12.urdf"


class FingerEduLoader(RobotLoader):
    path = "finger_edu_description"
    urdf_filename = "finger_edu.urdf"
    srdf_filename = "finger_edu.srdf"
    ref_posture = "hanging"
    free_flyer = False


class KinovaLoader(RobotLoader):
    path = "kinova_description"
    urdf_filename = "kinova.urdf"
    srdf_filename = "kinova.srdf"
    ref_posture = "arm_up"


class TiagoLoader(RobotLoader):
    path = "tiago_description"
    urdf_filename = "tiago.urdf"


class TiagoDualLoader(TiagoLoader):
    urdf_filename = "tiago_dual.urdf"


class TiagoNoHandLoader(TiagoLoader):
    urdf_filename = "tiago_no_hand.urdf"


class TiagoProLoader(RobotLoader):
    path = "tiago_pro_description"
    urdf_filename = "tiago_pro.urdf"


class ICubLoader(RobotLoader):
    path = "icub_description"
    urdf_filename = "icub.urdf"
    srdf_filename = "icub.srdf"
    free_flyer = True


class ICubReducedLoader(ICubLoader):
    urdf_filename = "icub_reduced.urdf"


class PandaLoader(RobotLoader):
    path = "panda_description"
    urdf_filename = "panda.urdf"
    urdf_subpath = "urdf"
    srdf_filename = "panda.srdf"
    ref_posture = "default"


class PandaLoaderCollision(PandaLoader):
    urdf_filename = "panda_collision.urdf"

    def __init__(self, verbose=False):
        super().__init__(verbose=verbose)

        cmodel = self.robot.collision_model.copy()
        list_names_capsules = []
        # Iterate through geometry objects in the collision model
        for geom_object in cmodel.geometryObjects:
            geometry = geom_object.geometry
            # Remove superfluous suffix from the name
            base_name = "_".join(geom_object.name.split("_")[:-1])

            # Convert cylinders to capsules
            if isinstance(geometry, hppfcl.Cylinder):
                name = self.generate_capsule_name(base_name, list_names_capsules)
                list_names_capsules.append(name)
                capsule = pin.GeometryObject(
                    name=name,
                    parent_frame=int(geom_object.parentFrame),
                    parent_joint=int(geom_object.parentJoint),
                    collision_geometry=hppfcl.Capsule(
                        geometry.radius, geometry.halfLength
                    ),
                    placement=geom_object.placement,
                )
                capsule.meshColor = np.array([249, 136, 126, 125]) / 255  # Red color
                self.robot.collision_model.addGeometryObject(capsule)
                self.robot.collision_model.removeGeometryObject(geom_object.name)

            # Remove spheres associated with links
            elif isinstance(geometry, hppfcl.Sphere) and "link" in geom_object.name:
                self.robot.collision_model.removeGeometryObject(geom_object.name)

        # Recreate collision data since the collision pairs changed
        self.robot.collision_data = self.robot.collision_model.createData()

        self.srdf_path = None
        self.robot.q0 = pin.neutral(self.robot.model)
        root = getModelPath(self.path)
        self.robot.urdf = join(root, self.path, self.urdf_subpath, self.urdf_filename)


class AlexNubHandsLoader(RobotLoader):
    path = "alex_description"
    urdf_filename = "alex_nub_hands.urdf"
    urdf_subpath = "urdf"
    srdf_filename = "alex_nub_hands.srdf"
    ref_posture = "default"


class AlexPsyonicHandsLoader(RobotLoader):
    path = "alex_description"
    urdf_filename = "alex_psyonic_hands.urdf"
    urdf_subpath = "urdf"
    srdf_filename = "alex_psyonic_hands.srdf"
    ref_posture = "default"


class AlexSakeHandsLoader(RobotLoader):
    path = "alex_description"
    urdf_filename = "alex_sake_hands.urdf"
    urdf_subpath = "urdf"
    srdf_filename = "alex_sake_hands.srdf"
    ref_posture = "default"


class AllegroRightHandLoader(RobotLoader):
    path = "allegro_hand_description"
    urdf_filename = "allegro_right_hand.urdf"
    urdf_subpath = "urdf"


class AllegroLeftHandLoader(RobotLoader):
    path = "allegro_hand_description"
    urdf_filename = "allegro_left_hand.urdf"
    urdf_subpath = "urdf"


class UR3Loader(RobotLoader):
    path = "ur_description"
    urdf_filename = "ur3_robot.urdf"
    urdf_subpath = "urdf"
    ref_posture = None


class UR3GripperLoader(UR3Loader):
    urdf_filename = "ur3_gripper.urdf"
    srdf_filename = "ur3_gripper.srdf"


class UR3LimitedLoader(UR3Loader):
    urdf_filename = "ur3_joint_limited_robot.urdf"


class UR5Loader(UR3Loader):
    urdf_filename = "ur5_robot.urdf"
    srdf_filename = "ur5.srdf"


class UR5GripperLoader(UR5Loader):
    urdf_filename = "ur5_gripper.urdf"
    srdf_filename = "ur5_gripper.srdf"


class UR5LimitedLoader(UR5Loader):
    urdf_filename = "ur5_joint_limited_robot.urdf"


class UR10Loader(UR3Loader):
    urdf_filename = "ur10_robot.urdf"


class UR10LimitedLoader(UR10Loader):
    urdf_filename = "ur10_joint_limited_robot.urdf"


class HectorLoader(RobotLoader):
    path = "hector_description"
    urdf_filename = "quadrotor_base.urdf"
    free_flyer = True


class HextiltLoader(RobotLoader):
    path = "hextilt_description"
    urdf_subpath = "urdf"
    srdf_subpath = "srdf"
    urdf_filename = "hextilt_flying_arm_5.urdf"
    srdf_filename = "hextilt_flying_arm_5.srdf"
    ref_posture = "home"
    free_flyer = True


class DoublePendulumLoader(RobotLoader):
    path = "double_pendulum_description"
    urdf_filename = "double_pendulum.urdf"
    urdf_subpath = "urdf"


class DoublePendulumContinuousLoader(DoublePendulumLoader):
    urdf_filename = "double_pendulum_continuous.urdf"


class DoublePendulumSimpleLoader(DoublePendulumLoader):
    urdf_filename = "double_pendulum_simple.urdf"


class QuadrupedLoader(RobotLoader):
    path = "quadruped_description"
    urdf_subpath = "urdf"
    urdf_filename = "quadruped.urdf"
    free_flyer = True


class RomeoLoader(RobotLoader):
    path = "romeo_description"
    urdf_filename = "romeo.urdf"
    urdf_subpath = "urdf"
    free_flyer = True


class SimpleHumanoidLoader(RobotLoader):
    path = "simple_humanoid_description"
    urdf_subpath = "urdf"
    urdf_filename = "simple_humanoid.urdf"
    srdf_filename = "simple_humanoid.srdf"
    free_flyer = True


class SimpleHumanoidClassicalLoader(SimpleHumanoidLoader):
    urdf_filename = "simple_humanoid_classical.urdf"
    srdf_filename = "simple_humanoid_classical.srdf"


class IrisLoader(RobotLoader):
    path = "iris_description"
    urdf_filename = "iris_simple.urdf"
    free_flyer = True


class PR2Loader(RobotLoader):
    path = "pr2_description"
    urdf_filename = "pr2.urdf"
    urdf_subpath = "urdf"
    srdf_filename = "pr2.srdf"
    free_flyer = True
    ref_posture = "tuck_left_arm"


class HumanLoader(RobotLoader):
    path = "human_description"
    urdf_filename = "human.urdf"
    free_flyer = True
    ref_posture = "anatomical"
    # Enforced, unchangeable free-flyer orientation (90° about X, and Y↔Z swap)
    freeflyer_ori: np.ndarray = np.array(
        [
            [1, 0, 0],
            [0, 0, -1],
            [0, 1, 0],
        ]
    )

    def __init__(
        self,
        height: typing.Optional[float] = None,
        weight: typing.Optional[float] = None,
        gender: str = "m",
        verbose: bool = False,
    ):
        """
        Initialize the robot loader with optional scaling and configuration parameters. The scaling of the model is based on Dumas 2007, except for the
        abdomen and thorax segments, which are scaled based de Leva 1996.
        Parameters:
            height (float, optional): The height to scale the robot model to. If None, uses the default height in the urdf of 1.80m.
            weight (float, optional): The weight to scale the robot model to. If None, uses the default weight in the urdf 75kg.
            gender (str, optional): The gender specification for scaling, default is 'm'.
            verbose (bool, optional): If True, enables verbose output. Default is False.
        Behavior:
            - Calls the base loader's initializer.
            - If height or weight is provided, scales the robot model accordingly.
            - The free-flyer is present, it enforces its orientation and reapplies joint limits.
        """

        # call base loader
        super().__init__(verbose=verbose)

        if height is not None or weight is not None:
            self._scale_pin_model(
                self.robot.model, self.robot.visual_model, height, weight, gender
            )

        # automatically apply the enforced free-flyer orientation
        if self.free_flyer:
            # get joint index
            j_id = self.robot.model.getJointId("root_joint")
            # assign enforced rotation
            self.robot.model.jointPlacements[j_id].rotation = self.freeflyer_ori
            # re-apply limits
            self.addFreeFlyerJointLimits()

    @staticmethod
    def get_dict_inertial_param(
        height: float, weight: float, gender: str = "m"
    ) -> dict:
        inertial_segment_names = [
            "middle_pelvis",
            "left_upperleg",
            "left_lowerleg",
            "left_foot",
            "middle_abdomen",
            "middle_thorax",
            "middle_head",
            "left_upperarm",
            "left_lowerarm",
            "left_hand",
            "right_upperarm",
            "right_lowerarm",
            "right_hand",
            "right_upperleg",
            "right_lowerleg",
            "right_foot",
        ]

        sgt_lengths = {}

        dicts = []

        # Ratios from Dumas 2007 and De Leva 1996 for abdomen and thorax
        for name in inertial_segment_names:
            dict_sgmt = {}
            if name == "middle_pelvis":
                length = (0.0634 if gender == "f" else 0.0505) * height
                sgt_lengths["middle_pelvis_0"] = length
                sgt_lengths["middle_pelvis_0_width"] = (
                    0.1478 if gender == "f" else 0.1265
                ) * height
                dict_sgmt["mass"] = np.round(
                    (0.146 if gender == "f" else 0.142) * weight, 2
                )
                dict_sgmt["com"] = np.round(
                    (
                        np.array([-0.009, -0.232, 0.002])
                        if gender == "f"
                        else np.array([0.028, -0.28, -0.006])
                    )
                    * length,
                    3,
                )
                dict_sgmt["inertia"] = np.round(
                    (
                        np.array(
                            [
                                (length * 0.91) ** 2,
                                -((length * 0.34) ** 2),
                                -((length * 0.01) ** 2),
                                (length * 1) ** 2,
                                -((length * 0.01) ** 2),
                                (length * 0.79) ** 2,
                            ]
                        )
                        if gender == "f"
                        else np.array(
                            [
                                (length * 1.01) ** 2,
                                -((length * 0.25) ** 2),
                                -((length * 0.12) ** 2),
                                (length * 1.06) ** 2,
                                -((length * 0.08) ** 2),
                                (length * 0.95) ** 2,
                            ]
                        )
                    )
                    * dict_sgmt["mass"],
                    5,
                )
                dicts.append(dict_sgmt)
            elif name == "left_upperleg":
                length = (0.2354 if gender == "f" else 0.2441) * height
                sgt_lengths["left_upperleg_0"] = length
                dict_sgmt["mass"] = np.round(
                    (0.146 if gender == "f" else 0.123) * weight, 2
                )
                dict_sgmt["com"] = np.round(
                    (
                        np.array([-0.077, -0.377, 0.009])
                        if gender == "f"
                        else np.array([-0.041, -0.429, 0.033])
                    )
                    * length,
                    3,
                )
                dict_sgmt["inertia"] = np.round(
                    (
                        np.array(
                            [
                                (length * 0.31) ** 2,
                                (length * 0.07) ** 2,
                                -((length * 0.02) ** 2),
                                (length * 0.19) ** 2,
                                -((length * 0.07) ** 2),
                                (length * 0.32) ** 2,
                            ]
                        )
                        if gender == "f"
                        else np.array(
                            [
                                (length * 0.29) ** 2,
                                (length * 0.07) ** 2,
                                -((length * 0.02) ** 2),
                                (length * 0.15) ** 2,
                                -((length * 0.07) ** 2),
                                (length * 0.3) ** 2,
                            ]
                        )
                    )
                    * dict_sgmt["mass"],
                    5,
                )
                dicts.append(dict_sgmt)
            elif name == "left_lowerleg":
                length = (0.2410 if gender == "f" else 0.2446) * height
                sgt_lengths["left_lowerleg_0"] = length
                dict_sgmt["mass"] = np.round(
                    (0.045 if gender == "f" else 0.048) * weight, 2
                )
                dict_sgmt["com"] = np.round(
                    (
                        np.array([-0.049, -0.404, 0.031])
                        if gender == "f"
                        else np.array([-0.048, -0.41, 0.007])
                    )
                    * length,
                    3,
                )
                dict_sgmt["inertia"] = np.round(
                    (
                        np.array(
                            [
                                (length * 0.28) ** 2,
                                (length * 0.02) ** 2,
                                (length * 0.01) ** 2,
                                (length * 0.1) ** 2,
                                (length * 0.06) ** 2,
                                (length * 0.28) ** 2,
                            ]
                        )
                        if gender == "f"
                        else np.array(
                            [
                                (length * 0.28) ** 2,
                                (length * 0.04) ** 2,
                                (length * 0.02) ** 2,
                                (length * 0.1) ** 2,
                                (length * 0.05) ** 2,
                                (length * 0.28) ** 2,
                            ]
                        )
                    )
                    * dict_sgmt["mass"],
                    5,
                )
                dicts.append(dict_sgmt)
            elif name == "left_foot":
                length = (0.1447 if gender == "f" else 0.1497) * height
                sgt_lengths["left_foot_0"] = length
                dict_sgmt["mass"] = np.round(
                    (0.01 if gender == "f" else 0.012) * weight, 2
                )
                dict_sgmt["com"] = np.round(
                    (
                        np.array([0.27, -0.218, 0.039])
                        if gender == "f"
                        else np.array([0.382, -0.151, 0.026])
                    )
                    * length,
                    3,
                )
                dict_sgmt["inertia"] = np.round(
                    (
                        np.array(
                            [
                                (length * 0.17) ** 2,
                                -((length * 0.10) ** 2),
                                (length * 0.06) ** 2,
                                (length * 0.36) ** 2,
                                -((length * 0.04) ** 2),
                                (length * 0.35) ** 2,
                            ]
                        )
                        if gender == "f"
                        else np.array(
                            [
                                (length * 0.17) ** 2,
                                (length * 0.13) ** 2,
                                -((length * 0.08) ** 2),
                                (length * 0.37) ** 2,
                                (length * 0) ** 2,
                                (length * 0.36) ** 2,
                            ]
                        )
                    )
                    * dict_sgmt["mass"],
                    5,
                )
                dicts.append(dict_sgmt)
            elif name == "middle_abdomen":
                length = (0.1183 if gender == "f" else 0.1237) * height
                sgt_lengths["middle_abdomen_0"] = length
                dict_sgmt["mass"] = np.round(
                    (0.1465 if gender == "f" else 0.1633) * weight, 2
                )
                dict_sgmt["com"] = np.round(
                    (
                        np.array([-0.016, -0.4512, -0.006])
                        if gender == "f"
                        else np.array([-0.036, 0.4502, -0.002])
                    )
                    * length,
                    3,
                )
                dict_sgmt["inertia"] = np.round(
                    (
                        np.array(
                            [
                                (length * 0.29) ** 2,
                                (length * 0.22) ** 2,
                                (length * 0.05) ** 2,
                                (length * 0.27) ** 2,
                                -((length * 0.05) ** 2),
                                (length * 0.29) ** 2,
                            ]
                        )
                        if gender == "f"
                        else np.array(
                            [
                                (length * 0.27) ** 2,
                                (length * 0.18) ** 2,
                                -((length * 0.02) ** 2),
                                (length * 0.25) ** 2,
                                -((length * 0.04) ** 2),
                                (length * 0.28) ** 2,
                            ]
                        )
                    )
                    * dict_sgmt["mass"],
                    5,
                )
                dicts.append(dict_sgmt)
            elif name == "middle_thorax":
                length = (0.1314 if gender == "f" else 0.1390) * height
                sgt_lengths["middle_thorax_0"] = length
                dict_sgmt["mass"] = np.round(
                    (0.1545 if gender == "f" else 0.1596) * weight, 2
                )
                dict_sgmt["com"] = np.round(
                    (
                        np.array([-0.016, 0.5050, -0.006])
                        if gender == "f"
                        else np.array([-0.036, 0.5066, -0.002])
                    )
                    * length,
                    3,
                )
                dict_sgmt["inertia"] = np.round(
                    (
                        np.array(
                            [
                                (length * 0.29) ** 2,
                                (length * 0.22) ** 2,
                                (length * 0.05) ** 2,
                                (length * 0.27) ** 2,
                                -((length * 0.05) ** 2),
                                (length * 0.29) ** 2,
                            ]
                        )
                        if gender == "f"
                        else np.array(
                            [
                                (length * 0.27) ** 2,
                                (length * 0.18) ** 2,
                                -((length * 0.02) ** 2),
                                (length * 0.25) ** 2,
                                -((length * 0.04) ** 2),
                                (length * 0.28) ** 2,
                            ]
                        )
                    )
                    * dict_sgmt["mass"],
                    5,
                )
                dicts.append(dict_sgmt)
            elif name == "middle_head":
                length = (0.1308 if gender == "f" else 0.1310) * height
                sgt_lengths["middle_head_0"] = length
                dict_sgmt["mass"] = np.round(
                    0.067 * weight, 2
                )  # same for male and female
                dict_sgmt["com"] = np.round(
                    (
                        np.array([-0.07, 0.597, 0])
                        if gender == "f"
                        else np.array([-0.062, 0.555, 0.001])
                    )
                    * length,
                    3,
                )
                dict_sgmt["inertia"] = np.round(
                    (
                        np.array(
                            [
                                (length * 0.32) ** 2,
                                -((length * 0.06) ** 2),
                                (length * 0.01) ** 2,
                                (length * 0.27) ** 2,
                                -((length * 0.01) ** 2),
                                (length * 0.34) ** 2,
                            ]
                        )
                        if gender == "f"
                        else np.array(
                            [
                                (length * 0.31) ** 2,
                                -((length * 0.09) ** 2),
                                -((length * 0.02) ** 2),
                                (length * 0.25) ** 2,
                                (length * 0.03) ** 2,
                                (length * 0.33) ** 2,
                            ]
                        )
                    )
                    * dict_sgmt["mass"],
                    5,
                )
                dicts.append(dict_sgmt)
            elif name == "left_upperarm":
                length = (0.1510 if gender == "f" else 0.1531) * height
                sgt_lengths["left_upperarm_0"] = length
                dict_sgmt["mass"] = np.round(
                    (0.022 if gender == "f" else 0.024) * weight, 2
                )
                dict_sgmt["com"] = np.round(
                    (
                        np.array([-0.073, -0.454, -0.028])
                        if gender == "f"
                        else np.array([0.017, -0.452, -0.026])
                    )
                    * length,
                    3,
                )
                dict_sgmt["inertia"] = np.round(
                    (
                        np.array(
                            [
                                (length * 0.33) ** 2,
                                (length * 0.03) ** 2,
                                -((length * 0.05) ** 2),
                                (length * 0.17) ** 2,
                                -((length * 0.14) ** 2),
                                (length * 0.33) ** 2,
                            ]
                        )
                        if gender == "f"
                        else np.array(
                            [
                                (length * 0.31) ** 2,
                                (length * 0.06) ** 2,
                                (length * 0.05) ** 2,
                                (length * 0.14) ** 2,
                                (length * 0.02) ** 2,
                                (length * 0.32) ** 2,
                            ]
                        )
                    )
                    * dict_sgmt["mass"],
                    5,
                )
                dicts.append(dict_sgmt)
            elif name == "left_lowerarm":
                length = (0.1534 if gender == "f" else 0.1593) * height
                sgt_lengths["left_lowerarm_0"] = length
                dict_sgmt["mass"] = np.round(
                    (0.013 if gender == "f" else 0.017) * weight, 2
                )
                dict_sgmt["com"] = np.round(
                    (
                        np.array([0.021, -0.411, 0.019])
                        if gender == "f"
                        else np.array([0.01, -0.417, 0.014])
                    )
                    * length,
                    3,
                )
                dict_sgmt["inertia"] = np.round(
                    (
                        np.array(
                            [
                                (length * 0.26) ** 2,
                                (length * 0.1) ** 2,
                                (length * 0.04) ** 2,
                                (length * 0.14) ** 2,
                                -((length * 0.13) ** 2),
                                (length * 0.25) ** 2,
                            ]
                        )
                        if gender == "f"
                        else np.array(
                            [
                                (length * 0.28) ** 2,
                                (length * 0.03) ** 2,
                                (length * 0.02) ** 2,
                                (length * 0.11) ** 2,
                                -((length * 0.08) ** 2),
                                (length * 0.27) ** 2,
                            ]
                        )
                    )
                    * dict_sgmt["mass"],
                    5,
                )
                dicts.append(dict_sgmt)
            elif name == "left_hand":
                length = (0.0989 if gender == "f" else 0.1014) * height
                sgt_lengths["left_hand_0"] = length
                dict_sgmt["mass"] = np.round(
                    (0.005 if gender == "f" else 0.006) * weight, 2
                )
                dict_sgmt["com"] = np.round(
                    (
                        np.array([0.077, -0.768, 0.048])
                        if gender == "f"
                        else np.array([0.082, -0.839, 0.074])
                    )
                    * length,
                    3,
                )
                dict_sgmt["inertia"] = np.round(
                    (
                        np.array(
                            [
                                (length * 0.63) ** 2,
                                (length * 0.29) ** 2,
                                (length * 0.23) ** 2,
                                (length * 0.43) ** 2,
                                -((length * 0.28) ** 2),
                                (length * 0.58) ** 2,
                            ]
                        )
                        if gender == "f"
                        else np.array(
                            [
                                (length * 0.61) ** 2,
                                (length * 0.22) ** 2,
                                (length * 0.15) ** 2,
                                (length * 0.38) ** 2,
                                -((length * 0.2) ** 2),
                                (length * 0.56) ** 2,
                            ]
                        )
                    )
                    * dict_sgmt["mass"],
                    5,
                )
                dicts.append(dict_sgmt)
            elif name == "right_upperarm":
                length = (0.1510 if gender == "f" else 0.1531) * height
                sgt_lengths["right_upperarm_0"] = length
                dict_sgmt["mass"] = np.round(
                    (0.022 if gender == "f" else 0.024) * weight, 2
                )
                dict_sgmt["com"] = np.round(
                    (
                        np.array([-0.073, -0.454, -0.028])
                        if gender == "f"
                        else np.array([0.017, -0.452, -0.026])
                    )
                    * length,
                    3,
                )
                dict_sgmt["inertia"] = np.round(
                    (
                        np.array(
                            [
                                (length * 0.33) ** 2,
                                (length * 0.03) ** 2,
                                -((length * 0.05) ** 2),
                                (length * 0.17) ** 2,
                                -((length * 0.14) ** 2),
                                (length * 0.33) ** 2,
                            ]
                        )
                        if gender == "f"
                        else np.array(
                            [
                                (length * 0.31) ** 2,
                                (length * 0.06) ** 2,
                                (length * 0.05) ** 2,
                                (length * 0.14) ** 2,
                                (length * 0.02) ** 2,
                                (length * 0.32) ** 2,
                            ]
                        )
                    )
                    * dict_sgmt["mass"],
                    5,
                )
                dicts.append(dict_sgmt)
            elif name == "right_lowerarm":
                length = (0.1534 if gender == "f" else 0.1593) * height
                sgt_lengths["right_lowerarm_0"] = length
                dict_sgmt["mass"] = np.round(
                    (0.013 if gender == "f" else 0.017) * weight, 2
                )
                dict_sgmt["com"] = np.round(
                    (
                        np.array([0.021, -0.411, 0.019])
                        if gender == "f"
                        else np.array([0.01, -0.417, 0.014])
                    )
                    * length,
                    3,
                )
                dict_sgmt["inertia"] = np.round(
                    (
                        np.array(
                            [
                                (length * 0.26) ** 2,
                                (length * 0.1) ** 2,
                                (length * 0.04) ** 2,
                                (length * 0.14) ** 2,
                                -((length * 0.13) ** 2),
                                (length * 0.25) ** 2,
                            ]
                        )
                        if gender == "f"
                        else np.array(
                            [
                                (length * 0.28) ** 2,
                                (length * 0.03) ** 2,
                                (length * 0.02) ** 2,
                                (length * 0.11) ** 2,
                                -((length * 0.08) ** 2),
                                (length * 0.27) ** 2,
                            ]
                        )
                    )
                    * dict_sgmt["mass"],
                    5,
                )
                dicts.append(dict_sgmt)
            elif name == "right_hand":
                length = (0.0989 if gender == "f" else 0.1014) * height
                sgt_lengths["right_hand_0"] = length
                dict_sgmt["mass"] = np.round(
                    (0.005 if gender == "f" else 0.006) * weight, 2
                )
                dict_sgmt["com"] = np.round(
                    (
                        np.array([0.077, -0.768, 0.048])
                        if gender == "f"
                        else np.array([0.082, -0.839, 0.074])
                    )
                    * length,
                    3,
                )
                dict_sgmt["inertia"] = np.round(
                    (
                        np.array(
                            [
                                (length * 0.63) ** 2,
                                (length * 0.29) ** 2,
                                (length * 0.23) ** 2,
                                (length * 0.43) ** 2,
                                -((length * 0.28) ** 2),
                                (length * 0.58) ** 2,
                            ]
                        )
                        if gender == "f"
                        else np.array(
                            [
                                (length * 0.61) ** 2,
                                (length * 0.22) ** 2,
                                (length * 0.15) ** 2,
                                (length * 0.38) ** 2,
                                -((length * 0.2) ** 2),
                                (length * 0.56) ** 2,
                            ]
                        )
                    )
                    * dict_sgmt["mass"],
                    5,
                )
                dicts.append(dict_sgmt)
            elif name == "right_upperleg":
                length = (0.2354 if gender == "f" else 0.2441) * height
                sgt_lengths["right_upperleg_0"] = length
                dict_sgmt["mass"] = np.round(
                    (0.146 if gender == "f" else 0.123) * weight, 2
                )
                dict_sgmt["com"] = np.round(
                    (
                        np.array([-0.077, -0.377, 0.009])
                        if gender == "f"
                        else np.array([-0.041, -0.429, 0.033])
                    )
                    * length,
                    3,
                )
                dict_sgmt["inertia"] = np.round(
                    (
                        np.array(
                            [
                                (length * 0.31) ** 2,
                                -((length * 0.07) ** 2),
                                -((length * 0.02) ** 2),
                                (length * 0.19) ** 2,
                                -((length * 0.07) ** 2),
                                (length * 0.32) ** 2,
                            ]
                        )
                        if gender == "f"
                        else np.array(
                            [
                                (length * 0.29) ** 2,
                                -((length * 0.07) ** 2),
                                -((length * 0.02) ** 2),
                                (length * 0.15) ** 2,
                                -((length * 0.07) ** 2),
                                (length * 0.3) ** 2,
                            ]
                        )
                    )
                    * dict_sgmt["mass"],
                    5,
                )
                dicts.append(dict_sgmt)
            elif name == "right_lowerleg":
                length = (0.2410 if gender == "f" else 0.2446) * height
                sgt_lengths["right_lowerleg_0"] = length
                dict_sgmt["mass"] = np.round(
                    (0.045 if gender == "f" else 0.048) * weight, 2
                )
                dict_sgmt["com"] = np.round(
                    (
                        np.array([-0.049, -0.404, 0.031])
                        if gender == "f"
                        else np.array([-0.048, -0.41, 0.007])
                    )
                    * length,
                    3,
                )
                dict_sgmt["inertia"] = np.round(
                    (
                        np.array(
                            [
                                (length * 0.28) ** 2,
                                (length * 0.02) ** 2,
                                (length * 0.01) ** 2,
                                (length * 0.1) ** 2,
                                (length * 0.06) ** 2,
                                (length * 0.28) ** 2,
                            ]
                        )
                        if gender == "f"
                        else np.array(
                            [
                                (length * 0.28) ** 2,
                                (length * 0.04) ** 2,
                                (length * 0.02) ** 2,
                                (length * 0.1) ** 2,
                                (length * 0.05) ** 2,
                                (length * 0.28) ** 2,
                            ]
                        )
                    )
                    * dict_sgmt["mass"],
                    5,
                )
                dicts.append(dict_sgmt)
            elif name == "right_foot":
                length = (0.1447 if gender == "f" else 0.1497) * height
                sgt_lengths["right_foot_0"] = length
                dict_sgmt["mass"] = np.round(
                    (0.01 if gender == "f" else 0.012) * weight, 2
                )
                dict_sgmt["com"] = np.round(
                    (
                        np.array([0.27, -0.218, 0.039])
                        if gender == "f"
                        else np.array([0.382, -0.151, 0.026])
                    )
                    * length,
                    3,
                )
                dict_sgmt["inertia"] = np.round(
                    (
                        np.array(
                            [
                                (length * 0.17) ** 2,
                                -((length * 0.10) ** 2),
                                (length * 0.06) ** 2,
                                (length * 0.36) ** 2,
                                -((length * 0.04) ** 2),
                                (length * 0.35) ** 2,
                            ]
                        )
                        if gender == "f"
                        else np.array(
                            [
                                (length * 0.17) ** 2,
                                (length * 0.13) ** 2,
                                -((length * 0.08) ** 2),
                                (length * 0.37) ** 2,
                                (length * 0) ** 2,
                                (length * 0.36) ** 2,
                            ]
                        )
                    )
                    * dict_sgmt["mass"],
                    5,
                )
                dicts.append(dict_sgmt)

        return dict(zip(inertial_segment_names, dicts)), sgt_lengths

    @staticmethod
    def get_dict_joint_placements(height: float, gender: str = "m") -> dict:
        joint_names = [
            "middle_thoracic_Z",
            "middle_cervical_Z",
            "left_clavicle_joint_X",
            "right_clavicle_joint_X",
            "left_shoulder_Z",
            "left_elbow_Z",
            "left_wrist_Z",
            "right_shoulder_Z",
            "right_elbow_Z",
            "right_wrist_Z",
            "left_hip_Z",
            "left_knee",
            "left_ankle_Z",
            "right_hip_Z",
            "right_knee",
            "right_ankle_Z",
        ]

        lengths_names = [
            "L_abdomen",
            "L_thorax_cerv",
            "L_thorax_supr",
            "L_upperarm",
            "L_lowerarm",
            "L_upperleg",
            "L_lowerleg",
        ]
        ratios = np.array(
            [
                0.1183
                if gender == "f"
                else 0.1237,  # L_abdomen MPT  from XYP to OMPH (De Leva 1996)
                0.1314
                if gender == "f"
                else 0.1390,  # L_thorax UPT from CERV to XYPH (De Leva 1996)
                0.0821 if gender == "f" else 0.0980,  # from SUPR to XYPH (De Leva 1996)
                0.1510 if gender == "f" else 0.1531,  # L_upperarm (Dumas 2007)
                0.1534 if gender == "f" else 0.1593,  # L_lowerarm (Dumas 2007)
                0.2354 if gender == "f" else 0.2441,  # L_upperleg (Dumas 2007)
                0.2410 if gender == "f" else 0.2446,  # L_lowerleg (Dumas 2007)
            ]
        )
        # Ratios from De Leva : HJC from Seidel
        # SJC Schnorenberg-style AC/H = 0.132 (F) or 0.0129 (M)
        SJ_thorax_X_r = 0.0043 if gender == "f" else 0.0046
        SJ_thorax_Y_r = -0.0449 if gender == "f" else -0.0416
        SJ_thorax_Z_r = 0.1108 if gender == "f" else 0.1164
        HJ_pelvis_X_r = 0.0138 if gender == "f" else 0.0126
        HJ_pelvis_Y_r = -0.0570 if gender == "f" else -0.0558
        HJ_pelvis_Z_r = 0.0548 if gender == "f" else 0.0457

        lengths = np.round(ratios * height, 3)  # mm accuracy
        dict_lengths = dict(zip(lengths_names, lengths))

        joint_placements = []
        for j in joint_names:
            if j == "middle_thoracic_Z":
                joint_placements.append(np.array([0, dict_lengths["L_abdomen"], 0]))
            elif j == "middle_cervical_Z":
                joint_placements.append(np.array([0, dict_lengths["L_thorax_cerv"], 0]))
            elif j == "left_clavicle_joint_X":
                joint_placements.append(np.array([0, dict_lengths["L_thorax_supr"], 0]))
            elif j == "right_clavicle_joint_X":
                joint_placements.append(np.array([0, dict_lengths["L_thorax_supr"], 0]))
            elif j == "left_shoulder_Z":
                joint_placements.append(
                    np.round(
                        height
                        * np.array([SJ_thorax_X_r, SJ_thorax_Y_r, -SJ_thorax_Z_r]),
                        3,
                    )
                )
            elif j == "left_elbow_Z":
                joint_placements.append(np.array([0, -dict_lengths["L_upperarm"], 0]))
            elif j == "left_wrist_Z":
                joint_placements.append(np.array([0, -dict_lengths["L_lowerarm"], 0]))
            elif j == "right_shoulder_Z":
                joint_placements.append(
                    np.round(
                        height
                        * np.array([SJ_thorax_X_r, SJ_thorax_Y_r, SJ_thorax_Z_r]),
                        3,
                    )
                )
            elif j == "right_elbow_Z":
                joint_placements.append(np.array([0, -dict_lengths["L_upperarm"], 0]))
            elif j == "right_wrist_Z":
                joint_placements.append(np.array([0, -dict_lengths["L_lowerarm"], 0]))
            elif j == "left_hip_Z":
                joint_placements.append(
                    np.round(
                        height
                        * np.array([HJ_pelvis_X_r, HJ_pelvis_Y_r, -HJ_pelvis_Z_r]),
                        3,
                    )
                )
            elif j == "left_knee":
                joint_placements.append(np.array([0, -dict_lengths["L_upperleg"], 0]))
            elif j == "left_ankle_Z":
                joint_placements.append(np.array([0, -dict_lengths["L_lowerleg"], 0]))
            elif j == "right_hip_Z":
                joint_placements.append(
                    np.round(
                        height
                        * np.array([HJ_pelvis_X_r, HJ_pelvis_Y_r, HJ_pelvis_Z_r]),
                        3,
                    )
                )
            elif j == "right_knee":
                joint_placements.append(np.array([0, -dict_lengths["L_upperleg"], 0]))
            elif j == "right_ankle_Z":
                joint_placements.append(np.array([0, -dict_lengths["L_lowerleg"], 0]))

        return dict(zip(joint_names, joint_placements))

    @staticmethod
    def get_dict_meshes_scale(
        height: float, sgt_lengths: dict, gender: str = "m"
    ) -> dict:
        meshes_names = [
            "middle_pelvis_0",
            "left_upperleg_0",
            "left_lowerleg_0",
            "left_lowerleg_1",
            "left_foot_0",
            "middle_abdomen_0",
            "middle_abdomen_1",
            "middle_head_0",
            "middle_head_1",
            "left_upperarm_0",
            "left_upperarm_1",
            "left_lowerarm_0",
            "left_lowerarm_1",
            "left_hand_0",
            "right_upperarm_0",
            "right_upperarm_1",
            "right_lowerarm_0",
            "right_lowerarm_1",
            "right_hand_0",
            "right_upperleg_0",
            "right_lowerleg_0",
            "right_lowerleg_1",
            "right_foot_0",
        ]

        scales = []

        for name in meshes_names:
            if (
                name == "middle_pelvis_0"
            ):  # pelvis mesh size is : X = 37.050m, Y = 32.551m, Z = 39.252m
                scale = np.round(
                    np.array(
                        [
                            1.09 * sgt_lengths["middle_pelvis_0_width"] / 39.252,
                            sgt_lengths["middle_pelvis_0_width"] / 39.252,
                            sgt_lengths["middle_pelvis_0_width"] / 39.252,
                        ]
                    ),
                    4,
                )
                scales.append(scale)
            elif (
                name == "left_upperleg_0"
            ):  # upperleg mesh size is : X = 20.977m, Y = 68.524m, Z = 20.977m
                scale = np.round(
                    np.array(
                        [
                            0.96 * sgt_lengths["left_upperleg_0"] / 68.524,
                            sgt_lengths["left_upperleg_0"] / 68.524,
                            1.08 * sgt_lengths["left_upperleg_0"] / 68.524,
                        ]
                    ),
                    4,
                )
                scales.append(scale)
            elif (
                name == "left_lowerleg_0"
            ):  # lowerleg mesh size is : X = 17.986m, Y = 67m, Z = 17.977m
                scale = np.round(
                    np.array(
                        [
                            0.96 * sgt_lengths["left_lowerleg_0"] / 67,
                            sgt_lengths["left_lowerleg_0"] / 67,
                            1.07 * sgt_lengths["left_lowerleg_0"] / 67,
                        ]
                    ),
                    4,
                )
                scales.append(scale)
            elif name == "left_lowerleg_1":  # knee mesh is a sphere of 12.5m
                scale = np.round(
                    np.array(
                        [
                            0.7
                            * (0.053 * height if gender == "f" else 0.056 * height)
                            / 12.5,
                            0.7
                            * (0.053 * height if gender == "f" else 0.056 * height)
                            / 12.5,
                            0.7
                            * (0.053 * height if gender == "f" else 0.056 * height)
                            / 12.5,
                        ]
                    ),
                    4,
                )
                scales.append(scale)
            elif (
                name == "left_foot_0"
            ):  # foot mesh size is : X = 40.5m, Y = 16.5m, Z = 17.999m
                scale = np.round(
                    np.array(
                        [
                            sgt_lengths["left_foot_0"] / 40.5,
                            sgt_lengths["left_foot_0"] / 40.5,
                            sgt_lengths["left_foot_0"] / 40.5,
                        ]
                    ),
                    4,
                )
                scales.append(scale)
            elif name == "middle_abdomen_0":  # abdomen meshis a sphere of radius 30 m
                scale = np.round(
                    np.array(
                        [
                            sgt_lengths["middle_abdomen_0"] / 30,
                            sgt_lengths["middle_abdomen_0"] / 30,
                            sgt_lengths["middle_abdomen_0"] / 30,
                        ]
                    ),
                    4,
                )
                scales.append(scale)
            elif (
                name == "middle_abdomen_1"
            ):  # torso mesh size is : X = 36.991m, Y = 35m, Z = 40.029m
                scale = np.round(
                    np.array(
                        [
                            sgt_lengths["middle_thorax_0"] / 35,
                            sgt_lengths["middle_thorax_0"] / 35,
                            sgt_lengths["middle_thorax_0"] / 35,
                        ]
                    ),
                    4,
                )
                scales.append(scale)
            elif (
                name == "middle_head_0"
            ):  # head mesh size is : X = 32.049m, Y = 42m, Z = 32.049m
                scale = np.round(
                    np.array(
                        [
                            sgt_lengths["middle_head_0"] / 42,
                            sgt_lengths["middle_head_0"] / 42,
                            sgt_lengths["middle_head_0"] / 42,
                        ]
                    ),
                    4,
                )
                scales.append(scale)
            elif name == "middle_head_1":  # neck mesh
                scale = np.round(
                    np.array(
                        [
                            (0.078 * height if gender == "f" else 0.086 * height)
                            / 31.0,
                            (0.078 * height if gender == "f" else 0.086 * height)
                            / 31.0,
                            (0.078 * height if gender == "f" else 0.086 * height)
                            / 31.0,
                        ]
                    ),
                    4,
                )
                scales.append(scale)
            elif name == "left_upperarm_0":  # shoulder mesh is a shpere of radius 17.5
                scale = np.round(
                    np.array(
                        [
                            (0.037 * height if gender == "f" else 0.038 * height)
                            / 17.5,
                            (0.037 * height if gender == "f" else 0.038 * height)
                            / 17.5,
                            (0.037 * height if gender == "f" else 0.038 * height)
                            / 17.5,
                        ]
                    ),
                    4,
                )
                scales.append(scale)
            elif (
                name == "left_upperarm_1"
            ):  # upperarm mesh is of size : X = 17.158m, Y = 46m, Z = 17.183m
                scale = np.round(
                    np.array(
                        [
                            1.05 * sgt_lengths["left_upperarm_0"] / 46,
                            sgt_lengths["left_upperarm_0"] / 46,
                            1.16 * sgt_lengths["left_upperarm_0"] / 46,
                        ]
                    ),
                    4,
                )
                scales.append(scale)
            elif (
                name == "left_lowerarm_0"
            ):  # lowerarm mesh is of size : X = 15.547m, Y = 45m, Z = 15.541m
                scale = np.round(
                    np.array(
                        [
                            sgt_lengths["left_lowerarm_0"] / 45,
                            sgt_lengths["left_lowerarm_0"] / 45,
                            sgt_lengths["left_lowerarm_0"] / 45,
                        ]
                    ),
                    4,
                )
                scales.append(scale)
            elif name == "left_lowerarm_1":  # elbow mesh is a sphere of radius 12.5
                scale = np.round(
                    np.array(
                        [
                            (0.037 * height if gender == "f" else 0.038 * height)
                            / 12.5,
                            (0.037 * height if gender == "f" else 0.038 * height)
                            / 12.5,
                            (0.037 * height if gender == "f" else 0.038 * height)
                            / 12.5,
                        ]
                    ),
                    4,
                )
                scales.append(scale)
            elif (
                name == "left_hand_0"
            ):  # hand mesh is of size : X = 12.055m, Y = 30m, Z = 16.646m
                scale = np.round(
                    np.array(
                        [
                            sgt_lengths["left_hand_0"] / 30,
                            sgt_lengths["left_hand_0"] / 30,
                            sgt_lengths["left_hand_0"] / 30,
                        ]
                    ),
                    4,
                )
                scales.append(scale)
            elif name == "right_upperarm_0":  # shoulder mesh
                scale = np.round(
                    np.array(
                        [
                            (0.037 * height if gender == "f" else 0.038 * height)
                            / 17.5,
                            (0.037 * height if gender == "f" else 0.038 * height)
                            / 17.5,
                            (0.037 * height if gender == "f" else 0.038 * height)
                            / 17.5,
                        ]
                    ),
                    4,
                )
                scales.append(scale)
            elif name == "right_upperarm_1":  # upperarm mesh
                scale = np.round(
                    np.array(
                        [
                            1.05 * sgt_lengths["right_upperarm_0"] / 46,
                            sgt_lengths["right_upperarm_0"] / 46,
                            1.16 * sgt_lengths["right_upperarm_0"] / 46,
                        ]
                    ),
                    4,
                )
                scales.append(scale)
            elif name == "right_lowerarm_0":  # lowerarm mesh
                scale = np.round(
                    np.array(
                        [
                            sgt_lengths["right_lowerarm_0"] / 45,
                            sgt_lengths["right_lowerarm_0"] / 45,
                            sgt_lengths["right_lowerarm_0"] / 45,
                        ]
                    ),
                    4,
                )
                scales.append(scale)
            elif name == "right_lowerarm_1":  # elbow mesh
                scale = np.round(
                    np.array(
                        [
                            (0.037 * height if gender == "f" else 0.038 * height)
                            / 12.5,
                            (0.037 * height if gender == "f" else 0.038 * height)
                            / 12.5,
                            (0.037 * height if gender == "f" else 0.038 * height)
                            / 12.5,
                        ]
                    ),
                    4,
                )
                scales.append(scale)
            elif name == "right_hand_0":  # hand mesh
                scale = np.round(
                    np.array(
                        [
                            sgt_lengths["right_hand_0"] / 30,
                            sgt_lengths["right_hand_0"] / 30,
                            sgt_lengths["right_hand_0"] / 30,
                        ]
                    ),
                    4,
                )
                scales.append(scale)
            elif name == "right_upperleg_0":  # upperleg mesh
                scale = np.round(
                    np.array(
                        [
                            0.96 * sgt_lengths["right_upperleg_0"] / 68.524,
                            sgt_lengths["right_upperleg_0"] / 68.524,
                            1.08 * sgt_lengths["right_upperleg_0"] / 68.524,
                        ]
                    ),
                    4,
                )
                scales.append(scale)
            elif name == "right_lowerleg_0":  # lowerleg mesh
                scale = np.round(
                    np.array(
                        [
                            0.96 * sgt_lengths["right_lowerleg_0"] / 67,
                            sgt_lengths["right_lowerleg_0"] / 67,
                            1.07 * sgt_lengths["right_lowerleg_0"] / 67,
                        ]
                    ),
                    4,
                )
                scales.append(scale)
            elif name == "right_lowerleg_1":  # knee mesh
                scale = np.round(
                    np.array(
                        [
                            0.7
                            * (0.053 * height if gender == "f" else 0.056 * height)
                            / 12.5,
                            0.7
                            * (0.053 * height if gender == "f" else 0.056 * height)
                            / 12.5,
                            0.7
                            * (0.053 * height if gender == "f" else 0.056 * height)
                            / 12.5,
                        ]
                    ),
                    4,
                )
                scales.append(scale)
            elif name == "right_foot_0":  # foot mesh
                scale = np.round(
                    np.array(
                        [
                            sgt_lengths["right_foot_0"] / 40.5,
                            sgt_lengths["right_foot_0"] / 40.5,
                            sgt_lengths["right_foot_0"] / 40.5,
                        ]
                    ),
                    4,
                )
                scales.append(scale)

        return dict(zip(meshes_names, scales))

    def _scale_pin_model(
        self,
        model: pin.Model,
        visual_model: pin.GeometryModel,
        height: float,
        weight: float,
        gender: str = "m",
    ) -> None:
        """
        Overwrite model.inertias, model.jointPlacements, and model.geometryObjects.meshScale
        based on anthropometry.
        """
        # compute parameters
        joints = self.get_dict_joint_placements(height, gender)
        inert, sgt_lengths = self.get_dict_inertial_param(height, weight, gender)
        scales = self.get_dict_meshes_scale(height, sgt_lengths, gender)

        # apply inertias
        for seg, P in inert.items():
            b_id = model.frames[model.getFrameId(seg)].parentJoint

            Ixx, Ixy, Ixz, Iyy, Iyz, Izz = P["inertia"]
            I_mat = np.array([[Ixx, Ixy, Ixz], [Ixy, Iyy, Iyz], [Ixz, Iyz, Izz]])
            model.inertias[b_id] = pin.Inertia(P["mass"], P["com"], I_mat)

        # apply joint placements
        for j, pos in joints.items():
            j_id = model.getJointId(j)
            M = model.jointPlacements[j_id]
            M.translation = pos
            model.jointPlacements[j_id] = M

        # scale visuals
        assert len(visual_model.geometryObjects.tolist()) == len(scales)
        for geom_obj in visual_model.geometryObjects:
            geom_obj.meshScale = scales[geom_obj.name]


ROBOTS = {
    "b1": B1Loader,
    "bravo7_gripper": Bravo7GripperLoader,
    "bravo7_no_ee": Bravo7NoEndEffectorLoader,
    "falcon_bravo7_no_ee": FalconBravo7NoEndEffectorLoader,
    "falcon_bravo7_gripper": FalconBravo7GripperLoader,
    "bluevolta_bravo7_no_ee": BluevoltaBravo7NoEndEffectorLoader,
    "bluevolta_bravo7_gripper": BluevoltaBravo7GripperLoader,
    "go1": Go1Loader,
    "go2": Go2Loader,
    "a1": A1Loader,
    "z1": Z1Loader,
    "b1_z1": B1Z1Loader,
    "anymal": ANYmalLoader,
    "anymal_c": ANYmalCLoader,
    "anymal_kinova": ANYmalKinovaLoader,
    "asr_twodof": AsrTwoDofLoader,
    "baxter": BaxterLoader,
    "cassie": CassieLoader,
    "double_pendulum": DoublePendulumLoader,
    "double_pendulum_continuous": DoublePendulumContinuousLoader,
    "double_pendulum_simple": DoublePendulumSimpleLoader,
    "hector": HectorLoader,
    "hextilt": HextiltLoader,
    "human": HumanLoader,
    "hyq": HyQLoader,
    "icub": ICubLoader,
    "icub_reduced": ICubReducedLoader,
    "iris": IrisLoader,
    "kinova": KinovaLoader,
    "laikago": LaikagoLoader,
    "panda": PandaLoader,
    "panda_collision": PandaLoaderCollision,
    "alex_nub_hands": AlexNubHandsLoader,
    "alex_psyonic_hands": AlexPsyonicHandsLoader,
    "alex_sake_hands": AlexSakeHandsLoader,
    "allegro_right_hand": AllegroRightHandLoader,
    "allegro_left_hand": AllegroLeftHandLoader,
    "quadruped": QuadrupedLoader,
    "romeo": RomeoLoader,
    "simple_humanoid": SimpleHumanoidLoader,
    "simple_humanoid_classical": SimpleHumanoidClassicalLoader,
    "bolt": BoltLoader,
    "borinot": BorinotLoader,
    "solo8": Solo8Loader,
    "solo12": Solo12Loader,
    "finger_edu": FingerEduLoader,
    "pr2": PR2Loader,
    "talos": TalosLoader,
    "talos_box": TalosBoxLoader,
    "talos_arm": TalosArmLoader,
    "talos_legs": TalosLegsLoader,
    "talos_full": TalosFullLoader,
    "talos_full_box": TalosFullBoxLoader,
    "tiago": TiagoLoader,
    "tiago_dual": TiagoDualLoader,
    "tiago_no_hand": TiagoNoHandLoader,
    "tiago_pro": TiagoProLoader,
    "ur3": UR5Loader,
    "ur3_gripper": UR3GripperLoader,
    "ur3_limited": UR3LimitedLoader,
    "ur5": UR5Loader,
    "ur5_gripper": UR5GripperLoader,
    "ur5_limited": UR5LimitedLoader,
    "ur10": UR10Loader,
    "ur10_limited": UR10LimitedLoader,
}


def loader(name, display=False, rootNodeName="", verbose=False):
    """Load a robot by its name, and optionally display it in a viewer."""
    if name not in ROBOTS:
        robots = ", ".join(sorted(ROBOTS.keys()))
        raise ValueError(f"Robot '{name}' not found. Possible values are {robots}")
    inst = ROBOTS[name](verbose=verbose)
    if display:
        if rootNodeName:
            inst.robot.initViewer()
            inst.robot.viz.loadViewerModel(rootNodeName=rootNodeName)
        else:
            inst.robot.initViewer(loadModel=True)
        inst.robot.display(inst.robot.q0)
    return inst


def load(name, display=False, rootNodeName="", verbose=False):
    """Load a robot by its name, and optionnaly display it in a viewer."""
    return loader(name, display, rootNodeName, verbose).robot


def load_full(name, display=False, rootNodeName="", verbose=False):
    """Load a robot by its name, optionnaly display it in a viewer,
    and provide its q0 and paths."""
    inst = loader(name, display, rootNodeName, verbose)
    return inst.robot, inst.robot.q0, inst.df_path, inst.srdf_path
