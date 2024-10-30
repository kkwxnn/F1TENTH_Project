import math
import numpy as np
from geometry_msgs.msg import PoseStamped, TransformStamped, Twist, Quaternion, Point
from roboticstoolbox.robot.ET import ET
from roboticstoolbox.robot.ERobot import ERobot
from roboticstoolbox.robot.Link import Link
from roboticstoolbox import DHRobot, RevoluteDH

from scipy.spatial.transform import Rotation as R
from spatialmath import base as smb
from spatialmath.base import symbolic as sym

# import pinocchio as pin
# import hppfcl as fcl
import numpy as np

_eps = np.finfo(np.float64).eps
from numpy import (
    array,
    ndarray,
    isnan,
    zeros,
    eye,
    expand_dims,
    empty,
    concatenate,
    cross,
    arccos,
    dot,
)

# class ResidualCollision:
#     """Class computing the residual of the collision constraint. This residual is the signed distance minus a safety margin."""

#     def __init__(self, robot_model: pin.Model, geom_model: pin.GeometryModel, geom_data: pin.GeometryData, pair_id: int, epsilon: float = 0.02):
#         """Initialize the ResidualCollision class.

#         Args:
#             robot_model (pin.Model): Robot model
#             geom_model (pin.GeometryModel): Collision model of pinocchio
#             geom_data (pin.GeometryData): Collision data of the collision model of pinocchio
#             pair_id (int): ID of the collision pair
#             epsilon (float): Safety margin for the inequality constraint
#         """
#         # Robot model
#         self._robot_model = robot_model

#         # Geometry model of the robot
#         self._geom_model = geom_model

#         # Geometry data of the robot
#         self._geom_data = geom_data

#         # Pair ID of the collisionPair
#         self._pair_id = pair_id

#         # Safety margin
#         self._epsilon = epsilon

#         # Making sure that the pair of collision exists
#         assert self._pair_id < len(self._geom_model.collisionPairs)

#         # Collision pair
#         self._collisionPair = self._geom_model.collisionPairs[self._pair_id]

#         # Geometry ID of the shape 1 of collision pair
#         self._shape1_id = self._collisionPair.first

#         # Making sure that the frame exists
#         assert self._shape1_id < len(self._geom_model.geometryObjects)

#         # Geometry object shape 1
#         self._shape1 = self._geom_model.geometryObjects[self._shape1_id]

#         # Shape 1 parent joint
#         self._shape1_parentJoint = self._shape1.parentJoint

#         # Geometry ID of the shape 2 of collision pair
#         self._shape2_id = self._collisionPair.second

#         # Making sure that the frame exists
#         assert self._shape2_id < len(self._geom_model.geometryObjects)

#         # Geometry object shape 2
#         self._shape2 = self._geom_model.geometryObjects[self._shape2_id]

#         # Shape 2 parent joint
#         self._shape2_parentJoint = self._shape2.parentJoint

#     def calc(self, q):
#         """Compute the residual of the collision constraint.

#         Args:
#             q (np.array): Configuration vector

#         Returns:
#             float: Signed distance minus the safety margin
#         """
#         return self.f(q)

#     def f(self, q):
#         """Compute the signed distance minus the safety margin between the two closest points of the 2 shapes.

#         Args:
#             q (np.array): Configuration vector

#         Returns:
#             float: Signed distance minus the safety margin
#         """
#         # Create a data object for the robot model
#         data = self._robot_model.createData()

#         # Updating the position of the joints & the geometry objects.
#         pin.updateGeometryPlacements(self._robot_model, data, self._geom_model, self._geom_data, q)

#         # Distance Request & Result from hppfcl / pydiffcol
#         req = fcl.DistanceRequest()
#         res = fcl.DistanceResult()

#         # Getting the geometry of the shape 1
#         shape1_geom = self._shape1.geometry

#         # Getting its pose in the world reference
#         shape1_placement = self._geom_data.oMg[self._shape1_id]

#         # Doing the same for the second shape.
#         shape2_geom = self._shape2.geometry
#         shape2_placement = self._geom_data.oMg[self._shape2_id]

#         # Computing the distance
#         distance = fcl.distance(
#             shape1_geom,
#             fcl.Transform3f(shape1_placement.rotation, shape1_placement.translation),
#             shape2_geom,
#             fcl.Transform3f(shape2_placement.rotation, shape2_placement.translation),
#             req,
#             res,
#         )
#         return distance - self._epsilon

#     def calcDiff(self, q):
#         """Compute the Jacobian of the collision constraint.

#         Args:
#             q (np.array): Configuration vector

#         Returns:
#             np.array: Jacobian of the collision constraint
#         """
#         return self.calcDiff_ana(q)

#     def calcDiff_ana(self, q):
#         """Compute the analytical Jacobian of the collision constraint.

#         Args:
#             q (np.array): Configuration vector

#         Returns:
#             np.array: Jacobian of the collision constraint
#         """
#         # Create a data object for the robot model
#         data = self._robot_model.createData()

#         # Compute the Jacobian for shape 1 and shape 2 frames
#         jacobian1 = pin.computeFrameJacobian(
#             self._robot_model, data, q, self._shape1_parentJoint, pin.LOCAL_WORLD_ALIGNED
#         )

#         jacobian2 = pin.computeFrameJacobian(
#             self._robot_model, data, q, self._shape2_parentJoint, pin.LOCAL_WORLD_ALIGNED
#         )

#         # Distance Request & Result from hppfcl / pydiffcol
#         req = fcl.DistanceRequest()
#         res = fcl.DistanceResult()

#         # Getting the geometry of the shape 1
#         shape1_geom = self._shape1.geometry

#         # Getting its pose in the world reference
#         shape1_placement = self._geom_data.oMg[self._shape1_id]

#         # Doing the same for the second shape.
#         shape2_geom = self._shape2.geometry
#         shape2_placement = self._geom_data.oMg[self._shape2_id]

#         # Computing the distance
#         distance = fcl.distance(
#             shape1_geom,
#             fcl.Transform3f(shape1_placement.rotation, shape1_placement.translation),
#             shape2_geom,
#             fcl.Transform3f(shape2_placement.rotation, shape2_placement.translation),
#             req,
#             res,
#         )
#         cp1 = res.getNearestPoint1()
#         cp2 = res.getNearestPoint2()

#         ## Transport the jacobian of frame 1 into the jacobian associated to cp1
#         # Vector from frame 1 center to p1
#         f1p1 = cp1 - data.oMf[self._shape1.parentJoint].translation
#         # Create transformation from frame center to cp1
#         f1Mp1 = pin.SE3(np.eye(3), f1p1)
#         # Transform the Jacobian to the nearest point
#         jacobian1 = f1Mp1.actionInverse @ jacobian1

#         ## Transport the jacobian of frame 2 into the jacobian associated to cp2
#         # Vector from frame 2 center to p2
#         f2p2 = cp2 - data.oMf[self._shape2.parentJoint].translation
#         # Create transformation from frame center to cp2
#         f2Mp2 = pin.SE3(np.eye(3), f2p2)
#         # Transform the Jacobian to the nearest point
#         jacobian2 = f2Mp2.actionInverse @ jacobian2

#         # Compute the Jacobian of the collision constraint
#         # The difference of Jacobians represents the relative motion of the closest points in the direction of the shortest distance
#         return (cp1 - cp2).T / distance @ (jacobian1[:3] - jacobian2[:3])


#     def compute_constraint_matrices(self, q):
#         """Compute the inequality constraint matrices A_in and b_in.

#         Args:
#             q (np.array): Configuration vector

#         Returns:
#             np.array, np.array: A_in and b_in matrices for the inequality constraint
#         """
#         residual = self.calc(q)
#         jacobian = self.calcDiff(q)
#         A_in = jacobian
#         b_in = np.array([residual])
#         return A_in, b_in

#     def lidar_constraints(self, lidar_readings, q, safety_margin=0.1):
#         A_in_list = []
#         b_in_list = []
#         num_readings = len(lidar_readings)
#         robot_data = self._robot_model.createData()

#         for i, distance in enumerate(lidar_readings):
#             if distance < float(1.0):  # Assuming 'inf' represents no detection
#                 angle = i * 2 * np.pi / num_readings
#                 direction = np.array([np.cos(angle), np.sin(angle), 0])
                
#                 # Assuming the lidar is mounted on the robot's base or a specific joint
#                 jacobian = pin.computeFrameJacobian(self._robot_model, robot_data, q, self._shape1.parentJoint, pin.LOCAL_WORLD_ALIGNED)
                
#                 # Project the Jacobian in the direction of the LiDAR beam
#                 projected_jacobian = direction[:2] @ jacobian[:2, :]  # Use only x and y components for a 2D LiDAR
                
#                 A_in_list.append(projected_jacobian)
#                 b_in_list.append(distance - safety_margin)

#         if not A_in_list:
#             return np.array([]), np.array([])

#         A_in = np.vstack(A_in_list)
#         b_in = np.array(b_in_list)
#         return A_in, b_in
    
class robot_class:

    def link_collision_damper(
        self,
        shape,
        q=None,
        di=0.3,
        ds=0.05,
        xi=1.0,
        end=None,
        start=None,
        collision_list=None,
    ):
        """
        Formulates an inequality contraint which, when optimised for will
        make it impossible for the robot to run into a collision. Requires
        See examples/neo.py for use case
        :param ds: The minimum distance in which a joint is allowed to
            approach the collision object shape
        :type ds: float
        :param di: The influence distance in which the velocity
            damper becomes active
        :type di: float
        :param xi: The gain for the velocity damper
        :type xi: float
        :param from_link: The first link to consider, defaults to the base
            link
        :type from_link: Link
        :param to_link: The last link to consider, will consider all links
            between from_link and to_link in the robot, defaults to the
            end-effector link
        :type to_link: Link
        :returns: Ain, Bin as the inequality contraints for an omptimisor
        :rtype: ndarray(6), ndarray(6)
        """

        end, start, _ = self._get_limit_links(start=start, end=end)

        links, n, _ = self.get_path(start=start, end=end)
        
        # if q is None:
        #     q = copy(self.q)
        # else:
        #     q = getvector(q, n)

        j = 0
        Ain = None
        bin = None

        def indiv_calculation(link, link_col, q):
            d, wTlp, wTcp = link_col.closest_point(shape, di)
            if d is not None:
                lpTcp = -wTlp + wTcp

                norm = lpTcp / d
                norm_h = expand_dims(concatenate((norm, [0, 0, 0])), axis=0)

                # tool = (self.fkine(q, end=link).inv() * SE3(wTlp)).A[:3, 3]

                # Je = self.jacob0(q, end=link, tool=tool)
                # Je[:3, :] = self._T[:3, :3] @ Je[:3, :]

                # n_dim = Je.shape[1]
                # dp = norm_h @ shape.v
                # l_Ain = zeros((1, self.n))

                Je = self.jacobe(q, start=self.base_link, end=link, tool=link_col.T)
                n_dim = Je.shape[1]
                dp = norm_h @ shape.v
                l_Ain = zeros((1, n))

                l_Ain[0, :n_dim] = norm_h @ Je
                l_bin = (xi * (d - ds) / (di - ds)) + dp
            else:
                l_Ain = None
                l_bin = None

            return l_Ain, l_bin
        
        for link in links:
            if link.isjoint:
                j += 1
        
            if collision_list is None:
                col_list = link.collision
            else:
                col_list = collision_list[j - 1]

            for link_col in col_list:
                
                l_Ain, l_bin = indiv_calculation(link, link_col, q)

                if l_Ain is not None and l_bin is not None:
                    if Ain is None:
                        Ain = l_Ain
                    else:
                        Ain = concatenate((Ain, l_Ain))

                    if bin is None:
                        bin = array(l_bin)
                    else:
                        bin = concatenate((bin, l_bin))

        return Ain, bin

    def isrot(self, R, check=False, tol=100):
        """
        Test if matrix belongs to SO(3)

        :param R: SO(3) matrix to test
        :type R: numpy(3,3)
        :param check: check validity of rotation submatrix
        :type check: bool
        :param tol: Tolerance in units of eps for rotation matrix test, defaults to 100
        :type: float
        :return: whether matrix is an SO(3) rotation matrix
        :rtype: bool

        - ``isrot(R)`` is True if the argument ``R`` is of dimension 3x3
        - ``isrot(R, check=True)`` as above, but also checks orthogonality of the
        rotation matrix.

        .. runblock:: pycon

            >>> from spatialmath.smb import *
            >>> import numpy as np
            >>> T = np.array([[1, 0, 0, 3], [0, 1, 0, 4], [0, 0, 1, 5], [0, 0, 0, 1]])
            >>> isrot(T)
            >>> R = np.array([[1, 0, 0], [0, 1, 0], [0, 0, 1]])
            >>> isrot(R)
            >>> R = R = np.array([[1, 1, 0], [0, 1, 0], [0, 0, 1]]) # invalid SO(3)
            >>> isrot(R)  # a quick check says it is an SO(3)
            >>> isrot(R, check=True) # but if we check more carefully...

        :seealso: :func:`~spatialmath.smb.transformsNd.isR` :func:`~spatialmath.smb.transforms2d.isrot2`,  :func:`~ishom`
        """
        return (
            isinstance(R, np.ndarray)
            and R.shape == (3, 3)
            and (not check or smb.isR(R, tol=tol))
        )


    def tr2rpy(self, T, unit="rad", order="zyx", check=False):
        r"""
        Convert SO(3) or SE(3) to roll-pitch-yaw angles

        :param R: SE(3) or SO(3) matrix
        :type R: ndarray(4,4) or ndarray(3,3)
        :param unit: 'rad' or 'deg'
        :type unit: str
        :param order: 'xyz', 'zyx' or 'yxz' [default 'zyx']
        :type order: str
        :param check: check that rotation matrix is valid
        :type check: bool
        :return: Roll-pitch-yaw angles
        :rtype: ndarray(3)
        :raises ValueError: bad arguments

        ``tr2rpy(R)`` are the roll-pitch-yaw angles corresponding to
        the rotation part of ``R``.

        The 3 angles RPY = :math:`[\theta_R, \theta_P, \theta_Y]` correspond to
        sequential rotations about the Z, Y and X axes respectively.  The axis order
        sequence can be changed by setting:

        - ``order='xyz'``  for sequential rotations about X, Y, Z axes
        - ``order='yxz'``  for sequential rotations about Y, X, Z axes

        By default the angles are in radians but can be changed setting
        ``unit='deg'``.

        .. runblock:: pycon

            >>> from spatialmath.smb import *
            >>> T = rpy2tr(0.2, 0.3, 0.5)
            >>> print(T)
            >>> tr2rpy(T)

        .. note::

            - There is a singularity for the case where :math:`\theta_P = \pi/2` in
            which case we arbitrarily set :math:`\theta_R=0` and
            :math:`\theta_Y = \theta_R + \theta_Y`.
            - If the input is SE(3) the translation component is ignored.

        :seealso: :func:`~rpy2r` :func:`~rpy2tr` :func:`~tr2eul`,
                :func:`~tr2angvec`
        :SymPy: not supported
        """

        if smb.ismatrix(T, (4, 4)):
            R = smb.t2r(T)
        else:
            R = T
        if not self.isrot(R, check=check):
            raise ValueError("not a valid SO(3) matrix")

        rpy = np.zeros((3,))
        if order in ("xyz", "arm"):

            # XYZ order
            if abs(abs(R[0, 2]) - 1) < 10 * _eps:  # when |R13| == 1
                # singularity
                rpy[0] = 0  # roll is zero
                if R[0, 2] > 0:
                    rpy[2] = math.atan2(R[2, 1], R[1, 1])  # R+Y
                else:
                    rpy[2] = -math.atan2(R[1, 0], R[2, 0])  # R-Y
                rpy[1] = math.asin(np.clip(R[0, 2], -1.0, 1.0))
            else:
                rpy[0] = -math.atan2(R[0, 1], R[0, 0])
                rpy[2] = -math.atan2(R[1, 2], R[2, 2])

                k = np.argmax(np.abs([R[0, 0], R[0, 1], R[1, 2], R[2, 2]]))
                if k == 0:
                    rpy[1] = math.atan(R[0, 2] * math.cos(rpy[0]) / R[0, 0])
                elif k == 1:
                    rpy[1] = -math.atan(R[0, 2] * math.sin(rpy[0]) / R[0, 1])
                elif k == 2:
                    rpy[1] = -math.atan(R[0, 2] * math.sin(rpy[2]) / R[1, 2])
                elif k == 3:
                    rpy[1] = math.atan(R[0, 2] * math.cos(rpy[2]) / R[2, 2])

        elif order in ("zyx", "vehicle"):

            # old ZYX order (as per Paul book)
            if abs(abs(R[2, 0]) - 1) < 10 * _eps:  # when |R31| == 1
                # singularity
                rpy[0] = 0  # roll is zero
                if R[2, 0] < 0:
                    rpy[2] = -math.atan2(R[0, 1], R[0, 2])  # R-Y
                else:
                    rpy[2] = math.atan2(-R[0, 1], -R[0, 2])  # R+Y
                rpy[1] = -math.asin(np.clip(R[2, 0], -1.0, 1.0))
            else:
                rpy[0] = math.atan2(R[2, 1], R[2, 2])  # R
                rpy[2] = math.atan2(R[1, 0], R[0, 0])  # Y

                k = np.argmax(np.abs([R[0, 0], R[1, 0], R[2, 1], R[2, 2]]))
                if k == 0:
                    rpy[1] = -math.atan(R[2, 0] * math.cos(rpy[2]) / R[0, 0])
                elif k == 1:
                    rpy[1] = -math.atan(R[2, 0] * math.sin(rpy[2]) / R[1, 0])
                elif k == 2:
                    rpy[1] = -math.atan(R[2, 0] * math.sin(rpy[0]) / R[2, 1])
                elif k == 3:
                    rpy[1] = -math.atan(R[2, 0] * math.cos(rpy[0]) / R[2, 2])

        elif order in ("yxz", "camera"):

            if abs(abs(R[1, 2]) - 1) < 10 * _eps:  # when |R23| == 1
                # singularity
                rpy[0] = 0
                if R[1, 2] < 0:
                    rpy[2] = -math.atan2(R[2, 0], R[0, 0])  # R-Y
                else:
                    rpy[2] = math.atan2(-R[2, 0], -R[2, 1])  # R+Y
                rpy[1] = -math.asin(np.clip(R[1, 2], -1.0, 1.0))  # P
            else:
                rpy[0] = math.atan2(R[1, 0], R[1, 1])
                rpy[2] = math.atan2(R[0, 2], R[2, 2])

                k = np.argmax(np.abs([R[1, 0], R[1, 1], R[0, 2], R[2, 2]]))
                if k == 0:
                    rpy[1] = -math.atan(R[1, 2] * math.sin(rpy[0]) / R[1, 0])
                elif k == 1:
                    rpy[1] = -math.atan(R[1, 2] * math.cos(rpy[0]) / R[1, 1])
                elif k == 2:
                    rpy[1] = -math.atan(R[1, 2] * math.sin(rpy[2]) / R[0, 2])
                elif k == 3:
                    rpy[1] = -math.atan(R[1, 2] * math.cos(rpy[2]) / R[2, 2])

        else:
            raise ValueError("Invalid order")

        if unit == "deg":
            rpy *= 180 / math.pi

        return rpy


    def quaternion_to_euler(self, w, x, y, z):
        # Roll (x-axis rotation)
        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = math.atan2(sinr_cosp, cosr_cosp)

        # Pitch (y-axis rotation)
        sinp = 2 * (w * y - z * x)
        if abs(sinp) >= 1:
            pitch = math.copysign(math.pi / 2, sinp)  # Use 90 degrees if out of range
        else:
            pitch = math.asin(sinp)

        # Yaw (z-axis rotation)
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        return roll, pitch, yaw
    
    def quaternion_from_euler(self, ai, aj, ak):
        ai /= 2.0
        aj /= 2.0
        ak /= 2.0
        ci = math.cos(ai)
        si = math.sin(ai)
        cj = math.cos(aj)
        sj = math.sin(aj)
        ck = math.cos(ak)
        sk = math.sin(ak)
        cc = ci*ck
        cs = ci*sk
        sc = si*ck
        ss = si*sk

        q = np.empty((4, ))
        q[0] = cj*sc - sj*cs
        q[1] = cj*ss + sj*cc
        q[2] = cj*cs - sj*sc
        q[3] = cj*cc + sj*ss

        return q

    def quaternion_to_matrix(self, x, y, z, w):
        # Convert quaternion to a rotation matrix
        qx, qy, qz, qw = x, y, z, w
        return np.array([
            [1.0 - 2.0*qy**2.0 - 2.0*qz**2.0, 2.0*qx*qy - 2.0*qz*qw, 2.0*qx*qz + 2.0*qy*qw, 0.0],
            [2.0*qx*qy + 2.0*qz*qw, 1.0 - 2.0*qx**2.0 - 2.0*qz**2.0, 2.0*qy*qz - 2.0*qx*qw, 0.0],
            [2.0*qx*qz - 2.0*qy*qw, 2.0*qy*qz + 2.0*qx*qw, 1.0 - 2.0*qx**2.0 - 2.0*qy**2.0, 0.0],
            [0.0, 0.0, 0.0, 1.0]
        ], dtype=np.float64)
    
    def pose_stamped_to_matrix(self, pose_stamped):
        
        translation = pose_stamped.pose.position
        rotation = pose_stamped.pose.orientation

        translation_matrix = np.identity(4)
        translation_matrix[:3, 3] = [translation.x, translation.y, translation.z]

        rotation_matrix = self.quaternion_to_matrix(rotation.x, rotation.y, rotation.z, rotation.w)

        homogeneous_matrix = np.dot(translation_matrix, rotation_matrix)
        return homogeneous_matrix
    
    def euler_to_quaternion(self, roll, pitch, yaw):
        quaternion = Quaternion()
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)

        quaternion.w = cr * cp * cy + sr * sp * sy
        quaternion.x = sr * cp * cy - cr * sp * sy
        quaternion.y = cr * sp * cy + sr * cp * sy
        quaternion.z = cr * cp * sy - sr * sp * cy

        return quaternion
    
    def matrix_to_pose_stamped(self, matrix, frame_id):
        pose_stamped = PoseStamped()
        pose_stamped.header.stamp = self.get_clock().now().to_msg()
        pose_stamped.header.frame_id = frame_id

        translation = matrix[:3, 3]
        rotation = matrix[:3, :3]

        # Set the position (translation)
        pose_stamped.pose.position = Point(x=translation[0], y=translation[1], z=translation[2])

        # Set the orientation (rotation)
        quaternion = self.matrix_to_quaternion(rotation)
        pose_stamped.pose.orientation = quaternion

        return pose_stamped
    
    def rpy_to_matrix(self, r, p, y):
        Rx = np.array([[1.0, 0.0, 0.0, 0.0],
                    [0.0, np.cos(r), -np.sin(r), 0.0],
                    [0.0, np.sin(r), np.cos(r), 0.0],
                    [0.0, 0.0, 0.0, 1.0]], dtype=np.float64)

        Ry = np.array([[np.cos(p), 0.0, np.sin(p), 0.0],
                    [0.0, 1.0, 0.0, 0.0],
                    [-np.sin(p), 0.0, np.cos(p), 0.0],
                    [0.0, 0.0, 0.0, 1.0]], dtype=np.float64)

        Rz = np.array([[np.cos(y), -np.sin(y), 0.0, 0.0],
                    [np.sin(y), np.cos(y), 0.0, 0.0],
                    [0.0, 0.0, 1.0, 0.0],
                    [0.0, 0.0, 0.0, 1.0]], dtype=np.float64)

        R = np.dot(Rx,np.dot(Rz,Ry))# Rz @ Ry @ Rx

        return R

    def create_transformation(self, x=0, y=0, z=0, roll=0, pitch=0, yaw=0):
        translation_matrix = np.identity(4)
        translation_matrix[:3, 3] = [x, y, z]
        rotation_matrix = self.rpy_to_matrix(roll, pitch, yaw)
        homogeneous_matrix = np.dot(translation_matrix, rotation_matrix)
        return homogeneous_matrix

    def position_to_matrix(self, position):
        # transform = position
        # print(position)
        translation = position.position
        rotation = position.orientation

        translation_matrix = np.identity(4)
        translation_matrix[:3, 3] = [translation.x, translation.y, translation.z]

        rotation_matrix = self.quaternion_to_matrix(rotation.x, rotation.y, rotation.z, rotation.w)

        homogeneous_matrix = np.dot(translation_matrix, rotation_matrix)
        return homogeneous_matrix
    
    def transform_stamped_to_matrix(self, transform_stamped):
        transform = transform_stamped.transform
        translation = transform.translation
        rotation = transform.rotation

        translation_matrix = np.identity(4)
        translation_matrix[:3, 3] = [translation.x, translation.y, translation.z]

        rotation_matrix = self.quaternion_to_matrix(rotation.x, rotation.y, rotation.z, rotation.w)

        homogeneous_matrix = np.dot(translation_matrix, rotation_matrix)
        return homogeneous_matrix

    def euler_to_quaternion(self, roll, pitch, yaw):
        quaternion = Quaternion()
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)

        quaternion.w = cr * cp * cy + sr * sp * sy
        quaternion.x = sr * cp * cy - cr * sp * sy
        quaternion.y = cr * sp * cy + sr * cp * sy
        quaternion.z = cr * cp * sy - sr * sp * cy

        return quaternion
    
    def position_to_matrix(self, position):
        # transform = position
        # print(position)
        translation = position.position
        rotation = position.orientation

        translation_matrix = np.identity(4)
        translation_matrix[:3, 3] = [translation.x, translation.y, translation.z]

        rotation_matrix = self.quaternion_to_matrix(rotation.x, rotation.y, rotation.z, rotation.w)

        homogeneous_matrix = np.dot(translation_matrix, rotation_matrix)
        return homogeneous_matrix
    
    def matrix_to_pose_stamped(self, matrix, frame_id, stamp):
        pose_stamped = PoseStamped()
        pose_stamped.header.stamp = stamp
        pose_stamped.header.frame_id = frame_id

        translation = matrix[:3, 3]
        rotation = matrix[:3, :3]

        # Set the position (translation)
        pose_stamped.pose.position = Point(x=translation[0], y=translation[1], z=translation[2])

        # Set the orientation (rotation)
        quaternion = self.matrix_to_quaternion(rotation)
        pose_stamped.pose.orientation = quaternion

        return pose_stamped
    
    def matrix_to_quaternion(self, r):
        # Compute the trace of the matrix
        trace = r[0, 0] + r[1, 1] + r[2, 2]

        if trace > 0:
            # If the trace is positive
            s = 0.5 / np.sqrt(trace + 1.0)
            qw = 0.25 / s
            qx = (r[2, 1] - r[1, 2]) * s
            qy = (r[0, 2] - r[2, 0]) * s
            qz = (r[1, 0] - r[0, 1]) * s
        else:
            # If the trace is not positive
            if r[0, 0] > r[1, 1] and r[0, 0] > r[2, 2]:
                s = 2.0 * np.sqrt(1.0 + r[0, 0] - r[1, 1] - r[2, 2])
                qw = (r[2, 1] - r[1, 2]) / s
                qx = 0.25 * s
                qy = (r[0, 1] + r[1, 0]) / s
                qz = (r[0, 2] + r[2, 0]) / s
            elif r[1, 1] > r[2, 2]:
                s = 2.0 * np.sqrt(1.0 + r[1, 1] - r[0, 0] - r[2, 2])
                qw = (r[0, 2] - r[2, 0]) / s
                qx = (r[0, 1] + r[1, 0]) / s
                qy = 0.25 * s
                qz = (r[1, 2] + r[2, 1]) / s
            else:
                s = 2.0 * np.sqrt(1.0 + r[2, 2] - r[0, 0] - r[1, 1])
                qw = (r[1, 0] - r[0, 1]) / s
                qx = (r[0, 2] + r[2, 0]) / s
                qy = (r[1, 2] + r[2, 1]) / s
                qz = 0.25 * s

        return Quaternion(x=qx, y=qy, z=qz, w=qw)

class UR5(ERobot):

    def __init__(self, symbolic=False):

        if symbolic:
            import spatialmath.base.symbolic as sym

            zero = sym.zero()
            pi = sym.pi()
        else:
            from math import pi

            zero = 0.0

        deg = pi / 180
        inch = 0.0254

        # robot length values (metres)
        a = [0, -0.42500, -0.39225, 0, 0, 0]
        d = [0.089459, 0, 0, 0.10915, 0.09465, 0.0823]

        alpha = [pi / 2, zero, zero, pi / 2, -pi / 2, zero]

        # mass data, no inertia available
        mass = [3.7000, 8.3930, 2.33, 1.2190, 1.2190, 0.1897]
        center_of_mass = [
            [0, -0.02561, 0.00193],
            [0.2125, 0, 0.11336],
            [0.15, 0, 0.0265],
            [0, -0.0018, 0.01634],
            [0, -0.0018, 0.01634],
            [0, 0, -0.001159],
        ]
        qdlim = np.array(
            [4.0, 4.0, 2.1750, 2.1750, 2.1750, 2.1750, 2.6100, 2.6100]
        )
        # lx = Link(ET.tx() , name="px_link", parent=None)
        # ly = Link(ET.ty() , name="py_link", parent=None)
        # rz = Link(ET.Rz() , name="rz_link", parent=None)
        l_base0 = Link(ET.Rz() , name="base_0", parent=None)
        l_base1 = Link(ET.tx() , name="base_1", parent=None, qlim = [-100.0, 100.0])
        l0 = Link(ET.tz(0.8) * ET.Rz(math.pi) * ET.Rz() , name="ur_base_link", parent=None)
        l1 = Link(ET.tz(d[0]) * ET.tx(a[0]) * ET.Rx(alpha[0]) * ET.Rz() , name="shoulder_link", parent=None)
        l2 = Link(ET.tz(d[1]) * ET.tx(a[1]) * ET.Rx(alpha[1]) * ET.Rz() , name="upper_arm_link", parent=None)
        l3 = Link(ET.tz(d[2]) * ET.tx(a[2]) * ET.Rx(alpha[2]) * ET.Rz() , name="forearm_link", parent=None)
        l4 = Link(ET.tz(d[3]) * ET.tx(a[3]) * ET.Rx(alpha[3]) * ET.Rz() , name="wrist_1_link", parent=None)
        l5 = Link(ET.tz(d[4]) * ET.tx(a[4]) * ET.Rx(alpha[4]) * ET.Rz() , name="wrist_2_link", parent=None)
        l6 = Link(ET.tz(d[5]) * ET.tx(a[5]) * ET.Rx(alpha[5]), name="wrist_3_link", parent=None)
        l7 = Link(ET.tz(0.15), name="end_effector", parent=None)

        elinks = [l_base0, l_base1, l0, l1, l2, l3, l4, l5, l6, l7]

        super(UR5, self).__init__(elinks, name="UR5", manufacturer="Universal Robot")

class UR5Real(ERobot):

    def __init__(self, symbolic=False):

        if symbolic:
            import spatialmath.base.symbolic as sym

            zero = sym.zero()
            pi = sym.pi()
        else:
            from math import pi

            zero = 0.0

        deg = pi / 180
        inch = 0.0254

        # robot length values (metres)
        a = [0, -0.42500, -0.39225, 0, 0, 0]
        d = [0.089459, 0, 0, 0.10915, 0.09465, 0.0823]

        alpha = [pi / 2, zero, zero, pi / 2, -pi / 2, zero]

        # mass data, no inertia available
        mass = [3.7000, 8.3930, 2.33, 1.2190, 1.2190, 0.1897]
        center_of_mass = [
            [0, -0.02561, 0.00193],
            [0.2125, 0, 0.11336],
            [0.15, 0, 0.0265],
            [0, -0.0018, 0.01634],
            [0, -0.0018, 0.01634],
            [0, 0, -0.001159],
        ]
        qdlim = np.array(
            [4.0, 4.0, 2.1750, 2.1750, 2.1750, 2.1750, 2.6100, 2.6100]
        )
        # lx = Link(ET.tx() , name="px_link", parent=None)
        # ly = Link(ET.ty() , name="py_link", parent=None)
        # rz = Link(ET.Rz() , name="rz_link", parent=None)
        l_base0 = Link(ET.Rz() , name="base_0", parent=None)
        l_base1 = Link(ET.tx() , name="base_1", parent=None, qlim = [-100.0, 100.0])
        l0 = Link(ET.tz(0.0) * ET.Rz(math.pi) * ET.Rz() , name="ur_base_link", parent=None)
        l1 = Link(ET.tz(d[0]) * ET.tx(a[0]) * ET.Rx(alpha[0]) * ET.Rz() , name="shoulder_link", parent=None)
        l2 = Link(ET.tz(d[1]) * ET.tx(a[1]) * ET.Rx(alpha[1]) * ET.Rz() , name="upper_arm_link", parent=None)
        l3 = Link(ET.tz(d[2]) * ET.tx(a[2]) * ET.Rx(alpha[2]) * ET.Rz() , name="forearm_link", parent=None)
        l4 = Link(ET.tz(d[3]) * ET.tx(a[3]) * ET.Rx(alpha[3]) * ET.Rz() , name="wrist_1_link", parent=None)
        l5 = Link(ET.tz(d[4]) * ET.tx(a[4]) * ET.Rx(alpha[4]) * ET.Rz() , name="wrist_2_link", parent=None)
        l6 = Link(ET.tz(d[5]) * ET.tx(a[5]) * ET.Rx(alpha[5]), name="wrist_3_link", parent=None)
        l7 = Link(ET.tz(0.236) * ET.Rz(-135*pi/180), name="end_effector", parent=None)
        

        elinks = [l_base0, l_base1, l0, l1, l2, l3, l4, l5, l6, l7]

        super(UR5Real, self).__init__(elinks, name="UR5", manufacturer="Universal Robot")
