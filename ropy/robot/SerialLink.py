#!/usr/bin/env python
"""
Created on August 1 2019
@author: Jesse Haviland
"""

import numpy as np
from ropy.robot.Link import Link
from spatialmath.base.argcheck import \
    getvector, ismatrix, isscalar, verifymatrix
from spatialmath.base.transforms3d import tr2delta
from spatialmath import SE3
from scipy.optimize import minimize, Bounds


class SerialLink(object):
    """
    A superclass for arm type robots. A concrete class that represents a
    serial-link arm-type robot.  Each link and joint in the chain is
    described by a Link-class object using Denavit-Hartenberg parameters
    (standard or modified).

    Note: Link subclass elements passed in must be all standard, or all
          modified, DH parameters.

    :param name: Name of the robot
    :type name: string
    :param manufacturer: Manufacturer of the robot
    :type manufacturer: string
    :param base: Locaation of the base
    :type base: float np.ndarray(4,4)
    :param tool: Location of the tool
    :type tool: float np.ndarray(4,4)
    :param links: Series of links which define the robot
    :type links: List[n]
    :param mdh: 0 if standard D&H, else 1
    :type mdh: int
    :param n: Number of joints in the robot
    :type n: int
    :param T: The current pose of the robot
    :type T: float np.ndarray(4,4)
    :param q: The current joint angles of the robot
    :type q: float np.ndarray(1,n)

    Examples
    --------
    >>> L[0] = Revolute('d', 0, 'a', a1, 'alpha', np.pi/2)

    >>> L[1] = Revolute('d', 0, 'a', a2, 'alpha', 0)

    >>> twolink = SerialLink(L, 'name', 'two link');

    See Also
    --------
    ropy.robot.ets : A superclass which represents the kinematics of a
                     serial-link manipulator
    ropy.robot.Link : A link superclass for all link types
    ropy.robot.Revolute : A revolute link class

    Reference::
    - Robotics, Vision & Control, Chaps 7-9,
      P. Corke, Springer 2011.
    - Robot, Modeling & Control,
      M.Spong, S. Hutchinson & M. Vidyasagar, Wiley 2006.
    """

    def __init__(
            self,
            L,
            name='noname',
            manufacturer='',
            base=SE3(),
            tool=SE3(),
            gravity=np.array([[0], [0], [9.81]])
            ):

        self.name = name
        self.manuf = manufacturer
        self.base = base
        self.tool = tool
        self.gravity = gravity

        super(SerialLink, self).__init__()

        # Verify L
        if not isinstance(L, list):
            raise TypeError('The links L must be stored in a list.')

        length = len(L)
        self._links = []
        self._n = 0

        for i in range(length):
            if isinstance(L[i], Link):
                self._links.append(L[i])
                self._n += 1

            elif isinstance(L[i], SerialLink):
                for j in range(L[i].n):
                    self._links.append(L[i].links[j])
                    self._n += 1

            else:
                raise TypeError("Input can be only Link or SerialLink")

        # Current joint angles of the robot
        self.q = np.zeros(self.n)

        # Check the DH convention
        self._check_dh()

    def __add__(self, L):
        nlinks = []

        # TODO - Should I do a deep copy here a physically copy the Links
        # and not just the references?
        # Copy Link references to new list
        for i in range(self.n):
            nlinks.append(self.links[i])

        if isinstance(L, Link):
            nlinks.append(L)
        elif isinstance(L, SerialLink):
            for j in range(L.n):
                nlinks.append(L.links[j])
        else:
            raise TypeError("Can only combine SerialLinks with other "
                            "SerialLinks or Links")

        return SerialLink(
            nlinks,
            name=self.name,
            manufacturer=self.manuf,
            base=self.base,
            tool=self.tool,
            gravity=self.gravity)

    def _check_dh(self):
        self._mdh = self.links[0].mdh
        for i in range(self.n):
            if not self.links[i].mdh == self.mdh:
                raise ValueError('Robot has mixed D&H links conventions.')

    @property
    def name(self):
        return self._name

    @property
    def manuf(self):
        return self._manuf

    @property
    def links(self):
        return self._links

    @property
    def base(self):
        return self._base

    @property
    def tool(self):
        return self._tool

    @property
    def gravity(self):
        return self._gravity

    @property
    def n(self):
        return self._n

    @property
    def mdh(self):
        return self._mdh

    @property
    def q(self):
        return self._q

    @property
    def d(self):
        v = []
        for i in range(self.n):
            v.append(self.links[i].d)
        return v

    @property
    def a(self):
        v = []
        for i in range(self.n):
            v.append(self.links[i].a)
        return v

    @property
    def theta(self):
        v = []
        for i in range(self.n):
            v.append(self.links[i].theta)
        return v

    @property
    def r(self):
        v = np.copy(self.links[0].r)
        for i in range(1, self.n):
            v = np.c_[v, self.links[i].r]
        return v

    @property
    def offset(self):
        v = []
        for i in range(self.n):
            v.append(self.links[i].offset)
        return v

    @property
    def qlim(self):
        v = np.copy(self.links[0].qlim)
        for i in range(1, self.n):
            v = np.c_[v, self.links[i].qlim]
        return v

    @manuf.setter
    def manuf(self, manuf_new):
        self._manuf = manuf_new

    @gravity.setter
    def gravity(self, gravity_new):
        self._gravity = getvector(gravity_new, 3, 'col')

    @name.setter
    def name(self, name_new):
        self._name = name_new

    @base.setter
    def base(self, T):
        if not isinstance(T, SE3):
            T = SE3(T)
        self._base = T

    @tool.setter
    def tool(self, T):
        if not isinstance(T, SE3):
            T = SE3(T)
        self._tool = T

    @q.setter
    def q(self, q_new):
        self._q = getvector(q_new, self.n)

    def A(self, joints, q=None):
        """
        Transforms between link frames for the J'th joint.  q is a vector
        (1xN) of joint variables. For:
        - standard DH parameters, this is from frame {J-1} to frame {J}.
        - modified DH parameters, this is from frame {J} to frame {J+1}.

        Notes:
        - Base and tool transforms are not applied.

        :param joints:
        :type joints: int, tuple or 2 element list
        :param q: The joint angles/configuration of the robot (Optional,
            if not supplied will use the stored q values)
        :type q: float np.ndarray(1,n)

        :return T: The transform between link 0 and joints or joints[0]
            and joints[1]
        :rtype T: SE3
        """

        if not isscalar(joints):
            joints = getvector(joints, 2)
            j0 = joints[0]
            jn = joints[1]
        else:
            j0 = 0
            jn = joints

        jn += 1

        if jn > self.n:
            raise ValueError("The joints value out of range")

        if q is None:
            q = np.copy(self.q)
        else:
            q = getvector(q, self.n)

        T = SE3()

        for i in range(j0, jn):
            T = T * self.links[i].A(q[i])

        return T

    def islimit(self, q=None):
        """
        Joint limit test

        :param q: The joint angles/configuration of the robot (Optional,
            if not supplied will use the stored q values)
        :type q: float np.ndarray(1,n)

        :return v: is a vector of boolean values, one per joint, False if q[i]
            is within the joint limits, else True
        :rtype v: bool list
        """

        if q is None:
            q = np.copy(self.q)
        else:
            q = getvector(q, self.n)

        v = []

        for i in range(self.n):
            v.append(self.links[i].islimit(q[i]))

        return v

    def isspherical(self):
        """
        Test for spherical wrist. Tests if the robot has a spherical wrist,
        that is, the last 3 axes are revolute and their axes intersect at
        a point.

        :return: True if spherical wrist
        :rtype: bool
        """
        if self.n < 3:
            return False

        L = self.links[self.n-3:self.n]

        alpha = [-np.pi/2, np.pi/2]

        if L[0].a == 0 and L[1].a == 0 and \
                L[1].d == 0 and \
                (
                    (L[0].alpha == alpha[0] and L[1].alpha == alpha[1]) or
                    (L[0].alpha == alpha[1] and L[1].alpha == alpha[0])
                ) and \
                L[0].sigma == 0 and L[1].sigma == 0 and L[2].sigma == 0:

            return True
        else:
            return False

    def payload(self, m, p=np.zeros(3)):
        """
        Add payload mass adds a payload with point mass m at position p
        in the end-effector coordinate frame. payload(0) removes added
        payload.

        :param m: mass (kg)
        :type m: float
        :param p: position in end-effector frame
        :type p: float np.ndarray(3,1)
        """

        p = getvector(p, 3, out='col')
        lastlink = self.links[self.n-1]

        lastlink.m = m
        lastlink.r = p

    def jointdynamics(self, q, qd):
        """
        Transfer function of joint actuator. Returns a vector of N
        continuous-time transfer function objects that represent the
        transfer function 1/(Js+B) for each joint based on the dynamic
        parameters of the robot and the configuration q (1xN).
        n is the number of robot joints.

        :param q: The joint angles/configuration of the robot
        :type q: float np.ndarray(1,n)
        :param qd: The joint velocities of the robot
        :type qd: float np.ndarray(1,n)
        """

        # TODO a tf object implementation?
        pass

    def isprismatic(self):
        """
        Identify prismatic joints

        :return: a list of bool variables, one per joint, true if
            the corresponding joint is prismatic, otherwise false.
        :rtype: bool list
        """

        p = []

        for i in range(self.n):
            p.append(self.links[i].isprismatic())

        return p

    def isrevolute(self):
        """
        Identify revolute joints

        :return: a list of bool variables, one per joint, true if
            the corresponding joint is revolute, otherwise false.
        :rtype: bool list
        """

        p = []

        for i in range(self.n):
            p.append(self.links[i].isrevolute())

        return p

    def todegrees(self, q=None):
        """
        Convert joint angles to degrees

        :param q: The joint angles/configuration of the robot (Optional,
            if not supplied will use the stored q values)
        :type q: float np.ndarray(1,n)

        :return: a vector of joint coordinates where those elements
            corresponding to revolute joints are converted from radians to
            degrees. Elements corresponding to prismatic joints are copied
            unchanged.
        :rtype: float np.ndarray(n,)
        """

        if q is None:
            qdeg = np.copy(self.q)
        else:
            qdeg = getvector(q, self.n)

        k = self.isrevolute()
        qdeg[k] *= 180 / np.pi
        return qdeg

    def toradians(self, q=None):
        """
        Convert joint angles to radians

        :param q: The joint angles/configuration of the robot (Not optional,
            stored q is always radians)
        :type q: float np.ndarray(1,n)

        :return: a vector of joint coordinates where those elements
            corresponding to revolute joints are converted from degrees to
            radians. Elements corresponding to prismatic joints are copied
            unchanged.
        :rtype: float np.ndarray(n,)
        """

        if q is None:
            qrad = np.copy(self.q)
        else:
            qrad = getvector(q, self.n)

        k = self.isrevolute()
        qrad[k] *= np.pi / 180
        return qrad

    def twists(self, q=None):
        """
        Joint axis twists. Calculates a vector of Twist objects tw (1xN) that
        represent the axes of the joints for the robot with joint coordinates
        q (1xN). Also returns T0 which is an SE3 object representing the pose
        of the tool.

        :param q: The joint angles/configuration of the robot (Optional,
            if not supplied will use the stored q values)
        :type q: float np.ndarray(n,)

        :return tw: a vector of Twist objects
        :rtype tw: float np.ndarray(n,)
        :return T0: Represents the pose of the tool
        :rtype T0: SE3
        """

        if q is None:
            q = np.copy(self.q)
        else:
            q = getvector(q, self.n)

        # TODO Implement this

    def fkine(self, q=None):
        '''
        Evaluate fkine for each point on a trajectory of joints q

        Note:
        - The robot's base or tool transform, if present, are incorporated
            into the result.
        - Joint offsets, if defined, are added to q before the forward
            kinematics are computed.

        :param q: The joint angles/configuration of the robot (Optional,
            if not supplied will use the stored q values). If q is a matrix
            the rows are interpreted as the generalized joint coordinates
            for a sequence of points along a trajectory. q(j,i) is the
            j'th joint parameter for the i'th trajectory point.
        :type q: float np.ndarray(n) or (nxm)

        :return T: Homogeneous transformation matrix or trajectory
        :rtype T: SE3 or SE3 list
        '''

        cols = 0
        if q is None:
            q = np.copy(self.q)
        elif isinstance(q, np.ndarray) and q.ndim == 2 and q.shape[1] > 1:
            cols = q.shape[1]
            ismatrix(q, (self.n, cols))
        else:
            q = getvector(q, self.n)

        if cols == 0:
            # Single configuration
            t = self.base
            for i in range(self.n):
                t = t * self.links[i].A(q[i])
            t = t * self.tool
        else:
            # Trajectory

            for i in range(cols):
                tr = self.base
                for j in range(self.n):
                    tr *= self.links[j].A(q[j, i])
                tr = tr * self.tool

                if i == 0:
                    t = SE3(tr)
                else:
                    t.append(tr)

        return t

    def allfkine(self, q=None):
        '''
        Evaluate fkine for each joint within a robot

        Note:
        - The robot's base or tool transform, if present, are incorporated
            into the result.
        - Joint offsets, if defined, are added to q before the forward
            kinematics are computed.

        :param q: The joint angles/configuration of the robot (Optional,
            if not supplied will use the stored q values).
        :type q: float np.ndarray(n)

        :return T: Homogeneous transformationtrajectory
        :rtype T: SE3 list
        '''

        if q is None:
            q = np.copy(self.q)
        else:
            q = getvector(q, self.n)

        t = self.base
        Tall = SE3()
        for i in range(self.n):
            t = t * self.links[i].A(q[i])

            if i == 0:
                Tall = SE3(t)
            else:
                Tall.append(t)

        return Tall

    def jacobe(self, q=None):
        """
        The manipulator Jacobian matrix maps joint velocity to end-effector
        spatial velocity v = Je*qd in the end-effector frame.

        :param q: The joint angles/configuration of the robot (Optional,
            if not supplied will use the stored q values).
        :type q: float np.ndarray(1,n)

        :return J: The manipulator Jacobian in ee frame
        :rtype: float np.ndarray(6,n)
        """

        if q is None:
            q = np.copy(self.q)
        else:
            q = getvector(q, self.n)

        n = self.n
        L = self.links
        J = np.zeros((6, self.n))

        U = self.tool.A

        for j in range(n-1, -1, -1):
            if self.mdh == 0:
                # standard DH convention
                U = L[j].A(q[j]).A @ U

            if not L[j].sigma:
                # revolute axis
                d = np.array([
                    [-U[0, 0] * U[1, 3] + U[1, 0] * U[0, 3]],
                    [-U[0, 1] * U[1, 3] + U[1, 1] * U[0, 3]],
                    [-U[0, 2] * U[1, 3] + U[1, 2] * U[0, 3]]
                ])
                delta = np.expand_dims(U[2, :3], axis=1)  # nz oz az
            else:
                # prismatic axis
                d = np.expand_dims(U[2, :3], axis=1)      # nz oz az
                delta = np.zeros((3, 1))

            J[:, j] = np.squeeze(np.concatenate((d, delta)))

            if self.mdh != 0:
                # modified DH convention
                U = L[j].A(q[j]).A @ U

        return J

    def jacob0(self, q=None):
        """
        The manipulator Jacobian matrix maps joint velocity to end-effector
        spatial velocity v = Je*qd in the end-effector frame.

        :param q: The joint angles/configuration of the robot (Optional,
            if not supplied will use the stored q values).
        :type q: float np.ndarray(1,n)

        :return J: The manipulator Jacobian in ee frame
        :rtype: float np.ndarray(6,n)
        """

        if q is None:
            q = np.copy(self.q)
        else:
            q = getvector(q, self.n)

        J0 = self.jacob0v(q) @ self.jacobe(q)

        return J0

    def jacob0v(self, q=None):
        """
        The spatial velocity Jacobian which relates the velocity in
        end-effector frame to velocity in the base frame.

        :param q: The joint angles/configuration of the robot (Optional,
            if not supplied will use the stored q values).
        :type q: float np.ndarray(1,n)

        :returns J: The velocity Jacobian in 0 frame
        :rtype J: float np.ndarray(6,n)
        """

        if q is None:
            q = np.copy(self.q)
        else:
            q = getvector(q, self.n)

        r = self.fkine(q).R

        Jv = np.zeros((6, 6))
        Jv[:3, :3] = r
        Jv[3:, 3:] = r

        return Jv

    def jacobev(self, q=None):
        """
        The spatial velocity Jacobian which relates the velocity in
        the base frame to the velocity in the end-effector frame.

        :param q: The joint angles/configuration of the robot (Optional,
            if not supplied will use the stored q values).
        :type q: float np.ndarray(1,n)

        :returns J: The velocity Jacobian in ee frame
        :rtype J: float np.ndarray(6,n)
        """

        if q is None:
            q = np.copy(self.q)
        else:
            q = getvector(q, self.n)

        r = self.fkine(q).R
        r = np.linalg.inv(r)

        Jv = np.zeros((6, 6))
        Jv[:3, :3] = r
        Jv[3:, 3:] = r

        return Jv

    def accel(self, qd, torque, q=None):
        """
        Calculates a vector (nx1) of joint accelerations that result
        from applying the actuator force/torque (1xN) to the manipulator
        in state q (1xN) and qd (1xN), and n is the number of robot joints.

        If q, qd, torque are matrices (KxN) then qdd is a matrix (KxN) where
        each row is the acceleration corresponding to the equivalent rows of
        q, qd, torque.

        Notes:
        - Useful for simulation of manipulator dynamics, in
          conjunction with a numerical integration function.
        - Uses the method 1 of Walker and Orin to compute the forward dynamics.
        - Featherstone's method is more efficient for robots with large numbers
          of joints.
        - Joint friction is considered.

        :param qd: The joint velocities of the robot
        :type qd: float np.ndarray(1,n)
        :param torque: The joint torques of the robot
        :type torque: float np.ndarray(1,n)
        :param q: The joint angles/configuration of the robot (Optional,
            if not supplied will use the stored q values).
        :type q: float np.ndarray(1,n)

        :return a: The joint accelerations of the robot
        :rtype a: float np.ndarray(1,n)

        References:
        - Efficient dynamic computer simulation of robotic mechanisms,
          M. W. Walker and D. E. Orin,
          ASME Journa of Dynamic Systems, Measurement and Control, vol.
          104, no. 3, pp. 205-211, 1982.
        """

        if q is None:
            q = np.copy(self.q)
        else:
            q = getvector(q, self.n)

        qd = getvector(qd, self.n)
        torque = getvector(torque, self.n)

        # TODO After rne

        # Compute current manipulator inertia torques resulting from unit
        # acceleration of each joint with no gravity.
        # M = rne(robot, ones(n,1)*q, zeros(n,n), eye(n), 'gravity', [0 0 0]);

        # Compute gravity and coriolis torque torques resulting from zero
        # acceleration at given velocity & with gravity acting.
        # tau = rne(robot, q, qd, zeros(1,n));

        # qdd = M \ (torque - tau)';

    def nofriction(self, coulomb=True, viscous=False):
        """
        Copies the robot and returns a robot with the same parameters except,
        the Coulomb and/or viscous friction parameter to zero

        :param coulomb: if True, will set the coulomb friction to 0
        :type coulomb: bool
        :param viscous: if True, will set the viscous friction to 0
        :type viscous: bool

        :return: A copy of the robot with modified friction
        :rtype: SerialLink
        """

        L = []

        for i in range(self.n):
            L.append(self.links[i].nofriction(coulomb, viscous))

        return SerialLink(
            L,
            name='NF' + self.name,
            manufacturer=self.manuf,
            base=self.base,
            tool=self.tool,
            gravity=self.gravity)

    def pay(self, W, q=None, J=None, frame=1):
        """
        pay(W, J) Returns the generalised joint force/torques due to a payload
        wrench W applied to the end-effector. Where the manipulator Jacobian
        is J (6xn), and n is the number of robot joints.

        pay(W, q, frame) as above but the Jacobian is calculated at pose q
        in the frame given by frame which is 0 for base frame, 1 for
        end-effector frame.

        Uses the formula tau = J'W, where W is a wrench vector applied at the
        end effector, W = [Fx Fy Fz Mx My Mz]'.

        Trajectory operation:
          In the case q is nxm or J is 6xnxm then tau is nxm where each row
          is the generalised force/torque at the pose given by corresponding
          row of q.

        :param W: A wrench vector applied at the end effector,
            W = [Fx Fy Fz Mx My Mz]
        :type q: float np.ndarray(1,n)
        :param q: The joint angles/configuration of the robot (Optional,
            if not supplied will use the stored q values).
        :type q: float np.ndarray(1,n)
        :param J: The manipulator Jacobian (Optional, if not supplied will
            use the q value).
        :type J: float np.ndarray(1,n)
        :param frame: The frame in which to torques are expressed in when J
            is not supplied. 0 means base frame of the robot, 1 means end-
            effector frame
        :type frame: int

        :return tau: The joint forces/torques due to w
        :rtype tau: float np.ndarray(1,n)

        Notes:
        - Wrench vector and Jacobian must be from the same reference frame.
        - Tool transforms are taken into consideration when F = 'e'.
        - Must have a constant wrench - no trajectory support for this yet.
        """

        try:
            W = getvector(W, 6)
            trajn = 0
        except ValueError:
            trajn = W.shape[1]
            verifymatrix(W, (6, trajn))

        if trajn:
            # A trajectory
            if J is not None:
                # Jacobian supplied
                verifymatrix(J, (6, self.n, trajn))
            else:
                # Use q instead
                verifymatrix(q, (self.n, trajn))
                J = np.zeros((6, self.n, trajn))
                for i in range(trajn):
                    if frame:
                        J[:, :, i] = self.jacobe(q[:, i])
                    else:
                        J[:, :, i] = self.jacob0(q[:, i])
        else:
            # Single configuration
            if J is not None:
                # Jacobian supplied
                verifymatrix(J, (6, self.n))
            else:
                # Use q instead
                if q is None:
                    q = np.copy(self.q)
                else:
                    q = getvector(q, self.n)

                if frame:
                    J = self.jacobe(q)
                else:
                    J = self.jacob0(q)

        if trajn == 0:
            tau = -J.T @ W
        else:
            tau = np.zeros((self.n, trajn))

            for i in range(trajn):
                tau[:, i] = -J[:, :, i].T @ W[:, i]

        return tau

    def friction(self, qd):
        """
        Calculates the vector of joint friction forces/torques for the robot
        moving with joint velocities qd.

        The friction model includes:
        - Viscous friction which is a linear function of velocity.
        - Coulomb friction which is proportional to sign(qd).

        Notes:
        - The friction value should be added to the motor output torque, it
          has a negative value when qd>0.
        - The returned friction value is referred to the output of the
          gearbox.
        - The friction parameters in the Link object are referred to the
          motor.
        - Motor viscous friction is scaled up by G^2.
        - Motor Coulomb friction is scaled up by G.
        - The appropriate Coulomb friction value to use in the non-symmetric
          case depends on the sign of the joint velocity, not the motor
          velocity.
        - The absolute value of the gear ratio is used. Negative gear ratios
          are tricky: the Puma560 has negative gear ratio for joints 1 and 3.

        :param qd: The joint velocities of the robot
        :type qd: float np.ndarray(1,n)

        :return: The joint friction forces.torques for the robot
        :rtype: float np.ndarray(n,)
        """

        qd = getvector(qd, self.n)
        tau = np.zeros(self.n)

        for i in range(self.n):
            tau[i] = self.links[i].friction(qd[i])

        return tau

    def cinertia(self, q=None):
        """
        The nxn Cartesian (operational space) inertia matrix which relates
        Cartesian force/torque to Cartesian acceleration at the joint
        configuration q.

        :param q: The joint angles/configuration of the robot (Optional,
            if not supplied will use the stored q values).
        :type q: float np.ndarray(1,n)

        :return M: The inertia matrix
        :rtype M: float np.ndarray(n,n)
        """

        if q is None:
            q = np.copy(self.q)
        else:
            q = getvector(q, self.n)

        J = self.jacob0(q)
        Ji = np.linalg.pinv(J)
        M = self.inertia(q)
        Mx = Ji.T @ M @ Ji

        return Mx

    def coriolis(self, qd, q=None):
        """
        Calculates the Coriolis/centripetal matrix (nxn) for the robot in
        configuration q and velocity qd, where N is the number of joints.
        The product c*qd is the vector of joint force/torque due to
        velocity coupling. The diagonal elements are due to centripetal
        effects and the off-diagonal elements are due to Coriolis effects.
        This matrix is also known as the velocity coupling matrix, since
        it describes the disturbance forces on any joint due to velocity
        of all other joints.

        If q and qd are matrices (nxk), each row is interpretted as a
        joint state vector, and the result (nxnxk) is a 3d-matrix where
        each plane corresponds to a row of q and qd.

        Notes:
        - Joint viscous friction is also a joint force proportional to
            velocity but it is eliminated in the computation of this value.
        - Computationally slow, involves n^2/2 invocations of RNE.

        :param qd: The joint velocities of the robot
        :type qd: float np.ndarray(1,n)
        :param q: The joint angles/configuration of the robot (Optional,
            if not supplied will use the stored q values).
        :type q: float np.ndarray(1,n)

        :return C: The Coriolis/centripetal matrix
        :rtype C: float np.ndarray(n,n)
        """

        try:
            qd = getvector(qd, self.n)
            trajn = 0

            if q is None:
                q = np.copy(self.q)
            else:
                q = getvector(q, self.n)
        except ValueError:
            trajn = qd.shape[1]
            verifymatrix(qd, (self.n, trajn))
            verifymatrix(q, (self.n, trajn))

        r1 = self.nofriction(True, True)

        C = np.zeros((self.n, self.n))
        Csq = np.zeros((self.n, self.n))

        # Find the torques that depend on a single finite joint speed,
        # these are due to the squared (centripetal) terms
        # set QD = [1 0 0 ...] then resulting torque is due to qd_1^2
        for i in range(self.n):
            QD = np.zeros((1, self.n))
            QD[i] = 1
            tau = r1.rne(q, QD, np.zeros(self.n), gravity=[0, 0, 0])
            Csq[:, i] = Csq[:, i] + tau.T

        # Find the torques that depend on a pair of finite joint speeds,
        # these are due to the product (Coridolis) terms
        # set QD = [1 1 0 ...] then resulting torque is due to
        # qd_1 qd_2 + qd_1^2 + qd_2^2
        for i in range(self.n):
            for j in range(i+1, self.n):
                # Find a product term  qd_i * qd_j
                QD = np.zeros((1, self.n))
                QD[i] = 1
                QD[j] = 1
                tau = r1.rne(q, QD, np.zeros(self.n), gravity=[0, 0, 0])
                C[:, j] = C[:, j] + (tau.T - Csq[:, j] - Csq[:, i]) \
                    * qd[i]/2
                C[:, i] = C[:, i] + (tau.T - Csq[:, j] - Csq[:, i]) \
                    * qd[j]/2

        C = C + Csq @ np.diag(qd)

    def gravjac(self, q=None, grav=None):
        """
        Calculates the generalised joint force/torques due to gravity tau
        (1xN) and the manipulator Jacobian in the base frame jacob0 (6xn)
        for robot pose q (1xn), where N is the number of robot joints.

        Trajectory operation:
        If q is nxm where n is the number of robot joints then a
        trajectory is assumed where each row of q corresponds to a robot
        configuration. tau (nxm) is the generalised joint torque, each row
        corresponding to an input pose, and jacob0 (6xnxm) where each
        plane is a Jacobian corresponding to an input pose.

        Notes:
        - The gravity vector is defined by the SerialLink property if not
            explicitly given.
        - Does not use inverse dynamics function RNE.
        - Faster than computing gravity and Jacobian separately.

        :param q: The joint angles/configuration of the robot (Optional,
            if not supplied will use the stored q values).
        :type q: float np.ndarray(n)
        :param grav: The gravity vector (Optional, if not supplied will
            use the stored gravity values).
        :type grav: float np.ndarray(3,)

        :return tau: The generalised joint force/torques due to gravity
        :rtype tau: float np.ndarray(1,n)
        """

        if grav is None:
            grav = np.copy(self.gravity)
        else:
            grav = getvector(grav, 3)

        try:
            if q is not None:
                q = getvector(q, self.n, 'col')
            else:
                q = np.copy(self.q)
                q = getvector(q, self.n, 'col')

            poses = 1
        except ValueError:
            poses = q.shape[1]
            verifymatrix(q, (self.n, poses))

        if not self.mdh:
            baseAxis = self.base.a
            baseOrigin = self.base.t

        tauB = np.zeros((self.n, poses))
        J = np.zeros((6, self.n, poses))

        # Forces
        force = np.zeros((3, self.n))

        for joint in range(self.n):
            force[:, joint] = np.squeeze(self.links[joint].m * grav)

        # Centre of masses (local frames)
        r = np.zeros((4, self.n))
        for joint in range(self.n):
            r[:, joint] = np.r_[np.squeeze(self.links[joint].r), 1]

        for pose in range(poses):
            com_arr = np.zeros((3, self.n))

            T = self.allfkine(q[:, pose])

            jointOrigins = np.zeros((3, self.n))
            jointAxes = np.zeros((3, self.n))
            for i in range(self.n):
                jointOrigins[:, i] = T[i].t
                jointAxes[:, i] = T[i].a

            if not self.mdh:
                jointOrigins = np.c_[
                    baseOrigin, jointOrigins[:, :-1]
                ]
                jointAxes = np.c_[
                    baseAxis, jointAxes[:, :-1]
                ]

            # Backwards recursion
            for joint in range(self.n-1, -1, -1):
                # C.o.M. in world frame, homog
                com = T[joint].A @ r[:, joint]

                # Add it to the distal others
                com_arr[:, joint] = com[0:3]

                t = np.zeros(3)

                # for all links distal to it
                for link in range(joint, self.n):
                    if not self.links[joint].sigma:
                        # Revolute joint
                        d = com_arr[:, link] - jointOrigins[:, joint]
                        t = t + self._cross3(d, force[:, link])
                        # Though r x F would give the applied torque
                        # and not the reaction torque, the gravity
                        # vector is nominally in the positive z
                        # direction, not negative, hence the force is
                        # the reaction force
                    else:
                        # Prismatic joint
                        # Force on prismatic joint
                        t = t + force[:, link]

                tauB[joint, pose] = t.T @ jointAxes[:, joint]

        return tauB

    def _cross3(self, a, b):
        c = np.zeros(3)
        c[2] = a[0] * b[1] - a[1] * b[0]
        c[0] = a[1] * b[2] - a[2] * b[1]
        c[1] = a[2] * b[0] - a[0] * b[2]
        return c

    def gravload(self, q=None, grav=None):
        """
        Calculates the joint gravity loading (n) for the robot Rin the joint
        configuration q (n), where n is the number of robot joints.
        Gravitational acceleration is a property of the robot object.

        If q is a matrix (nxm) each column is interpreted as a joint
        configuration vector, and the result is a matrix (nxm) each column
        being the corresponding joint torques.

        :param q: The joint angles/configuration of the robot (Optional,
            if not supplied will use the stored q values).
        :type q: float np.ndarray(n)
        :param grav: The gravity vector (Optional, if not supplied will
            use the stored gravity values).
        :type grav: float np.ndarray(3,)

        :return taug: The generalised joint force/torques due to gravity
        :rtype taug: float np.ndarray(1,n)
        """

        if grav is None:
            grav = np.copy(self.gravity)
        else:
            grav = getvector(grav, 3)

        try:
            if q is not None:
                q = getvector(q, self.n, 'col')
            else:
                q = np.copy(self.q)
                q = getvector(q, self.n, 'col')

            poses = 1
        except ValueError:
            poses = q.shape[1]
            verifymatrix(q, (self.n, poses))

        taug = rne(q, np.zeros(self.n), np.zeros(self.n), grav)

    def ikcon(self, T, q0=None):
        """
        Inverse kinematics by optimization with joint limits

        Calculates the joint coordinates (1xn) corresponding to the robot
        end-effector pose T which is an SE3 object or homogenenous transform
        matrix (4x4), and N is the number of robot joints. OPTIONS is an
        optional list of name/value pairs than can be passed to fmincon.

        Q = robot.ikunc(T, Q0, OPTIONS) as above but specify the
        initial joint coordinates Q0 used for the minimisation.

        [Q,ERR,EXITFLAG] = robot.ikcon(T, ...) as above but also returns the
        status EXITFLAG from fmincon.

        Trajectory operation:
        In all cases if T is a vector of SE3 objects (1xM) or a homogeneous
        transform sequence (4x4xm) then returns the joint coordinates
        corresponding to each of the transforms in the sequence. q is nxm
        where N is the number of robot joints. The initial estimate of q
        for each time step is taken as the solution from the previous time
        step.

        ERR and EXITFLAG are also Mx1 and indicate the results of optimisation
        for the corresponding trajectory step.

        Notes:
        - Joint limits are considered in this solution.
        - Can be used for robots with arbitrary degrees of freedom.
        - In the case of multiple feasible solutions, the solution returned
          depends on the initial choice of q0.
        - Works by minimizing the error between the forward kinematics of the
          joint angle solution and the end-effector frame as an optimisation.
          The objective function (error) is described as:
                  sumsqr( (inv(T)*robot.fkine(q) - eye(4)) * omega )
          Where omega is some gain matrix, currently not modifiable.
        """

        if not isinstance(T, SE3):
            T = SE3(T)

        trajn = len(T)

        try:
            if q0 is not None:
                q0 = getvector(q0, self.n, 'col')
            else:
                q0 = np.zeros((self.n, trajn))
        except ValueError:
            verifymatrix(q0, (self.n, trajn))

        # create output variables
        qstar = np.zeros((self.n, trajn))
        error = []
        exitflag = []

        reach = np.sum(np.abs([self.a, self.d]))
        omega = np.diag([1, 1, 1, 3/reach])

        fun = lambda q: \
            np.sum(
                ((np.linalg.pinv(Ti.A) @ self.fkine(q).A - np.eye(4)) @ omega)
                ** 2
            )

        bnds = Bounds(self.qlim[0, :], self.qlim[1, :])

        for i in range(trajn):
            Ti = T[i]
            res = minimize(fun, q0[:, i], bounds=bnds, options={'gtol': 1e-6})
            qstar[:, i] = res.x
            error.append(res.fun)
            exitflag.append(res.success)

        if trajn > 1:
            return qstar, error, exitflag
        else:
            return qstar[:, 0], error[0], exitflag[0]

    def ikine(
            self, T,
            ilimit=500,
            rlimit=100,
            tol=1e-10,
            Y=0.1,
            Ymin=0,
            mask=None,
            q0=None,
            search=False,
            slimit=100,
            transpose=None):
        """
        Inverse kinematics by optimization without joint limits

        q = ikine(T) are the joint coordinates (1xn) corresponding to the
        robot end-effector pose T which is an SE3 object or homogenenous
        transform matrix (4x4), and n is the number of robot joints.

        This method can be used for robots with any number of degrees of
        freedom.

        :param ilimit: maximum number of iterations
        :type ilimit: int (default 500)
        :param rlimit: maximum number of consecutive step rejections
        :type rlimit: int (default 100)
        :param tol: final error tolerance
        :type tol: float (default 1e-10)
        :param Y: initial value of lambda
        :type Y: float (default 0.1)
        :param Ymin: minimum allowable value of lambda
        :type Ymin: float (default 0)
        :param mask: mask vector that correspond to translation in X, Y and Z
            and rotation about X, Y and Z respectively.
        :type mask: float np.ndarray(6)
        :param q0: initial joint configuration (default all zeros)
        :type q0: float np.ndarray(n) (default all zeros)
        :param search: search over all configurations
        :type search: bool
        :param slimit: maximum number of search attempts
        :type slimit: int (default 100)
        :param transpose: use Jacobian transpose with step size A, rather
            than Levenberg-Marquadt
        :type transpose:

        Trajectory operation:
        In all cases if T is a vector of SE3 objects (1xm) or a homogeneous
        transform sequence (4x4xm) then returns the joint coordinates
        corresponding to each of the transforms in the sequence. q is nxm
        where n is the number of robot joints. The initial estimate of q for
        each time step is taken as the solution from the previous time step.

        Underactuated robots:
        For the case where the manipulator has fewer than 6 DOF the solution
        space has more dimensions than can be spanned by the manipulator joint
        coordinates.

        In this case we specify the 'mask' option where the mask vector (1x6)
        specifies the Cartesian DOF (in the wrist coordinate frame) that will
        be ignored in reaching a solution.  The mask vector has six elements
        that correspond to translation in X, Y and Z, and rotation about X, Y
        and Z respectively. The value should be 0 (for ignore) or 1. The
        number of non-zero elements should equal the number of manipulator
        DOF.

        For example when using a 3 DOF manipulator rotation orientation might
        be unimportant in which case use the option: mask = [1 1 1 0 0 0].

        For robots with 4 or 5 DOF this method is very difficult to use since
        orientation is specified by T in world coordinates and the achievable
        orientations are a function of the tool position.

        References:
        - Robotics, Vision & Control, P. Corke, Springer 2011, Section 8.4.

        Notes:
        - Solution is computed iteratively.
        - Implements a Levenberg-Marquadt variable step size solver.
        - The tolerance is computed on the norm of the error between current
          and desired tool pose.  This norm is computed from distances
          and angles without any kind of weighting.
        - The inverse kinematic solution is generally not unique, and
          depends on the initial guess q0 (defaults to 0).
        - The default value of q0 is zero which is a poor choice for most
          manipulators (eg. puma560, twolink) since it corresponds to a
          kinematic singularity.
        - Such a solution is completely general, though much less efficient
          than specific inverse kinematic solutions derived symbolically,
          like ikine6s or ikine3.
        - This approach allows a solution to be obtained at a singularity, but
          the joint angles within the null space are arbitrarily assigned.
        - Joint offsets, if defined, are added to the inverse kinematics to
          generate q.
        - Joint limits are not considered in this solution.
        - The 'search' option peforms a brute-force search with initial
          conditions chosen from the entire configuration space.
        - If the search option is used any prismatic joint must have joint
          limits defined.
        """

        if not isinstance(T, SE3):
            T = SE3(T)

        trajn = len(T)
        err = ''

        try:
            if q0 is not None:
                if trajn == 1:
                    q0 = getvector(q0, self.n, 'col')
                else:
                    verifymatrix(q0, (self.n, trajn))
            else:
                q0 = np.zeros((self.n, trajn))
        except ValueError:
            verifymatrix(q0, (self.n, trajn))

        if mask is not None:
            mask = getvector(mask, 6)
        else:
            mask = np.ones(6)

        if search:
            # Randomised search for a starting point
            search = False
            # quiet = True

            for k in range(slimit):

                q0n = np.zeros(self.n)
                for j in range(self.n):
                    qlim = self.links[j].qlim
                    if np.sum(np.abs(qlim)) == 0:
                        if not self.links[j].sigma:
                            q0n[j] = np.random.rand() * 2 * np.pi - np.pi
                        else:
                            raise ValueError('For a prismatic joint, '
                                             'search requires joint limits')
                    else:
                        q0n[j] = np.random.rand() * (qlim[1]-qlim[0]) + qlim[0]

                # fprintf('Trying q = %s\n', num2str(q))

                q, _, _ = self.ikine(
                    T,
                    ilimit,
                    rlimit,
                    tol,
                    Y,
                    Ymin,
                    mask,
                    q0n,
                    search,
                    slimit,
                    transpose)

                if not np.sum(np.abs(q)) == 0:
                    return q, True, err

            q = np.array([])
            return q, False, err

        if not self.n >= np.sum(mask):
            raise ValueError('Number of robot DOF must be >= the same number '
                             'of 1s in the mask matrix')
        W = np.diag(mask)

        # Preallocate space for results
        qt = np.zeros((self.n, len(T)))

        # Total iteration count
        tcount = 0

        # Rejected step count
        rejcount = 0

        failed = False
        nm = 0

        revolutes = []
        for i in range(self.n):
            revolutes.append(not self.links[i].sigma)

        for i in range(len(T)):
            iterations = 0
            q = np.copy(q0[:, i])
            Yl = Y

            while True:
                # Update the count and test against iteration limit
                iterations += 1

                if iterations > ilimit:
                    err = 'ikine: iteration limit {0} exceeded (pose {1}), '\
                          'final err {2}'.format(ilimit, i, nm)
                    failed = True
                    break

                e = tr2delta(self.fkine(q).A, T[i].A)

                # Are we there yet
                if np.linalg.norm(W @ e) < tol:
                    # print(iterations)
                    break

                # Compute the Jacobian
                J = self.jacobe(q)

                JtJ = J.T @ W @ J

                if transpose is not None:
                    # Do the simple Jacobian transpose with constant gain
                    dq = transpose * J.T @ e
                else:
                    # Do the damped inverse Gauss-Newton with
                    # Levenberg-Marquadt
                    dq = np.linalg.inv(
                        JtJ + ((Yl + Ymin) * np.eye(self.n))
                        ) @ J.T @ W @ e

                    # Compute possible new value of
                    qnew = q + dq

                    # And figure out the new error
                    enew = tr2delta(self.fkine(qnew).A, T[i].A)

                    # Was it a good update?
                    if np.linalg.norm(W @ enew) < np.linalg.norm(W @ e):
                        # Step is accepted
                        q = qnew
                        e = enew
                        Yl = Yl/2
                        rejcount = 0
                    else:
                        # Step is rejected, increase the damping and retry
                        Yl = Yl*2
                        rejcount += 1
                        if rejcount > rlimit:
                            err = 'ikine: rejected-step limit {0} exceeded ' \
                                  '(pose {1}), final err {2}'.format(
                                      rlimit, i, np.linalg.norm(W @ enew))
                            failed = True
                            break

                # Wrap angles for revolute joints
                k = (q > np.pi) & revolutes
                q[k] -= 2 * np.pi

                k = (q < -np.pi) & revolutes
                q[k] += + 2 * np.pi

                nm = np.linalg.norm(W @ e)

            qt[:, i] = q
            tcount += iterations

            if failed:
                err = 'failed to converge: try a different ' \
                      'initial value of joint coordinates'

        if trajn == 1:
            qt = qt[:, 0]

        return qt, not failed, err
