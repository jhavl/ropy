#!/usr/bin/env python

import numpy as np

class ets(object):

    def __init__(self, robot):

        _ets = {}
        _ets_compact = {}
        _ets_string = ''
        self._q_idx = []

        conv = 180.0/np.pi
        
        i = 0
        self._joints = robot.n

        for j in range(self._joints):

            _ets[i] = {}
            L = robot.links[j]

            if robot.mdh:
                # Method for modified DH parameters
                
                # Append Tx(a)
                if L.a != 0:
                    _ets[i]['T'] = self._Tx(L.a)
                    _ets[i]['type'] = 'real'
                    _ets[i]['s'] = 'Tx'
                    _ets[i]['val'] = L.a
                    _ets[i]['val_deg'] = L.a
                    i += 1
                    _ets[i] = {}
                
                # Append Rx(alpha)
                if L.alpha != 0:
                    _ets[i]['T'] = self._Rx(L.alpha)
                    _ets[i]['type'] = 'real'
                    _ets[i]['s'] = 'Rx'
                    _ets[i]['val'] = L.alpha
                    _ets[i]['val_deg'] = L.alpha * conv
                    i += 1
                    _ets[i] = {}
                
                if L.is_revolute:

                    # Append Tz(d)
                    if L.d != 0:
                        _ets[i]['T'] = self._Tz(L.d)
                        _ets[i]['type'] = 'real'
                        _ets[i]['s'] = 'Tz'
                        _ets[i]['val'] = L.d
                        _ets[i]['val_deg'] = L.d
                        i += 1
                        _ets[i] = {}
                    
                    # Append Rz(q)
                    _ets[i]['T']   = lambda q : self._R('Rz', q)
                    _ets[i]['dT']  = lambda q : self._dR('Rz', q)
                    _ets[i]['ddT'] = lambda q : self._ddR('Rz', q)
                    _ets[i]['type'] = 'sym'
                    _ets[i]['s'] = 'Rz'
                    self._q_idx.append(i)
                    i += 1
                    _ets[i] = {}
                else:
                    
                    # Append Rz(theta)
                    if L.theta != 0:
                        _ets[i]['T'] = self._Rz(L.alpha)
                        _ets[i]['type'] = 'real'
                        _ets[i]['s'] = 'Rz'
                        _ets[i]['val'] = L.alpha
                        _ets[i]['val_deg'] = L.alpha * conv
                        i += 1
                        _ets[i] = {}
                    
                    # Append Tz(q)
                    _ets[i]['T']   = lambda q : self._T('Tz', q)
                    _ets[i]['dT']  = lambda q : self._dT('Tz', q)
                    _ets[i]['ddT'] = lambda q : self._ddT('Tz', q)
                    _ets[i]['type'] = 'sym'
                    _ets[i]['s'] = 'Tz'
                    self._q_idx.append(i)
                    i += 1
                    _ets[i] = {}
            else:
                # Method for modified DH parameters
                
                if L.is_revolute:

                    # Append Rz(q)
                    _ets[i]['T']   = lambda q : self._R('Rz', q)
                    _ets[i]['dT']  = lambda q : self._dR('Rz', q)
                    _ets[i]['ddT'] = lambda q : self._ddR('Rz', q)
                    _ets[i]['type'] = 'sym'
                    _ets[i]['s'] = 'Rz'
                    self._q_idx.append(i)
                    i += 1
                    _ets[i] = {}

                    # Append Tz(d)
                    if L.d != 0:
                        _ets[i]['T'] = self._Tz(L.d)
                        _ets[i]['type'] = 'real'
                        _ets[i]['s'] = 'Tz'
                        _ets[i]['val'] = L.d
                        _ets[i]['val_deg'] = L.d
                        i += 1
                        _ets[i] = {}
                    
                else:
                    # Append Rz(theta)
                    if L.theta != 0:
                        _ets[i]['T'] = self._Rz(L.alpha)
                        _ets[i]['type'] = 'real'
                        _ets[i]['s'] = 'Rz'
                        _ets[i]['val'] = L.alpha
                        _ets[i]['val_deg'] = L.alpha * conv
                        i += 1
                        _ets[i] = {}
                    
                    # Append Tz(q)
                    _ets[i]['T']   = lambda q : self._T('Tz', q)
                    _ets[i]['dT']  = lambda q : self._dT('Tz', q)
                    _ets[i]['ddT'] = lambda q : self._ddT('Tz', q)
                    _ets[i]['type'] = 'sym'
                    _ets[i]['s'] = 'Tz'
                    self._q_idx.append(i)
                    i += 1
                    _ets[i] = {}

                # Append Tx(a)
                if L.a != 0:
                    _ets[i]['T'] = self._Tx(L.a)
                    _ets[i]['type'] = 'real'
                    _ets[i]['s'] = 'Tx'
                    _ets[i]['val'] = L.a
                    _ets[i]['val_deg'] = L.a
                    i += 1
                    _ets[i] = {}

                # Append Rx(alpha)
                if L.alpha != 0:
                    _ets[i]['T'] = self._Rx(L.alpha)
                    _ets[i]['type'] = 'real'
                    _ets[i]['s'] = 'Rx'
                    _ets[i]['val'] = L.alpha
                    _ets[i]['val_deg'] = L.alpha * conv
                    i += 1
                    _ets[i] = {}


        # Add the tool transform
        _ets[i]['T'] = robot.tool
        _ets[i]['type'] = 'real'
        _ets[i]['s'] = 'tool'
        _ets[i]['val'] = 1
        _ets[i]['val_deg'] = 1
        i += 1
        _ets[i] = {}

        # Number of transforms in the ETS
        self.n = i  

        # The ETS of the robot
        self._ets = _ets

    
    def to_string(self, deg = False):

        s = ''
        j = 1

        for i in range(self.n):

            tr = self._ets[i]['s']

            if self._ets[i]['type'] == 'sym':
                val = 'q{}'.format(j)
                j += 1

            else:
                if deg:
                    val = self._ets[i]['val_deg']
                else:
                    val = self._ets[i]['val']

            s = '{}{}({})'.format(s, tr, val)

        return s

    
    def fkine(self, q):

        if not isinstance(q, np.ndarray):
            raise TypeError('q array must be a numpy ndarray.')
        if q.shape != (self._joints,):
            raise ValueError('q must be a 1 dim (n,) array')

        j = 0

        trans = np.eye(4)

        for i in range(self.n):

            if self._ets[i]['type'] == 'sym':
                T = self._ets[i]['T'](q[j])
                j += 1

            else:
                T = self._ets[i]['T']

            trans = trans @ T

        return trans

    
    def jacob0(self, q):

        if not isinstance(q, np.ndarray):
            raise TypeError('q array must be a numpy ndarray.')
        if q.shape != (self._joints,):
            raise ValueError('q must be a 1 dim (n,) array')

        J = np.zeros((6,self._joints))

        T = self.fkine(q)
        R = T[0:3,0:3]

        dT = self._d_ets(q)

        for i in range(self._joints):

            # Linear velocity component of the Jacobian
            J[0:3,i] = dT[i][0:3,3]

            # Angular velocity component of the Jacobian
            J[3:7,i] = np.squeeze(self._vex( dT[i][0:3,0:3] @ np.transpose(R) ))

        return J


    def hessian0(self, q):

        if not isinstance(q, np.ndarray):
            raise TypeError('q array must be a numpy ndarray.')
        if q.shape != (self._joints,):
            raise ValueError('q must be a 1 dim (n,) array')

        H = np.zeros((6,self._joints,self._joints))

        T = self.fkine(q)
        R = T[0:3,0:3]

        dT = self._d_ets(q)
        ddT = self._dd_ets(q)

        for j in range(self._joints):
            for i in range(self._joints):

                # Linear velocity component of the Hessian
                H[0:3,i,j] = ddT[i][j][0:3,3]

                sw = ddT[j][i][0:3,0:3] @ np.transpose(R) + \
                    dT[i][0:3,0:3] @ np.transpose(dT[j][0:3,0:3])

                H[3:7,i,j] = np.squeeze(self._vex(sw))

        return H


    def m(self, q):
        J = self.jacob0(q)

        return np.sqrt(np.linalg.det(J @ np.transpose(J)))

    def dot_m(self, q):
        m = self.m(q)
        J = self.jacob0(q)
        H = self.hessian0(q)
        b = np.linalg.inv(J @ np.transpose(J))

        a = np.zeros((self._joints,1))

        for i in range(self._joints):
            c = J @ np.transpose(H[:,:,i])
            a[i,0] = m * np.transpose(c.flatten('F')) @ b.flatten('F')

        return a



    def _vex(self, T):
        if not isinstance(T, np.ndarray):
            raise TypeError('T matrix must be a numpy ndarray.')
        if T.shape != (3,3) and T.shape != (4,4):
            raise ValueError('T must be a 3x3 or 4x4 matrix')

        w = np.zeros((3,1))

        w[0,0] = 0.5 * (T[2,1] - T[1,2])
        w[1,0] = 0.5 * (T[0,2] - T[2,0])
        w[2,0] = 0.5 * (T[1,0] - T[0,1])

        return w


    # Calculates the partial derivative of an ETS wrt q
    def _d_ets(self, q):
        if not isinstance(q, np.ndarray):
            raise TypeError('q array must be a numpy ndarray.')
        if q.shape != (self._joints,):
            raise ValueError('q must be a 1 dim (n,) array')

        # The partial derivative of the pose T wrt each joint
        dT = []

        # Compute partial derivatives wrt each joint
        for k in range(self._joints):

            dT.append(np.eye(4))
            l = 0

            for i in range(self.n):

                if self._ets[i]['type'] == 'sym':
                    if i == self._q_idx[k]:
                        #  Multiply by the partial differentiation
                        dT[k] = dT[k] @ self._ets[i]['dT'](q[k])
                        l += 1
                    else:
                        dT[k] = dT[k] @ self._ets[i]['T'](q[l])
                        l += 1
                    
                else:
                    # The current T is static
                    dT[k] = dT[k] @ self._ets[i]['T']
        
        return dT


    # Calculates the second partial derivative of an ETS wrt q
    def _dd_ets(self, q):
        if not isinstance(q, np.ndarray):
            raise TypeError('q array must be a numpy ndarray.')
        if q.shape != (self._joints,):
            raise ValueError('q must be a 1 dim (n,) array')

        # The second partial derivative of the pose wrt each joint
        ddT = []

        # Precompute partial derivatives wrt each joint
        for k in range(self._joints):
        # for k in range(1):

            ddT.append([])

            for j in range(self._joints):

                # Initialise each matrix
                ddT[k].append(np.eye(4))
                l = 0

                # Double partial derivative
                for i in range(self.n):

                    if self._ets[i]['type'] == 'sym':

                        if i == self._q_idx[k] and i == self._q_idx[j]:
                            # Multiply by the double partial differentiation
                            ddT[k][j] = ddT[k][j] @ self._ets[i]['ddT'](q[k])
                            l += 1
                        elif i == self._q_idx[k]: 
                            #  Multiply by the partial differentiation
                            ddT[k][j] = ddT[k][j] @ self._ets[i]['dT'](q[k])
                            l += 1
                        elif i == self._q_idx[j]:
                            #  Multiply by the partial differentiation
                            ddT[k][j] = ddT[k][j] @ self._ets[i]['dT'](q[j])
                            l += 1
                        else:
                            ddT[k][j] = ddT[k][j] @ self._ets[i]['T'](q[l])
                            l += 1
                        
                    else:
                        # The current T is static
                        ddT[k][j] = ddT[k][j] @ self._ets[i]['T']

        return ddT

    # Helper functions
    def _R(self, axis, q):
        if axis == 'Rx':
            return self._Rx(q)
        elif axis == 'Ry':
            return self._Ry(q)
        elif axis == 'Rz':
            return self._Rz(q)
        else:
            raise ValueError('Not a valid axis, axis must be: "Rx", "Ry", or "Rz"')

    def _dR(self, axis, q):
        if axis == 'Rx':
            return self._dRx(q)
        elif axis == 'Ry':
            return self._dRy(q)
        elif axis == 'Rz':
            return self._dRz(q)
        else:
            raise ValueError('Not a valid axis, axis must be: "Rx", "Ry", or "Rz"')

    def _ddR(self, axis, q):
        if axis == 'Rx':
            return self._ddRx(q)
        elif axis == 'Ry':
            return self._ddRy(q)
        elif axis == 'Rz':
            return self._ddRz(q)
        else:
            raise ValueError('Not a valid axis, axis must be: "Rx", "Ry", or "Rz"')

    def _T(self, axis, q):
        if axis == 'Tx':
            return self._Tx(q)
        elif axis == 'Ty':
            return self._Ry(q)
        elif axis == 'Tz':
            return self._Rz(q)
        else:
            raise ValueError('Not a valid axis, axis must be: "Tx", "Ty", or "Tz"')

    def _dT(self, axis, q):
        if axis == 'Tx':
            return self._dTx(q)
        elif axis == 'Ty':
            return self._dTy(q)
        elif axis == 'Tz':
            return self._dTz(q)
        else:
            raise ValueError('Not a valid axis, axis must be: "Tx", "Ty", or "Tz"')

    def _ddT(self, axis, q):
        if axis == 'Tx':
            return self._ddTx(q)
        elif axis == 'Ty':
            return self._ddTy(q)
        elif axis == 'Tz':
            return self._ddTz(q)
        else:
            raise ValueError('Not a valid axis, axis must be: "Tx", "Ty", or "Tz"')

    # Each of the 6 elementary transforms as a function of q
    def _Rx(self, q):
        return np.array([
            [1,  0,          0,         0],
            [0,  np.cos(q), -np.sin(q), 0],
            [0,  np.sin(q),  np.cos(q), 0],
            [0,  0,          0,         1]
        ])

    def _Ry(self, q):
        return np.array([
            [ np.cos(q),  0, np.sin(q), 0],
            [ 0,          1, 0,         0],
            [-np.sin(q),  0, np.cos(q), 0],
            [ 0,          0, 0,         1]
        ])

    def _Rz(self, q):
        return np.array([
            [np.cos(q), -np.sin(q), 0, 0],
            [np.sin(q),  np.cos(q), 0, 0],
            [0,          0,         1, 0],
            [0,          0,         0, 1]
        ])

    def _Tx(self, q):
        return np.array([
            [1, 0, 0, q],
            [0, 1, 0, 0],
            [0, 0, 1, 0],
            [0, 0, 0, 1]
        ])

    def _Ty(self, q):
        return np.array([
            [1, 0, 0, 0],
            [0, 1, 0, q],
            [0, 0, 1, 0],
            [0, 0, 0, 1]
        ])

    def _Tz(self, q):
        return np.array([
            [1, 0, 0, 0],
            [0, 1, 0, 0],
            [0, 0, 1, q],
            [0, 0, 0, 1]
        ])

    # Precomputation of the first partial derivative wrt q of the 6  
    # elementary transforms
    # Partial derivative of R is skew(a) * R where skew(a) is the 
    # corresponding skew symmetric matrix for R
    def _dRx(self, q):
        return np.array([
            [0,  0,          0,         0],
            [0, -np.sin(q), -np.cos(q), 0],
            [0,  np.cos(q), -np.sin(q), 0],
            [0,  0,          0,         0]
        ])

    def _dRy(self, q):
        return np.array([
            [ -np.sin(q),  0,  np.cos(q), 0],
            [  0,          0,  0,         0],
            [ -np.cos(q),  0, -np.sin(q), 0],
            [  0,          0,  0,         0]
        ])

    def _dRz(self, q):
        return np.array([
            [ -np.sin(q), -np.cos(q), 0, 0],
            [  np.cos(q), -np.sin(q), 0, 0],
            [  0,          0,         0, 0],
            [  0,          0,         0, 0]
        ])

    def _dTx(self, q):
        return np.array([
            [0, 0, 0, 1],
            [0, 0, 0, 0],
            [0, 0, 0, 0],
            [0, 0, 0, 0]
        ])

    def _dTy(self, q):
        return np.array([
            [0, 0, 0, 0],
            [0, 0, 0, 1],
            [0, 0, 0, 0],
            [0, 0, 0, 0]
        ])

    def _dTz(self, q):
        return np.array([
            [0, 0, 0, 0],
            [0, 0, 0, 0],
            [0, 0, 0, 1],
            [0, 0, 0, 0]
        ])

    # Precomputation of the second partial derivative wrt q of the 6  
    # elementary transforms
    # Second partial derivative of R is skew(a) * skew(a) * R where 
    # skew(a) is the corresponding skew symmetric matrix for R
    def _ddRx(self, q):
        return np.array([
            [0,  0,          0,         0],
            [0, -np.cos(q),  np.sin(q), 0],
            [0, -np.sin(q), -np.cos(q), 0],
            [0,  0,          0,         0]
        ])

    def _ddRy(self, q):
        return np.array([
            [ -np.cos(q),  0, -np.sin(q), 0],
            [  0,          0,  0,         0],
            [  np.sin(q),  0, -np.cos(q), 0],
            [  0,          0,  0,         0]
        ])

    def _ddRz(self, q):
        return np.array([
            [ -np.cos(q),  np.sin(q), 0, 0],
            [ -np.sin(q), -np.cos(q), 0, 0],
            [  0,          0,         0, 0],
            [  0,          0,         0, 0]
        ])

    def _ddTx(self, q):
        return np.zeros(4)

    def _ddTy(self, q):
        return np.zeros(4)

    def _ddTz(self, q):
        return np.zeros(4)


# def etss(robot, deg = False):
#     """
#     The elementary transform sequence (ETS) represents a robot's forward 
#     kinematics in terms of 6 elementary transfroms. This method calculates
#     a robot's ETS based on its DH or MDH parameters.
    
#     Parameters
#     ----------
#     robot : An object of SerialLink or subclass type
#         The robot to calculate the fkine of
#     deg : Return the ETS using degrees instead of radians

#     Returns
#     -------
#     s : String
#         The ETS of the robot

#     Examples
#     --------
#     >>> s = ets(panda)
    
#     See Also
#     --------
#     robotics.tools.relative_yaw_to_trans : Returns the relative yaw to b, 
#         from a
#     """
    
#     if deg:
#         conv = 180.0/np.pi
#     else:
#         conv = 1
    
#     s = ''
            
#     for j in range(robot.n):

#         L = robot.links[j]
        
#         if robot.mdh:
#             # Method for modified DH parameters
            
#             # Append Tx(a)
#             if L.a != 0:
#                     s = '{}Tx({})'.format(s, L.a)
            
#             # Append Rx(alpha)
#             if L.alpha != 0:
#                 s = '{}Rx({})'.format(s, L.alpha*conv)
            
#             if L.is_revolute:

#                 # Append Tz(d)
#                 if L.d != 0:
#                     s = '{}Tz({})'.format(s, L.d)
                
#                 # Append Rz(q)
#                 s = '{}Rz(q{})'.format(s, j+1)
#             else:
                
#                 # Append Rz(theta)
#                 if L.theta != 0:
#                     s = '{}Rz({})'.format(s, L.alpha*conv)
                
#                 # Append Tz(q)
#                 s = '{}Tz(q{})'.format(s, j+1)

#         else:
#             # Method for standard DH parameters
            
#             if L.is_revolute:

#                 # Append Tz(d)
#                 if L.d != 0:
#                     s = '{}Tz({})'.format(s, L.d)
                
#                 # Append Rz(q)
#                 s = '{}Rz(q{})'.format(s, j+1)
#             else:
                
#                 # Append Rz(theta)
#                 if L.theta != 0:
#                     s = '{}Rz({})'.format(s, L.alpha*conv)
                
#                 # Append Tz(q)
#                 s = '{}Tz(q{})'.format(s, j+1)

#             # Append Tx(a)
#             if L.a != 0:
#                     s = '{}Tx({})'.format(s, L.a)
            
#             # Append Rx(alpha)
#             if L.alpha != 0:
#                 s = '{}Rx({})'.format(s, L.alpha*conv)

#     return s