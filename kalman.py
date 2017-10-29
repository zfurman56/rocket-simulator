import numpy as np
from numpy import dot, zeros, eye, isscalar, shape
from scipy.stats import multivariate_normal
import scipy.linalg as linalg

# Returns the matrix multiplication of A*B*C
dot3 = lambda A, B, C: dot(A, dot(B, C))

def Q_discrete_white_noise(dim, dt=1., var=1.):
    """ Returns the Q matrix for the Discrete Constant White Noise
    Model. dim may be either 2 or 3, dt is the time step, and sigma is the
    variance in the noise.

    Q is computed as the G * G^T * variance, where G is the process noise per
    time step. In other words, G = [[.5dt^2][dt]]^T for the constant velocity
    model.

    Parameters
    -----------

    dim : int (2 or 3)
        dimension for Q, where the final dimension is (dim x dim)

    dt : float, default=1.0
        time step in whatever units your filter is using for time. i.e. the
        amount of time between innovations

    var : float, default=1.0
        variance in the noise
    """

    assert dim == 2 or dim == 3
    if dim == 2:
        Q = np.array([[.25*dt**4, .5*dt**3],
                      [.5*dt**3, dt**2]], dtype=float)
    else:
        Q = np.array([[.25*dt**4, .5*dt**3, .5*dt**2],
                      [.5*dt**3, dt**2, dt],
                      [.5*dt**2, dt, 1]], dtype=float)

    return Q * var

def logpdf(x, mean, cov, allow_singular=True):
    """Computes the log of the probability density function of the normal
    N(mean, cov) for the data x. The normal may be univariate or multivariate.

    Wrapper for older versions of scipy.multivariate_normal.logpdf which
    don't support support the allow_singular keyword prior to verion 0.15.0.

    If it is not supported, and cov is singular or not PSD you may get
    an exception.

    `x` and `mean` may be column vectors, row vectors, or lists.
    """

    flat_mean = np.asarray(mean).flatten()
    flat_x = np.asarray(x).flatten()

    return multivariate_normal.logpdf(flat_x, flat_mean, cov, allow_singular)


class KalmanFilter(object):
    """ Implements a Kalman filter. You are responsible for setting the
    various state variables to reasonable values; the defaults  will
    not give you a functional filter.

    You will have to set the following attributes after constructing this
    object for the filter to perform properly. Please note that there are
    various checks in place to ensure that you have made everything the
    'correct' size. However, it is possible to provide incorrectly sized
    arrays such that the linear algebra can not perform an operation.
    It can also fail silently - you can end up with matrices of a size that
    allows the linear algebra to work, but are the wrong shape for the problem
    you are trying to solve.

    Attributes
    ----------
    x : numpy.array(dim_x, 1)
        State estimate vector

    P : numpy.array(dim_x, dim_x)
        Covariance matrix

    R : numpy.array(dim_z, dim_z)
        Measurement noise matrix

    Q : numpy.array(dim_x, dim_x)
        Process noise matrix

    F : numpy.array()
        State Transition matrix

    H : numpy.array(dim_x, dim_x)
        Measurement function


    You may read the following attributes.

    Attributes
    ----------
    y : numpy.array
        Residual of the update step.

    K : numpy.array(dim_x, dim_z)
        Kalman gain of the update step

    S :  numpy.array
        Systen uncertaintly projected to measurement space

    likelihood : scalar
        Likelihood of last measurement update.

    log_likelihood : scalar
        Log likelihood of last measurement update.
    """

    def __init__(self, dim_x, dim_z, dim_u=0):
        """ Create a Kalman filter. You are responsible for setting the
        various state variables to reasonable values; the defaults below will
        not give you a functional filter.

        Parameters
        ----------
        dim_x : int
            Number of state variables for the Kalman filter. For example, if
            you are tracking the position and velocity of an object in two
            dimensions, dim_x would be 4.
            This is used to set the default size of P, Q, and u
        dim_z : int
            Number of of measurement inputs. For example, if the sensor
            provides you with position in (x,y), dim_z would be 2.
        dim_u : int (optional)
            size of the control input, if it is being used.
            Default value of 0 indicates it is not used.
        """

        assert dim_x > 0
        assert dim_z > 0
        assert dim_u >= 0

        self.dim_x = dim_x
        self.dim_z = dim_z
        self.dim_u = dim_u

        self.x = zeros((dim_x, 1)) # state
        self.P = eye(dim_x)       # uncertainty covariance
        self.Q = eye(dim_x)       # process uncertainty
        self.B = 0                # control transition matrix
        self.F = 0                # state transition matrix
        self.F2 = 0               # state transition matrix
        self.H = 0                # Measurement function
        self.H2 = 0               # Measurement function
        self.R = eye(dim_z)       # state uncertainty
        self.R2 = eye(dim_z)      # state uncertainty
        self._alpha_sq = 1.       # fading memory control
        self.M = 0                # process-measurement cross correlation

        # gain and residual are computed during the innovation step. We
        # save them so that in case you want to inspect them for various
        # purposes
        self.K = 0 # kalman gain
        self.y = zeros((dim_z, 1))
        self.S = np.zeros((dim_z, dim_z)) # system uncertainty

        # identity matrix. Do not alter this.
        self.I = np.eye(dim_x)


    def update(self, z, R=None, H=None):
        """
        Add a new measurement (z) to the Kalman filter. If z is None, nothing
        is changed.

        Parameters
        ----------
        z : np.array
            measurement for this update. z can be a scalar if dim_z is 1,
            otherwise it must be a column vector.

        R : np.array, scalar, or None
            Optionally provide R to override the measurement noise for this
            one call, otherwise  self.R will be used.

        H : np.array, or None
            Optionally provide H to override the measurement function for this
            one call, otherwise self.H will be used.
        """

        if z is None:
            return

        if R is None:
            R = self.R
        elif isscalar(R):
            R = eye(self.dim_z) * R

        # rename for readability and a tiny extra bit of speed
        if H is None:
            H = self.H
        P = self.P
        x = self.x

        # handle special case: if z is in form [[z]] but x is not a column
        # vector dimensions will not match
        if x.ndim == 1 and shape(z) == (1, 1):
            z = z[0]

        if shape(z) == (): # is it scalar, e.g. z=3 or z=np.array(3)
            z = np.asarray([z])

        # y = z - Hx
        # error (residual) between measurement and prediction
        Hx = dot(H, x)

        assert shape(Hx) == shape(z) or (shape(Hx) == (1, 1) and shape(z) == (1,)), \
               'shape of z should be {}, but it is {}'.format(
                   shape(Hx), shape(z))
        self.y = z - Hx

        # S = HPH' + R
        # project system uncertainty into measurement space
        self.S = dot3(H, P, H.T) + R

        # K = PH'inv(S)
        # map system uncertainty into kalman gain
        self.K = dot3(P, H.T, linalg.inv(self.S))

        # x = x + Ky
        # predict new x with residual scaled by the kalman gain
        self.x = x + dot(self.K, self.y)

        # P = (I-KH)P(I-KH)' + KRK'
        I_KH = self.I - dot(self.K, H)
        self.P = dot3(I_KH, P, I_KH.T) + dot3(self.K, R, self.K.T)

        self.log_likelihood = logpdf(z, dot(H, x), self.S)

    def update2(self, z):
        self.update(z, R=self.R2, H=self.H2)


    def test_matrix_dimensions(self, z=None, H=None, R=None, F=None, Q=None):
        """ Performs a series of asserts to check that the size of everything
        is what it should be. This can help you debug problems in your design.

        If you pass in H, R, F, Q those will be used instead of this object's
        value for those matrices.

        Testing `z` (the measurement) is problamatic. x is a vector, and can be
        implemented as either a 1D array or as a nx1 column vector. Thus Hx
        can be of different shapes. Then, if Hx is a single value, it can
        be either a 1D array or 2D vector. If either is true, z can reasonably
        be a scalar (either '3' or np.array('3') are scalars under this
        definition), a 1D, 1 element array, or a 2D, 1 element array. You are
        allowed to pass in any combination that works.
        """

        if H is None: H = self.H
        if R is None: R = self.R
        if F is None: F = self.F
        if Q is None: Q = self.Q
        x = self.x
        P = self.P

        assert x.ndim == 1 or x.ndim == 2, \
            "x must have one or two dimensions, but has {}".format(
                x.ndim)

        if x.ndim == 1:
            assert x.shape[0] == self.dim_x, \
                   "Shape of x must be ({},{}), but is {}".format(
                       self.dim_x, 1, x.shape)
        else:
            assert x.shape == (self.dim_x, 1), \
                "Shape of x must be ({},{}), but is {}".format(
                    self.dim_x, 1, x.shape)

        assert P.shape == (self.dim_x, self.dim_x), \
            "Shape of P must be ({},{}), but is {}".format(
                self.dim_x, self.dim_x, P.shape)

        assert Q.shape == (self.dim_x, self.dim_x), \
            "Shape of P must be ({},{}), but is {}".format(
                self.dim_x, self.dim_x, P.shape)

        assert F.shape == (self.dim_x, self.dim_x), \
               "Shape of F must be ({},{}), but is {}".format(
                   self.dim_x, self.dim_x, F.shape)


        assert np.ndim(H) == 2, \
               "Shape of H must be (dim_z, {}), but is {}".format(
                   P.shape[0], shape(H))

        assert H.shape[1] == P.shape[0], \
               "Shape of H must be (dim_z, {}), but is {}".format(
                   P.shape[0], H.shape)

        # shape of R must be the same as HPH'
        hph_shape = (H.shape[0], H.shape[0])
        r_shape = shape(R)

        if H.shape[0] == 1:
            # r can be scalar, 1D, or 2D in this case
            assert r_shape == () or r_shape == (1,) or r_shape == (1, 1), \
            "R must be scalar or one element array, but is shaped {}".format(
                r_shape)
        else:
            assert r_shape == hph_shape, \
            "shape of R should be {} but it is {}".format(hph_shape, r_shape)


        if z is not None:
            z_shape = shape(z)
        else:
            z_shape = (self.dim_z, 1)

        # H@x must have shape of z
        Hx = dot(H, x)

        if z_shape == (): # scalar or np.array(scalar)
            assert Hx.ndim == 1 or shape(Hx) == (1, 1), \
            "shape of z should be {}, not {} for the given H".format(
                shape(Hx), z_shape)

        elif shape(Hx) == (1,):
            assert z_shape[0] == 1, 'Shape of z must be {} for the given H'.format(shape(Hx))

        else:
            assert (z_shape == shape(Hx) or
                    (len(z_shape) == 1 and shape(Hx) == (z_shape[0], 1))), \
                    "shape of z should be {}, not {} for the given H".format(
                        shape(Hx), z_shape)

        if np.ndim(Hx) > 1 and shape(Hx) != (1, 1):
            assert shape(Hx) == z_shape, \
               'shape of z should be {} for the given H, but it is {}'.format(
                   shape(Hx), z_shape)


    def predict(self, u=0, B=None, F=None, Q=None):
        """ Predict next position using the Kalman filter state propagation
        equations.

        Parameters
        ----------

        u : np.array
            Optional control vector. If non-zero, it is multiplied by B
            to create the control input into the system.

        B : np.array(dim_x, dim_z), or None
            Optional control transition matrix; a value of None in
            any position will cause the filter to use `self.B`.

        F : np.array(dim_x, dim_x), or None
            Optional state transition matrix; a value of None in
            any position will cause the filter to use `self.F`.

        Q : np.array(dim_x, dim_x), scalar, or None
            Optional process noise matrix; a value of None in
            any position will cause the filter to use `self.Q`.
        """

        if B is None:
            B = self.B
        if F is None:
            F = self.F
        if Q is None:
            Q = self.Q
        elif isscalar(Q):
            Q = eye(self.dim_x) * Q

        # x = Fx + Bu
        self.x = dot(F, self.x) + dot(B, u)

        # P = FPF' + Q
        self.P = self._alpha_sq * dot3(F, self.P, F.T) + Q

    def project_state(self, n):
        x = self.x
        for i in range(n):
            x = dot(self.F, x)
        return x
