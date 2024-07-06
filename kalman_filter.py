import numpy as np
import simdkalman

class ExampleEKF:
    """
    Vectorized EKF model for estimating the positions of the trackables
    based on noisy x, y coordinates.
    """
    def __init__(self, n_trackables):
        self.n_trackables = n_trackables  # Number of trackable objects
        self.STATE_DIM = 2  # State dimension (x, y coordinates)

        # Initialize mean state vector and covariance matrix for each trackable
        self.m = np.zeros((self.n_trackables, self.STATE_DIM, 1))
        self.P = np.zeros((self.n_trackables, self.STATE_DIM, self.STATE_DIM))

        # Initial position guess with high uncertainty
        INITIAL_POS_NOISE = 10.0
        for i in range(self.n_trackables):
            self.P[i, ...] = np.eye(self.STATE_DIM) * INITIAL_POS_NOISE

        OBS_DIM = 2  # Observation dimension (noisy x, y coordinates)
        OBS_NOISE = 0.1  # Observation noise standard deviation

        RANDOM_WALK_NOISE = 0.05  # Process noise standard deviation

        # Initialize observation noise covariance matrix, state transition matrix, and process noise covariance matrix
        self.R = np.zeros((self.n_trackables, OBS_DIM, OBS_DIM))
        self.A = np.zeros((self.n_trackables, self.STATE_DIM, self.STATE_DIM))
        self.Q = np.zeros((self.n_trackables, self.STATE_DIM, self.STATE_DIM))

        for i in range(self.n_trackables):
            self.A[i, ...] = np.eye(self.STATE_DIM)  # Identity matrix for state transition
            self.Q[i, ...] = np.eye(self.STATE_DIM) * RANDOM_WALK_NOISE**2  # Process noise covariance
            self.R[i, ...] = np.eye(OBS_DIM) * OBS_NOISE**2  # Observation noise covariance

    def predict(self):
        """
        Predict the next state and covariance using the state transition matrix and process noise covariance.
        """
        self.m, self.P = simdkalman.primitives.predict(self.m, self.P, self.A, self.Q)

    def update(self, observations):
        """
        Update the state and covariance with the new observations.
        """
        # Observation matrix (identity in this case)
        H = np.tile(np.eye(self.STATE_DIM), (self.n_trackables, 1, 1))
        # Linearized measurement residual
        y_lin = observations - simdkalman.primitives.ddot(H, self.m)
        # Update step of EKF
        self.m, self.P = simdkalman.primitives.update(self.m, self.P, H, self.R, y_lin)