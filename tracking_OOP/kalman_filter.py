import numpy as np
import simdkalman

class KalmanFilterManager:
    """
    A class that manages the Kalman filtering process for tracking positions
    based on noisy x, y coordinates.
    """
    def __init__(self, n_trackables=1, initial_pos_noise=10.0, obs_noise=0.1, process_noise=0.05):
        self.n_trackables = n_trackables  # Number of trackable objects
        self.STATE_DIM = 2  # State dimension (x, y coordinates)
        self.OBS_DIM = 2  # Observation dimension (noisy x, y coordinates)

        # Initialize mean state vector and covariance matrix for each trackable
        self.m = np.zeros((self.n_trackables, self.STATE_DIM, 1))
        self.P = np.zeros((self.n_trackables, self.STATE_DIM, self.STATE_DIM))

        # Initial position guess with high uncertainty
        for i in range(self.n_trackables):
            self.P[i, ...] = np.eye(self.STATE_DIM) * initial_pos_noise

        # Initialize observation noise covariance matrix, state transition matrix, and process noise covariance matrix
        self.R = np.zeros((self.n_trackables, self.OBS_DIM, self.OBS_DIM))
        self.A = np.zeros((self.n_trackables, self.STATE_DIM, self.STATE_DIM))
        self.Q = np.zeros((self.n_trackables, self.STATE_DIM, self.STATE_DIM))

        for i in range(self.n_trackables):
            self.A[i, ...] = np.eye(self.STATE_DIM)  # Identity matrix for state transition
            self.Q[i, ...] = np.eye(self.STATE_DIM) * process_noise**2  # Process noise covariance
            self.R[i, ...] = np.eye(self.OBS_DIM) * obs_noise**2  # Observation noise covariance

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
                
    def apply_filter(self, loc_x_y_unfilt: tuple):
        """
        Apply the Kalman filter to a new set of observations.
        """
        observations = np.array(loc_x_y_unfilt).reshape(1, self.OBS_DIM, 1)
        self.predict()
        self.update(observations)
        return self.m[:, :, 0].flatten()*(2, 2)