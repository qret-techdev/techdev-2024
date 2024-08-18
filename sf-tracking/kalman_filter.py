import numpy as np
import simdkalman

class KalmanFilterManager:
    """
    Manages the Kalman filtering process for tracking positions based on noisy x, y coordinates.
    """
    def __init__(self, num_objects=1, initial_noise=1.0, obs_noise=0.5, process_noise=0.05):
        self.num_objects = num_objects
        self.STATE_DIM = 2  # State dimension (x, y coordinates)
        self.OBS_DIM = 2    # Observation dimension (noisy x, y coordinates)

        # Initialize mean state vector and covariance matrix for each object
        self.mean_state = np.zeros((self.num_objects, self.STATE_DIM, 1))
        self.cov_matrix = np.eye(self.STATE_DIM) * initial_noise

        # Initialize matrices
        self.transition_matrix = np.eye(self.STATE_DIM)  # Identity matrix for state transition
        self.process_cov = np.eye(self.STATE_DIM) * process_noise**2
        self.obs_cov = np.eye(self.OBS_DIM) * obs_noise**2

    def predict(self):
        """
        Predict the next state and covariance.
        """
        self.mean_state, self.cov_matrix = simdkalman.primitives.predict(self.mean_state, self.cov_matrix, self.transition_matrix, self.process_cov)

    def update(self, observations):
        """
        Update the state and covariance with new observations.
        """
        H = np.tile(np.eye(self.STATE_DIM), (self.num_objects, 1, 1))
        y_residual = observations - simdkalman.primitives.ddot(H, self.mean_state)
        self.mean_state, self.cov_matrix = simdkalman.primitives.update(self.mean_state, self.cov_matrix, H, self.obs_cov, y_residual)

    def apply_filter(self, loc_x_y_unfilt):
        """
        Apply the Kalman filter to a new set of observations.
        """
        observations = np.array(loc_x_y_unfilt).reshape(1, self.OBS_DIM, 1)
        self.predict()
        self.update(observations)
        return self.mean_state[:, :, 0].flatten()*(2, 2)
