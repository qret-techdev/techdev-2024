import numpy as np
import simdkalman

class ExampleEKF:
    """
    Vectorized EKF model for estimating the positions of the trackables
    based on noisy x, y coordinates.
    """
    def __init__(self, simulation):
        self.n_trackables = len(simulation.trackables)  # Number of trackable objects
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


class Simulation:
    """
    Simulate movement of the rocket. This is not vectorized and does not
    aim to be very efficient.
    """
    def __init__(self):
        class Trackable:
            def __init__(self):
                self.position = np.array([0.0, 0.0])
                self.velocity = np.array([0.5, 1.0])

            def move(self, delta_t):
                self.position += self.velocity * delta_t

        N_TRACKABLES = 1
        self.trackables = [Trackable() for _ in range(N_TRACKABLES)]
        self.time = 0

    def simulate_step(self):
        DELTA_T = 0.08
        T_MAX = 20
        MEASUREMENT_NOISE = 0.1

        self.time += DELTA_T
        if self.time > T_MAX: return None

        true_positions = []
        observations = np.zeros((len(self.trackables), 2, 1))

        for obj_i, obj in enumerate(self.trackables):
            obj.move(DELTA_T)
            true_positions.append(obj.position)

            noisy_position = obj.position + np.random.normal(scale=MEASUREMENT_NOISE, size=2)
            observations[obj_i, :, 0] = noisy_position

        return (true_positions, observations)

def uncertainty_ellipse_95(mean, cov):
    N = 30
    theta = np.linspace(0, 2*np.pi, num=N)
    circle = np.vstack([c[np.newaxis, :] for c in (np.sin(theta), np.cos(theta))])

    u, s_vec, _ = np.linalg.svd(cov)
    scale_for_95 = 2.0 * np.sqrt(5.991)
    s_mat = np.diag(np.sqrt(s_vec)) * scale_for_95

    return np.dot(u, np.dot(s_mat, circle)).transpose() + mean[np.newaxis, :]

# Initialize
np.random.seed(100)

simulation = Simulation()
ekf = ExampleEKF(simulation)

true_trajectories = []
estimated_trajectories = []
estimated_uncertainties = []

# Run simulation and estimation
while True:
    simulated = simulation.simulate_step()
    if simulated is None: break

    true_positions, observations = simulated
    true_trajectories.append(true_positions)

    ekf.predict()
    ekf.update(observations)

    estimated_trajectories.append(ekf.m[:, :, 0])
    for i in range(ekf.P.shape[0]):
        estimated_uncertainties.append(uncertainty_ellipse_95(ekf.m[i, :, 0], ekf.P[i, ...]))

# Visualize results
import matplotlib.pyplot as plt

for ell in estimated_uncertainties:
    plt.plot(ell[:, 0], ell[:, 1], color='blue', lw=1, alpha=0.1)

true_trajectories = np.array(true_trajectories)
estimated_trajectories = np.array(estimated_trajectories)
for i in range(len(true_trajectories[0])):
    kwargs = {}
    if i == 0: kwargs['label'] = 'true trajectories'
    plt.plot(true_trajectories[:, i, 0], true_trajectories[:, i, 1], 'k', **kwargs)
    if i == 0: kwargs['label'] = 'estimated trajectories'
    plt.plot(estimated_trajectories[:, i, 0], estimated_trajectories[:, i, 1], 'bx', alpha=0.5, **kwargs)

plt.legend()
plt.axis('equal')
plt.show()
