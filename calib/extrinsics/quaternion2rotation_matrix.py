import numpy as np
from scipy.spatial.transform import Rotation as R



np.set_printoptions(precision=10, suppress=True)

# qx, qy, qz, qw
q = [-0.49967402836949243,  0.5061239984700692, -0.49146081543072123,  0.5026237492419483]  # 90° around Z-axis
rotation_matrix = R.from_quat(q).as_matrix()


print("Rotation matrix:\n", rotation_matrix)