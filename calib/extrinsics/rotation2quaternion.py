import numpy as np
np.set_printoptions(suppress=True, precision=9)


def inv_SE3(T: np.ndarray):
    T = np.asarray(T, dtype=float)
    R = T[:3, :3]
    t = T[:3, 3]
    Ti = np.eye(4, dtype=float)
    Rt = R.T
    Ti[:3, :3] = Rt
    Ti[:3, 3]  = -Rt @ t
    return Ti

def rotmat_to_quat(R: np.ndarray, *, orthonormalize: bool = True) -> np.ndarray:
    """
    Convert a 3x3 rotation matrix to a quaternion (w, x, y, z).

    Parameters
    ----------
    R : (3,3) array_like
        Rotation matrix (assumed right-handed; acts on column vectors).
    orthonormalize : bool
        If True, project R to the nearest proper rotation via SVD (helps with tiny
        numerical errors). Set False if R is already perfect.

    Returns
    -------
    q : (4,) np.ndarray
        Quaternion in scalar-first order: (w, x, y, z), normalized.
    """
    R = np.asarray(R, dtype=np.float64)
    if R.shape != (3, 3):
        raise ValueError("R must be 3x3")

    if orthonormalize:
        U, _, Vt = np.linalg.svd(R)
        R = U @ Vt
        # Ensure det(R)=+1
        if np.linalg.det(R) < 0:
            U[:, -1] *= -1
            R = U @ Vt

    t = np.trace(R)
    if t > 0.0:
        s = np.sqrt(t + 1.0) * 2.0
        w = 0.25 * s
        x = (R[2,1] - R[1,2]) / s
        y = (R[0,2] - R[2,0]) / s
        z = (R[1,0] - R[0,1]) / s
    else:
        # Find the largest diagonal entry to improve stability
        i = np.argmax([R[0,0], R[1,1], R[2,2]])
        if i == 0:
            s = np.sqrt(1.0 + R[0,0] - R[1,1] - R[2,2]) * 2.0
            w = (R[2,1] - R[1,2]) / s
            x = 0.25 * s
            y = (R[0,1] + R[1,0]) / s
            z = (R[0,2] + R[2,0]) / s
        elif i == 1:
            s = np.sqrt(1.0 + R[1,1] - R[0,0] - R[2,2]) * 2.0
            w = (R[0,2] - R[2,0]) / s
            x = (R[0,1] + R[1,0]) / s
            y = 0.25 * s
            z = (R[1,2] + R[2,1]) / s
        else:
            s = np.sqrt(1.0 + R[2,2] - R[0,0] - R[1,1]) * 2.0
            w = (R[1,0] - R[0,1]) / s
            x = (R[0,2] + R[2,0]) / s
            y = (R[1,2] + R[2,1]) / s
            z = 0.25 * s

    q = np.array([w, x, y, z], dtype=np.float64)
    q /= np.linalg.norm(q) + 1e-16
    return q

# --- quick checks ---
if __name__ == "__main__":
    TransformationMatrix = np.array([
            [0.999994564612669,-0.00327143011166783,-0.000410475508767800,0.253736175410149],
            [0.00326819763481066,0.999965451959397,-0.00764289028177120, -0.000362553856124796],
            [0.000435464509051199,0.00764150722461529,0.999970708440001, -0.000621002717451192],
            [0.0,0.0,0.0,1.0]
        ])


    inv_TransformationMatrix = inv_SE3(TransformationMatrix)
    print(inv_TransformationMatrix)
    print(f"{rotmat_to_quat(inv_TransformationMatrix[:3, :3])}")  # ~ [1, 0, 0, 0], [w,x,y,z]


