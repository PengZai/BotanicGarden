import numpy as np



def inverseTransformation( T ):

    R = T[0:3, 0:3]
    t = T[0:3, 3]

    R_inv = R.T
    t_inv = -R_inv @ t

    T_inv = np.eye(4)
    T_inv[0:3, 0:3] = R_inv
    T_inv[0:3, 3] = t_inv


    return T_inv



T = np.array([
    [0.999634566,  -0.024872196,   0.0106254411, -0.0714691819],
    [-0.9997844018, -0.0201338971,  0.0051549656,  0.1460676939],
    [0.0197725191, -0.9978475484, -0.0625178777, -0.0049711072],
    [0.0       ,0.0      ,0.0       ,1.0        ]
])




np.set_printoptions(precision=10, suppress=True)

inv_T = inverseTransformation(T)
print("inv_T", inv_T)


