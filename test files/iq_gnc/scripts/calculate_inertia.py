import numpy as np

def skew_symmetric(v):
    """Matriz antisimétrica para el producto vectorial [v x]."""
    return np.array([[0, -v[2], v[1]],
                    [v[2], 0, -v[0]],
                    [-v[1], v[0], 0]])

def parallel_axis_theorem(I_local, m, r):
    """
    Aplica el teorema de Steiner para trasladar la inercia I_local a un nuevo marco.
    
    Args:
        I_local: Matriz de inercia local (3x3).
        m: Masa del objeto.
        r: Vector de posición del centro de masa local respecto al nuevo marco (3x1).
    
    Returns:
        I_new: Matriz de inercia en el nuevo marco (3x3).
    """
    r = np.array(r).reshape(3, 1)
    I_new = I_local + m * (np.dot(r.T, r) * np.eye(3) - np.dot(r, r.T))
    return I_new

# -----------------------------------------------------------------------------
# Matrices de inercia locales (en sus marcos de referencia)
# -----------------------------------------------------------------------------
# --- Modelo iris (base_link y rotores) ---
I_base = np.diag([0.008, 0.015, 0.017])  # kg·m²
mass_base = 1.5  # kg

I_rotor = np.diag([9.75e-6, 1.66704e-4, 1.67604e-4])  # kg·m²
mass_rotor = 0.025  # kg

# Posiciones de los rotores respecto al base_link del dron (x, y, z) en metros
rotor_positions = [
    [0.13, -0.22, 0.023],   # rotor_0
    [-0.13, 0.2, 0.023],    # rotor_1
    [0.13, 0.22, 0.023],    # rotor_2
    [-0.13, -0.2, 0.023]    # rotor_3
]

# --- Modelo gimbal_small_2d ---
I_gimbal_base = np.diag([0.0001, 0.0001, 0.0001])  # kg·m²
mass_gimbal_base = 0.2  # kg
gimbal_base_pos = [0, 0, 0.18]  # Pose del gimbal respecto al base_link del dron

I_tilt = np.diag([0.00001, 0.00001, 0.00001])  # kg·m²
mass_tilt = 0.01  # kg
tilt_pos_rel = [0, 0, 0.02]  # Pose del tilt_link respecto al base_link del gimbal

# -----------------------------------------------------------------------------
# Cálculo de la matriz de inercia total en el base_link del dron
# -----------------------------------------------------------------------------
# 1. Inercia del base_link del dron (ya está en el marco correcto)
I_total = I_base.copy()

# 2. Contribución de los rotores (Steiner)
for pos in rotor_positions:
    I_total += parallel_axis_theorem(I_rotor, mass_rotor, pos)

# 3. Contribución del gimbal (base_link del gimbal)
I_total += parallel_axis_theorem(I_gimbal_base, mass_gimbal_base, gimbal_base_pos)

# 4. Contribución del tilt_link (trasladado al base_link del dron)
tilt_pos_global = np.array(gimbal_base_pos) + np.array(tilt_pos_rel)
I_total += parallel_axis_theorem(I_tilt, mass_tilt, tilt_pos_global)

# -----------------------------------------------------------------------------
# Resultado
# -----------------------------------------------------------------------------
print("Matriz de inercia total (base_link del dron):\n", I_total)
print("\nForma simplificada:")
print(f"Ixx = {I_total[0, 0]:.6f} kg·m²")
print(f"Iyy = {I_total[1, 1]:.6f} kg·m²")
print(f"Izz = {I_total[2, 2]:.6f} kg·m²")
print(f"Ixy = {I_total[0, 1]:.6f} kg·m²")
print(f"Ixz = {I_total[0, 2]:.6f} kg·m²")
print(f"Iyz = {I_total[1, 2]:.6f} kg·m²")