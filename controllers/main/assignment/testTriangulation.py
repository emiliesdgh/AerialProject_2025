import numpy as np

def triangulate_point(u1, v1, u2, v2, C1, C2, f_pixels):
    W, H = 300, 300  # Image dimensions

    # Convert pixel coordinates to camera frame
    v1_cam = np.array([u1 - W/2, v1 - H/2, f_pixels])
    v2_cam = np.array([u2 - W/2, v2 - H/2, f_pixels])

    # World frame vectors (since R = I)
    r = v1_cam
    s = v2_cam

    # Solve for lambda and mu
    A = np.column_stack((r, -s))
    b = C2 - C1
    lamb_mu = np.linalg.lstsq(A, b, rcond=None)[0]
    lambda_, mu = lamb_mu

    # Compute 3D position
    P1 = C1 + lambda_ * r
    P2 = C2 + mu * s
    P = (P1 + P2) / 2

    return P

# Example usage
C1 = np.array([0.03, 0, 0.01])  # Camera 1 position
C2 = np.array([0.1, 0, 0.01])   # Camera 2 position (example)

f_pixels = 161.0139  # Given focal length in pixels
u1, v1 = 150, 120  # Pixel coordinates in first image
u2, v2 = 140, 115  # Pixel coordinates in second image

P = triangulate_point(u1, v1, u2, v2, C1, C2, f_pixels)
print("3D Position:", P)
