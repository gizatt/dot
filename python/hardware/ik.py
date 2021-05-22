import numpy as np

def normalize_angle(angle):
    """  
    https://www.programcreek.com/python/?code=DerekK88%2FPICwriter%2FPICwriter-master%2Fpicwriter%2Ftoolkit.py
    """
    angle = angle % (2 * np.pi)
    if angle > np.pi:
        angle -= 2 * np.pi
    return angle 

def single_leg_forward_kin(
        ee_target,
        hip_1_in_root_frame,
        hip_2_in_hip_1_frame,
        hip_length,
        shin_length):
    # Hip 1 in root frame: [x y z] hip roll (abduction) servo
    # horn center in robot body root frame. +x is forward,
    # +y left, +z up.
    # hip_2_in_hip_1_frame: [x y z] hip pitch axis intersection
    # with leg plane as offset from hip roll horn center at 0
    # hip1 roll angle.
    # +x forward, +y left, +z up.
    # hip, shin length: length of those links
    
    # First solve to put the hip abduction joint such that the
    # leg hip and shin plane aligns with the ee target point.

    # Get offsets from in the coronal plane of end effector and
    # the hip roll joint.
    # https://www.math-only-math.com/a-cos-theta-plus-b-sin-theta-equals-c.html
    A = ee_target[1] - hip_1_in_root_frame[1]
    B = ee_target[2] - hip_1_in_root_frame[2]
    hip_roll_to_leg_plane = hip_2_in_hip_1_frame[1]
    R = np.sqrt(A**2 + B**2)
    if hip_roll_to_leg_plane >= R:
        print("IK fail: ee point to close to hip roll point (%f vs %f)" % (hip_roll_to_leg_plane, R))
        return False
    if A == 0 and B != 0:
        q_roll = np.arcsin(hip_roll_to_leg_plane / B)
    elif B == 0 and A != 0:
        q_roll = np.arccos(hip_roll_to_leg_plane / A)
    else:
        q_roll = np.arctan2(B, A) + np.arccos(hip_roll_to_leg_plane / R)
    
    # Given that plane, do 2-joint IK in the plane. First, project everything into
    # leg plane at that angle by inverse rotating the ee point around the hip roll joint
    # by (minus) that angle.
    rotmat = np.array([
        [1., 0., 0.],
        [0., np.cos(q_roll), -np.sin(q_roll)],
        [0., np.sin(q_roll), np.cos(q_roll)]
    ])
    ee_pt_aligned = np.dot(rotmat.T, ee_target - hip_1_in_root_frame) + hip_1_in_root_frame
    hip_pitch_origin = hip_1_in_root_frame + hip_2_in_hip_1_frame
    assert(np.allclose(ee_pt_aligned[1], hip_pitch_origin[1]), 
            "%f vs %f" % (ee_pt_aligned[1], hip_pitch_origin[1]))

    x_off = ee_pt_aligned[0] - hip_pitch_origin[0]
    z_off = ee_pt_aligned[2] - hip_pitch_origin[2]
    # classic 2-link IK in normal joint angles
    # https://robotacademy.net.au/lesson/inverse-kinematics-for-a-2-joint-robot-arm-using-geometry/
    q2_classic = np.arccos((x_off**2 + z_off**2 - hip_length**2 - shin_length**2) / (2*hip_length*shin_length))
    if not np.isfinite(q2_classic):
        print("IK fail: knee pitch calculation infeasible.")
        return False
    # Weird mapping: our +x is the diagram's +y, our +z is diagram's -y.
    q1_classic = np.arctan2(x_off, -z_off) - np.arctan2(shin_length * np.sin(q2_classic), hip_length +  shin_length*np.cos(q2_classic))
    if not np.isfinite(q1_classic):
        print("IK fail: hip pitch calculation infeasible.")
        return False

    # Convert to joint positions for our weird four bar
    q_hip_pitch = -q1_classic
    q_knee_pitch = q_hip_pitch + np.pi/2 - q2_classic
    return np.array(
        [normalize_angle(a) for a in [q_roll, q_hip_pitch, q_knee_pitch]]
    )

def test_ik():
    shin_length = 1.
    hip_length = 1.
    out = single_leg_forward_kin(
        np.array([0., 0., -np.sqrt(shin_length**2 + hip_length**2)]),
        np.zeros(3),
        np.zeros(3),
        hip_length, shin_length
    )
    print("Test even down: ", out)
    assert(np.allclose(out, np.array([0., np.pi/4., np.pi/4.])))

    shin_length = 1.
    hip_length = 1.
    out = single_leg_forward_kin(
        np.array([shin_length/2., 0., -np.sqrt(hip_length**2 - (shin_length/2.)**2)]),
        np.zeros(3),
        np.zeros(3),
        hip_length, shin_length
    )
    print("Test flat shin down: ", out)
    assert(np.allclose(out, np.array([0., np.arcsin(shin_length/(2.*hip_length)), 0.])))

    # Leg at zero angles should reach forward a shin length
    # and 
    out = single_leg_forward_kin(
        np.array([shin_length, 0., -hip_length]),
        np.zeros(3),
        np.zeros(3),
        hip_length, shin_length
    )
    print("Test zero angles: ", out)
    assert(np.allclose(out, np.array([0., 0., 0.])))

    # Straight out to the left, half bent leg
    out = single_leg_forward_kin(
        np.array([0., np.sqrt(shin_length**2 + hip_length**2), 0.]),
        np.zeros(3),
        np.zeros(3),
        hip_length, shin_length
    )
    print("Test straight out left even bent: ", out)
    assert(np.allclose(out, np.array([np.pi/2, np.pi/4., np.pi/4.])))

    # Straight out to the left with an offset hip should account for
    # the offset hip.
    offset = np.array([0.2, 0.3, 0.4])
    out = single_leg_forward_kin(
        np.array([0., np.sqrt(shin_length**2 + hip_length**2), 0.]) + offset,
        offset,
        np.zeros(3),
        hip_length, shin_length
    )
    print("Test straight out offset hip: ", out)
    assert(np.allclose(out, np.array([np.pi/2, np.pi/4., np.pi/4.])))

    # Straight down with an offset hip pitch plane should account for
    # the offset.
    offset = np.array([0.00, 0.1, 0.0])
    out = single_leg_forward_kin(
        np.array([0., 0., -np.sqrt(shin_length**2 + hip_length**2)]) + offset,
        np.zeros(3),
        offset,
        hip_length, shin_length
    )
    print("Test down offset hip pitch plane: ", out)
    assert(np.allclose(out, np.array([0., np.pi/4., np.pi/4.])))

    # Straight out to the left with an offset hip pitch plane should account for
    # the offset.
    out = single_leg_forward_kin(
        np.array([0., np.sqrt(shin_length**2 + hip_length**2), 0.]) + np.array([0., 0., 0.1]), # ( rotated hip lateral offset )
        np.zeros(3),
        np.array([0., 0.1, 0.0]),
        hip_length, shin_length
    )
    print("Test straight out left offset hip pitch plane: ", out)
    assert(np.allclose(out, np.array([np.pi/2., np.pi/4., np.pi/4.])))


if __name__ == "__main__":
    test_ik()