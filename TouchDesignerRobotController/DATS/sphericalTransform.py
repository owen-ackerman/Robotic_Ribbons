def sphericalToXYZ(theta, phi, radius=1.0):
    """
    Convert spherical coordinates to Cartesian XYZ.
    
    Args:
        theta: Azimuth angle (radians) - rotation around Z axis
        phi: Polar angle (radians) - angle from Z axis
        radius: Rod length (default 1.0m)
    
    Returns:
        Dictionary with 'x', 'y', 'z' coordinates
    """
    theta = math.radians(theta)
    phi = math.radians(phi)
    x = radius * math.sin(phi) * math.cos(theta)
    z = radius * math.sin(phi) * math.sin(theta)
    y = radius * math.cos(phi)
    return [x,y,z]

    

def onValueChange(channel: Channel, sampleIndex: int, val: float, 
                  prev: float):

    i = me.digits
    x,y,z = sphericalToXYZ(op(f'null{i}')[0],op(f'null{i}')[1])
    i = i - 1
    op(f'const_xyz{i}').par.const0value = x
    op(f'const_xyz{i}').par.const1value = y
    op(f'const_xyz{i}').par.const2value = z
