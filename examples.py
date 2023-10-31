import dotsim as ds
import numpy as np

def pendulum(n_segments, seg_length):
    x, y = 0,0
    alpha = 0.1
    dots = []
    for i in range(n_segments + 1):
        dots.append(ds.Dot(x,y))
        x += np.cos(alpha)*seg_length
        y += np.sin(alpha)*seg_length
        alpha += 0.1*seg_length

    constraints = [ds.Gravity(dots), ds.FixedDot(dots[0])]
    for i in range(n_segments):
        constraints.append(ds.ElasticJoint(dots[i], dots[i+1]))
    return ds.Body(dots, constraints)


def soft_body_and_floor():
    stiffness = 20
    damping = 0.5
    mass = 0.05
    d = [ds.Dot(-1,0,mass), ds.Dot(0,-1,mass), ds.Dot(1,0,mass), ds.Dot(0,1,mass), ds.Dot(2,1,mass), ds.Dot(1,2,mass)]
    c = [
        ds.ElasticJoint(d[0], d[1], stiffness=stiffness, damping=damping),
        ds.ElasticJoint(d[0], d[2], stiffness=stiffness, damping=damping),
        ds.ElasticJoint(d[0], d[3], stiffness=stiffness, damping=damping),
        ds.ElasticJoint(d[1], d[2], stiffness=stiffness, damping=damping),
        ds.ElasticJoint(d[1], d[3], stiffness=stiffness, damping=damping),
        ds.ElasticJoint(d[2], d[3], stiffness=stiffness, damping=damping),
        ds.ElasticJoint(d[4], d[5], stiffness=stiffness, damping=damping),
        ds.ElasticJoint(d[3], d[5], stiffness=stiffness, damping=damping),
        ds.ElasticJoint(d[3], d[4], stiffness=stiffness, damping=damping),
        ds.ElasticJoint(d[2], d[5], stiffness=stiffness, damping=damping),
        ds.ElasticJoint(d[2], d[4], stiffness=stiffness, damping=damping),
        ds.Gravity(d),
        ds.Wall(d, y=-4),
    ]
    return ds.Body(d, c)


def test_muscle():
    stiffness = 900
    damping = 0.5
    mass = 0.05
    d = [ds.Dot(0,0,mass), ds.Dot(1,0,mass), ds.Dot(0,1,mass*10)]
    c = [
        ds.Muscle(d[1], d[0], d[2]),
        ds.ElasticJoint(d[0], d[1], stiffness=stiffness, damping=damping),
        ds.ElasticJoint(d[2], d[1], stiffness=stiffness, damping=damping),
        #ds.Wall(d, y=-0.2, d),
        #ds.Gravity(d),
    ]
    return ds.Body(d,c)

def test_fluid():
    d = []
    for i in range(10):
        for j in range(10):
            d.append(ds.Dot(i/3, j/3, 0.05))
    c = [
        ds.RepulsiveDots(d),
        ds.ViscousFluid(d),
        ds.Gravity(d),
        ds.Wall(d, x = -2),
        ds.Wall(d, x = 3, positive=False),
        ds.Wall(d, y = -2),
        ds.Wall(d, y = 3, positive=False),
    ]
    return ds.Body(d,c)

def test_lone_ball():
    d = [ds.Dot(0,0,1)]
    c = [
        ds.Gravity(d),
        # ds.ViscousFluid(d),
        ds.Wall(d, x = -3),
        ds.Wall(d, x = 3, positive=False),
        ds.Wall(d, y = -3),
        ds.Wall(d, y = 3, positive=False),
    ]
    return ds.Body(d,c)
