import matplotlib.pyplot as plt
import numpy as np
import matplotlib.animation as animation
import matplotlib as mpl
cmap = mpl.colormaps['jet']


def saturation(val, sat_val):
    return max(min(val, sat_val), -sat_val)

def distance_dots(d1,d2):
    return np.sqrt((d1.x - d2.x)**2 + (d1.y - d2.y)**2)

def angle_diff(a1, a2):
    return (a1 - a2 + np.pi) % (2*np.pi) - np.pi

class Dot:
    def __init__(self, x0, y0, mass=0.001):
        self.x = x0
        self.y = y0
        self.vx = 0
        self.vy = 0
        self._mass = mass
        self._fx = 0
        self._fy = 0

    def clear_force(self):
        self._fx = 0
        self._fy = 0

    def add_force(self, fx, fy):
        self._fx += fx
        self._fy += fy

    def update_position(self, dt):
        self.vx += self._fx / self._mass * dt
        self.vy += self._fy / self._mass * dt
        self.x += self.vx * dt
        self.y += self.vy * dt
    
    def plot(self):
        plt.plot([self.x], [self.y], "o", color="orange")


class AbstractConstraint:
    def __init__(self):
        pass

    def update_self(self, dt):
        pass

    def update_dots_force(self, dt):
        pass

    def update_dots_pos(self, dt):
        pass

    def plot(self):
        pass


class ElasticJoint(AbstractConstraint):
    def __init__(self, d1, d2, length=None, stiffness = 900, damping = 0.5, lpf_alpha = 0.3):
        """
        if lenght is unset, the length will be set to the intial distance beween points 
        """
        super().__init__()
        self.d1 = d1
        self.d2 = d2
        self._rest_length = length
        if self._rest_length is None:
            self._rest_length = distance_dots(self.d1, self.d2)
        self.length = None
        self.tension = 0 # >0 if extended
        self._stiffness = stiffness
        self._damping = damping
        self._lpf_alpha = lpf_alpha
        self.healthy = False
        self.precision = 0

    def update_self(self, dt):
        # update of local params
        previous_length = self.length
        previous_tension = self.tension
        self.length = distance_dots(self.d1, self.d2)
        if previous_length is None:
            previous_length = self.length
        
        # stiffness term
        self.tension = saturation(self.length - self._rest_length, self._rest_length*0.1) * self._stiffness
        
        # friction term
        self.tension += saturation(self.length - previous_length, self._rest_length*1000/dt) * self._damping / dt
        
        # low pass filter
        self.tension = previous_tension*(self._lpf_alpha) + self.tension*(1-self._lpf_alpha)

        # stablity checks
        self.precision = self.length - self._rest_length
        if abs(self.length - self._rest_length) > 1e-2 and abs((self.length - previous_length)/dt) > 1e-2:
            self.healthy = False
        else:
            self.healthy = True

    def update_dots_force(self, dt):
        # update dot forces
        fx = (self.d2.x - self.d1.x) / self.length * self.tension
        fy = (self.d2.y - self.d1.y) / self.length * self.tension
        self.d1.add_force(fx,fy)
        self.d2.add_force(-fx,-fy)

    def plot(self):
        plt.plot([self.d1.x, self.d2.x], [self.d1.y, self.d2.y], color=cmap(abs(self.tension)))


class Gravity(AbstractConstraint):
    def __init__(self, dots):
        super().__init__()
        self.dots = dots

    def update_dots_force(self, dt):
        for d in self.dots:
            d.add_force(0, - 9.81 * d._mass)


class FixedDot(AbstractConstraint):
    def __init__(self, dot):
        super().__init__()
        self.dot = dot
        self.x0 = dot.x
        self.y0 = dot.x

    def update_dots_pos(self, dt):
        self.dot.x = self.x0
        self.dot.y = self.y0
        self.dot.vx = 0
        self.dot.vy = 0

class Wall(AbstractConstraint):
    def __init__(self, dots, x = None, y = None, positive = True, bouncyness=0.8):
        assert not (x is None and y is None)
        assert not (x is not None and y is not None)
        super().__init__()
        self.dots = dots
        self.y = y
        self.x = x
        self.bouncyness = bouncyness
        if x is not None:
            if positive:
                self.update_dots_pos = self._update_dots_pos_x_pos
            else:
                self.update_dots_pos = self._update_dots_pos_x_neg
        else:
            if positive:
                self.update_dots_pos = self._update_dots_pos_y_pos
            else:
                self.update_dots_pos = self._update_dots_pos_y_neg

    def _update_dots_pos_y_pos(self, dt):
        for d in self.dots:
            if d.y < self.y:
                d.y = self.y
                if d.vy < 0:
                    d.vy *= -self.bouncyness
    
    def _update_dots_pos_y_neg(self, dt):
        for d in self.dots:
            if d.y > self.y:
                d.y = self.y
                if d.vy > 0:
                    d.vy *= -self.bouncyness

    def _update_dots_pos_x_pos(self, dt):
        for d in self.dots:
            if d.x < self.x:
                d.x = self.x
                if d.vx < 0:
                    d.vx *= -self.bouncyness
    
    def _update_dots_pos_x_neg(self, dt):
        for d in self.dots:
            if d.x > self.x:
                d.x = self.x
                if d.vx > 0:
                    d.vx *= -self.bouncyness

    def plot(self):
        if self.y is not None:
            plt.plot([-15, 15], [self.y, self.y], color="black")
        else:
            plt.plot([self.x, self.x], [-15, 15], color="black")

class Muscle(AbstractConstraint):
    def __init__(self, base_d, d1, d2):
        """
        Idea : use instead other ElasticJoint Between points
        """
        super().__init__()
        self.target_angle = 120/180*3.1415
        self.base_d = base_d
        self.d1 = d1
        self.d2 = d2
        self.stiffness = 4
        self.damping = 0.5
        self.angle = None

    def update_dots_force(self, dt):
        previous_angle = self.angle
        d1_x = (self.d1.x - self.base_d.x)
        d1_y = (self.d1.y - self.base_d.y)
        d2_x = (self.d2.x - self.base_d.x)
        d2_y = (self.d2.y - self.base_d.y)
        self.angle = np.arctan2(d1_y, d1_x) - np.arctan2(d2_y, d2_x)
        if previous_angle is None:
            previous_angle = self.angle
        moment = angle_diff(self.angle, self.target_angle) * self.stiffness
        moment += angle_diff(self.angle, previous_angle) * self.damping / dt
        d1_l_2 = d1_x**2 + d1_y**2
        d2_l_2 = d2_x**2 + d2_y**2
        self.d1.add_force(moment*d1_y/d1_l_2, - moment*d1_x/d1_l_2)
        self.d2.add_force(- moment*d2_y/d2_l_2, moment*d2_x/d2_l_2)
        self.base_d.add_force(-moment*d1_y/d1_l_2, moment*d1_x/d1_l_2)
        self.base_d.add_force(moment*d2_y/d2_l_2, -moment*d2_x/d2_l_2)


class RepulsiveDots(AbstractConstraint):
    def __init__(self, dots):
        self.dots = dots
        self.repulsive_func = lambda r : 0.01/(0.01 + r)

    def update_dots_force(self, dt):
        for id1, d1 in enumerate(self.dots):
            for id2, d2 in enumerate(self.dots):
                if id1 == id2:
                    continue
                dist = distance_dots(d1, d2)
                force = self.repulsive_func(dist)
                ux = (d2.x - d1.x) / dist
                uy = (d2.y - d1.y) / dist
                d2.add_force(ux * force, uy * force)
                d1.add_force(-ux * force, -uy * force)


class ViscousFluid(AbstractConstraint):
    def __init__(self, dots):
        self.dots = dots
        self.cx = 0.01

    def update_dots_force(self, dt):
        for d in self.dots:
            d.add_force(- self.cx * d.vx, - self.cx * d.vy)

class Body:
    def __init__(self, dots, constraints):
        self.constraints = constraints
        self.dots = dots
        self.healthy = False
        self.mean_precision = 0

    def update(self, dt):
        healthy = True
        mean_precision = 0
        wp = 0
        for d in self.dots:
            d.clear_force()
        for c in self.constraints:
            c.update_self(dt)
        for c in self.constraints:
            c.update_dots_force(dt)
        for d in self.dots:
            d.update_position(dt)
        for c in self.constraints:
            c.update_dots_pos(dt)
        #     if not c.healthy:
        #         healthy = False
        #     mean_precision += c.precision
        #     if c.precision > wp:
        #         wp = c.precision
        # self.mean_precision = wp# mean_precision/len(self._connections)
        # self.healthy = healthy

def pendulum(n_segments, seg_length):
    x, y = 0,0
    alpha = 0.1
    dots = []
    for i in range(n_segments + 1):
        dots.append(Dot(x,y))
        x += np.cos(alpha)*seg_length
        y += np.sin(alpha)*seg_length
        alpha += 0.1*seg_length

    constraints = [Gravity(dots), FixedDot(dots[0])]
    for i in range(n_segments):
        constraints.append(ElasticJoint(dots[i], dots[i+1]))
    return Body(dots, constraints)


def soft_body_and_floor():
    stiffness = 20
    damping = 0.5
    mass = 0.05
    d = [Dot(-1,0,mass), Dot(0,-1,mass), Dot(1,0,mass), Dot(0,1,mass), Dot(2,1,mass), Dot(1,2,mass)]
    c = [
        ElasticJoint(d[0], d[1], stiffness=stiffness, damping=damping),
        ElasticJoint(d[0], d[2], stiffness=stiffness, damping=damping),
        ElasticJoint(d[0], d[3], stiffness=stiffness, damping=damping),
        ElasticJoint(d[1], d[2], stiffness=stiffness, damping=damping),
        ElasticJoint(d[1], d[3], stiffness=stiffness, damping=damping),
        ElasticJoint(d[2], d[3], stiffness=stiffness, damping=damping),
        ElasticJoint(d[4], d[5], stiffness=stiffness, damping=damping),
        ElasticJoint(d[3], d[5], stiffness=stiffness, damping=damping),
        ElasticJoint(d[3], d[4], stiffness=stiffness, damping=damping),
        ElasticJoint(d[2], d[5], stiffness=stiffness, damping=damping),
        ElasticJoint(d[2], d[4], stiffness=stiffness, damping=damping),
        Gravity(d),
        Floor(-4, d),
    ]
    return Body(d, c)

def test_muscle():
    stiffness = 20
    damping = 0.5
    mass = 0.05
    d = [Dot(0,0,mass), Dot(1,0,mass), Dot(0,1,mass)]
    c = [
        ElasticJoint(d[0], d[1], stiffness=stiffness, damping=damping),
        ElasticJoint(d[2], d[1], stiffness=stiffness, damping=damping),
        Floor(-1, d),
        #Gravity(d),
        Muscle(d[1], d[0], d[2])
    ]
    return Body(d,c)

#b = pendulum(3, 1)
#b = soft_body_and_floor()
b = test_muscle()


def update(i):
    if i%25 == 0:
        if b.constraints[3].target_angle > 90/180*3.1415:
            b.constraints[3].target_angle = 10/180*3.1415
        else:
            b.constraints[3].target_angle = 120/180*3.1415

    for _ in range(50):
        b.update(0.001)
    plt.clf()
    plt.xlim((-7, 7))
    plt.ylim((-7, 7))
    plt.gca().set_aspect('equal')
    for c in b.constraints:
        c.plot()
    for d in b.dots:
        d.plot()

fig = plt.figure()
ani = animation.FuncAnimation(fig=fig, func=update, interval=10)
plt.show()


