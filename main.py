import matplotlib.pyplot as plt
import numpy as np
import matplotlib.animation as animation

def saturation(val, sat_val):
    return max(min(val, sat_val), -sat_val)

def distance_dots(d1,d2):
    return np.sqrt((d1.x - d2.x)**2 + (d1.y - d2.y)**2)

class Dot:
    def __init__(self, x0, y0):
        self.x = x0
        self.y = y0
        self.vx = 0
        self.vy = 0
        self._mass = 0.001
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


class FixedDistance(AbstractConstraint):
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
        plt.plot([self.d1.x, self.d2.x], [self.d1.y, self.d2.y], color="blue")


class Gravity(AbstractConstraint):
    def __init__(self, dots):
        super().__init__()
        self.dots = dots

    def update_dots_force(self, dt):
        for d in self.dots:
            d.add_force(0, - 9.81 * d._mass)


class FixedDot(AbstractConstraint):
    def __init__(self, d):
        super().__init__()
        self.dot = d
        self.x0 = d.x
        self.y0 = d.x

    def update_dots_pos(self, dt):
        self.dot.x = self.x0
        self.dot.y = self.y0
        self.dot.vx = 0
        self.dot.vy = 0


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
        constraints.append(FixedDistance(dots[i], dots[i+1]))
    return Body(dots, constraints)

b = pendulum(3, 1)

def update(i):
    for _ in range(50):
        b.update(0.001)
    plt.clf()
    plt.xlim((-7, 7))
    plt.ylim((-7, 2))
    plt.gca().set_aspect('equal')
    for c in b.constraints:
        c.plot()
    for d in b.dots:
        d.plot()

fig = plt.figure()
ani = animation.FuncAnimation(fig=fig, func=update, interval=10)
plt.show()


