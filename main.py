import matplotlib.pyplot as plt
import numpy as np
import matplotlib.animation as animation

def saturation(val, sat_val):
    return max(min(val, sat_val), -sat_val)

def distance_dots(d1,d2):
    return np.sqrt((d1.x - d2.x)**2 + (d1.y - d2.y)**2)

class Dot:
    def __init__(self, x0, y0):
        self._connections = [] # (connection, other dot)
        self.x = x0
        self.y = y0
        self.vx = 0
        self.vy = 0
        self._mass = 0.001
        self._fx = 0
        self._fy = 0

    def update_force(self, dt):
        # connections must be updated before
        self._fx = 0
        self._fy = 0
        for c, d in self._connections:
            self._fx += (d.x - self.x) / c.length * c.tension
            self._fy += (d.y - self.y) / c.length * c.tension
        self._fy += - 9.81 * self._mass
    
    def update_position(self, dt):
        self.vx += self._fx / self._mass * dt
        self.vy += self._fy / self._mass * dt
        self.x += self.vx * dt
        self.y += self.vy * dt


class Connection:
    def __init__(self, d1, d2, length=None, stiffness = 500, damping = 0.5, lpf_alpha = 0.3):
        """
        if lenght is unset, the length will be set to the intial distance beween points 
        """
        self.d1 = d1
        self.d2 = d2
        self.d1._connections.append((self, d2))
        self.d2._connections.append((self, d1))
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

    def update(self, dt):
        previous_length = self.length
        previous_tension = self.tension
        self.length = distance_dots(self.d1, self.d2)
        if previous_length is None:
            previous_length = self.length
        
        # stiffness term
        self.tension = saturation(self.length - self._rest_length, self._rest_length*0.1) * self._stiffness
        
        # friction term
        self.tension += saturation(self.length - previous_length, self._rest_length*1000/dt) * self._damping / dt
        
        # low pass filtering
        self.tension = previous_tension*(self._lpf_alpha) + self.tension*(1-self._lpf_alpha)

        # stablity checks
        #print(abs(self.length - self._rest_length), abs((self.length - previous_length)/dt)) 
        self.precision = self.length - self._rest_length
        if abs(self.length - self._rest_length) > 1e-2 and abs((self.length - previous_length)/dt) > 1e-2:
            self.healthy = False
        else:
            self.healthy = True

class Body:
    def __init__(self, connections):
        self._connections = connections
        self.dots = set()
        self.healthy = False
        self.mean_precision = 0
        for c in self._connections:
            self.dots.add(c.d1)
            self.dots.add(c.d2)

    def update(self, dt):
        healthy = True
        mean_precision = 0
        wp = 0
        for c in self._connections:
            c.update(dt)
            if not c.healthy:
                healthy = False
            mean_precision += c.precision
            if c.precision > wp:
                wp = c.precision
        self.mean_precision = wp# mean_precision/len(self._connections)
        self.healthy = healthy
        for d in self.dots:
            d.update_force(dt)
        for d in self.dots:
            d.update_position(dt)

# double pendulum
# d1 = Dot(0,0)
# d2 = Dot(1,0)
# d3 = Dot(2,1)
# c1 = Connection(d1, d2, 1)
# c2 = Connection(d2, d3, 1)
# b = Body([c1, c2])

# rigid triangle
def rigid_triangle():
    d1 = Dot(0,0)
    d2 = Dot(1,0)
    d3 = Dot(-1,1)
    c1 = Connection(d1, d2, 1, damping=damp, stiffness=stiff)
    c2 = Connection(d2, d3, 1, damping=damp, stiffness=stiff)
    c3 = Connection(d3, d1, 1, damping=damp, stiffness=stiff)
    b = Body([c1, c2, c3])
    return b, [d1, d2, d3]

# pendulum
def pendulum(n_segments, seg_length):
    x, y = 0,0
    alpha = 0.1
    dots = []
    for i in range(n_segments + 1):
        dots.append(Dot(x,y))
        x += np.cos(alpha)*seg_length
        y += np.sin(alpha)*seg_length
        alpha += 0.1*seg_length

    conn = []
    for i in range(n_segments):
        conn.append(Connection(dots[i], dots[i+1], seg_length, damping=damp, stiffness=stiff))

    b = Body(conn)
    return b, dots

def test_damp_stiff():
    damps = np.linspace(0.05, 0.9, 20)
    stiffnesses = np.linspace(200, 4000, 20)
    precisions_pend = []
    precisions_tri = []
    #damp = 0.5
    stiff = 900
    for damp in damps:
    #for stiff in stiffnesses: 
        print(damp, stiff)

        b, dots = pendulum(30, 0.1)
        mp = 0
        for _ in range(30*150):
            b.update(0.001)
            dots[0].vx = 0
            dots[0].xy = 0
            dots[0].x = 0
            dots[0].y = 0
            if b.mean_precision > mp:
                mp = b.mean_precision
        precisions_pend.append(mp)

        # b, dots = rigid_triangle()
        # mp = 0
        # for nit in range(30*150):
        #     b.update(0.001)
        #     dots[0].vx = 0
        #     dots[0].xy = 0
        #     dots[0].x = 0
        #     dots[0].y = 0
        #     if b.mean_precision < 1e-4:
        #         break
        # precisions_tri.append(nit*0.001)
        

    plt.plot(damps, precisions_pend)
    plt.plot(damps, precisions_tri)
    plt.show()

def update(i):
    for _ in range(30):
        b.update(0.001)
        d1.vx = 0
        d1.xy = 0
        d1.x = 0
        d1.y = 0
    plt.clf()
    plt.xlim((-7, 7))
    plt.ylim((-7, 2))
    plt.gca().set_aspect('equal')
    for c in b._connections:
        plt.plot([c.d1.x, c.d2.x], [c.d1.y, c.d2.y], color="blue")
        plt.plot([c.d1.x], [c.d1.y], "o", color="orange")
        plt.plot([c.d2.x], [c.d2.y], "o", color="orange")

fig = plt.figure()
ani = animation.FuncAnimation(fig=fig, func=update, interval=10)
plt.show()


