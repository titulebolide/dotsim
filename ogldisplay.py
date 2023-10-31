import pyglet
import examples
import threading
import time
import dotsim as ds
import matplotlib as mpl
cmap = mpl.colormaps['jet']

#b = examples.pendulum(3, 2)
#b = examples.soft_body_and_floor()
b = examples.test_muscle()

class ThreadedSim(threading.Thread):
    def __init__(self, body):
        super().__init__()
        self.body = body
        self.dt = 0.001

    def run(self):
        self._done = False
        t_begin = time.time()
        warning_prompted = False
        while not self._done:
            self.body.update(self.dt)
            pause = self.dt - time.time() + t_begin
            if pause < 0:
                if not warning_prompted:
                    print("Lagging behind!")
                    warning_prompted = True
            else:
                time.sleep(pause)
            t_begin = time.time()

    def stop(self):
        self._done = True

    # def draw():

sim = ThreadedSim(b)
sim.start()

window = pyglet.window.Window()
print(window.width, window.height)

zoom = 30

def trans(x,y):
    return x*zoom + window.width / 2, y*zoom + window.height / 2


t = time.time()
i = 0
@window.event
def on_draw():
    global i, t
    i+=1
    if i%100 == 0:
        print(f"{round(100/(time.time() - t), 2)} fps")
        t = time.time()

    window.clear()
    batch = pyglet.graphics.Batch()
    circles = []

    for c in sim.body.constraints:
        if type(c) == ds.ElasticJoint:
            # color = list(cmap(abs(c.tension)))[:3]
            # color = [int(i*256) for i in color]
            color = (255, 30, 30)
            circles.append(
                pyglet.shapes.Line(*trans(c.d1.x, c.d1.y), *trans(c.d2.x, c.d2.y), 3, color=color, batch=batch)
            )
        if type(c) == ds.Wall:
            if c.x is not None:
                circles.append(
                    pyglet.shapes.Line(trans(c.x, 0)[0], 0, trans(c.x, 0)[0], window.height, 3, color=(30, 30, 255), batch=batch)
                )
            else:
                circles.append(
                    pyglet.shapes.Line(0, trans(0, c.y)[1], window.width, trans(0, c.y)[1], 3, color=(30, 30, 255), batch=batch)
                )
    
    for d in sim.body.dots:
        circles.append(
            pyglet.shapes.Circle(*trans(d.x, d.y), radius=5, color=(50, 225, 30), batch = batch)
        )
    batch.draw()

pyglet.app.run()
sim.stop()
sim.join()