import dotsim
import matplotlib.animation as animation
import matplotlib.pyplot as plt

b = dotsim.pendulum(1, 1)
#b = soft_body_and_floor()
#b = test_muscle()
#b = test_fluid()
# b = test_lone_ball()

def update(i):
    # if i%30 == 0:
    #     if b.constraints[0].target_angle > 90/180*3.1415:
    #         b.constraints[0].target_angle = 90/180*3.1415
    #     else:
    #         b.constraints[0].target_angle = 270/180*3.1415

    for _ in range(50):
        b.constraints[1].x0 = np.sin(b.t*3)
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
