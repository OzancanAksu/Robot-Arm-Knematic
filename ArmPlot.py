import numpy as np
import pyglet

import Arm


def plot():
     "" " Bir kolu çizmek için bir iþlevi ve bunu hesaplamak
    Ters kinematik iþlemi için, fare (x, y) koordinatlarý verildiðinde
    Bu noktaya ulaþmak için uygun eklem açýlarýný bulur. """

    # kolun bir örneðini oluþturmak
    arm = Arm.Arm3Link ( L = np.array ([ 300 , 200 , 100 ]))

    # penceremizi oluþturmak
    window = pyglet.window.Window ()

    window = pyglet.window.Window()

    label = pyglet.text.Label(
        'Mouse (x,y)', font_name='Times New Roman',
        font_size=36, x=window.width//2, y=window.height//2,
        anchor_x='center', anchor_y='center')

    def get_joint_positions():
          "" " Bu yöntem  her bir eklemin (x, y) koordinatlarýný bulur" ""

        x = np.array([
            0,
            arm.L[0]*np.cos(arm.q[0]),
            arm.L[0]*np.cos(arm.q[0]) + arm.L[1]*np.cos(arm.q[0]+arm.q[1]),
            arm.L[0]*np.cos(arm.q[0]) + arm.L[1]*np.cos(arm.q[0]+arm.q[1]) +
            arm.L[2]*np.cos(np.sum(arm.q))]) + window.width/2

        y = np.array([
            0,
            arm.L[0]*np.sin(arm.q[0]),
            arm.L[0]*np.sin(arm.q[0]) + arm.L[1]*np.sin(arm.q[0]+arm.q[1]),
            arm.L[0]*np.sin(arm.q[0]) + arm.L[1]*np.sin(arm.q[0]+arm.q[1]) +
            arm.L[2]*np.sin(np.sum(arm.q))])

        return np.array([x, y]).astype('int')

    window.jps = get_joint_positions()

    @window.event
    def on_draw():
        window.clear()
        label.draw()
        for i in range(3):
            pyglet.graphics.draw(
                2,
                pyglet.gl.GL_LINES,
                ('v2i', (window.jps[0][i], window.jps[1][i],
                         window.jps[0][i+1], window.jps[1][i+1])))

    @window.event
    def on_mouse_motion(x, y, dx, dy):
        # kolun ters kinematik iþlevini çaðýrýr
        # haraket etmek için ideal eklem açýlarýný bul
        # farenin konumunu bul
        label.text = '(x,y) = (%.3f, %.3f)' % (x, y)
        arm.q = arm.inv_kin([x - window.width/2, y])  # get new arm angles
        window.jps = get_joint_positions()  # get new joint (x,y) positions

    pyglet.app.run()

plot()