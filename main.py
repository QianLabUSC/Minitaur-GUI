import math
import time
import matplotlib.pyplot as plt
import sys
import numpy as np
import matplotlib.animation as ani
from GUI import calculated_ori as cal
import GUI
import tkinter as tk
import Enviornment as en
from mpl_toolkits.mplot3d import Axes3D

duration = 10 # Unknown
a, b, c, d, h = 4, 4, 8, 1, 5 # short leg length, motor distance, long leg length, extra leg length, leg lift height
duty_cycle, lift_cycle, T, d_t = 0.5, 0.15, 3000, 0.001 # time for leg contact on ground, time for lifting leg, period, unknown
x_list, y_list, x_omem, y_omem, z_omem = [], [], [], [], [] # x and y position for designed trajectory, x, y, z position memory for obstacle
step_length, lift_length = 8, 5
x_start, y_start = 2, 11 # x and y for relative trajectory start position
body_length, body_width = 10, 7.5 # robot body length  and body width

# calculate the design trajectory
for t in range(T):
    if t < duty_cycle * T:
        x_list.append(x_start - step_length * t / (duty_cycle * T))
        y_list.append(y_start)
    elif t < duty_cycle * T + lift_cycle * T:
        x_list.append(x_start - step_length)
        y_list.append(y_start - lift_length * (t - duty_cycle * T) / (lift_cycle * T))
    elif t < T - lift_cycle * T:
        x_list.append(x_start - step_length + step_length * (t - duty_cycle * T - lift_cycle * T)/ ((1 - duty_cycle - 2* lift_cycle) * T))
        y_list.append(y_start - lift_length)
    else:
        x_list.append(x_start)
        y_list.append(y_start - lift_length + lift_length * (t - T + lift_cycle * T) / (lift_cycle * T))
print([x_list, y_list])
tp = 0
# x, y = -b/2, (a + c)/2  #test point
# test_list = [0, 750, 1500, 1950, 2250, 2550, 2999]  # test point
phi0, phi1, phi2, phi3 = 0, 0, 0, 0
# setup the control window
window = tk.Tk()
entryVar_0A = tk.StringVar(window, '0')
entryVar_0B = tk.StringVar(window, '0')
entryVar_1A = tk.StringVar(window, '0')
entryVar_1B = tk.StringVar(window, '0')
entryVar_2A = tk.StringVar(window, '0')
entryVar_2B = tk.StringVar(window, '0')
entryVar_3A = tk.StringVar(window, '0')
entryVar_3B = tk.StringVar(window, '0')


def phase_change1(value1):
    global phi1
    global T
    phi1 = float(value1) * T
    return 0


def phase_change2(value2):
    global phi2
    global T
    phi2 = float(value2) * T
    return 0


def phase_change3(value3):
    global phi3
    global T
    phi3 = float(value3) * T
    return 0


def gait(): #show any gait base on the gait space
    # values for motor angle show in the text area
    global entryVar_0A, entryVar_0B, entryVar_1A, entryVar_1B, entryVar_2A, entryVar_2B, entryVar_3A, entryVar_3B
    # angle for two motors in a leg, leg joint point x and y position
    theta_A_list, theta_B_list, x0_list, y0_list = cal(x_list, y_list, T, a, b, c, d)
    fig = plt.figure(1)
    ax1 = plt.subplot(2, 2, 1)
    ax2 = plt.subplot(2, 2, 2)
    ax3 = plt.subplot(2, 2, 3)
    ax4 = plt.subplot(2, 2, 4)
    plt.ion()
    for t in range(0, T, 20):
        theta_A, theta_B = theta_A_list[t], theta_B_list[t]
        x, y, x0, y0 = x_list[t], y_list[t], x0_list[t], y0_list[t]
        # ax = fig.add_subplot(y_start1)
        # ax.clear()  # necessary for save memory
        # figure, axes = plt.subplots()
        plt.sca(ax1)
        plt.cla()
        # plot leg and joint points RF leg
        plt.plot([-b, -b + a * math.cos(theta_B)], [0, a * math.sin(theta_B)])
        plt.plot([0, a * math.cos(theta_A)], [0, a * math.sin(theta_A)])
        plt.plot([-b + a * math.cos(theta_B), x], [a * math.sin(theta_B), y])
        plt.plot([a * math.cos(theta_A), x0], [a * math.sin(theta_A), y0])
        plt.plot(x, y, 'ro')
        if x < x_start and y == y_start:
            plt.plot([x_start, x], [y_start, y], 'r-')
        else:
            plt.plot([x_start, x_start - step_length], [y_start, y_start], 'r-')
        plt.xlim([-12, 12])
        plt.ylim([-12, 12])
        entryVar_0A.set('RF_A:      ' + str(format(theta_A/math.pi*180, '.2f')))
        entryVar_0B.set('RF_B:      ' + str(format(theta_B/math.pi*180, '.2f')))
        plt.xlabel('RF leg')
        # plot leg and joint points LF leg
        theta_A, theta_B = theta_A_list[int((t+phi1)%T)], theta_B_list[int((t+phi1)%T)]
        x, y, x0, y0 = x_list[int((t+phi1)%T)], y_list[int((t+phi1)%T)], x0_list[int((t+phi1)%T)], y0_list[int((t+phi1)%T)]
        plt.sca(ax2)
        plt.cla()
        plt.plot([-b, -b + a * math.cos(theta_B)], [0, a * math.sin(theta_B)])
        plt.plot([0, a * math.cos(theta_A)], [0, a * math.sin(theta_A)])
        plt.plot([-b + a * math.cos(theta_B), x], [a * math.sin(theta_B), y])
        plt.plot([a * math.cos(theta_A), x0], [a * math.sin(theta_A), y0])
        plt.plot(x, y, 'ro')
        if x < x_start and y == y_start:
            plt.plot([x_start, x], [y_start, y], 'r-')
        else:
            plt.plot([x_start, x_start - step_length], [y_start, y_start], 'r-')
        plt.xlim([-12, 12])
        plt.ylim([-12, 12])
        entryVar_1A.set('LF_A:      ' + str(format(theta_A/math.pi*180, '.2f')))
        entryVar_1B.set('LF_B:      ' + str(format(theta_B/math.pi*180, '.2f')))
        plt.xlabel('LF leg')
        # plot leg and joint points LB leg
        theta_A, theta_B = theta_A_list[int((t+phi2)%T)], theta_B_list[int((t+phi2)%T)]
        x, y, x0, y0 = x_list[int((t+phi2)%T)], y_list[int((t+phi2)%T)], x0_list[int((t+phi2)%T)], y0_list[int((t+phi2)%T)]
        plt.sca(ax3)
        plt.cla()
        plt.plot([-b, -b + a * math.cos(theta_B)], [0, a * math.sin(theta_B)])
        plt.plot([0, a * math.cos(theta_A)], [0, a * math.sin(theta_A)])
        plt.plot([-b + a * math.cos(theta_B), x], [a * math.sin(theta_B), y])
        plt.plot([a * math.cos(theta_A), x0], [a * math.sin(theta_A), y0])
        plt.plot(x, y, 'ro')
        if x < x_start and y == y_start:
            plt.plot([x_start, x], [y_start, y], 'r-')
        else:
            plt.plot([x_start, x_start - step_length], [y_start, y_start], 'r-')
        plt.xlim([-12, 12])
        plt.ylim([-12, 12])
        entryVar_2A.set('LB_A:      ' + str(format(theta_A/math.pi*180, '.2f')))
        entryVar_2B.set('LB_B:      ' + str(format(theta_B/math.pi*180, '.2f')))
        plt.xlabel('LB leg')
        # plot leg and joint points RB leg
        theta_A, theta_B = theta_A_list[int((t+phi3)%T)], theta_B_list[int((t+phi3)%T)]
        x, y, x0, y0 = x_list[int((t+phi3)%T)], y_list[int((t+phi3)%T)], x0_list[int((t+phi3)%T)], y0_list[int((t+phi3)%T)]
        plt.sca(ax4)
        plt.cla()
        plt.plot([-b, -b + a * math.cos(theta_B)], [0, a * math.sin(theta_B)])
        plt.plot([0, a * math.cos(theta_A)], [0, a * math.sin(theta_A)])
        plt.plot([-b + a * math.cos(theta_B), x], [a * math.sin(theta_B), y])
        plt.plot([a * math.cos(theta_A), x0], [a * math.sin(theta_A), y0])
        plt.plot(x, y, 'ro')
        if x < x_start and y == y_start:
            plt.plot([x_start, x], [y_start, y], 'r-')
        else:
            plt.plot([x_start, x_start - step_length], [y_start, y_start], 'r-')
        plt.xlim([-12, 12])
        plt.ylim([-12, 12])
        entryVar_3A.set('RB_A:      ' + str(format(theta_A/math.pi*180, '.2f')))
        entryVar_3B.set('RB_B:      ' + str(format(theta_B/math.pi*180, '.2f')))
        plt.xlabel('RB leg')
        # circle1 = plt.Circle((-b + a * math.cos(theta_B), a * math.sin(theta_B)), c, facecolor='none', edgecolor='r')
        # circle2 = plt.Circle((a * math.cos(theta_A), a * math.sin(theta_A)), c, facecolor='none', edgecolor='r')
        # circle3 = plt.Circle((-b + a * math.cos(theta_B), a * math.sin(theta_B)), c+d, facecolor='none', edgecolor='g')
        # draw_circle1 = plt.Circle((x0, y0), 0.1)
        # draw_circle2 = plt.Circle((x, y), 0.1)
        # ax.add_patch(draw_circle1)
        # ax.add_patch(draw_circle2)
        # ax.add_patch(circle1)
        # ax.add_patch(circle2)
        # ax.add_patch(circle3)
        # axes.set_aspect('equal')
        plt.pause(0.05)
    plt.ioff()
    # plt.show()
    return 0


def body_show(T=3000, a=4, b=4, c=8, d=1, xx=0, yy=0, z=0, theta_x=0, theta_y=0, theta_z=0, body_length=15, body_width=7.5):
    global x_omem, y_omem, z_omem
    distance, edge = 4, 2
    theta_A_list, theta_B_list, x0_list, y0_list = cal(x_list, y_list, T, a, b, c, d)
    for tt in range(20):
        print(theta_A_list[int(tt/20*3000)]/np.pi*180, theta_B_list[int(tt/20*3000)]/np.pi*180)

    fig2 = plt.figure(2, figsize=(16,9))
    ax1 = Axes3D(fig2)
    ax1.view_init(20, 0)
    for t in range(1, 2 * T, 30):
        yy += 2 * step_length/3000.0 * 30
        ax1.cla()
        ########################################
        for loop in range(10):
            x_omem, y_omem, z_omem = en.obstacle(ax1, loop, x_omem, y_omem, z_omem)
        entryVar_0A.set('RF_A:      ' + str(format(theta_A_list[int(t)%T]/math.pi*180, '.2f')))
        entryVar_0B.set('RF_B:      ' + str(format(theta_B_list[int(t)%T]/math.pi*180, '.2f')))
        entryVar_1A.set('LF_A:      ' + str(format(theta_A_list[int((t+phi1)%T)]/math.pi*180, '.2f')))
        entryVar_1B.set('LF_B:      ' + str(format(theta_B_list[int((t+phi1)%T)]/math.pi*180, '.2f')))
        entryVar_2A.set('LB_A:      ' + str(format(theta_A_list[int((t+phi2)%T)]/math.pi*180, '.2f')))
        entryVar_2B.set('LB_B:      ' + str(format(theta_B_list[int((t+phi2)%T)]/math.pi*180, '.2f')))
        entryVar_3A.set('RB_A:      ' + str(format(theta_A_list[int((t+phi3)%T)]/math.pi*180, '.2f')))
        entryVar_3B.set('RB_B:      ' + str(format(theta_B_list[int((t+phi3)%T)]/math.pi*180, '.2f')))
        ########################################
        ax1.plot3D([xx + body_width, xx + body_width], [yy - body_length, yy + body_length], [y_start, y_start], 'k-')
        ax1.plot3D([xx - body_width, xx + body_width], [yy + body_length, yy + body_length], [y_start, y_start], 'k-')
        ax1.plot3D([xx - body_width, xx - body_width], [yy + body_length, yy - body_length], [y_start, y_start], 'k-')
        ax1.plot3D([xx - body_width, xx + body_width], [yy - body_length, yy - body_length], [y_start, y_start], 'k-')
        ax1.plot3D(xx + body_width, yy + body_length - edge, y_start, 'ro'), ax1.plot3D(xx + body_width, yy + body_length - edge - distance, y_start, 'ro')
        ax1.plot3D(xx + body_width, yy - body_length + edge, y_start, 'ro'), ax1.plot3D(xx + body_width, yy - body_length + edge + distance, y_start, 'ro')
        ax1.plot3D(xx - body_width, yy + body_length - edge, y_start, 'ro'), ax1.plot3D(xx - body_width, yy + body_length - edge - distance, y_start, 'ro')
        ax1.plot3D(xx - body_width, yy - body_length + edge, y_start, 'ro'), ax1.plot3D(xx - body_width, yy - body_length + edge + distance, y_start, 'ro')
        ################################################################
        theta_A, theta_B = theta_A_list[int(t)%T], theta_B_list[int(t)%T]
        x, y, x0, y0 = x_list[int(t)%T], y_list[int(t)%T], x0_list[int(t)%T], y0_list[int(t)%T]
        ax1.plot3D([xx + body_width, xx + body_width], [yy + body_length - edge - b, yy + body_length - edge - b + a * math.cos(theta_B)], [y_start - 0, y_start - a * math.sin(theta_B)])    # b_1 leg
        ax1.plot3D([xx + body_width, xx + body_width], [yy + body_length - edge + 0, yy + body_length - edge + a * math.cos(theta_A)], [y_start - 0, y_start - a * math.sin(theta_A)])          # a_1 leg
        ax1.plot3D([xx + body_width, xx + body_width], [yy + body_length - edge - b + a * math.cos(theta_B), yy + body_length - edge + x], [y_start - a * math.sin(theta_B), y_start - y])     # b_2 leg
        ax1.plot3D([xx + body_width, xx + body_width], [yy + body_length - edge + a * math.cos(theta_A), yy + body_length - edge + x0], [y_start - a * math.sin(theta_A), y_start - y0])        # a_2 leg
        ax1.plot3D(xx + body_width, yy + body_length - edge + x, y_start - y, 'ro')
        if x < x_start and y == y_start:
            ax1.plot3D([xx + body_width, xx + body_width], [yy + body_length - edge + x_start, yy + body_length - edge + x], [y_start - y_start, y_start - y], 'r-')
        else:
            ax1.plot3D([xx + body_width, xx + body_width], [yy + body_length - edge + x_start, yy + body_length - edge + x_start - step_length], [y_start - y_start, y_start - y_start], 'r-')
        ################################################################
        theta_A, theta_B = theta_A_list[int((t+phi1)%T)], theta_B_list[int((t+phi1)%T)]
        x, y, x0, y0 = x_list[int((t+phi1)%T)], y_list[int((t+phi1)%T)], x0_list[int((t+phi1)%T)], y0_list[int((t+phi1)%T)]
        ax1.plot3D([xx - body_width, xx - body_width], [yy + body_length - edge - b, yy + body_length - edge - b + a * math.cos(theta_B)], [y_start - 0, y_start - a * math.sin(theta_B)])    # b_1 leg
        ax1.plot3D([xx - body_width, xx - body_width], [yy + body_length - edge + 0, yy + body_length - edge + a * math.cos(theta_A)], [y_start - 0, y_start - a * math.sin(theta_A)])          # a_1 leg
        ax1.plot3D([xx - body_width, xx - body_width], [yy + body_length - edge - b + a * math.cos(theta_B), yy + body_length - edge + x], [y_start - a * math.sin(theta_B), y_start - y])     # b_2 leg
        ax1.plot3D([xx - body_width, xx - body_width], [yy + body_length - edge + a * math.cos(theta_A), yy + body_length - edge + x0], [y_start - a * math.sin(theta_A), y_start - y0])        # a_2 leg
        ax1.plot3D(xx - body_width, yy + body_length - edge + x, y_start - y, 'ro')
        if x < x_start and y == y_start:
            ax1.plot3D([xx - body_width, xx - body_width], [yy + body_length - edge + x_start, yy + body_length - edge + x], [y_start - y_start, y_start - y], 'r-')
        else:
            ax1.plot3D([xx - body_width, xx - body_width], [yy + body_length - edge + x_start, yy + body_length - edge + x_start - step_length], [y_start - y_start, y_start - y_start], 'r-')
        #################################################################
        theta_A, theta_B = theta_A_list[int((t + phi2) % T)], theta_B_list[int((t + phi2) % T)]
        x, y, x0, y0 = x_list[int((t + phi2) % T)], y_list[int((t + phi2) % T)], x0_list[int((t + phi2) % T)], y0_list[
            int((t + phi2) % T)]
        ax1.plot3D([xx - body_width, xx - body_width], [yy + -body_length + edge, yy + -body_length + edge + a * math.cos(theta_B)], [y_start - 0, y_start - a * math.sin(theta_B)])  # b_1 leg
        ax1.plot3D([xx - body_width, xx - body_width], [yy + -body_length + edge + b, yy + -body_length + edge + b + a * math.cos(theta_A)], [y_start - 0, y_start - a * math.sin(theta_A)])  # a_1 leg
        ax1.plot3D([xx - body_width, xx - body_width], [yy + -body_length + edge + a * math.cos(theta_B), yy + -body_length + edge + b + x], [y_start - a * math.sin(theta_B), y_start - y])  # b_2 leg
        ax1.plot3D([xx - body_width, xx - body_width], [yy + -body_length + edge + b + a * math.cos(theta_A), yy + -body_length + edge + b + x0], [y_start - a * math.sin(theta_A), y_start - y0])  # a_2 leg
        ax1.plot3D(xx - body_width, yy + -body_length + edge + b + x, y_start - y, 'ro')
        if x < x_start and y == y_start:
            ax1.plot3D([xx - body_width, xx - body_width], [yy + -body_length + edge + b + x_start, yy + -body_length + edge + b + x], [y_start - y_start, y_start - y], 'r-')
        else:
            ax1.plot3D([xx - body_width, xx - body_width], [yy + -body_length + edge + b + x_start, yy + -body_length + edge + b + x_start - step_length], [y_start - y_start, y_start - y_start], 'r-')
        ################################################################
        theta_A, theta_B = theta_A_list[int((t+phi3)%T)], theta_B_list[int((t+phi3)%T)]
        x, y, x0, y0 = x_list[int((t+phi3)%T)], y_list[int((t+phi3)%T)], x0_list[int((t+phi3)%T)], y0_list[int((t+phi3)%T)]
        ax1.plot3D([xx + body_width, xx + body_width], [yy + -body_length + edge, yy + -body_length + edge + a * math.cos(theta_B)], [y_start - 0, y_start - a * math.sin(theta_B)])  # b_1 leg
        ax1.plot3D([xx + body_width, xx + body_width], [yy + -body_length + edge + b, yy + -body_length + edge + b + a * math.cos(theta_A)], [y_start - 0, y_start - a * math.sin(theta_A)])  # a_1 leg
        ax1.plot3D([xx + body_width, xx + body_width], [yy + -body_length + edge + a * math.cos(theta_B), yy + -body_length + edge + b + x], [y_start - a * math.sin(theta_B), y_start - y])  # b_2 leg
        ax1.plot3D([xx + body_width, xx + body_width], [yy + -body_length + edge + b + a * math.cos(theta_A), yy + -body_length + edge + b + x0], [y_start - a * math.sin(theta_A), y_start - y0])  # a_2 leg
        ax1.plot3D(xx + body_width, yy + -body_length + edge + b + x, y_start - y, 'ro')
        if x < x_start and y == y_start:
            ax1.plot3D([xx + body_width, xx + body_width], [yy + -body_length + edge + b + x_start, yy + -body_length + edge + b + x], [y_start - y_start, y_start - y], 'r-')
        else:
            ax1.plot3D([xx + body_width, xx + body_width], [yy + -body_length + edge + b + x_start, yy + -body_length + edge + b + x_start - step_length], [y_start - y_start, y_start - y_start], 'r-')

        ax1.set_xlim(-30, 30)
        ax1.set_ylim(-30, 70)
        ax1.set_zlim(0, 40)
        plt.pause(0.001)
    plt.show()
    return 0


window.title('Parameter setup')
window.geometry('600x600')
l = tk.Label(window, bg='green', fg='white', width=40, text='Setup')
l.pack()
s1 = tk.Scale(window, label='Phi1', from_=0, to=1, orient=tk.HORIZONTAL, length=400, showvalue=0, tickinterval=0.1, resolution=0.1, command=phase_change1)
s2 = tk.Scale(window, label='Phi2', from_=0, to=1, orient=tk.HORIZONTAL, length=400, showvalue=0, tickinterval=0.1, resolution=0.1, command=phase_change2)
s3 = tk.Scale(window, label='Phi3', from_=0, to=1, orient=tk.HORIZONTAL, length=400, showvalue=0, tickinterval=0.1, resolution=0.1, command=phase_change3)
butt1 = tk.Button(window, text='Show gait', font=('Arial', 12), width=10, height=1, command=gait)
butt2 = tk.Button(window, text='Show robot', font=('Arial', 12), width=10, height=1, command=body_show)
l_0A = tk.Entry(window, textvariable=entryVar_0A)
l_0B = tk.Entry(window, textvariable=entryVar_0B)
l_1A = tk.Entry(window, textvariable=entryVar_1A)
l_1B = tk.Entry(window, textvariable=entryVar_1B)
l_2A = tk.Entry(window, textvariable=entryVar_2A)
l_2B = tk.Entry(window, textvariable=entryVar_2B)
l_3A = tk.Entry(window, textvariable=entryVar_3A)
l_3B = tk.Entry(window, textvariable=entryVar_3B)
s1.pack(), s2.pack(), s3.pack(), butt1.place(x=150, y=210, width=120, height=25), butt2.place(x=300, y=210, width=120, height=25)
l_0A.place(x=100, y=250, width=120, height=25), l_0B.place(x=100, y=280, width=120, height=25)
l_1A.place(x=100, y=310, width=120, height=25), l_1B.place(x=100, y=340, width=120, height=25)
l_2A.place(x=100, y=370, width=120, height=25), l_2B.place(x=100, y=400, width=120, height=25)
l_3A.place(x=100, y=430, width=120, height=25), l_3B.place(x=100, y=460, width=120, height=25)
l_0A.config(fg='red'), l_0B.config(fg='red'), l_1A.config(fg='red'), l_1B.config(fg='red')
l_2A.config(fg='red'), l_2B.config(fg='red'), l_3A.config(fg='red'), l_3B.config(fg='red')


window.mainloop()


def rad_deg(a):
    bb, ttp = [], 0
    for i in a:
        ttp = i/math.pi * 180
        bb.append(ttp)
    return bb
# print(rad_deg(theta_A_list))
# print(rad_deg(theta_B_list))



