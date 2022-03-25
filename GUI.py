import math
import matplotlib.pyplot as plt
# import main
def calculated_ori(x_list, y_list, T, a, b, c, d):
    theta_A_list, theta_B_list, x0_list, y0_list = [], [], [], []
    for t in range(0, T):
        x, y = x_list[t], y_list[t]
        d2 = math.sqrt((x + b) ** 2 + y ** 2)
        theta_b = math.acos((a ** 2 + d2 ** 2 - (c + d) ** 2) / (2 * a * d2))
        if x + b == 0:
            theta_b_tp = 0.5 * math.pi
        elif math.atan(y / (x + b)) < 0:
            theta_b_tp = math.pi + math.atan(y / (x + b))
        else:
            theta_b_tp = math.atan(y / (x + b))
        # print(theta_b_tp/3.1415926*180)
        # print(theta_b/3.1415926*180)
        # print(math.cos(theta_b/3.1415926*180))
        theta_B = theta_b_tp + theta_b
        theta_B_list.append(theta_B)
        x_B, y_B = -b + a * math.cos(theta_B), a * math.sin(theta_B)
        x0, y0 = x_B + c / (c + d) * (x - x_B), y_B + c / (c + d) * (y - y_B)
        x0_list.append(x0)
        y0_list.append(y0)

        d1 = math.sqrt(x0 ** 2 + y0 ** 2)
        theta_a = math.acos((d1 ** 2 + a ** 2 - c ** 2) / (2 * a * d1))
        if x0 == 0:
            theta_a_tp = 0.5 * math.pi
        elif math.atan(y0 / (-x0)) < 0:
            theta_a_tp = math.pi + math.atan(y0 / (-x0))
        else:
            theta_a_tp = math.atan(y0 / (-x0))
        # print(theta_a_tp/3.1415926*180)
        # print(theta_a/3.1415926*180)
        theta_A = math.pi - (theta_a_tp + theta_a)
        theta_A_list.append(theta_A)
    return theta_A_list, theta_B_list, x0_list, y0_list

