import os
img_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'icon/model3.png')
target_area = [-10, 21, -10, 10]


def robot_in_area(p):
    assert len(p)>1, "state should be at least 2D"
    xmax, xmin, ymax, ymin = target_area
    if (p[0] < xmax and p[0] >= xmin) and (p[1] < ymax and p[1] >= ymin):
        return True
    return False