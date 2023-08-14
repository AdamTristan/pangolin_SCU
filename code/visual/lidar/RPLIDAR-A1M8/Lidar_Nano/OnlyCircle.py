circles = [[1, 2, 3], [2, 3, 4], [3, 4, 5], [0, 1, 2]]


def NearestCircle(circless):
    mindist = 3000
    i = -1
    order = 0
    for circle in circless:
        i = i + 1
        if (circle[0] ** 2 + circle[1] ** 2) ** 0.5 < mindist:
            mindist = (circle[0] ** 2 + circle[1] ** 2) ** 0.5
            order = i
    return circless[order]


circle = NearestCircle(circles)
print(circle)
