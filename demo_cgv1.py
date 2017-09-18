def scale_x(points):
    escala_x = [
        [2, 0],
        [0, 1]]
    for i in range(0, 6):
        for j in range(0, 2):
            sum = 0
            for k in range(0, 2):
                sum += points[i][k] * escala_x[k][j]
            points[i][j] = sum

def scale_y(points):
    escala_y = [
        [1, 0],
        [0, 2]]
    for i in range(0, 6):
        for j in range(0, 2):
            sum = 0
            for k in range(0, 2):
                sum += points[i][k] * escala_y[k][j]
            points[i][j] = sum

def cis_x(points):
    cis_x = [
        [1, 2],
        [0, 1]]
    for i in range(0, 6):
        for j in range(0, 2):
            sum = 0
            for k in range(0, 2):
                sum += points[i][k] * cis_x[k][j]
            points[i][j] = sum

def cis_y(points):
    cis_y = [
        [1, 0],
        [2, 1]]
    for i in range(0, 6):
        for j in range(0, 2):
            sum = 0
            for k in range(0, 2):
                sum += points[i][k] * cis_y[k][j]
            points[i][j] = sum

for x,y,z in verts:
    z += 5
    f = 250/z
    x,y = x*f,y*f
    pt = Circle(Point(cx+int(x), cy+int(y)), 6)
    pt.setFill('black')
    pt.draw(janela)


for edge in edges:
    points = []
    for x,y,z in (verts[edge[0]], verts[edge[1]]):
        z += 5
        f = 250/z
        x,y = x*f,y*f
        points += [(cx+int(x),cy+int(y))]
    pt1 = Point(points[0][0], points[0][1])
    pt2 = Point(points[1][0], points[1][1])
    ln = Line(pt1, pt2)
    ln.draw(janela)
