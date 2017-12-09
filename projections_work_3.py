# -*- coding: utf-8 -*-
from graphics import *
import numpy as np
import numpy.linalg as la
import sys, math, time

class Vertice:
    def __init__(self, coordenadas):
        self.x = coordenadas[0]
        self.y = coordenadas[1]
        self.z = coordenadas[2]

class Aresta:
    def __init__(self, ini, fim):
        self.ini = ini
        self.fim = fim

class Solido:
    def __init__(self):
        self.vertices = []
        self.vertices = np.zeros((0,4))
        self.arestas = []

    def addVertices(self, vertLista):
        coluna_uns = np.ones((len(vertLista), 1))
        uns_added = np.hstack((vertLista, coluna_uns))
        self.vertices = np.vstack((self.vertices, uns_added))

    def addArestas(self, arestLista):
        self.arestas = arestLista

    def outVertices(self):
        print "\n --- Vertices --- "
        for i, (x, y, z, d) in enumerate(self.vertices):
            print " %d: (%.2f, %.2f, %.2f, %.2f)" % (i, x, y, z, d)

    def outArestas(self):
        print "\n --- Arestas --- "
        for i, (vertice1, vertice2) in enumerate(self.arestas):
            print "%d: %d -> %d" % (i, vertice1, vertice2)

    def rotZ(self, ang):
        radianos = ang*(np.pi/180)
        c = np.cos(radianos)
        s = np.sin(radianos)
        rZ = np.array([[c, -s, 0, 0],
                       [s, c, 0, 0],
                       [0, 0, 1, 0],
                       [0, 0, 0, 1]])

        centro = self.getCentro()
        self.translacao(-centro[0], -centro[1], -centro[2])
        self.vertices = self.vertices.transpose()
        self.vertices = np.dot(rZ, self.vertices)
        self.vertices = self.vertices.transpose()
        self.translacao(centro[0], centro[1], centro[2])

    def rotY(self, ang):
        radianos = ang*(np.pi/180)
        c = np.cos(radianos)
        s = np.sin(radianos)
        rY = np.array([[c, 0, s, 0],
                       [0, 1, 0, 0],
                       [-s, 0, c, 0],
                       [0, 0, 0, 1]])

        centro = self.getCentro()
        self.translacao(-centro[0], -centro[1], -centro[2])
        self.vertices = self.vertices.transpose()
        self.vertices = np.dot(rY, self.vertices)
        self.vertices = self.vertices.transpose()
        self.translacao(centro[0], centro[1], centro[2])

    def rotX(self, ang):
        radianos = ang*(np.pi/180)
        c = np.cos(radianos)
        s = np.sin(radianos)
        rX = np.array([[1, 0, 0, 0],
                       [0, c, -s, 0],
                       [0, s, c, 0],
                       [0, 0, 0, 1]])

        centro = self.getCentro()
        self.translacao(-centro[0], -centro[1], -centro[2])
        self.vertices = self.vertices.transpose()
        self.vertices = np.dot(rX, self.vertices)
        self.vertices = self.vertices.transpose()
        self.translacao(centro[0], centro[1], centro[2])

    def translacao(self, dx=0, dy=0, dz=0):
        t = np.array([[1, 0, 0, dx],
                      [0, 1, 0, dy],
                      [0, 0, 1, dz],
                      [0, 0, 0, 1]])
        self.vertices = self.vertices.transpose()
        self.vertices = np.dot(t, self.vertices)
        self.vertices = self.vertices.transpose()

    def escala(self, sx=0, sy=0, sz=0):

        e = np.array([[sx, 0, 0, 0],
                      [0, sy, 0, 0],
                      [0, 0, sz, 0],
                      [0, 0, 0,  1]])
        centro = self.getCentro()
        self.translacao(-centro[0], -centro[1], -centro[2])
        self.vertices = np.dot(self.vertices, e)
        self.translacao(centro[0], centro[1], centro[2])

    def projIsometrica(self, angX, angY):
        radianosX = angX*(np.pi/180)
        radianosY = angY*(np.pi/180)
        cx = np.cos(-radianosX)
        sx = np.sin(-radianosX)
        cy = np.cos(radianosY)
        sy = np.sin(radianosY)
        rI = np.array([[cy, sy*sx, -sy*cx, 0],
                       [0, cx, sx, 0],
                       [sy, -sx*cy, cx*cy, 0],
                       [0, 0, 0, 1]])

        centro = self.getCentro()
        self.translacao(-centro[0], -centro[1], -centro[2])
        self.vertices = self.vertices.transpose()
        self.vertices = np.dot(rI, self.vertices)
        self.vertices = self.vertices.transpose()
        self.translacao(centro[0], centro[1], centro[2])

    def projObliquaCabinet(self, ang, l):
        radianos = ang*(np.pi/180)
        c = np.cos(radianos)
        s = np.sin(radianos)

        obl = np.array([[1, 0, (l*c)/2, 0],
                       [0, 1, (l*s)/2, 0],
                       [0, 0, 0, 0], 
                       [0, 0, 0, 1]])
        
        centro = self.getCentro()
        self.translacao(-centro[0], -centro[1], -centro[2])
        self.vertices = self.vertices.transpose()
        self.vertices = np.dot(obl, self.vertices)
        self.vertices = self.vertices.transpose()
        self.translacao(centro[0], centro[1], centro[2])

    def projPerspecUmPontoFuga(self, zcp):
        upf = np.array([[1, 0, 0, 0],
                       [0, 1, 0, 0],
                       [0, 0, 0, -1/zcp], 
                       [0, 0, 0, 1]])
		
        self.vertices = np.dot(self.vertices, upf)
        
        for vertice in self.vertices:
            norm = vertice[3]
            vertice[0] = abs(vertice[0]/norm)
            vertice[1] = abs(vertice[1]/norm)
            vertice[2] = abs(vertice[2]/norm)
            vertice[3] = abs(vertice[3]/norm)
        
        	
    def getCentro(self):
        n_vertices = len(self.vertices)
        vert = []
        vert = [sum(self.vertices[:,i])/n_vertices for i in range(4)]
        return(vert[0], vert[1], vert[2])

    def desenha(self, grafico):
        for n1,n2 in self.arestas:
            ptI = Point(int(self.vertices[n1][0]), int(self.vertices[n1][1]))
            ptF = Point(int(self.vertices[n2][0]), int(self.vertices[n2][1]))
            ln = Line(ptI, ptF)
            ln.draw(grafico)
        
    def desenhaComRealismo(self, visoes, faces, grafico, normal, luz, tipo):
        j = 0
        coef = 0.8
        for face in faces:
            if(visoes[j] == True):
                i = 0
                pts = []
                for i in range(len(face)):
                    if(i+1 < len(face)):   
                        ptI = Point(int(self.vertices[face[i]][0]), int(self.vertices[face[i]][1]))
                        ptF = Point(int(self.vertices[face[i+1]][0]), int(self.vertices[face[i+1]][1]))
                    else:
                        ptI = Point(int(self.vertices[face[i]][0]), int(self.vertices[face[i]][1]))
                        ptF = Point(int(self.vertices[face[0]][0]), int(self.vertices[face[0]][1]))
                    pts.append(ptI)
                    line = Line(ptI, ptF)
                    line.draw(grafico)
                    update(60)
                
                v1 = np.array(normal[j])
                v2 = np.array(luz[j])
                cosLuz = angle_between(v1, v2)
                
                if(tipo == 1):
                    hsv = [0,1,255]
                    R = hsv[0]
                    G = hsv[1]
                    B = hsv[2]*coef*cosLuz
                    cores = hsvToRgb(R,G,B)
                    R = cores[0]
                    G = cores[1]
                    B = cores[2]

                elif(tipo == 2):
                    hsv = [60,1,255]
                    R = hsv[0]
                    G = hsv[1]
                    B = hsv[2]*coef*cosLuz
                    cores = hsvToRgb(R,G,B)
                    R = cores[0]
                    G = cores[1]
                    B = cores[2]

                elif(tipo == 3):
                    hsv = [120,1,255]
                    R = hsv[0]
                    G = hsv[1]
                    B = hsv[2]*coef*cosLuz
                    cores = hsvToRgb(R,G,B)
                    R = cores[0]
                    G = cores[1]
                    B = cores[2]
                
                elif(tipo == 4):
                    hsv = [180,1,255]
                    R = hsv[0]
                    G = hsv[1]
                    B = hsv[2]*coef*cosLuz
                    cores = hsvToRgb(R,G,B)
                    R = cores[0]
                    G = cores[1]
                    B = cores[2]
                
                elif(tipo == 5):
                    hsv = [240,1,255]
                    R = hsv[0]
                    G = hsv[1]
                    B = hsv[2]*coef*cosLuz
                    cores = hsvToRgb(R,G,B)
                    R = cores[0]
                    G = cores[1]
                    B = cores[2]
                
                elif(tipo == 6):
                    hsv = [300,1,255]
                    R = hsv[0]
                    G = hsv[1]
                    B = hsv[2]*coef*cosLuz
                    cores = hsvToRgb(R,G,B)
                    R = cores[0]
                    G = cores[1]
                    B = cores[2]
                    
                p = Polygon(pts)
                p.setFill(color_rgb(R, G ,B))
                p.setOutline(color_rgb(R, G, B))
                p.draw(grafico)
                update(60)
            j+=1

    def pinta(self, faces, window):
        cores = ["green", "blue", "grey", "orange", "purple"]
        j = 0
        for face in faces:
            pts = []
            for i in range(len(face)):
               pt = Point(self.vertices[face[i]][0], self.vertices[face[i]][1]) 
               pts.append(pt)
            p = Polygon(pts)
            p.setFill(cores[j])
            p.setOutline("black")
            p.draw(window)
            time.sleep(.3)
            update(60)
            if (j > 3):
                j = 0
            j += 1
    
    def pintaPontos(self, window):
        i = 40
        j = 0
        for vertice in self.vertices:
            s = ' ,'.join(str(e) for e in (vertice[0], vertice[1], vertice[2]))
            txt3 = Text(Point(120, 20), "Pontos atuais do polígono: ")
            txt2 = Text(Point(13,i), str(j) + ": ")
            txt = Text(Point(200,i),s)
            txt.setFace("arial")
            txt.setStyle("bold")
            txt.draw(window)
            txt2.setFace("arial")
            txt2.setStyle("bold")
            txt2.draw(window)
            txt3.setFace("arial")
            txt3.setStyle("bold")
            txt3.draw(window)
            i += 40
            j += 1
            
def unit_vector(vector):
    return vector / np.linalg.norm(vector)

def angle_between(v1, v2):
    v1_u = unit_vector(v1)
    v2_u = unit_vector(v2)
    return np.arccos(np.clip(np.dot(v1_u, v2_u), -1.0, 1.0))

def clear(win):
    for item in win.items[:]:
        item.undraw()
    win.update()

def rgbToHsv(r,g,b):
    maxValue = max(r,g,b)
    minValue = min(r,g,b)
    value = maxValue 
    delta = maxValue - minValue
    cores = []
    if(maxValue != 0):
        saturation = delta / maxValue
    else:
        cores.append(-1)
        cores.append(0)
        cores.append(value)
        return cores

    if(r == max(r,g,b)):
        hue = (g - b) / delta
    elif(g == max(r,g,b)):
        hue = 2 + (b - r) / delta 
    else:
        hue = 4 + (r - g) / delta
    hue *= 60
    if(hue < 0):
        hue += 360
    cores.append(hue)
    cores.append(saturation)
    cores.append(value)
    return cores

def hsvToRgb(h,s,v):
    cores = []
    if (s == 0):
        r = v
        g = v
        b = v
        cores.append(r)
        cores.append(g)
        cores.append(b)
        return cores
    h /= 60
    i = math.floor(h)
    f = h - i
    p = v * (1-s)
    q = v * (1-s * f)
    t = v * (1-s * (1-f))

    if(i == 0):
        r = v
        g = t
        b = p
    elif(i == 1):
        r = q
        g = v
        b = p
    elif(i == 2):
        r = p
        g = v
        b = t
    elif(i == 3):
        r = p
        g = q
        b = v
    elif(i == 4):
        r = t
        g = p
        b = v
    else:
        r = v
        g = p
        b = q
    cores.append(r)
    cores.append(g)
    cores.append(b)
    return cores

def calcularNormal(vertice1, vertice0, vertice2):
        normal = []
        sub1 =  np.subtract(vertice1, vertice0)
        sub2 =  np.subtract(vertice2, vertice0)
        normal = np.cross(sub1[:3], sub2[:3])
        return normal

def checarVisao(vertice, normal, anguloVisao):
    sub = np.subtract(vertice[:3], anguloVisao)
    resultado = np.inner(sub, normal)
    if(resultado >= 0):
        return True
    return False

def retornaVetorLuz(vertice, luz):
    sub = np.subtract(vertice[:3], luz)
    return sub

def binomial(i, n):
    """Binomial coefficient"""
    return math.factorial(n) / float(
        math.factorial(i) * math.factorial(n - i))

def bezier(t, points):
    """Calculate coordinate of a point in the bezier curve"""
    n = len(points) - 1
    x = y = 0
    for i, pos in enumerate(points):
        bern = bernstein(t, i, n)
        x += pos[0] * bern
        y += pos[1] * bern
    return x, y

def bezier_curve_range(n, points):
    """Range of points in a curve bezier"""
    for i in xrange(n):
        t = i / float(n - 1)
        yield bezier(t, points)

def bernstein(t, i, n):
    """Bernstein polynom"""
    return binomial(i, n) * (t ** i) * ((1 - t) ** (n - i))


def main():
    w,h = 800,600
    janela = GraphWin("janela", w, h, autoflush=False)
    janela.setBackground(color_rgb(255,255,255))
    cx,cy = w//2, h//2
   
    desenhado = True
    janelaAberta = True

    hexagono = Solido()
    hexagonVertices = [[25,400,0],
                    [30,410,0],
                    [40,410,0],
                    [45,400,0],
                    [40,390,0],
                    [30,390,0],
                    [25,400,40],
                    [30,410,40],
                    [40,410,40],
                    [45,400,40],
                    [40,390,40],
                    [30,390,40]]

    hexagonArestas = [[0,1],[1,2],[2,3],[3,4],[4,5],[5,0],[0,6],[1,7],[2,8],[3,9],[4,10],[5,11],[6,7],[7,8],[8,9],[9,10],[10,11],[11,6]]
    hexagonoFaces1 = [[0,1,2,3,4,5], [4,5,11,10], [5,0,6,11], [0,6,7,1], [1,7,8,2], [2,8,9,3], [3,9,10,4], [6,7,8,9,10,11]] 
    
    hexagono.addVertices(np.array(hexagonVertices))
    hexagono.addArestas(hexagonArestas)
    hexagono.escala(8,8,8)
    hexagono.translacao(550,-150,1)

    hexagono.projPerspecUmPontoFuga(130)
    hexagono.escala(200,200,70)
    hexagono.translacao(500,300)

    steps = 8
    oldPoint = []
    verts = []
    oldPoint = hexagono.vertices[0]
    pointsFace1 = []
    faces = []
    faceOficial = []
    
    i = 1
    verts = (hexagono.vertices[0], hexagono.vertices[1], hexagono.vertices[2], hexagono.vertices[3])
    for point in bezier_curve_range(steps, verts):
        if(i < steps/2):    
            pointsFace1.append(point)
        faceOficial.append(point)
        p = Point(oldPoint[0], oldPoint[1])
        newP = Point(point[0], point[1])
        line = Line(p, newP)
        line.draw(janela)
        faces.append(point)
        oldPoint = point
        i += 1

    i = 1
    oldPoint = hexagono.vertices[0]
    verts = (hexagono.vertices[0], hexagono.vertices[5], hexagono.vertices[4], hexagono.vertices[3])
    for point in bezier_curve_range(steps, verts):
        if(i < steps/2 + 2):
            pointsFace1.append(point)
        faceOficial.append(point)
        p = Point(oldPoint[0], oldPoint[1])
        newP = Point(point[0], point[1])
        line = Line(p, newP)
        line.draw(janela)
        oldPoint = point
        i += 1

    oldPoint = hexagono.vertices[6]
    pointsFace2 = []
    i = 1
    verts = (hexagono.vertices[6], hexagono.vertices[7], hexagono.vertices[8], hexagono.vertices[9])
    for point in bezier_curve_range(steps, verts):
        if(i < steps/2):
            pointsFace2.append(point)
            p = Point(oldPoint[0], oldPoint[1])
            newP = Point(point[0], point[1])
            line = Line(p, newP)
            line.draw(janela)
            oldPoint = point
        i += 1
    
    i = 1
    verts = (hexagono.vertices[6], hexagono.vertices[11], hexagono.vertices[10], hexagono.vertices[9])
    oldPoint = hexagono.vertices[6]
    for point in bezier_curve_range(steps, verts):
        if(i < steps/2 + 2):
            pointsFace2.append(point)
            p = Point(oldPoint[0], oldPoint[1])
            newP = Point(point[0], point[1])
            line = Line(p, newP)
            line.draw(janela)
            oldPoint = point
        i += 1

    pointsFront = []
    pointsBack = []
    for n in range(len(pointsFace1)):    
        p1 = Point(pointsFace1[n][0], pointsFace1[n][1])
        p2 = Point(pointsFace2[n][0], pointsFace2[n][1])
        l = Line(p1,p2)
        l.draw(janela) 
        pointsFront.append([pointsFace1[n][0], pointsFace1[n][1], 0.0])
        pointsBack.append([pointsFace2[n][0], pointsFace2[n][1], 0.0])

    normais = []
    normais.append(calcularNormal(pointsFront[0], pointsFront[1], pointsFront[2]))
    normais.append(calcularNormal(pointsBack[1], pointsBack[2], pointsFront[2]))
    normais.append(calcularNormal(pointsBack[0], pointsBack[1], pointsFront[1]))
    normais.append(calcularNormal(pointsBack[4], pointsBack[3], pointsFront[3]))
    normais.append(calcularNormal(pointsBack[5], pointsBack[4], pointsFront[4]))
    normais.append(calcularNormal(pointsBack[6], pointsBack[5], pointsFront[5]))
    normais.append(calcularNormal(pointsBack[7], pointsBack[6], pointsFront[6]))

    visao = [480.0, 330.0, -70.0]
    visoes = []
    visoes.append(checarVisao(pointsFront[1], normais[0], visao))
    visoes.append(checarVisao(pointsBack[2], normais[1], visao))
    visoes.append(checarVisao(pointsBack[1], normais[2], visao))
    visoes.append(checarVisao(pointsBack[3], normais[3], visao))
    visoes.append(checarVisao(pointsBack[4], normais[4], visao))
    visoes.append(checarVisao(pointsBack[5], normais[5], visao))
    visoes.append(checarVisao(pointsBack[6], normais[6], visao))

    luz = [480.0, 330.0, -70.0]
    luzes = []
    luzes.append(retornaVetorLuz(pointsFront[1], luz))
    luzes.append(retornaVetorLuz(pointsBack[2], luz))
    luzes.append(retornaVetorLuz(pointsBack[1], luz))
    luzes.append(retornaVetorLuz(pointsBack[3], luz))
    luzes.append(retornaVetorLuz(pointsBack[5], luz))
    luzes.append(retornaVetorLuz(pointsBack[5], luz))
    luzes.append(retornaVetorLuz(pointsBack[6], luz))

    faces1 = []
    faces1.append(faceOficial)
    faces1.append([pointsBack[2], pointsBack[1], pointsFront[1], pointsFront[2]])
    faces1.append([pointsBack[1], pointsBack[0], pointsFront[0], pointsFront[1]])
    faces1.append([pointsBack[0], pointsBack[4], pointsFront[4], pointsFront[0]])
    faces1.append([pointsBack[4], pointsBack[5], pointsFront[5], pointsFront[4]])
    faces1.append([pointsBack[5], pointsBack[6], pointsFront[6], pointsFront[5]])
    faces1.append([pointsBack[6], pointsBack[7], pointsFront[7], pointsFront[6]])

    j = 0
    for j in range(len(faces1)):
        v1 = np.array(normais[j])
        v2 = np.array(luzes[j])
        cosLuz = angle_between(v1, v2)
        coef = 0.7
        pts = []
        for point in faces1[j]:
            p1 = Point(point[0], point[1])
            pts.append(p1)
        hsv = [180,1,255]
        R = hsv[0]
        G = hsv[1]
        B = hsv[2]*coef*cosLuz
        cores = hsvToRgb(R,G,B)
        R = cores[0]
        G = cores[1]
        B = cores[2]
        p = Polygon(pts)
        p.setFill(color_rgb(R, G ,B))
        p.setOutline(color_rgb(R, G, B))
        p.draw(janela)
        update(60)

    janela.getMouse()
    janela.close()
main()