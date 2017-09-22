# -*- coding: utf-8 -*-
from graphics import *
import numpy as np
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
        for i, (x, y, z, _) in enumerate(self.vertices):
            print " %d: (%.2f, %.2f, %.2f)" % (i, x, y, z)

    def outArestas(self):
        print "\n --- Arestas --- "
        for i, (vertice1, vertice2) in enumerate(self.arestas):
            print "%d: %d -> %d" % (i, vertice1, vertice2)

    def rotZ(self, angulo):
        c = np.cos(angulo)
        s = np.sin(angulo)
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

    def rotY(self, radianos):
        c = np.cos(radianos)
        s = np.sin(radianos)
        rY = np.array([[c, 0, -s, 0],
                       [0, 1, 0, 0],
                       [s, 0, c, 0],
                       [0, 0, 0, 1]])

        centro = self.getCentro()
        self.translacao(-centro[0], -centro[1], -centro[2])
        self.vertices = self.vertices.transpose()
        self.vertices = np.dot(rY, self.vertices)
        self.vertices = self.vertices.transpose()
        self.translacao(centro[0], centro[1], centro[2])

    def rotX(self, radianos):
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

    def getCentro(self):
        n_vertices = len(self.vertices)
        vert = []
        vert = [sum(self.vertices[:,i])/n_vertices for i in range(4)]
        return(vert[0], vert[1], vert[2])

    def desenha(self, grafico):
        for n1,n2 in self.arestas:
            ptI = Point(self.vertices[n1][0], self.vertices[n1][1])
            ptF = Point(self.vertices[n2][0], self.vertices[n2][1])
            ln = Line(ptI, ptF)
            ln.draw(grafico)
        for vertice in self.vertices:
            c = Circle(Point(int(vertice[0]), int(vertice[1])), 2)
            c.draw(grafico)
    
    def pinta(self, faces, window):
        for face in faces:
            pts = []
            for i in range(len(face)):
               pt = Point(self.vertices[face[i]][0], self.vertices[face[i]][1]) 
               pts.append(pt)
            p = Polygon(pts)
            p.setFill("green")
            p.draw(window)
    
    def pintaPontos(self, window):
        i = 40
        for vertice in self.vertices:
            s = ','.join(str(e) for e in vertice)
            ponto = Text(Point(150,i),s)
            ponto.setFace("arial")
            ponto.setStyle("bold")
            ponto.draw(window)
            i += 40

def clear(win):
    for item in win.items[:]:
        item.undraw()
    win.update()

def main():
    w,h = 700,700
    janela = GraphWin("janela", w, h, autoflush=False)
    janela.setBackground(color_rgb(255,255,255))
    cx,cy = w//2, h//2
   
    hexagonVertices = [[25,400,20],
                      [30,410,20],
                      [40,410,20],
                      [45,400,20],
                      [40,390,20],
                      [30,390,20],
                      [25,400,40],
                      [30,410,40],
                      [40,410,40],
                      [45,400,40],
                      [40,390,40],
                      [30,390,40]]
                      
    hexagonArestas = [[0,1],[1,2],[2,3],[3,4],[4,5],[5,0],[0,6],[1,7],[2,8],[3,9],[4,10],[5,11],[6,7],[7,8],[8,9],[9,10],[10,11],[11,6]]
    hexagonoFaces1 = [[0,6,7,1], [1,7,8,2], [2,8,9,3], [3,9,10,4], [4,5,11,10], [5,0,6,11]]
    hexagonoFaces2 = [[0,1,2,3,4,5], [6,7,8,9,10,11]]

    hexagono = Solido()
    hexagono.addVertices(np.array(hexagonVertices))
    hexagono.addArestas(hexagonArestas)
    #hexagono.outVertices()
    #hexagono.outArestas()

    hexagono.escala(6,6,6)
    hexagono.translacao(450,-150,0)
   
    while True:
        hexagono.rotX(0.1)
        hexagono.rotY(0.1)
        hexagono.rotZ(0.1)
        time.sleep(.05)
        clear(janela)
        hexagono.pinta(hexagonoFaces1, janela)
        hexagono.pinta(hexagonoFaces2, janela)
        hexagono.pintaPontos(janela)
        hexagono.desenha(janela)
        update(60)
    
    janela.getMouse()
    janela.close()
main()
