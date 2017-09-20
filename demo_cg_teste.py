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
        #for ver in vertLista:
        #    self.vertices.append(Vertice(ver))
        coluna_uns = np.ones((len(vertLista), 1))
        uns_added = np.hstack((vertLista, coluna_uns))
        self.vertices = np.vstack((self.vertices, uns_added))

    def addArestas(self, arestLista):
        #for (ini, fim) in arestLista:
        #    self.arestas.append(Aresta(self.vertices[ini], self.vertices[fim]))
        self.arestas += arestLista

    def outVertices(self):
        print "\n --- Vertices --- "
        for i, (x, y, z, _) in enumerate(self.vertices):
            print " %d: (%.2f, %.2f, %.2f)" % (i, x, y, z)
            #print " %d: (%.2f, %.2f, %.2f)" % (i, vertice.x, vertice.y, vertice.z)

    def outArestas(self):
        print "\n --- Arestas --- "
        for i, (vertice1, vertice2) in enumerate(self.arestas):
            print "%d: %d -> %d" % (i, vertice1, vertice2)
            #print "%d:  (%.2f, %.2f, %.2f)" % (aresta.ini.x,  aresta.ini.y, aresta.ini.z)
            #print "para (%.2f, %.2f, %.2f)" % (aresta.fim.x,  aresta.fim.y, aresta.fim.z)

    def rotZ(self, radianos):
        c = np.cos(radianos)
        s = np.cos(radianos)
        rZ = np.array([[c, -s, 0, 0],
                       [s, c, 0, 0],
                       [0, 0, 1, 0],
                       [0, 0, 0, 1]])

        centro = self.getCentro()
        self.translacao(-centro[0], -centro[1], -centro[2])
        self.vertices = np.dot(self.vertices, rZ)
        self.translacao(centro[0], centro[1], centro[2])
        '''
        for vertice in self.vertices:
            x = vertice.x - cx
            y = vertice.y - cy
            distancia = math.hypot(y,x)
            teta = math.atan2(y,x) + radianos
            vertice.x = cx + distancia * math.cos(teta)
            vertice.y = cy + distancia * math.sin(teta)
        '''

    def rotY(self, radianos):
        c = np.cos(radianos)
        s = np.cos(radianos)
        rY = np.array([[c, 0, -s, 0],
                       [0, 1, 0, 0],
                       [s, 0, c, 0],
                       [0, 0, 0, 1]])

        centro = self.getCentro()
        self.translacao(-centro[0], -centro[1], -centro[2])
        self.vertices = np.dot(self.vertices, rY)
        self.translacao(centro[0], centro[1], centro[2])
        '''
        for vertice in self.vertices:
            x = vertice.x - cx
            z = vertice.z - cz
            distancia = math.hypot(x,z)
            teta = math.atan2(x,z) + radianos
            vertice.z = cz + distancia * math.cos(teta)
            vertice.x = cx + distancia * math.sin(teta)
        '''
    def rotX(self, radianos):
    #def rotX(self, (cx, cy, cz), radianos):
        c = np.cos(radianos)
        s = np.cos(radianos)
        rX = np.array([[1, 0, 0, 0],
                       [0, c, s, 0],
                       [0, -s, c, 0],
                       [0, 0, 0, 1]])

        centro = self.getCentro()
        self.translacao(-centro[0], -centro[1], -centro[2])
        self.vertices = np.dot(self.vertices, rX)
        self.translacao(centro[0], centro[1], centro[2])
        '''
        for vertice in self.vertices:
            y = vertice.y - cy
            z = vertice.z - cz
            distancia = math.hypot(y,z)
            teta = math.atan2(y,z) + radianos
            vertice.z = cz + distancia * math.cos(teta)
            vertice.y = cy + distancia * math.sin(teta)
        '''

    #def translacao(self, eixo, distancia):
    def translacao(self, dx, dy, dz):
        t = np.array([[1, 0, 0, 0],
                      [0, 1, 0, 0],
                      [0, 0, 1, 0],
                      [dx, dy, dz, 1]])
        self.vertices = np.dot(self.vertices, t)
        '''
        if eixo in ['x','y','z']:
            for vertice in self.vertices:
                setattr(vertice, eixo, getattr(vertice, eixo) + distancia)
        '''
    def escala(self, sx=0, sy=0, sz=0):
    #def escala(self, (centro_x, centro_y), escala):
        e = np.array([[sx, 0, 0, 0],
                      [0, sy, 0, 0],
                      [0, 0, sz, 0],
                      [0, 0, 0,  1]])
        centro = self.getCentro()
        self.translacao(-centro[0], -centro[1], -centro[2])
        self.vertices = np.dot(self.vertices, e)
        self.translacao(centro[0], centro[1], centro[2])
        '''
        for vertice in self.vertices:
            vertice.x = centro_x + escala * (vertice.x - centro_x)
            vertice.y = centro_y + escala * (vertice.y - centro_y)
            vertice.z *= escala
        '''

    def getCentro(self):
        n_vertices = len(self.vertices)

        '''
        midX = vert[0] / n_vertices
        midY = vert[1] / n_vertices
        midZ = vert[2] / n_vertices
        print self.vertices
        midX = sum([vertices.x for vertice in self.vertices]) / n_vertices
        midY = sum([vertices.y for vertice in self.vertices]) / n_vertices
        midZ = sum([vertices.z for vertice in self.vertices]) / n_vertices
        '''
        vert = []
        vert = [sum(self.vertices[:,i])/n_vertices for i in range(4)]
        #vert = self.vertices.mean(axis=0)
        #self.outVertices()
        print(vert[0], vert[1], vert[2])
        return(vert[0], vert[1], vert[2])

    def desenha(self, grafico):
        #for aresta in self.arestas:
        for n1,n2 in self.arestas:
            '''
            ptI = Point(aresta.ini.x, aresta.ini.y)
            ptF = Point(aresta.fim.x, aresta.fim.y)
            ln = Line(ptI, ptF)
            '''
            ptI = Point(self.vertices[n1][0], self.vertices[n1][1])
            ptF = Point(self.vertices[n2][0], self.vertices[n2][1])
            ln = Line(ptI, ptF)
            ln.draw(grafico)
        for vertice in self.vertices:
            '''
            c = Circle(Point(int(vertice.x), int(vertice.y)), 2)
            '''
            c = Circle(Point(int(vertice[0]), int(vertice[1])), 2)
            c.draw(grafico)

def clear(win):
    for item in win.items[:]:
        item.undraw()
    win.update()

def main():
    w,h = 500,500
    janela = GraphWin("janela", w, h, autoflush=False)
    janela.setBackground(color_rgb(255,255,255))
    cx,cy = w//2, h//2
    cuboVertices = [(x,y,z) for x in (25 + cx, 125 + cx)
                    for y in (25 + cy, 125 + cy)
                    for z in (25, 125)]
    cubo = Solido()
    cubo.addVertices(np.array(cuboVertices))

    cubo.addArestas([(n,n+4) for n in range(0,4)])
    cubo.addArestas([(n,n+1) for n in range(0,8,2)])
    cubo.addArestas([(n,n+2) for n in (0,1,4,5)])

    #cubo.outVertices()
    #cubo.outArestas()
    #cubo.escala((cx, cy), 2) #Funcionando
    #cubo.rotY(cubo.getCentro(), 0.7)
    #cubo.rotX(cubo.getCentro(), 0.3)
    #cubo.desenha(janela)


    #cubo.translacao('z', 100) #Projeçao no eixo z
    #cubo.rotZ(cubo.getCentro(), 0.2) #Funcionando
    #cubo.rotY(cubo.getCentro(), 0.2) #Funcionando
    #cubo.rotX(cubo.getCentro(), 0.2) #Funcionando

    for i in range(4):
    #Animação
        #cubo.rotY(1)
        #ubo.rotX(0.1)
        cubo.rotY(0.1)
        #cubo.rotZ(0.1)
        #cubo.rotZ(1)
        #cubo.escala(2, 1, 1)
        time.sleep(.05)
        clear(janela)
        #cubo.translacao(200, 100, 20)
        #cubo.getCentro()
        cubo.desenha(janela)
        update(60)

    janela.getMouse()
    janela.close()
main()
