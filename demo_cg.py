# -*- coding: utf-8 -*-
from graphics import *
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
        self.arestas = []

    def addVertices(self, vertLista):
        for ver in vertLista:
            self.vertices.append(Vertice(ver))

    def addArestas(self, arestLista):
        for (ini, fim) in arestLista:
            self.arestas.append(Aresta(self.vertices[ini], self.vertices[fim]))

    def outVertices(self):
        print ("\n --- Vertices --- ")
        for i, vertice in enumerate(self.vertices):
            print " %d: (%.2f, %.2f, %.2f)" % (i, vertice.x, vertice.y, vertice.z)

    def outArestas(self):
        print "\n --- Arestas --- "
        for i, aresta in enumerate(self.arestas):
            print "%d: (%.2f, %.2f, %.2f)" % (i, aresta.ini.x, aresta.ini.y, aresta.ini.z)
            print "para (%.2f, %.2f, %.2f)" % (aresta.fim.x,  aresta.fim.y, aresta.fim.z)

    def getCentro(self):
        n_vertices = len(self.vertices)
        midX = sum([vertice.x for vertice in self.vertices]) / n_vertices
        midY = sum([vertice.y for vertice in self.vertices]) / n_vertices
        midZ = sum([vertice.z for vertice in self.vertices]) / n_vertices

        return(midX, midY, midZ)

    def rotZ(self, (cx, cy, cz), radianos):
        for vertice in self.vertices:
            x = vertice.x - cx
            y = vertice.y - cy
            distancia = math.hypot(y,x)
            teta = math.atan2(y,x) + radianos
            vertice.x = cx + distancia * math.cos(teta)
            vertice.y = cy + distancia * math.sin(teta)

    def rotY(self, (cx, cy, cz), radianos):
        for vertice in self.vertices:
            x = vertice.x - cx
            z = vertice.z - cz
            distancia = math.hypot(x,z)
            teta = math.atan2(x,z) + radianos
            vertice.z = cz + distancia * math.cos(teta)
            vertice.x = cx + distancia * math.sin(teta)

    def rotX(self, (cx, cy, cz), radianos):
        for vertice in self.vertices:
            y = vertice.y - cy
            z = vertice.z - cz
            distancia = math.hypot(y,z)
            teta = math.atan2(y,z) + radianos
            vertice.z = cz + distancia * math.cos(teta)
            vertice.y = cy + distancia * math.sin(teta)


    def translacao(self, eixo, distancia):
        if eixo in ['x','y','z']:
            for vertice in self.vertices:
                setattr(vertice, eixo, getattr(vertice, eixo) + distancia)


    def escala(self, (centro_x, centro_y), escala):
        for vertice in self.vertices:
            vertice.x = centro_x + escala * (vertice.x - centro_x)
            vertice.y = centro_y + escala * (vertice.y - centro_y)
            vertice.z *= escala

    def desenha(self, grafico):
        for aresta in self.arestas:
            ptI = Point(aresta.ini.x, aresta.ini.y)
            ptF = Point(aresta.fim.x, aresta.fim.y)
            ln = Line(ptI, ptF)
            ln.draw(grafico)
        for vertice in self.vertices:
            c = Circle(Point(int(vertice.x), int(vertice.y)), 2)
            c.draw(grafico)
def clear(win):
    for item in win.items[:]:
        item.undraw()
    win.update()

def main():
    w,h = 500,500
    cx,cy = w//2, h//2
    janela = GraphWin("janela", w, h, autoflush=False)
    janela.setBackground(color_rgb(255,255,255))

    cuboVertices = [(x,y,z) for x in (cx + 25,cy + 125) for y in (cx + 25, cy + 125) for z in (cx + 25, cy + 125)]
    cubo = Solido()
    cubo.addVertices(cuboVertices)

    cubo.addArestas([(n,n+4) for n in range(0,4)])
    cubo.addArestas([(n,n+1) for n in range(0,8,2)])
    cubo.addArestas([(n,n+2) for n in (0,1,4,5)])

    #cubo.outVertices()
    #cubo.outArestas()

    #cubo.escala((cx, cy), 2) #Funcionando

    while True:

        '''
        key =  janela.checkKey()
        if(key == "w"):
            cubo.rotY(cubo.getCentro(), 0.2)
        elif(key == "a"):
            cubo.rotX(cubo.getCentro(), 0.2)
        elif(key == "s"):
            cubo.translacao('y',50)
        elif(key == "d"):
            cubo.translacao('x', 50)
        '''

        #cubo.translacao('z', 100) #Projeçao no eixo z
        #cubo.rotZ(cubo.getCentro(), 0.2) #Funcionando
        #cubo.rotY(cubo.getCentro(), 0.2) #Funcionando
        #cubo.rotX(cubo.getCentro(), 0.2) #Funcionando

        cubo.rotY(cubo.getCentro(), 0.2)
        cubo.rotX(cubo.getCentro(), 0.2)
        cubo.rotZ(cubo.getCentro(), 0.2)
        time.sleep(.05)
        clear(janela)
        cubo.desenha(janela)
        update(60)


    janela.getMouse()
    janela.close()
main()
