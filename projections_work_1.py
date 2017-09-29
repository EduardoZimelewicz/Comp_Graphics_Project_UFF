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
            cI = Circle(ptI, 3)
            cF = Circle(ptF, 3)
            cI.setFill("yellow")
            cI.draw(grafico)
            time.sleep(.2)
            update(60)
            ln = Line(ptI, ptF)
            ln.draw(grafico)
            time.sleep(.2)
            update(60)
            cF.setFill("red")
            cF.draw(grafico)
            time.sleep(.2)
            update(60)
            cF.setFill("yellow")
            time.sleep(.2)
            update(60)
    
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

def clear(win):
    for item in win.items[:]:
        item.undraw()
    win.update()

def main():
    w,h = 800,600
    janela = GraphWin("janela", w, h, autoflush=False)
    janela.setBackground(color_rgb(255,255,255))
    cx,cy = w//2, h//2
   
    desenhado = True
    janelaAberta = True

    while janelaAberta:
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
        hexagonoFaces1 = [[6,7,8,9,10,11], [1,7,8,2], [2,8,9,3], [3,9,10,4], [5,0,6,11], [0,6,7,1], [4,5,11,10], [0,1,2,3,4,5]]

        
        hexagono.addVertices(np.array(hexagonVertices))
        hexagono.addArestas(hexagonArestas)
        hexagono.outVertices()
        hexagono.escala(8,8,8)
        hexagono.translacao(550,-150,1)
    
        print "Digite 1 para Projecao Isometrica " + "\n" + "Digite 2 para Porjecao Obliqua " + "\n" + "Digite 3 para Projecao em Perspectiva com um ponto de fuga" + "\n"
        opcao = raw_input("Digite a opcao de desenho ou digite 'q' para sair: ")
        if(opcao == '1'):
            #projeção isométrica com dados os ângulos
            hexagono.projIsometrica(35.26,45)
        
        elif(opcao == '2'):
            #projeção obliqua com meu número (número 3) da chamada vezes 5
            hexagono.projObliquaCabinet(15, 1)
        
        elif(opcao == '3'):
            #um ponto de fuga com zcp = 100 + 10x3(3 é meu número na chamada)
            hexagono.projPerspecUmPontoFuga(130)
            hexagono.escala(150,150,20)
            hexagono.translacao(500,300)

        elif(opcao == 'q'):
            janelaAberta = False
            janela.close()
            break

        hexagono.pintaPontos(janela)
        hexagono.desenha(janela)
        hexagono.pinta(hexagonoFaces1, janela)
        update(60)

        while desenhado:
            ponto = raw_input("Gostaria de ver algum vertice? Diga o numero dele ou 'c' para limpar a janela: ")
            if(ponto == 'c'):
                clear(janela)
                update(60)
                break
            pt = Circle(Point(hexagono.vertices[int(ponto)][0], hexagono.vertices[int(ponto)][1]), 4)
            pt.setFill("red")
            pt.draw(janela)
            update(60)
            time.sleep(.4)
            pt.undraw()
            update(60)
main()
