from graphics import *

def main():
    janela = GraphWin("janela", 700, 700)
    janela.setBackground(color_rgb(255,255,255))
    hexa = [
        [50, 125, 1],
        [125, 200, 1],
        [250, 200, 1],
        [325, 125, 1],
        [125, 50, 1],
        [250, 50, 1]]

    escala_x = [
        [2, 0],
        [0, 1]]
    escala_y = [
        [1, 0],
        [0, 2]]

    cis_x = [
        [1, 2],
        [0, 1]]
    cis_y = [
        [1, 0],
        [2, 1]]

    perspec = [
    [1, 0, 0],
    [0, 1, 0],
    [2, 2, 1]]

    for i in range(0, 6):
        for j in range(0, 2):
            sum = 0
            for k in range(0, 2):
                sum += hexa[i][k] * escala_x[k][j]
            hexa[i][j] = sum

    pt1 = Point(hexa[0][0], hexa[0][1])
    pt2 = Point(hexa[1][0], hexa[1][1])
    pt3 = Point(hexa[2][0], hexa[2][1])
    pt4 = Point(hexa[3][0], hexa[3][1])
    pt5 = Point(hexa[4][0],hexa[4][1])
    pt6 = Point(hexa[5][0],hexa[5][1])
    ln1 = Line(pt1, pt2)
    ln2 = Line(pt1, pt5)
    ln3 = Line(pt2, pt3)
    ln4 = Line(pt5, pt6)
    ln5 = Line(pt3, pt4)
    ln6 = Line(pt6, pt4)

    ln1.setOutline(color_rgb(0, 0, 0))
    ln2.setOutline(color_rgb(0, 0, 0))
    ln3.setOutline(color_rgb(0, 0, 0))
    ln4.setOutline(color_rgb(0, 0, 0))
    ln5.setOutline(color_rgb(0, 0, 0))
    ln6.setOutline(color_rgb(0, 0, 0))

    ln1.draw(janela)
    ln2.draw(janela)
    ln3.draw(janela)
    ln4.draw(janela)
    ln5.draw(janela)
    ln6.draw(janela)

    janela.getMouse()
    janela.close()
main()
