#include <stdio.h>
#include <math.h>
#include <graphics.h>

int main(){
    int gdriver = DETECT,gmode;
    initgraph(&gdriver, &gmode, NULL);
    setcolor(WHITE);
    setlinestyle(SOLID_LINE,0,2);

    double vtcs[6] [2] = {
        {10.0, 25.0},
        {25.0, 50.0},
        {50.0, 50.0},
        {65.0, 25.0},
        {25.0, 10.0},
        {50.0, 10.0}
    };

    //Transf de escala em x
    double escala[2][2] = {
            {4.0, 0.0},
            {0.0, 1.0}
        };
    /*
    //Transf de escala em y
        int escala[2][2] = {
            {1, 0},
            {0, k}
        };
    */

    //Transf de cisalhamento em x
    double cis[2][2] = {
        {1.0, 3.0},
        {0.0, 1.0}
    };

    int i = 0;
    int j = 0;
    int k = 0;
    double sum = 0;
    for (i = 0; i < 6; i++){
        for (j = 0; j < 2; j++){
            sum = 0;
             for (k = 0; k < 2; k++){
                sum += vtcs[i][k] * cis[k][j];
            }
            vtcs[i][j] = sum;
        }
    }

    line(vtcs[0][0], vtcs[0][1], vtcs[1][0], vtcs[1][1]);
    line(vtcs[0][0], vtcs[0][1], vtcs[4][0], vtcs[4][1]);
    line(vtcs[1][0], vtcs[1][1], vtcs[2][0], vtcs[2][1]);
    line(vtcs[4][0], vtcs[4][1], vtcs[5][0], vtcs[5][1]);
    line(vtcs[2][0], vtcs[2][1], vtcs[3][0], vtcs[3][1]);
    line(vtcs[5][0], vtcs[5][1], vtcs[3][0], vtcs[3][1]);

    //line(10, 100, 100, 200);
    //line(10, 100, 100, 10);
    //line(100, 200, 200, 200);
    //line(100, 10, 200, 10);
    //line(200, 200, 300, 100);
    //line(200, 10, 300, 100);


    getch();
    closegraph();
    return 0;
}
