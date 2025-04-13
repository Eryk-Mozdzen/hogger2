#include <stdio.h>
#include <stdlib.h>
#include <math.h>

int main(int argc, char **argv) {
    const double clock_hz = atof(argv[1])*1000000.;
    const double desired_ns = atof(argv[2])*1000.;
    const double tDTS_ns = 1000000000./clock_hz;

    printf("APBx frequency    = %12f MHz\n", clock_hz/1000000.);
    printf("t_DTS             = %12f ns\n", tDTS_ns);

    double x;

    x = desired_ns/tDTS_ns;
    printf("DTG 0xxxxxxx    x = %12f in [0; 127] ", x);
    if((x>=0) && (x<=127)) {
        printf("OK\n");

        const int x_lb = floor(x);
        const int x_ub = ceil(x);

        printf("\t for DTG = %3d deadtime = %12f us\n", x_lb, 0.001*x_lb*tDTS_ns);
        printf("\t for DTG = %3d deadtime = %12f us\n", x_ub, 0.001*x_ub*tDTS_ns);
    } else {
        printf("IMPOSSIBLE\n");
    }

    x = desired_ns/(2*tDTS_ns) - 64;
    printf("DTG 10xxxxxx    x = %12f in [0; 63]  ", x);
    if((x>=0) && (x<=63)) {
        printf("OK\n");

        const int x_lb = floor(x);
        const int x_ub = ceil(x);

        printf("\t for DTG = %3d deadtime = %12f us\n", 0b10000000 | x_lb, 0.001*(64 + x_lb)*2*tDTS_ns);
        printf("\t for DTG = %3d deadtime = %12f us\n", 0b10000000 | x_ub, 0.001*(64 + x_ub)*2*tDTS_ns);
    } else {
        printf("IMPOSSIBLE\n");
    }

    x = desired_ns/(8*tDTS_ns) - 32;
    printf("DTG 110xxxxx    x = %12f in [0; 32]  ", x);
    if((x>=0) && (x<=31)) {
        printf("OK\n");

        const int x_lb = floor(x);
        const int x_ub = ceil(x);

        printf("\t for DTG = %3d deadtime = %12f us\n", 0b11000000 | x_lb, 0.001*(32 + x_lb)*8*tDTS_ns);
        printf("\t for DTG = %3d deadtime = %12f us\n", 0b11000000 | x_ub, 0.001*(32 + x_ub)*8*tDTS_ns);
    } else {
        printf("IMPOSSIBLE\n");
    }

    x = desired_ns/(16*tDTS_ns) - 32;
    printf("DTG 111xxxxx    x = %12f in [0; 32]  ", x);
    if((x>=0) && (x<=31)) {
        printf("OK\n");

        const int x_lb = floor(x);
        const int x_ub = ceil(x);

        printf("\t for DTG = %3d deadtime = %12f us\n", 0b11100000 | x_lb, 0.001*(32 + x_lb)*16*tDTS_ns);
        printf("\t for DTG = %3d deadtime = %12f us\n", 0b11100000 | x_ub, 0.001*(32 + x_ub)*16*tDTS_ns);
    } else {
        printf("IMPOSSIBLE\n");
    }

    return 0;
}
