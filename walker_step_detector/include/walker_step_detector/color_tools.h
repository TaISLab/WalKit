#ifndef COLORTOOL_HH
#define COLORTOOL_HH


// from https://sashamaps.net/docs/resources/20-colors/
// convenient 95% accesibility rbg colors 
#define N_COLORS 22
const double R_FIXED[N_COLORS] = { 230,  60, 255,   0, 245, 145,  70, 240, 210, 250,   0, 220, 170, 255, 128, 170, 128, 255,   0, 128, 255, 0 };
const double G_FIXED[N_COLORS] = {  25, 180, 225, 130, 130,  30, 240,  50, 245, 190, 128, 190, 110, 250,   0, 255, 128, 215,   0, 128, 255, 0 };
const double B_FIXED[N_COLORS] = {  75,  75,  25, 200,  48, 180, 240, 230,  60, 212, 128, 255,  40, 200,   0, 195,   0, 180, 128, 128, 255, 0 };

// from David H. at https://stackoverflow.com/questions/3018313/algorithm-to-convert-rgb-to-hsv-and-hsv-to-rgb-in-range-0-255-for-both
// checking how to get a random list from https://martin.ankerl.com/2009/12/09/how-to-create-random-colors-programmatically/
typedef struct {
    double r;       // a fraction between 0 and 1
    double g;       // a fraction between 0 and 1
    double b;       // a fraction between 0 and 1
} RGB;

typedef struct {
    double h;       // angle in DEGREES! (0.0 - 360.0)
    double s;       // a fraction between 0 and 1
    double v;       // a fraction between 0 and 1
} HSV;

static HSV   rgb2hsv(RGB in);
static RGB   hsv2rgb(HSV in);

HSV rgb2hsv(RGB in){
    HSV         out;
    double      min, max, delta;

    min = in.r < in.g ? in.r : in.g;
    min = min  < in.b ? min  : in.b;

    max = in.r > in.g ? in.r : in.g;
    max = max  > in.b ? max  : in.b;

    out.v = max;   // v
    delta = max - min;
    if (delta < 0.00001)
    {
        out.s = 0;
        out.h = 0; // undefined, maybe nan?
        return out;
    }
    if( max > 0.0 ) { // NOTE: if Max is == 0, this divide would cause a crash
        out.s = (delta / max);                  // s
    } else {
        // if max is 0, then r = g = b = 0              
        // s = 0, h is undefined
        out.s = 0.0;
        out.h = -1;//NAN;                            // its now undefined
        return out;
    }
    if( in.r >= max )                           // > is bogus, just keeps compiler happy
        out.h = ( in.g - in.b ) / delta;        // between yellow & magenta
    else
    if( in.g >= max )
        out.h = 2.0 + ( in.b - in.r ) / delta;  // between cyan & yellow
    else
        out.h = 4.0 + ( in.r - in.g ) / delta;  // between magenta & cyan

    out.h *= 60.0;                              // degrees

    if( out.h < 0.0 )
        out.h += 360.0;

    return out;
}

RGB hsv2rgb(HSV in){
    double      hh, p, q, t, ff;
    long        i;
    RGB         out;

    if(in.s <= 0.0) {       // < is bogus, just shuts up warnings
        out.r = in.v;
        out.g = in.v;
        out.b = in.v;
        return out;
    }
    hh = in.h;
    if(hh >= 360.0) hh = 0.0;
    hh /= 60.0;
    i = (long)hh;
    ff = hh - i;
    p = in.v * (1.0 - in.s);
    q = in.v * (1.0 - (in.s * ff));
    t = in.v * (1.0 - (in.s * (1.0 - ff)));

    switch(i) {
    case 0:
        out.r = in.v;
        out.g = t;
        out.b = p;
        break;
    case 1:
        out.r = q;
        out.g = in.v;
        out.b = p;
        break;
    case 2:
        out.r = p;
        out.g = in.v;
        out.b = t;
        break;

    case 3:
        out.r = p;
        out.g = q;
        out.b = in.v;
        break;
    case 4:
        out.r = t;
        out.g = p;
        out.b = in.v;
        break;
    case 5:
    default:
        out.r = in.v;
        out.g = p;
        out.b = q;
        break;
    }
    return out;     
}

RGB pick_one_of_n(double id, double total){
        HSV hsv0;
        hsv0.h = 360.0 * (id/ total);
        hsv0.s = 0.95;
        hsv0.v = 0.95;
        RGB rgb0 = hsv2rgb(hsv0);
        hsv0 = rgb2hsv(rgb0);
        return rgb0;
}

RGB pick_one(int id){
        RGB rgb0;
        int index = id % N_COLORS;
        rgb0.r = R_FIXED[index]/ 255.0;
        rgb0.g = G_FIXED[index]/ 255.0;
        rgb0.b = B_FIXED[index]/ 255.0;

        return rgb0;
}

#endif
