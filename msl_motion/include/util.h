#ifndef UTIL_H
#define UTIL_H 1

#define INT2BYTEPOS(i,b,pos) { (b)[(pos)] = (char)(i); (b)[(pos)+1] = (char)((i)>>8); (b)[(pos)+2] = (char)((i)>>16); (b)[(pos)+3] = (char)((i)>>24);}
#define SHORT2BYTEPOS(i,b,pos) { (b)[(pos)] = (char)(i); (b)[(pos)+1] = (char)((i)>>8);}
#define TIMEDIFFMS(n,o) ((n).tv_sec-(o).tv_sec)*1000+((n).tv_usec-(o).tv_usec)/1000
#define SIGN(a) ((a)>0)-((a)<0)
#define SIGN2(a) ((a)>0?1:((a)<0?-1:0))
#define MAX(a,b) ((a)>(b)?(a):(b))
#define MIN(a,b) ((a)>(b)?(b):(a))
#define CLAMP(x,b) ((x)>(b)?(b):((x)<-(b)?-(b):(x)))

#endif
