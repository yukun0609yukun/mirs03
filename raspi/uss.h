#ifndef __USS__
#define __USS__

extern int  uss_open_rf();
extern int  uss_open_rb();
extern int  uss_open_br();
extern int  uss_open_bl();
extern long uss_get_rf();
extern long uss_get_rb();
extern long uss_get_br();
extern long uss_get_bl();
extern void uss_thread();

extern long uss_rf, uss_rb;

#endif
