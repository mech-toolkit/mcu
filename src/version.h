#ifndef VERSION_H
#define VERSION_H

#define VERSION_MAJOR 2
#define VERSION_MINOR 0
#define VERSION_PATCH 1

#define VERSION_STR_(a, b, c) #a "." #b "." #c
#define VERSION_STR(a, b, c) VERSION_STR_(a, b, c)

#define VERSION VERSION_STR(VERSION_MAJOR, VERSION_MINOR, VERSION_PATCH)

#endif