/* This file has been automatically generated. Do not edit by hand. */

#ifndef HEBI_GROUP_INFO_H
#define HEBI_GROUP_INFO_H

#include "hebi_info.h"

#ifdef __cplusplus /* Use C linkage when compiling this C library header from C++ */
extern "C" {
#endif

typedef struct _HebiGroupInfo* HebiGroupInfoPtr;

HebiGroupInfoPtr hebiGroupInfoCreate(int number_of_modules);
int hebiGroupInfoGetNumModules(HebiGroupInfoPtr);
HebiInfoPtr hebiGroupInfoGetModuleInfo(HebiGroupInfoPtr, int module_index);
void hebiGroupInfoDestroy(HebiGroupInfoPtr);

#ifdef __cplusplus /* End C linkage when compiling from C++ */
}
#endif

#endif // HEBI_GROUP_INFO_H
