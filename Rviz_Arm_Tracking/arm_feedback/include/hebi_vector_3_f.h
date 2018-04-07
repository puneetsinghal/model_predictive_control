#ifndef HEBI_VECTOR_3_F
#define HEBI_VECTOR_3_F

// TODO: document; see include/hebi_module!

#ifdef __cplusplus /* Use C linkage when compiling this C library header from C++ */
extern "C" {
#endif

typedef struct _HebiVector3f {
  float x;
  float y;
  float z;
} HebiVector3f;

#ifdef __cplusplus /* End C linkage when compiling from C++ */
}
#endif

#endif // HEBI_VECTOR_3_F
