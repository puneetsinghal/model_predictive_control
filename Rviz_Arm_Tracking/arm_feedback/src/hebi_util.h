#if !defined(_WIN32) && (defined(__unix__) || defined(__unix) || (defined(__APPLE__) && defined(__MACH__)))
#include <unistd.h>

inline void hebi_sleep_ms(float ms)
{
  usleep(ms * 1000);
}

inline void hebi_sleep_s(unsigned int s)
{
  sleep(s);
}

#else
#include <Windows.h>

inline void hebi_sleep_ms(float ms)
{
  Sleep(ms);
}

inline void hebi_sleep_s(unsigned int s)
{
  Sleep(s * 1000);
}

#endif
