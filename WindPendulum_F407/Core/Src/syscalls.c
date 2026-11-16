#include <errno.h>
#include <sys/types.h>
#include <unistd.h>

/* No POSIX file descriptors are provided. Telemetry uses HAL_UART explicitly.
 * Weak hooks can be replaced when a filesystem or stdio backend is added. */
__attribute__((weak)) int _close(int file)
{
  (void)file;
  errno = EBADF;
  return -1;
}

__attribute__((weak)) _off_t _lseek(int file, _off_t offset, int whence)
{
  (void)file;
  (void)offset;
  (void)whence;
  errno = EBADF;
  return (_off_t)-1;
}

__attribute__((weak)) _ssize_t _read(int file, void *buffer, size_t length)
{
  (void)file;
  (void)buffer;
  (void)length;
  errno = EBADF;
  return -1;
}

__attribute__((weak)) _ssize_t _write(int file, const void *buffer, size_t length)
{
  (void)file;
  (void)buffer;
  (void)length;
  errno = EBADF;
  return -1;
}
