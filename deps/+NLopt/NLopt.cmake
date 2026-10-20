# NLopt 2.5 sets THREADLOCAL only in the configure run that first tests for
# thread local storage: a later run finds the test cached, skips it, and leaves
# THREADLOCAL empty. Its random state is then one for the whole process, and
# genetic searches running on two threads at once (the support tree's ground
# route detours) draw from each other's sequence, so seed(0) no longer makes
# them repeatable. MSVC's keyword is given here instead of detected. Passed as
# a cache entry, not on the command line, for the parentheses.
set(_nlopt_cache_args "")
if (MSVC)
  set(_nlopt_cache_args CMAKE_CACHE_ARGS "-DTHREADLOCAL:STRING=__declspec(thread)")
endif ()

add_cmake_project(NLopt
  URL "https://github.com/stevengj/nlopt/archive/v2.5.0.tar.gz"
  URL_HASH SHA256=c6dd7a5701fff8ad5ebb45a3dc8e757e61d52658de3918e38bab233e7fd3b4ae
  # Before CMAKE_ARGS, which takes everything after it.
  ${_nlopt_cache_args}
  CMAKE_ARGS
    -DNLOPT_PYTHON:BOOL=OFF
    -DNLOPT_OCTAVE:BOOL=OFF
    -DNLOPT_MATLAB:BOOL=OFF
    -DNLOPT_GUILE:BOOL=OFF
    -DNLOPT_SWIG:BOOL=OFF
    -DNLOPT_TESTS:BOOL=OFF
)
