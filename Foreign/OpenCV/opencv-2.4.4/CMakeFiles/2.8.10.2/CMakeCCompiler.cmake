set(CMAKE_C_COMPILER "C:/Dev-Cpp/MinGW64/bin/gcc.exe")
set(CMAKE_C_COMPILER_ARG1 "")
set(CMAKE_C_COMPILER_ID "GNU")
set(CMAKE_C_COMPILER_VERSION "4.7.1")
set(CMAKE_C_PLATFORM_ID "MinGW")

set(CMAKE_AR "C:/Dev-Cpp/MinGW64/bin/ar.exe")
set(CMAKE_RANLIB "C:/Dev-Cpp/MinGW64/bin/ranlib.exe")
set(CMAKE_LINKER "C:/Dev-Cpp/MinGW64/bin/ld.exe")
set(CMAKE_COMPILER_IS_GNUCC 1)
set(CMAKE_C_COMPILER_LOADED 1)
set(CMAKE_C_COMPILER_WORKS TRUE)
set(CMAKE_C_ABI_COMPILED TRUE)
set(CMAKE_COMPILER_IS_MINGW 1)
set(CMAKE_COMPILER_IS_CYGWIN )
if(CMAKE_COMPILER_IS_CYGWIN)
  set(CYGWIN 1)
  set(UNIX 1)
endif()

set(CMAKE_C_COMPILER_ENV_VAR "CC")

if(CMAKE_COMPILER_IS_MINGW)
  set(MINGW 1)
endif()
set(CMAKE_C_COMPILER_ID_RUN 1)
set(CMAKE_C_SOURCE_FILE_EXTENSIONS c)
set(CMAKE_C_IGNORE_EXTENSIONS h;H;o;O;obj;OBJ;def;DEF;rc;RC)
set(CMAKE_C_LINKER_PREFERENCE 10)

# Save compiler ABI information.
set(CMAKE_C_SIZEOF_DATA_PTR "8")
set(CMAKE_C_COMPILER_ABI "")
set(CMAKE_C_LIBRARY_ARCHITECTURE "")

if(CMAKE_C_SIZEOF_DATA_PTR)
  set(CMAKE_SIZEOF_VOID_P "${CMAKE_C_SIZEOF_DATA_PTR}")
endif()

if(CMAKE_C_COMPILER_ABI)
  set(CMAKE_INTERNAL_PLATFORM_ABI "${CMAKE_C_COMPILER_ABI}")
endif()

if(CMAKE_C_LIBRARY_ARCHITECTURE)
  set(CMAKE_LIBRARY_ARCHITECTURE "")
endif()




set(CMAKE_C_IMPLICIT_LINK_LIBRARIES "mingw32;moldname;mingwex;msvcrt;advapi32;shell32;user32;kernel32;mingw32;moldname;mingwex;msvcrt")
set(CMAKE_C_IMPLICIT_LINK_DIRECTORIES "c:/Dev-Cpp/MinGW64/lib/gcc/x86_64-w64-mingw32/4.7.1;c:/Dev-Cpp/MinGW64/lib/gcc;c:/Dev-Cpp/MinGW64/x86_64-w64-mingw32/lib;c:/Dev-Cpp/MinGW64/lib")



