

# Chipmunk for LuaJIT FFI

Sorry, this "ffi.lua" file is not tested enough.. ( I did only simple loading test. ) 
<br> I wrote this ffi.lua when chipmunk version is ``chipmunk-7.0.1.``

* Chinpmunk physics - https://chipmunk-physics.net/
* LuaJIT - http://luajit.org/

## Note .1

* build chipmunk Dll with enable ``CHIPMUNK_FFI`` preprocessor/define.
* preprocessor/define ``CHIPMUNK_FFI``. this option will be export static inline function for ffi. ( ref at  ``chipmunk_ffi.h`` and end of line ``chipmunk.c`` )
* change loading module name at ``chipmunk_ffi.lua`` line 2

## Note .2 

when if you want using TDM-GCC on win32

**On chipmunk_ffi.h**

added `CP_EXPORT` in chipmunk_ffi.h line 37 ( or near )

```
#ifdef _MSC_VER
 #if _MSC_VER >= 1600
  #define MAKE_REF(name) CP_EXPORT decltype(name) *_##name = name
 #else
  #define MAKE_REF(name)
 #endif
#else
 #define MAKE_REF(name) CP_EXPORT __typeof__(name) *_##name = name  <- this line added `CP_EXPORT`
#endif
```

**Compile commands**

Example compile commands : at src folder

```
@SET DLL_NAME=chipmunk-7.0.1.dll
> gcc -O3 -ffast-math -std=gnu99 -DCHIPMUNK_FFI -fPIC -I../include -c *.c
> gcc -shared  -DCHIPMUNK_FFI  -o %DLL_NAME% -Wl,--out-implib=%DLL_NAME%.a -Wl,--output-def=%DLL_NAME%.def  *.o  -static-libstdc++ -static-libgcc
// > del *.o

```
