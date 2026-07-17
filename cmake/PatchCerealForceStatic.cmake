# Neutralize cereal's forced MSVC dllexport / default visibility for monolithic
# slicer_core / slicer-engine builds (windows-policy W-EXP-1 / tasks 5.3).
# Cereal's StaticObject templates otherwise flood the DLL export table (~470 names).

function(slicer_patch_cereal_force_static)
    if (NOT DEFINED cereal_INCLUDE_DIR AND TARGET cereal::cereal)
        get_target_property(_cereal_incs cereal::cereal INTERFACE_INCLUDE_DIRECTORIES)
        if (_cereal_incs)
            list(GET _cereal_incs 0 cereal_INCLUDE_DIR)
        endif ()
    endif ()
    if (NOT cereal_INCLUDE_DIR)
        # Fallback: common deps destdir layout
        if (DEFINED CMAKE_PREFIX_PATH)
            foreach (_p IN LISTS CMAKE_PREFIX_PATH)
                if (EXISTS "${_p}/include/cereal/details/static_object.hpp")
                    set(cereal_INCLUDE_DIR "${_p}/include")
                    break()
                endif ()
            endforeach ()
        endif ()
    endif ()
    if (NOT cereal_INCLUDE_DIR)
        message(WARNING "PatchCerealForceStatic: cereal include dir not found; skip header patch")
        return()
    endif ()

    set(_hdr "${cereal_INCLUDE_DIR}/cereal/details/static_object.hpp")
    if (NOT EXISTS "${_hdr}")
        message(WARNING "PatchCerealForceStatic: missing ${_hdr}")
        return()
    endif ()

    file(READ "${_hdr}" _content)
    if (_content MATCHES "CEREAL_FORCE_STATIC")
        message(STATUS "cereal static_object.hpp already supports CEREAL_FORCE_STATIC")
        return()
    endif ()

    set(_old [[#ifdef _MSC_VER
#   define CEREAL_DLL_EXPORT __declspec(dllexport)
#   define CEREAL_USED
#else // clang or gcc
#   define CEREAL_DLL_EXPORT __attribute__ ((visibility("default")))
#   define CEREAL_USED __attribute__ ((__used__))
#endif]])

    set(_new [[#ifdef CEREAL_FORCE_STATIC
#   define CEREAL_DLL_EXPORT
#   if defined(_MSC_VER)
#       define CEREAL_USED
#   else
#       define CEREAL_USED __attribute__ ((__used__))
#   endif
#elif defined(_MSC_VER)
#   define CEREAL_DLL_EXPORT __declspec(dllexport)
#   define CEREAL_USED
#else // clang or gcc
#   define CEREAL_DLL_EXPORT __attribute__ ((visibility("default")))
#   define CEREAL_USED __attribute__ ((__used__))
#endif]])

    string(REPLACE "${_old}" "${_new}" _patched "${_content}")
    if (_patched STREQUAL _content)
        message(WARNING "PatchCerealForceStatic: pattern not found in ${_hdr}; manual patch required")
        return()
    endif ()

    file(WRITE "${_hdr}" "${_patched}")
    message(STATUS "Patched cereal header for CEREAL_FORCE_STATIC: ${_hdr}")
endfunction()
