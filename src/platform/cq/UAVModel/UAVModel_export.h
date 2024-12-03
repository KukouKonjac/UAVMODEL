
#ifndef UAVMODEL_EXPORT_H
#define UAVMODEL_EXPORT_H

#ifdef UAVMODEL_STATIC_DEFINE
#  define UAVMODEL_EXPORT
#  define UAVMODEL_NO_EXPORT
#else
#  ifndef UAVMODEL_EXPORT
#    ifdef UAVModel_EXPORTS
        /* We are building this library */
#      define UAVMODEL_EXPORT __declspec(dllexport)
#    else
        /* We are using this library */
#      define UAVMODEL_EXPORT __declspec(dllimport)
#    endif
#  endif

#  ifndef UAVMODEL_NO_EXPORT
#    define UAVMODEL_NO_EXPORT 
#  endif
#endif

#ifndef UAVMODEL_DEPRECATED
#  define UAVMODEL_DEPRECATED __declspec(deprecated)
#endif

#ifndef UAVMODEL_DEPRECATED_EXPORT
#  define UAVMODEL_DEPRECATED_EXPORT UAVMODEL_EXPORT UAVMODEL_DEPRECATED
#endif

#ifndef UAVMODEL_DEPRECATED_NO_EXPORT
#  define UAVMODEL_DEPRECATED_NO_EXPORT UAVMODEL_NO_EXPORT UAVMODEL_DEPRECATED
#endif

#if 0 /* DEFINE_NO_DEPRECATED */
#  ifndef UAVMODEL_NO_DEPRECATED
#    define UAVMODEL_NO_DEPRECATED
#  endif
#endif

#endif /* UAVMODEL_EXPORT_H */
