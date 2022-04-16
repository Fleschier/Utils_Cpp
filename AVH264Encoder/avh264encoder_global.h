#ifndef AVH264ENCODER_GLOBAL_H
#define AVH264ENCODER_GLOBAL_H

#include <QtCore/qglobal.h>

#if defined(AVH264ENCODER_LIBRARY)
#  define AVH264ENCODERSHARED_EXPORT Q_DECL_EXPORT
#else
#  define AVH264ENCODERSHARED_EXPORT Q_DECL_IMPORT
#endif

#endif // AVH264ENCODER_GLOBAL_H
