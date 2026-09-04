#ifndef ADA_CPP_OCCT_COMPAT_H
#define ADA_CPP_OCCT_COMPAT_H

// Shims over the OCCT API breaks between 7.9.x and 8.0 so the tree compiles against
// both. Keep this to genuine incompatibilities: anything OCCT 8 merely *deprecates*
// (Standard_Real, Standard_Integer, the TColStd_* aliases, ...) still compiles and
// does not belong here.

#include <Standard_Failure.hxx>
#include <Standard_Version.hxx>

// OCCT 8.0 re-rooted Standard_Failure on std::exception. It is no longer a
// Standard_Transient, so DynamicType() is gone — the exception's type name now comes
// from the virtual ExceptionType() — and GetMessageString() is deprecated in favour of
// the std::exception what().
inline const char *occt_failure_type(const Standard_Failure &failure) {
#if OCC_VERSION_HEX >= 0x080000
    return failure.ExceptionType();
#else
    return failure.DynamicType()->Name();
#endif
}

inline const char *occt_failure_message(const Standard_Failure &failure) {
#if OCC_VERSION_HEX >= 0x080000
    return failure.what();
#else
    return failure.GetMessageString();
#endif
}

#endif // ADA_CPP_OCCT_COMPAT_H
