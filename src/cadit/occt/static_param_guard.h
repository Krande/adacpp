#ifndef ADACPP_STATIC_PARAM_GUARD_H
#define ADACPP_STATIC_PARAM_GUARD_H

#include <Interface_Static.hxx>
#include <string>

// RAII snapshot/restore for an OCCT Interface_Static C-string parameter.
//
// OCCT's Interface_Static table is process-global. A STEP read/write that sets
// e.g. "write.step.schema" or "xstep.cascade.unit" and does not restore it leaks
// that setting into every later OCC operation in the same process — surfacing as
// order-dependent, hard-to-reproduce behaviour. Wrap each parameter you set in a
// guard so the previous value is restored on scope exit (including exceptions).
class InterfaceStaticCValGuard {
public:
    explicit InterfaceStaticCValGuard(const char *key) : key_(key) {
        const char *cur = Interface_Static::CVal(key);
        saved_ = (cur != nullptr) ? cur : "";
    }
    ~InterfaceStaticCValGuard() {
        Interface_Static::SetCVal(key_, saved_.c_str());
    }

    InterfaceStaticCValGuard(const InterfaceStaticCValGuard &) = delete;
    InterfaceStaticCValGuard &operator=(const InterfaceStaticCValGuard &) = delete;

private:
    const char *key_;
    std::string saved_;
};

#endif // ADACPP_STATIC_PARAM_GUARD_H
