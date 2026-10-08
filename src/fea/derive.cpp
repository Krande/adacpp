#include "derive.h"

#include <cmath>
#include <stdexcept>

namespace adacpp::fea {

namespace {

// derived_values.plane_von_mises, in double:
//   np.sqrt(sig_x * sig_x + sig_y * sig_y - sig_x * sig_y + 3.0 * tau_xy * tau_xy)
// Python evaluates left to right: ((sx*sx + sy*sy) - sx*sy) + (3.0*txy)*txy.
inline double von_mises(double sx, double sy, double txy) {
    const double a = sx * sx;
    const double b = sy * sy;
    const double c = sx * sy;
    const double d = (3.0 * txy) * txy;
    return std::sqrt(((a + b) - c) + d);
}

} // namespace

DeriveOp derive_op_from_name(const std::string &name) {
    if (name == "copy")
        return DeriveOp::Copy;
    if (name == "plane_von_mises" || name == "general_stress")
        return DeriveOp::PlaneVonMises;
    if (name == "plane_principal" || name == "membrane_principal")
        return DeriveOp::PlanePrincipal;
    if (name == "plane_principal_1")
        return DeriveOp::PlanePrincipal1;
    if (name == "plane_principal_2")
        return DeriveOp::PlanePrincipal2;
    if (name == "magnitude")
        return DeriveOp::Magnitude;
    if (name == "magnitude3")
        return DeriveOp::Magnitude3;
    if (name == "shell_decompose" || name == "decompose_shell")
        return DeriveOp::ShellDecompose;
    if (name == "shell_resultants" || name == "stress_resultants")
        return DeriveOp::ShellResultants;
    throw std::invalid_argument("unknown derivation op: " + name);
}

const char *derive_op_name(DeriveOp op) {
    switch (op) {
    case DeriveOp::Copy:
        return "copy";
    case DeriveOp::PlaneVonMises:
        return "plane_von_mises";
    case DeriveOp::PlanePrincipal:
        return "plane_principal";
    case DeriveOp::Magnitude:
        return "magnitude";
    case DeriveOp::ShellDecompose:
        return "shell_decompose";
    case DeriveOp::ShellResultants:
        return "shell_resultants";
    case DeriveOp::PlanePrincipal1:
        return "plane_principal_1";
    case DeriveOp::PlanePrincipal2:
        return "plane_principal_2";
    case DeriveOp::Magnitude3:
        return "magnitude3";
    }
    return "?";
}

std::size_t derive_op_n_args(DeriveOp op, std::size_t requested_args) {
    switch (op) {
    case DeriveOp::Copy:
        return 1;
    case DeriveOp::PlaneVonMises:
    case DeriveOp::PlanePrincipal:
    case DeriveOp::PlanePrincipal1:
    case DeriveOp::PlanePrincipal2:
    case DeriveOp::Magnitude3:
        return 3;
    case DeriveOp::Magnitude:
        return requested_args; // validated to 1..7
    case DeriveOp::ShellDecompose:
    case DeriveOp::ShellResultants:
        return 6;
    }
    return 0;
}

std::size_t derive_op_n_out(DeriveOp op) {
    switch (op) {
    case DeriveOp::Copy:
    case DeriveOp::PlaneVonMises:
    case DeriveOp::Magnitude:
    case DeriveOp::Magnitude3:
    case DeriveOp::PlanePrincipal1:
    case DeriveOp::PlanePrincipal2:
        return 1;
    case DeriveOp::PlanePrincipal:
        return 2;
    case DeriveOp::ShellDecompose:
        return 7;
    case DeriveOp::ShellResultants:
        return 6;
    }
    return 0;
}

void validate_derive(const DeriveSpec &spec, std::size_t in_ncomp, std::size_t out_ncomp, bool have_thickness) {
    const std::string name = derive_op_name(spec.op);
    if (spec.op == DeriveOp::Magnitude && (spec.args.empty() || spec.args.size() > 7))
        throw std::invalid_argument("magnitude takes 1..7 argument columns");
    if (spec.args.size() != derive_op_n_args(spec.op, spec.args.size()))
        throw std::invalid_argument(name + ": expected " + std::to_string(derive_op_n_args(spec.op, 0)) +
                                    " argument columns, got " + std::to_string(spec.args.size()));
    if (spec.out.size() != derive_op_n_out(spec.op))
        throw std::invalid_argument(name + ": expected " + std::to_string(derive_op_n_out(spec.op)) +
                                    " output columns (-1 to drop one), got " + std::to_string(spec.out.size()));
    for (int a : spec.args)
        if (a < 0 || static_cast<std::size_t>(a) >= in_ncomp)
            throw std::invalid_argument(name + ": argument column " + std::to_string(a) + " out of range");
    for (int o : spec.out)
        if (o < -1 || (o >= 0 && static_cast<std::size_t>(o) >= out_ncomp))
            throw std::invalid_argument(name + ": output column " + std::to_string(o) + " out of range");
    if (spec.op == DeriveOp::ShellResultants && !have_thickness)
        throw std::invalid_argument("shell_resultants needs a thickness array");
}

void apply_derive(const DeriveSpec &spec, const float *in, std::size_t in_ncomp, float *out, std::size_t out_ncomp,
                  std::size_t rows, const double *thickness, std::size_t thickness_rows) {
    validate_derive(spec, in_ncomp, out_ncomp, thickness != nullptr);
    const std::size_t na = spec.args.size();
    const std::size_t no = spec.out.size();
    int ai[7] = {0, 0, 0, 0, 0, 0, 0};
    int oi[7] = {-1, -1, -1, -1, -1, -1, -1};
    for (std::size_t k = 0; k < na; ++k)
        ai[k] = spec.args[k];
    for (std::size_t k = 0; k < no; ++k)
        oi[k] = spec.out[k];
    if (thickness_rows == 0)
        thickness_rows = 1;

    double x[7];
    double y[7];
    for (std::size_t r = 0; r < rows; ++r) {
        const float *src = in + r * in_ncomp;
        float *dst = out + r * out_ncomp;
        // Read every argument first, so an in-place op may overwrite its own inputs.
        for (std::size_t k = 0; k < na; ++k)
            x[k] = static_cast<double>(src[ai[k]]);

        switch (spec.op) {
        case DeriveOp::Copy:
            y[0] = x[0];
            break;
        case DeriveOp::PlaneVonMises:
            y[0] = von_mises(x[0], x[1], x[2]);
            break;
        case DeriveOp::PlanePrincipal:
        case DeriveOp::PlanePrincipal1:
        case DeriveOp::PlanePrincipal2: {
            // centre = 0.5 * (sig_x + sig_y); radius = np.sqrt((0.5 * (sig_x - sig_y)) ** 2 + tau_xy * tau_xy)
            const double centre = 0.5 * (x[0] + x[1]);
            const double h = 0.5 * (x[0] - x[1]);
            const double radius = std::sqrt(h * h + x[2] * x[2]);
            if (spec.op == DeriveOp::PlanePrincipal2) {
                y[0] = centre - radius;
            } else {
                y[0] = centre + radius;
                y[1] = centre - radius;
            }
            break;
        }
        case DeriveOp::Magnitude:
        case DeriveOp::Magnitude3: {
            // np.linalg.norm(values, axis=1): sqrt(add.reduce(x*x)), sequential for < 8 terms.
            double s = x[0] * x[0];
            for (std::size_t k = 1; k < na; ++k)
                s = s + x[k] * x[k];
            y[0] = std::sqrt(s);
            break;
        }
        case DeriveOp::ShellDecompose: {
            // decompose_shell(bottom, top): membrane = 0.5 * (top + bottom); bending = 0.5 * (top - bottom)
            const double m0 = 0.5 * (x[3] + x[0]);
            const double m1 = 0.5 * (x[4] + x[1]);
            const double m2 = 0.5 * (x[5] + x[2]);
            const double b0 = 0.5 * (x[3] - x[0]);
            const double b1 = 0.5 * (x[4] - x[1]);
            const double b2 = 0.5 * (x[5] - x[2]);
            y[0] = m0;
            y[1] = m1;
            y[2] = b0;
            y[3] = b1;
            y[4] = m2;
            y[5] = b2;
            y[6] = von_mises(m0, m1, m2); // from the DOUBLE membrane, as decompose_shell does
            break;
        }
        case DeriveOp::ShellResultants: {
            // stress_resultants(d, t): t2_over_6 = t * t / 6.0
            const double t = thickness[r / thickness_rows];
            const double t2 = (t * t) / 6.0;
            y[0] = x[0] * t;
            y[1] = x[4] * t;
            y[2] = x[1] * t;
            y[3] = x[5] * t2;
            y[4] = x[2] * t2;
            y[5] = x[3] * t2;
            break;
        }
        }
        for (std::size_t k = 0; k < no; ++k)
            if (oi[k] >= 0)
                dst[oi[k]] = static_cast<float>(y[k]);
    }
}

} // namespace adacpp::fea
