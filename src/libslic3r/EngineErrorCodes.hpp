#ifndef slic3r_EngineErrorCodes_hpp_
#define slic3r_EngineErrorCodes_hpp_

// The engine's own error codes, and the one-line structured report the CLI
// prints for them.
//
// The code strings are the agent's (agent/error_codes.py, owner=engine),
// copied verbatim: the agent reads them back off stdout and must find each one
// in its registry. A contract test in the agent scans the list below, so keep
// it one X(CODE, "description") per line. The description is for logs and CLI
// users only; the text a person reads comes from the frontend's own copy.

#include <cmath>
#include <optional>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include "Exception.hpp"

namespace Slic3r {

// clang-format off
#define PHZ_ENGINE_ERROR_CODES(X) \
    X(INVALID_MODEL,                    "the input model file contains no objects") \
    X(SUPPORT_HEAD_TOO_WIDE,            "support head front diameter exceeds the pillar diameter") \
    X(SUPPORT_HEAD_PENETRATION_INVALID, "support head penetration exceeds the head width") \
    X(SUPPORT_ELEVATION_TOO_LOW,        "object elevation is lower than the support head needs") \
    X(SUPPORT_POINTS_REQUIRED,          "support generation was asked for with no support points") \
    X(SUPPORT_PAD_GAP_CONFLICT,         "pad object gap exceeds the support base safety distance") \
    X(MODEL_OUT_OF_BOUNDS,              "no object lies fully inside the print volume") \
    X(SUPPORT_POINTS_MODEL_MISMATCH,    "imported support points were made for a different model") \
    X(SUPPORT_POINT_SAMPLING_FAILED,    "the support point generator could not sample an island") \
    X(SHRINKAGE_COMPENSATION_INVALID,   "the object transform is singular and cannot be inverted") \
    X(PAD_CONFIG_INVALID,               "pad brim is too small for the pad wall slope and thickness") \
    X(EXPOSURE_TIME_OUT_OF_RANGE,       "exposure time is outside the printer profile bounds") \
    X(MODEL_MESH_UNSLICEABLE,           "no slice level falls inside the model") \
    X(UNPRINTABLE_OBJECT,               "a slice record is unprintable") \
    X(PAD_GENERATION_FAILED,            "no pad could be generated for this model") \
    X(SUPPORT_MESH_EXPORT_FAILED,       "the support mesh could not be written to its STL file")
// clang-format on

enum class EngineErrorCode {
#define PHZ_X(code, description) code,
    PHZ_ENGINE_ERROR_CODES(PHZ_X)
#undef PHZ_X
};

inline const char *engine_error_code_name(EngineErrorCode code)
{
    switch (code) {
#define PHZ_X(c, description) case EngineErrorCode::c: return #c;
        PHZ_ENGINE_ERROR_CODES(PHZ_X)
#undef PHZ_X
    }
    return "";
}

inline const char *engine_error_code_message(EngineErrorCode code)
{
    switch (code) {
#define PHZ_X(c, description) case EngineErrorCode::c: return description;
        PHZ_ENGINE_ERROR_CODES(PHZ_X)
#undef PHZ_X
    }
    return "";
}

// A failure as the engine knows it. `message` is the existing human-readable
// text, kept verbatim; `code` is empty for failures that carry no registered
// code yet, which then report the message alone, as before.
struct EngineError
{
    std::optional<EngineErrorCode>             code;
    std::string                                message;
    // Backend SLAConfig field names - the frontend's data-field attributes use
    // the same spelling, so nothing downstream renames them.
    std::vector<std::string>                   fields;
    std::vector<std::pair<std::string, double>> values;

    bool failed() const { return !message.empty(); }
};

inline EngineError make_engine_error(EngineErrorCode                             code,
                                     std::string                                 message,
                                     std::vector<std::string>                    fields = {},
                                     std::vector<std::pair<std::string, double>> values = {})
{
    EngineError err;
    err.code    = code;
    err.message = std::move(message);
    err.fields  = std::move(fields);
    err.values  = std::move(values);
    return err;
}

namespace detail {
inline void append_json_string(std::ostringstream &os, const std::string &s)
{
    os << '"';
    for (char ch : s) {
        switch (ch) {
        case '"':  os << "\\\""; break;
        case '\\': os << "\\\\"; break;
        case '\n': os << "\\n";  break;
        case '\r': os << "\\r";  break;
        case '\t': os << "\\t";  break;
        default:   os << ch;
        }
    }
    os << '"';
}
} // namespace detail

// "PHZ_ERROR {...}", or an empty string when there is no code to report.
// Non-finite values are dropped: JSON cannot carry them.
inline std::string engine_error_line(const EngineError &err)
{
    if (!err.code)
        return {};

    std::ostringstream os;
    os.precision(12);
    os << "PHZ_ERROR {\"code\":";
    detail::append_json_string(os, engine_error_code_name(*err.code));

    if (!err.fields.empty()) {
        os << ",\"fields\":[";
        for (size_t i = 0; i < err.fields.size(); ++i) {
            if (i) os << ',';
            detail::append_json_string(os, err.fields[i]);
        }
        os << ']';
    }

    bool opened = false;
    for (const auto &[key, value] : err.values) {
        if (!std::isfinite(value))
            continue;
        os << (opened ? "," : ",\"values\":{");
        opened = true;
        detail::append_json_string(os, key);
        os << ':' << value;
    }
    if (opened)
        os << '}';

    os << '}';
    return os.str();
}

// Lets the CLI find the code on an exception thrown deep inside slicing,
// without changing the exception's type for anyone who catches it by type.
class EngineErrorCarrier
{
public:
    virtual ~EngineErrorCarrier() = default;
    virtual const EngineError &engine_error() const = 0;
};

template<class BaseException>
class EngineCodedException : public BaseException, public EngineErrorCarrier
{
public:
    explicit EngineCodedException(EngineError err)
        : BaseException(err.message), m_err(std::move(err))
    {}

    const EngineError &engine_error() const override { return m_err; }

private:
    EngineError m_err;
};

using CodedSlicingError = EngineCodedException<SlicingError>;
using CodedRuntimeError = EngineCodedException<RuntimeError>;

} // namespace Slic3r

#endif // slic3r_EngineErrorCodes_hpp_
