#include "source_factory.h"

#include <cstring>

#ifdef DEDECTIVE_HAVE_HACKRF
#include "hackrf_source.h"
#endif
#ifdef DEDECTIVE_HAVE_SDRPLAY
#include "sdrplay_source.h"
#endif
#ifdef DEDECTIVE_HAVE_PLUTO
#include "pluto_source.h"
#endif

namespace dedective {

const char* source_type_name(SourceType type) {
    switch (type) {
    case SourceType::HackRF:  return "HackRF";
    case SourceType::SDRplay: return "SDRplay";
    case SourceType::Pluto:   return "PlutoSDR";
    case SourceType::Auto:    break;
    }
    return "Auto";
}

bool parse_source_type(const char* text, SourceType& out) {
    if (!text) return false;
    if (std::strcmp(text, "hackrf") == 0 || std::strcmp(text, "HackRF") == 0) {
        out = SourceType::HackRF;
        return true;
    }
    if (std::strcmp(text, "sdrplay") == 0 || std::strcmp(text, "sdr") == 0 ||
        std::strcmp(text, "SDRplay") == 0) {
        out = SourceType::SDRplay;
        return true;
    }
    if (std::strcmp(text, "pluto") == 0 || std::strcmp(text, "plutosdr") == 0 ||
        std::strcmp(text, "PlutoSDR") == 0) {
        out = SourceType::Pluto;
        return true;
    }
    if (std::strcmp(text, "auto") == 0 || std::strcmp(text, "Auto") == 0) {
        out = SourceType::Auto;
        return true;
    }
    return false;
}

std::unique_ptr<IqSource> create_source(SourceType type) {
    if (type == SourceType::SDRplay) {
#ifdef DEDECTIVE_HAVE_SDRPLAY
        return std::make_unique<SdrplaySource>();
#else
        return nullptr;
#endif
    }
    if (type == SourceType::Pluto) {
#ifdef DEDECTIVE_HAVE_PLUTO
        return std::make_unique<PlutoSource>();
#else
        return nullptr;
#endif
    }
    if (type == SourceType::HackRF) {
#ifdef DEDECTIVE_HAVE_HACKRF
        return std::make_unique<HackrfSource>();
#else
        return nullptr;
#endif
    }

    // Auto: prefer SDRplay, then Pluto, then HackRF.
#ifdef DEDECTIVE_HAVE_SDRPLAY
    return std::make_unique<SdrplaySource>();
#elif defined(DEDECTIVE_HAVE_PLUTO)
    return std::make_unique<PlutoSource>();
#elif defined(DEDECTIVE_HAVE_HACKRF)
    return std::make_unique<HackrfSource>();
#else
    return nullptr;
#endif
}

std::unique_ptr<IqSource> open_source(SourceType type, std::string& error) {
    error.clear();

    std::vector<SourceType> order;
    if (type == SourceType::Auto) {
#ifdef DEDECTIVE_HAVE_SDRPLAY
        order.push_back(SourceType::SDRplay);
#endif
#ifdef DEDECTIVE_HAVE_PLUTO
        order.push_back(SourceType::Pluto);
#endif
#ifdef DEDECTIVE_HAVE_HACKRF
        order.push_back(SourceType::HackRF);
#endif
    } else {
        order.push_back(type);
    }

    for (SourceType t : order) {
        std::unique_ptr<IqSource> s = create_source(t);
        if (!s) {
            if (!error.empty()) error += "\n";
            error += std::string(source_type_name(t)) + ": not built in";
            continue;
        }
        if (s->open()) return s;

        if (!error.empty()) error += "\n";
        error += std::string(s->name()) + ": " + s->last_error();
        s->close();
    }

    if (error.empty()) error = "No SDR backend available";
    return nullptr;
}

std::vector<SourceType> known_source_types() {
    return { SourceType::SDRplay, SourceType::HackRF, SourceType::Pluto };
}

bool source_type_available(SourceType type) {
    switch (type) {
#ifdef DEDECTIVE_HAVE_SDRPLAY
    case SourceType::SDRplay: return true;
#endif
#ifdef DEDECTIVE_HAVE_HACKRF
    case SourceType::HackRF: return true;
#endif
#ifdef DEDECTIVE_HAVE_PLUTO
    case SourceType::Pluto: return true;
#endif
    default: return false;
    }
}

std::vector<std::string> available_sources() {
    std::vector<std::string> out;
#ifdef DEDECTIVE_HAVE_SDRPLAY
    out.emplace_back("SDRplay");
#endif
#ifdef DEDECTIVE_HAVE_PLUTO
    out.emplace_back("PlutoSDR");
#endif
#ifdef DEDECTIVE_HAVE_HACKRF
    out.emplace_back("HackRF");
#endif
    return out;
}

} // namespace dedective
