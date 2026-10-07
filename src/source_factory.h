#pragma once
#include "iq_source.h"
#include <memory>
#include <string>
#include <vector>

namespace dedective {

// Create a source backend.  SourceType::Auto prefers an SDRplay device if one
// is available, otherwise HackRF.
//
// Returns nullptr if the requested backend was not compiled in.
std::unique_ptr<IqSource> create_source(SourceType type);

// Create and open a source.  For SourceType::Auto this tries each compiled-in
// backend in preference order and returns the first one that opens
// successfully.  On failure returns nullptr and fills `error` with per-backend
// diagnostics.
std::unique_ptr<IqSource> open_source(SourceType type, std::string& error);

// All backends the application knows about, in display order.
std::vector<SourceType> known_source_types();

// Whether a backend was compiled into this build.
bool source_type_available(SourceType type);

// Names of the backends compiled into this build, in preference order.
std::vector<std::string> available_sources();

} // namespace dedective
