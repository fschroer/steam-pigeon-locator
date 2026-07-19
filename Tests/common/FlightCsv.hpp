// ---------------------------------------------------------------------------
// FlightCsv — shared reader for archived flight CSVs (host tests only).
//
// One parser for every harness, so the schema is understood in exactly one
// place.  Handles BOTH forms:
//
//   (a) The on-device export (UserInteraction::ExportData): a block of
//       "Key: value" metadata lines, then a NAMED column header, then samples.
//         Flight time: 2026-07-18 01:31:44
//         Launch detect time: 500
//         ...
//         time_ms,raw_baro_agl_m,fused_agl_m,...,accel_x_g,...,q_z
//
//   (b) The original positional 7-column harness form, header optional:
//         t_ms,raw_agl,raw_vel,raw_valid,fused_agl,fused_vspeed,accel_mag
//
// Columns are matched BY NAME, so reordered/added/removed columns are
// tolerated.  The export has already grown once (23 columns at
// ARCHIVE_VERSION 5, adding tilt + quaternion) and will grow again — a
// positional reader silently mis-assigns every field when that happens, which
// is what it did before this existed.
//
// Values are stored as double: latitude/longitude carry ~7 significant digits
// before the decimal point and would lose metres of precision as float.
// ---------------------------------------------------------------------------
#pragma once

#include <cmath>
#include <cstdio>
#include <fstream>
#include <initializer_list>
#include <map>
#include <sstream>
#include <string>
#include <vector>

namespace flightcsv {

struct Table {
    std::map<std::string, int>       col;         // column name -> index
    std::vector<std::vector<double>> rows;
    std::map<std::string, std::string> meta;      // "Launch detect time" -> "500"
    bool positional = false;                      // legacy headerless form

    /** Index of the first name that is present, or -1. */
    int find(std::initializer_list<const char*> names) const {
        for (const char* n : names) {
            auto it = col.find(n);
            if (it != col.end()) return it->second;
        }
        return -1;
    }

    bool has(std::initializer_list<const char*> names) const { return find(names) >= 0; }

    /** Cell value, or [dflt] when the column or cell is absent. */
    double get(size_t row, int c, double dflt = 0.0) const {
        if (c < 0 || row >= rows.size()) return dflt;
        const std::vector<double>& r = rows[row];
        if (c >= static_cast<int>(r.size())) return dflt;
        const double v = r[c];
        return std::isnan(v) ? dflt : v;
    }

    size_t size() const { return rows.size(); }
};

namespace detail {

inline std::vector<std::string> split(const std::string& line) {
    std::vector<std::string> f;
    std::stringstream ss(line);
    std::string tok;
    while (std::getline(ss, tok, ',')) {
        while (!tok.empty() && (tok.back() == '\r' || tok.back() == ' ' || tok.back() == '\t'))
            tok.pop_back();
        const size_t b = tok.find_first_not_of(" \t");
        f.push_back(b == std::string::npos ? std::string() : tok.substr(b));
    }
    return f;
}

/** NaN when the field is empty or not a number — callers treat that as absent. */
inline double toNum(const std::string& s) {
    if (s.empty()) return std::nan("");
    try { return std::stod(s); } catch (...) { return std::nan(""); }
}

} // namespace detail

/**
 * Load [path] into [t].  Returns false if the file cannot be opened or contains
 * no sample rows.
 */
inline bool load(const char* path, Table& t) {
    std::ifstream in(path);
    if (!in) { std::printf("FlightCsv: cannot open %s\n", path); return false; }

    std::string line;
    std::vector<std::string> pending;   // first data row when the file has no header

    // Metadata lines are "Key: value" and contain no comma, so the first line
    // WITH a comma is either the column header or (legacy) the first data row.
    while (std::getline(in, line)) {
        if (line.empty()) continue;
        if (line.find(',') == std::string::npos) {
            const size_t colon = line.find(':');
            if (colon != std::string::npos && colon + 1 < line.size()) {
                std::string k = line.substr(0, colon);
                std::string v = line.substr(colon + 1);
                const size_t b = v.find_first_not_of(" \t");
                t.meta[k] = (b == std::string::npos) ? std::string() : v.substr(b);
            }
            continue;
        }
        std::vector<std::string> f = detail::split(line);
        if (!f.empty() && !std::isnan(detail::toNum(f[0]))) {
            t.positional = true;          // numeric first cell -> headerless data
            pending = f;
        } else {
            for (size_t i = 0; i < f.size(); ++i) t.col[f[i]] = static_cast<int>(i);
        }
        break;
    }

    auto emit = [&](const std::vector<std::string>& f) {
        std::vector<double> r;
        r.reserve(f.size());
        for (const std::string& s : f) r.push_back(detail::toNum(s));
        t.rows.push_back(std::move(r));
    };

    if (!pending.empty()) emit(pending);
    while (std::getline(in, line)) {
        if (line.empty()) continue;
        if (line.find(',') == std::string::npos) continue;   // stray metadata
        emit(detail::split(line));
    }

    if (t.rows.empty()) { std::printf("FlightCsv: no sample rows in %s\n", path); return false; }
    return true;
}

/** Metadata value as a number, or [dflt] when absent/unparseable. */
inline double metaNum(const Table& t, const char* key, double dflt = -1.0) {
    auto it = t.meta.find(key);
    if (it == t.meta.end()) return dflt;
    const double v = detail::toNum(it->second);
    return std::isnan(v) ? dflt : v;
}

} // namespace flightcsv
