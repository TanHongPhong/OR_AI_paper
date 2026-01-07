#pragma once

#include <string>
#include <vector>
#include <sstream>
#include <algorithm>
#include <cctype>

inline void strip_utf8_bom(std::string& s) {
    if (s.size() >= 3 && (unsigned char)s[0] == 0xEF && (unsigned char)s[1] == 0xBB && (unsigned char)s[2] == 0xBF) {
        s.erase(0, 3);
    }
}

inline std::string trim(std::string s) {
    auto is_ws = [](unsigned char c) { return std::isspace(c) != 0; };
    while (!s.empty() && is_ws((unsigned char)s.front())) s.erase(s.begin());
    while (!s.empty() && is_ws((unsigned char)s.back())) s.pop_back();
    return s;
}

inline std::vector<std::string> split(const std::string& s, char delim) {
    std::vector<std::string> out;
    std::string item;
    std::istringstream iss(s);
    while (std::getline(iss, item, delim)) {
        out.push_back(trim(item));
    }
    return out;
}

inline std::string lower(std::string s) {
    for (auto& ch : s) ch = (char)std::tolower((unsigned char)ch);
    return s;
}

inline bool parse_bool(const std::string& s) {
    std::string t = lower(trim(s));
    return (t == "1" || t == "true" || t == "yes" || t == "y" || t == "t");
}
