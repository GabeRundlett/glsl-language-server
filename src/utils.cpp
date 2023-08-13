#include "utils.hpp"

#include <regex>
#include <cstdio>
#include <filesystem>
#include <fstream>

namespace fs = std::filesystem;

auto split_string(const std::string &string_to_split, const std::string &pattern) -> std::vector<std::string> {
    std::vector<std::string> result;

    const std::regex re(pattern);
    std::sregex_token_iterator iter(string_to_split.begin(), string_to_split.end(), re, -1);

    for (std::sregex_token_iterator const end; iter != end; ++iter) {
        result.push_back(iter->str());
    }

    return result;
}

auto trim_right(const std::string &s, const std::string &delimiters = " \f\n\r\t\v") -> std::string {
    return s.substr(0, s.find_last_not_of(delimiters) + 1);
}

auto trim_left(const std::string &s, const std::string &delimiters = " \f\n\r\t\v") -> std::string {
    return s.substr(s.find_first_not_of(delimiters));
}

auto trim(const std::string &s, const std::string &delimiters = " \f\n\r\t\v") -> std::string {
    return trim_left(trim_right(s, delimiters), delimiters);
}

/// Returns the byte offset for the given character on the given line.
// FIXME: use UTF-16 offsets
// https://fasterthanli.me/articles/the-bottom-emoji-breaks-rust-analyzer
auto find_position_offset(const char *text, int line, int character) -> int {
    int offset = 0;
    while (line > 0) {
        while ((text[offset] != 0) && text[offset] != '\n') {
            offset += 1;
        }
        offset += static_cast<int>(text[offset] == '\n');
        line -= 1;
    }

    while (character > 0 && (text[offset] != 0) && text[offset] != '\n') {
        offset += 1;
        character -= 1;
    }

    return offset;
}

/// Given a byte offset into a file, returns the corresponding line and column.
// FIXME: use UTF-16 offsets
// https://fasterthanli.me/articles/the-bottom-emoji-breaks-rust-analyzer
auto find_source_location(const char *text, int offset) -> SourceFileLocation {
    SourceFileLocation location{0, 0};
    const char *p = text;
    const char *end = text + offset;
    while ((*p != 0) && p < end) {
        if (*p == '\n') {
            location.line += 1;
            location.character = 0;
        } else {
            location.character += 1;
        }
        p++;
    }
    return location;
}

/// Returns `true` if the character may start an identifier.
auto is_identifier_start_char(char c) -> bool {
    return ('a' <= c && c <= 'z') || ('A' <= c && c <= 'Z') || c == '_';
}

/// Returns `true` if the character may be part of an identifier.
auto is_identifier_char(char c) -> bool {
    return is_identifier_start_char(c) || ('0' <= c && c <= '9');
}

/// Returns the offset in `text` where the last word started.
auto get_last_word_start(const char *text, int offset) -> int {
    int start = offset;
    while (start > 0 && is_identifier_char(text[start - 1])) {
        start -= 1;
    }

    // If `text` was `123abc` and `offset` pointed at `b`, start would point at `1`.
    // We want to point to `a`, so advance past any characters that are not a
    // valid start of an identifier.
    while (start < offset && !is_identifier_start_char(text[start])) {
        start += 1;
    }

    return start;
}

auto get_word_end(const char *text, int start) -> int {
    int end = start;
    while ((text[end] != 0) && is_identifier_char(text[end])) {
        end++;
    }
    return end;
}

auto read_file_to_string(const char *path) -> std::optional<std::string> {
    auto ifs = std::ifstream{path};
    if (!ifs.good()) {
        return std::nullopt;
    }
    auto str = std::string{};
    ifs.seekg(0, std::ios::end);
    str.reserve(static_cast<size_t>(ifs.tellg()));
    ifs.seekg(0, std::ios::beg);
    str.assign(std::istreambuf_iterator<char>(ifs), std::istreambuf_iterator<char>());
    return {str};
}

auto make_path_uri(const std::string &path) -> std::string {
    return "file://" + fs::absolute(path).string();
}

auto strip_prefix(const char *prefix, const char *haystack) -> const char * {
    while (*prefix != 0) {
        if (*prefix != *haystack) {
            return nullptr;
        }
        prefix++;
        haystack++;
    }
    return haystack;
}
