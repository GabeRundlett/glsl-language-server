#include "messagebuffer.hpp"

MessageBuffer::MessageBuffer() = default;
MessageBuffer::~MessageBuffer() = default;

void MessageBuffer::handle_char(char c) {
    m_raw_message += c;

    auto new_header = try_parse_header(m_raw_message);
    // Check whether we were actually able to parse a header.
    // If so, add it to our known headers.
    // We'll also reset our string then.
    if (!std::get<0>(new_header).empty()) {
        m_headers[std::get<0>(new_header)] = std::get<1>(new_header);
        m_raw_message.clear();
    }

    // A sole \r\n is the separator between the header block and the body block
    // but we don't need it.
    if (m_raw_message == "\r\n") {
        m_raw_message.clear();
        m_is_header_done = true;
    }

    if (m_is_header_done) {
        // Now that we know that we're in the body, we just have to count until
        // we reach the length of the body as provided in the Content-Length
        // header.
        auto content_length = std::stoul(m_headers["Content-Length"]);
        if (m_raw_message.length() == content_length) {
            m_body = json::parse(m_raw_message);
        }
    }
}

void MessageBuffer::handle_string(const std::string &s) {
    m_raw_message += s;

    auto new_header = try_parse_header(m_raw_message);
    // Check whether we were actually able to parse a header.
    // If so, add it to our known headers.
    // We'll also reset our string then.
    if (!std::get<0>(new_header).empty()) {
        m_headers[std::get<0>(new_header)] = std::get<1>(new_header);
        m_raw_message.clear();
    }

    // A sole \r\n is the separator between the header block and the body block
    // but we don't need it.
    if (m_raw_message == "\r\n") {
        m_raw_message.clear();
        m_is_header_done = true;
    }

    if (m_is_header_done) {
        // Now that we know that we're in the body, we just have to count until
        // we reach the length of the body as provided in the Content-Length
        // header.
        auto content_length = std::stoul(m_headers["Content-Length"]);
        if (m_raw_message.length() == content_length) {
            m_body = json::parse(m_raw_message);
        }
    }
}

auto MessageBuffer::headers() const -> const std::map<std::string, std::string> & {
    return m_headers;
}

auto MessageBuffer::body() const -> const json & {
    return m_body;
}

auto MessageBuffer::raw() const -> const std::string & {
    return m_raw_message;
}

auto MessageBuffer::message_completed() -> bool {
    if (m_is_header_done && !m_body.empty()) {
        return true;
    }
    return false;
}

auto MessageBuffer::try_parse_header(std::string & /*message*/) const -> std::tuple<std::string, std::string> {
    auto eol_pos = m_raw_message.find("\r\n");
    if (eol_pos != std::string::npos) {
        std::string const header_string = m_raw_message.substr(0, eol_pos);
        auto delim_pos = header_string.find(':');
        if (delim_pos != std::string::npos) {
            std::string const header_name = header_string.substr(0, delim_pos);
            std::string const header_value = header_string.substr(delim_pos + 1);
            return std::make_tuple(header_name, header_value);
        }
    }
    return std::make_tuple(std::string{}, std::string{});
}

void MessageBuffer::clear() {
    m_raw_message.clear();
    m_headers.clear();
    m_body.clear();
    m_is_header_done = false;
}
