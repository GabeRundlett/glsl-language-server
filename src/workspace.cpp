#include "workspace.hpp"

#include <utility>

Workspace::Workspace() = default;
Workspace::~Workspace() = default;

auto Workspace::is_initialized() const -> bool {
    return m_initialized;
};

void Workspace::set_initialized(bool new_value) {
    m_initialized = new_value;
};

auto Workspace::documents() -> std::map<std::string, std::string> & {
    return m_documents;
};

void Workspace::add_document(const std::string &key, std::string text) {
    m_documents[key] = std::move(text);
}

auto Workspace::remove_document(const std::string &key) -> bool {
    auto it = m_documents.find(key);
    if (it != m_documents.end()) {
        m_documents.erase(it);
        return true;
    }
    return false;
}

auto Workspace::change_document(const std::string &key, std::string text) -> bool {
    auto it = m_documents.find(key);
    if (it != m_documents.end()) {
        m_documents[key] = std::move(text);
        return true;
    }
    return false;
}
