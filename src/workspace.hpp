#pragma once

#include <map>
#include <string>
#include <utility>

class Workspace {
  public:
    Workspace();
    virtual ~Workspace();

    bool is_initialized() const;
    void set_initialized(bool new_value);

    std::map<std::string, std::string> &documents();
    void add_document(const std::string &key, std::string text);
    bool remove_document(const std::string &key);
    bool change_document(const std::string &key, std::string text);

  private:
    bool m_initialized = false;
    std::map<std::string, std::string> m_documents;
};
