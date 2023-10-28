#include "LibLsp/JsonRpc/Condition.h"
#include "LibLsp/lsp/general/exit.h"
#include "LibLsp/lsp/general/initialize.h"
#include "LibLsp/lsp/general/shutdown.h"
#include "LibLsp/lsp/ProtocolJsonHandler.h"
#include "LibLsp/lsp/AbsolutePath.h"

#include "LibLsp/lsp/textDocument/declaration_definition.h"
#include "LibLsp/lsp/textDocument/signature_help.h"
#include "LibLsp/lsp/textDocument/resolveCompletionItem.h"
#include "LibLsp/lsp/textDocument/typeHierarchy.h"
#include "LibLsp/lsp/textDocument/document_symbol.h"
#include "LibLsp/lsp/textDocument/did_open.h"
#include "LibLsp/lsp/textDocument/did_change.h"
#include "LibLsp/lsp/textDocument/completion.h"
#include "LibLsp/lsp/textDocument/hover.h"
#include "LibLsp/lsp/textDocument/publishDiagnostics.h"

#include <network/uri.hpp>

#include "LibLsp/JsonRpc/Endpoint.h"
#include "LibLsp/JsonRpc/stream.h"
#include "LibLsp/JsonRpc/TcpServer.h"
#include "LibLsp/lsp/workspace/execute_command.h"

#include <boost/filesystem.hpp>
#include <boost/asio.hpp>
#include <iostream>
#include <thread>

#include <fmt/format.h>
#include <fmt/ostream.h>
#include <nlohmann/json.hpp>
#include <mongoose.h>
#include <glslang/Public/ResourceLimits.h>
#include <glslang/Public/ShaderLang.h>
#include <glslang/MachineIndependent/Initialize.h>

#include <cstdint>
#include <filesystem>
#include <fstream>
#include <optional>
#include <regex>
#include <string>
#include <utility>
#include <vector>
#include <map>
#include <span>

#include "messagebuffer.hpp"
#include "workspace.hpp"
#include "utils.hpp"
#include "symbols.hpp"
#include "includer.hpp"

using json = nlohmann::json;
namespace fs = std::filesystem;

#define USE_LOGGING false

/// By default we target the most recent graphics APIs to be maximally permissive.
struct TargetVersions {
    // The target API (eg, Vulkan, OpenGL).
    glslang::EShClient client_api = glslang::EShClientVulkan;
    glslang::EShTargetClientVersion client_api_version = glslang::EShTargetVulkan_1_3;

    // The target SPIR-V version
    glslang::EShTargetLanguageVersion spv_version = glslang::EShTargetSpv_1_6;

    // Options for glslValidator
    EShMessages options = EShMessages(0);
};

struct AppState {
    Workspace workspace;
    bool verbose{};
    bool use_logfile{};
    std::ofstream logfile_stream;
    TargetVersions target{};
};

auto find_language(const std::string &name) -> EShLanguage {
    // As well as the one used in glslang, there are a number of different conventions used for naming GLSL shaders.
    // This function attempts to support the most common ones, by checking if the filename ends with one of a list of known extensions.
    // If a ".glsl" extension is found initially, it is first removed to allow for e.g. vs.glsl/vert.glsl naming.
    auto path = fs::path(name);
    auto ext = path.extension().string();
    if (ext == ".glsl") {
        ext = path.replace_extension().string();
    }
    if (ext.ends_with("vert") || ext.ends_with("vs") || ext.ends_with("vsh")) {
        return EShLangVertex;
    } else if (ext.ends_with("tesc")) {
        return EShLangTessControl;
    } else if (ext.ends_with("tese")) {
        return EShLangTessEvaluation;
    } else if (ext.ends_with("geom") || ext.ends_with("gs") || ext.ends_with("gsh")) {
        return EShLangGeometry;
    } else if (ext.ends_with("frag") || ext.ends_with("fs") || ext.ends_with("fsh")) {
        return EShLangFragment;
    } else if (ext.ends_with("comp") || true) {
        return EShLangCompute;
    }
    throw std::invalid_argument("Unknown file extension!");
    return {};
}

auto get_diagnostics(std::string uri, const std::string &content,
                     AppState &appstate) -> std::vector<lsDiagnostic> {
#if _WIN32
    auto *temp_out = fopen("nul:", "a");
    auto stdout_dupfd = _dup(1);
    _dup2(_fileno(temp_out), 1);
#else
    FILE const fp_old = *stdout;
    *stdout = *fopen("/dev/null", "w");
#endif

    const auto &document = std::move(uri);
    auto lang = find_language(document);

    glslang::TShader shader(lang);

    auto target = appstate.target;

    if ((target.options & EShMsgSpvRules) != 0u) {
        if ((target.options & EShMsgVulkanRules) != 0u) {
            shader.setEnvInput((target.options & EShMsgReadHlsl) != 0u ? glslang::EShSourceHlsl : glslang::EShSourceGlsl, lang, glslang::EShClientVulkan, 100);
            shader.setEnvClient(glslang::EShClientVulkan, target.client_api_version);
            shader.setEnvTarget(glslang::EShTargetSpv, target.spv_version);
        } else {
            shader.setEnvInput((target.options & EShMsgReadHlsl) != 0u ? glslang::EShSourceHlsl : glslang::EShSourceGlsl, lang, glslang::EShClientOpenGL, 100);
            shader.setEnvClient(glslang::EShClientOpenGL, target.client_api_version);
            shader.setEnvTarget(glslang::EshTargetSpv, target.spv_version);
        }
    }

    const auto *shader_cstring = content.c_str();
    const auto *shader_name = document.c_str();
    shader.setStringsWithLengthsAndNames(&shader_cstring, nullptr, &shader_name, 1);

    FileIncluder includer{&appstate.workspace};

    std::string preamble;
    preamble += "#extension GL_GOOGLE_include_directive : enable\n";
    preamble += "#extension GL_KHR_memory_scope_semantics : enable\n";
    shader.setPreamble(preamble.c_str());

    TBuiltInResource const Resources = *GetDefaultResources();
    auto const messages = static_cast<EShMessages>(EShMsgCascadingErrors | target.options);
    shader.parse(&Resources, 450, false, messages, includer);
    std::string debug_log = shader.getInfoLog();
#if _WIN32
    fflush(stdout);
    fclose(temp_out);
    _dup2(stdout_dupfd, 1);
    _close(stdout_dupfd);
#else
    *stdout = fp_old;
#endif

    if (appstate.use_logfile && appstate.verbose) {
        fmt::print(appstate.logfile_stream, "Diagnostics raw output: {}\n", debug_log);
    }

    std::regex const re("([A-Z]*): (.*):(\\d*): (.*)");
    std::smatch matches;
    auto error_lines = split_string(debug_log, "\n");
    auto content_lines = split_string(content, "\n");

    std::vector<lsDiagnostic> diagnostics;
    for (const auto &error_line : error_lines) {
        std::regex_search(error_line, matches, re);
        if (matches.size() == 5) {
            std::string const file = matches[2];
            if (file != document) {
                continue; // message is for another file
            }

            lsDiagnostic diagnostic;
            std::string severity = matches[1];
            lsDiagnosticSeverity severity_no = lsDiagnosticSeverity::Information;
            if (severity == "ERROR") {
                severity_no = lsDiagnosticSeverity::Error;
            } else if (severity == "WARNING") {
                severity_no = lsDiagnosticSeverity::Warning;
            }

            std::string const message = trim(matches[4], " ");

            // -1 because lines are 0-indexed as per LSP specification.
            int line_no = std::stoi(matches[3]) - 1;
            std::string const source_line = content_lines[line_no];

            int start_char = -1;
            int end_char = -1;

            // If this is an undeclared identifier, we can find the exact
            // position of the broken identifier.
            std::smatch message_matches;
            std::regex const msg_re("'(.*)' : (.*)");
            std::regex_search(message, message_matches, msg_re);
            if (message_matches.size() == 3) {
                std::string const identifier = message_matches[1];
                auto identifier_length = message_matches[1].length();
                auto source_pos = source_line.find(identifier);
                start_char = static_cast<int>(source_pos);
                end_char = static_cast<int>(source_pos) + static_cast<int>(identifier_length);
            } else {
                // If we can't find a precise position, we'll just use the whole line.
                start_char = 0;
                end_char = static_cast<int>(source_line.length());
            }

            auto start = lsPosition(line_no, start_char);
            auto end = lsPosition(line_no, end_char);
            diagnostic.range = lsRange(start, end);
            diagnostic.severity = severity_no;
            diagnostic.source = "glslang";
            diagnostic.message = message;
            diagnostics.push_back(diagnostic);
        }
    }
    // if (appstate.use_logfile && appstate.verbose && !diagnostics.empty()) {
    //     fmt::print(appstate.logfile_stream, "Sending diagnostics: {}\n", diagnostics.dump(4));
    // }
    appstate.logfile_stream.flush();
    return diagnostics;
}

auto get_symbols(const std::string &uri, AppState &appstate) -> SymbolMap {
    auto language = find_language(uri);

    // use the highest known version so that we get as many symbols as possible
    int const version = 460;
    // same thing here: use compatibility profile for more symbols
    EProfile const profile = ECompatibilityProfile;

    glslang::SpvVersion spv_version{};
    spv_version.spv = appstate.target.spv_version;
    spv_version.vulkanRelaxed = true; // be maximally permissive, allowing certain OpenGL in Vulkan

    glslang::TPoolAllocator pool{};
    glslang::SetThreadPoolAllocator(&pool);
    pool.push();

    const TBuiltInResource &resources = *GetDefaultResources();
    glslang::TBuiltIns builtins{};
    builtins.initialize(version, profile, spv_version);
    builtins.initialize(resources, version, profile, spv_version, language);

    // TODO: cache builtin symbols between runs.
    SymbolMap symbols;
    add_builtin_types(symbols);
    extract_symbols(builtins.getCommonString().c_str(), symbols);
    extract_symbols(builtins.getStageString(language).c_str(), symbols);

    extract_symbols(appstate.workspace.documents()[uri].c_str(), symbols, uri.c_str());

    glslang::GetThreadPoolAllocator().pop();
    glslang::SetThreadPoolAllocator(nullptr);

    return symbols;
}

void find_completions(const SymbolMap &symbols, const std::string & /*prefix*/, CompletionList &out) {
    for (const auto &entry : symbols) {
        const auto &name = entry.first;
        const auto &symbol = entry.second;

        // auto item = json{
        //     {"label", name},
        //     {"kind", symbol.kind == Symbol::Unknown ? json(nullptr) : json(symbol.kind)},
        //     {"detail", symbol.details},
        // };

        auto item = lsCompletionItem{
            .label = name,
            .detail = symbol.details,
        };

        switch (symbol.kind) {
        case Symbol::Function: item.kind = lsCompletionItemKind::Function; break;
        case Symbol::Type: item.kind = lsCompletionItemKind::Class; break;
        case Symbol::Constant: item.kind = lsCompletionItemKind::Constant; break;
        }

        out.items.push_back(item);
    }
}

auto get_completions(const std::string &uri, int line, int character, AppState &appstate) -> CompletionList {
    const std::string &document = appstate.workspace.documents()[uri];
    int const offset = find_position_offset(document.c_str(), line, character);
    int const word_start = get_last_word_start(document.c_str(), offset);
    int const length = offset - word_start;

    if (length <= 0) {
        // no word under the cursor.
        return {};
    }

    auto name = document.substr(word_start, length);

    auto result = CompletionList{};

    // std::vector<json> matches;
    auto symbols = get_symbols(uri, appstate);
    find_completions(symbols, name, result);

    // return matches;
    return result;
}

auto get_word_under_cursor(
    const std::string &uri,
    int line, int character,
    AppState &appstate) -> std::optional<std::string> {
    const std::string &document = appstate.workspace.documents()[uri];
    int const offset = find_position_offset(document.c_str(), line, character);
    int const word_start = get_last_word_start(document.c_str(), offset);
    int const word_end = get_word_end(document.c_str(), word_start);
    int const length = word_end - word_start;

    if (length <= 0) {
        // no word under the cursor.
        return std::nullopt;
    }

    return document.substr(word_start, length);
}

auto get_hover_info(const std::string &uri, int line, int character, AppState &appstate) -> TextDocumentHover::Result {
    auto word = get_word_under_cursor(uri, line, character, appstate);
    if (!word) {
        return {};
    }

    auto symbols = get_symbols(uri, appstate);
    auto symbol = symbols.find(*word);
    if (symbol == symbols.end()) {
        return {};
    }

    auto result = TextDocumentHover::Result{};
    auto val = lsMarkedString{.language = std::string{"glsl"}, .value = symbol->second.details};
    result.contents.first = std::vector{std::pair<std::optional<std::string>, std::optional<lsMarkedString>>{{}, val}};
    // return json{{"contents", {{"language", "glsl"}, {"value", symbol->second.details}}}};
    return result;
}

auto get_definition(const std::string &uri, int line, int character, AppState &appstate) -> LocationListEither::Either {
    auto word = get_word_under_cursor(uri, line, character, appstate);
    if (!word) {
        return {};
    }

    auto symbols = get_symbols(uri, appstate);
    auto symbol_iter = symbols.find(*word);
    if (symbol_iter == symbols.end()) {
        return {};
    }
    auto symbol = symbol_iter->second;
    if (symbol.location.uri == nullptr) {
        return {};
    }

    const std::string &text = appstate.workspace.documents()[symbol.location.uri];
    auto position = find_source_location(text.c_str(), symbol.location.offset);
    int const length = static_cast<int>(word->size());

    auto result = LocationListEither::Either{};

    auto start = lsPosition(position.line, position.character);
    auto end = lsPosition(position.line, position.character + length);
    result.first = std::vector{lsLocation(AbsolutePath(symbol.location.uri), lsRange(start, end))};

    return result;
}

using namespace boost::asio::ip;
using namespace std;
class DummyLog : public lsp::Log {
  public:
    void log(Level level, std::wstring &&msg) {
#if USE_LOGGING
        std::wcout << msg << std::endl;
#endif
    };
    void log(Level level, const std::wstring &msg) {
#if USE_LOGGING
        std::wcout << msg << std::endl;
#endif
    };
    void log(Level level, std::string &&msg) {
#if USE_LOGGING
        std::cout << msg << std::endl;
#endif
    };
    void log(Level level, const std::string &msg) {
#if USE_LOGGING
        std::cout << msg << std::endl;
#endif
    };
};

struct LanguageServer {
    virtual ~LanguageServer() {}
    virtual RemoteEndPoint &get_endpoint() = 0;
    virtual void start() = 0;
    virtual void stop() = 0;
};

struct StdioServer final : LanguageServer {
    StdioServer(const StdioServer &) = delete;
    StdioServer(StdioServer &&) = default;
    StdioServer &operator=(const StdioServer &) = delete;
    StdioServer &operator=(StdioServer &&) = default;
    StdioServer(const std::shared_ptr<MessageJsonHandler> &json_handler, const std::shared_ptr<Endpoint> &localEndPoint, lsp::Log &_log) : remote_end_point_(json_handler, localEndPoint, _log) {}
    struct ostream : lsp::base_ostream<std::ostream> {
        explicit ostream(std::ostream &_t) : base_ostream<std::ostream>(_t) {}
        std::string what() override { return {}; }
    };
    struct istream : lsp::base_istream<std::istream> {
        explicit istream(std::istream &_t) : base_istream<std::istream>(_t) {}
        std::string what() override { return {}; }
    };
    std::shared_ptr<ostream> output = std::make_shared<ostream>(std::cout);
    std::shared_ptr<istream> input = std::make_shared<istream>(std::cin);
    RemoteEndPoint remote_end_point_;
    virtual void start() override { remote_end_point_.startProcessingMessages(input, output); }
    virtual void stop() override {}
    virtual RemoteEndPoint &get_endpoint() override { return remote_end_point_; }
};

struct TcpServer final : LanguageServer {
    TcpServer(const TcpServer &) = delete;
    TcpServer(TcpServer &&) = default;
    TcpServer &operator=(const TcpServer &) = delete;
    TcpServer &operator=(TcpServer &&) = default;
    TcpServer(const std::shared_ptr<MessageJsonHandler> &json_handler, const std::shared_ptr<Endpoint> &localEndPoint, lsp::Log &_log, const std::string &port) : server("127.0.0.1", port, json_handler, localEndPoint, _log) {}
    lsp::TcpServer server;
    virtual void start() override {
        std::thread([&]() { server.run(); }).detach();
    }
    virtual void stop() override { server.stop(); }
    virtual RemoteEndPoint &get_endpoint() override { return server.point; }
};

auto main(int argc, char *argv[]) -> int {
    using namespace std::literals;

    enum class CommMode {
        NONE,
        SOCKET,
        STDIO,
    };

    auto comm_mode = CommMode::NONE;

    bool verbose = false;
    bool version = false;
    std::string logfile;

    std::string client_api;
    std::string spirv_version;

    std::string port = "7125";

    enum class DependentOption {
        NONE,
        SOCKET,
        PORT,
    };
    auto current_dependent_option = DependentOption::NONE;

    auto is_number = [](std::string_view s) -> bool {
        auto it = s.begin();
        while (it != s.end() && std::isdigit(*it))
            ++it;
        return !s.empty() && it == s.end();
    };

    auto args = std::span<char *>{argv, static_cast<size_t>(argc)};

    if (args.size() >= 2) {
        if (args[1] == "--help") {
            fmt::print("TODO: HELP MENU\n");
            return 0;
        } else if (args[1] == "--version") {
            fmt::print("grsls version {}.{}.{}\n", GRSLS_VERSION_MAJOR, GRSLS_VERSION_MINOR, GRSLS_VERSION_PATCH);
            return 0;
        }
    }

    for (auto c_arg : args) {
        auto arg = std::string_view{c_arg, static_cast<size_t>(strlen(c_arg))};

        switch (current_dependent_option) {
        case DependentOption::SOCKET: {
            if (comm_mode == CommMode::SOCKET && is_number(arg)) {
                port = arg;
            }
        } break;
        }

        // reset
        current_dependent_option = DependentOption::NONE;

        if (arg == "socket" || arg == "--socket") {
            comm_mode = CommMode::SOCKET;
            current_dependent_option = DependentOption::SOCKET;
        } else if (arg == "stdio" || arg == "--stdio") {
            comm_mode = CommMode::STDIO;
        } else if (arg.starts_with("--log=")) {
            logfile = arg.substr(6);
        }

        auto is_socket_opt = arg.starts_with("--socket="sv);
        auto is_port_opt = arg.starts_with("--port="sv);
        if (is_socket_opt || is_port_opt) {
            auto port_sv = [&]() {
                if (is_socket_opt) {
                    return arg.substr(9);
                } else /* if (is_port_opt) */ {
                    return arg.substr(7);
                }
                // unreachable
            }();
            if (is_number(port_sv)) {
                port = port_sv;
            } else {
                // maybe warn?
            }
        }
    }

    if (current_dependent_option != DependentOption::NONE) {
        // maybe warn?
    }

    if (comm_mode == CommMode::NONE) {
        // maybe warn?
        comm_mode = CommMode::SOCKET;
    }

    AppState appstate;

    DummyLog _log;
    std::shared_ptr<lsp::ProtocolJsonHandler> protocol_json_handler = std::make_shared<lsp::ProtocolJsonHandler>();
    std::shared_ptr<GenericEndpoint> endpoint = std::make_shared<GenericEndpoint>(_log);

    std::unique_ptr<LanguageServer> server;
    switch (comm_mode) {
    case CommMode::STDIO: server = std::make_unique<StdioServer>(protocol_json_handler, endpoint, _log); break;
    case CommMode::SOCKET: server = std::make_unique<TcpServer>(protocol_json_handler, endpoint, _log, port); break;
    }

    Condition<bool> esc_event;

    appstate.verbose = verbose;
    appstate.use_logfile = !logfile.empty();
    if (appstate.use_logfile) {
        appstate.logfile_stream.open(logfile);
    }

    const auto getVulkanSpv = []() {
        return EShMessages(EShMsgSpvRules | EShMsgVulkanRules);
    };

    const auto getSpvRules = []() {
        return EShMessages(EShMsgSpvRules);
    };

    if (!spirv_version.empty()) {
        appstate.target.options = getSpvRules();
        if (spirv_version == "spv1.5") {
            appstate.target.spv_version = glslang::EShTargetSpv_1_5;
        } else if (spirv_version == "spv1.4") {
            appstate.target.spv_version = glslang::EShTargetSpv_1_4;
        } else if (spirv_version == "spv1.3") {
            appstate.target.spv_version = glslang::EShTargetSpv_1_3;
        } else if (spirv_version == "spv1.2") {
            appstate.target.spv_version = glslang::EShTargetSpv_1_2;
        } else if (spirv_version == "spv1.1") {
            appstate.target.spv_version = glslang::EShTargetSpv_1_1;
        } else if (spirv_version == "spv1.0") {
            appstate.target.spv_version = glslang::EShTargetSpv_1_0;
        } else if (spirv_version == "spv1.6") {
            appstate.target.spv_version = glslang::EShTargetSpv_1_6;
        }
    }

    if (!client_api.empty()) {
        if (client_api == "vulkan1.2") {
            appstate.target.client_api = glslang::EShClientVulkan;
            appstate.target.client_api_version = glslang::EShTargetVulkan_1_2;
            appstate.target.spv_version = glslang::EShTargetSpv_1_5;
            appstate.target.options = getVulkanSpv();
        } else if (client_api == "vulkan1.1") {
            appstate.target.client_api = glslang::EShClientVulkan;
            appstate.target.client_api_version = glslang::EShTargetVulkan_1_1;
            appstate.target.spv_version = glslang::EShTargetSpv_1_3;
            appstate.target.options = getVulkanSpv();
        } else if (client_api == "vulkan1.0") {
            appstate.target.client_api = glslang::EShClientVulkan;
            appstate.target.client_api_version = glslang::EShTargetVulkan_1_0;
            appstate.target.spv_version = glslang::EShTargetSpv_1_1;
            appstate.target.options = getVulkanSpv();
        } else if (client_api == "opengl4.5" || client_api == "opengl") {
            appstate.target.client_api = glslang::EShClientOpenGL;
            appstate.target.client_api_version = glslang::EShTargetOpenGL_450;
            appstate.target.spv_version = glslang::EShTargetSpv_1_3;
        } else if (client_api == "vulkan1.3" || client_api == "vulkan") {
            appstate.target.client_api = glslang::EShClientVulkan;
            appstate.target.client_api_version = glslang::EShTargetVulkan_1_3;
            appstate.target.spv_version = glslang::EShTargetSpv_1_6;
            appstate.target.options = getVulkanSpv();
        }
    }

#define HANDLE_MONITOR(monitor)                                        \
    if (monitor && monitor()) {                                        \
        _log.info("textDocument request had been canceled.");          \
        Rsp_Error rsp;                                                 \
        rsp.error.code = lsErrorCodes::RequestCancelled;               \
        rsp.error.message = "textDocument request had been canceled."; \
        return rsp;                                                    \
    }

    auto &server_point = server->get_endpoint();

    server_point.registerHandler(
        [&](const td_initialize::request &req)
            -> lsp::ResponseOrError<td_initialize::response> {
            td_initialize::response rsp;

            rsp.result.capabilities.textDocumentSync =
                std::pair<std::optional<lsTextDocumentSyncKind>, std::optional<lsTextDocumentSyncOptions>>{
                    lsTextDocumentSyncKind::Full,
                    lsTextDocumentSyncOptions{
                        .openClose = true,
                        .change = lsTextDocumentSyncKind::Full,
                        .willSave = false,
                        .willSaveWaitUntil = false,
                        .save = {},
                    },
                };
            rsp.result.capabilities.hoverProvider = true;
            rsp.result.capabilities.completionProvider = lsCompletionOptions{
                .resolveProvider = false,
                .triggerCharacters = std::vector<std::string>{},
            };
            rsp.result.capabilities.definitionProvider = {true, {}};

            // rsp.result.capabilities.referencesProvider = false;
            // rsp.result.capabilities.documentHighlightProvider = false;
            // rsp.result.capabilities.documentSymbolProvider = false;
            // rsp.result.capabilities.workspaceSymbolProvider = false;
            // rsp.result.capabilities.codeActionProvider = false;
            // rsp.result.capabilities.codeLensProvider = CodeLensOptions{
            //     .resolveProvider = false,
            // };
            // rsp.result.capabilities.documentFormattingProvider = false;
            // rsp.result.capabilities.documentRangeFormattingProvider = false;
            // rsp.result.capabilities.renameProvider = false;

            return rsp;
        });
    server_point.registerHandler([&](Notify_Exit::notify &notify) {
        esc_event.notify(std::make_unique<bool>(true));
    });
    server_point.registerHandler([&](td_shutdown::request const &req, const CancelMonitor &monitor) -> lsp::ResponseOrError<td_shutdown::response> {
        HANDLE_MONITOR(monitor)
        td_shutdown::response rsp;
        rsp.id = req.id;
        rsp.result = lsp::Any{};
        rsp.result->SetJsonString("{}", lsp::Any::kNullType);
        return rsp;
    });

    server_point.registerHandler([&](Notify_TextDocumentDidOpen::notify &notify) {
        auto uri = notify.params.textDocument.uri.GetAbsolutePath();
        auto text = notify.params.textDocument.text + "\n ";
        appstate.workspace.add_document(uri, text);

        auto response = Notify_TextDocumentPublishDiagnostics::notify{};
        response.params.uri = AbsolutePath(uri);
        response.params.diagnostics = get_diagnostics(uri, text, appstate);
        server_point.send(response);
    });
    server_point.registerHandler([&](Notify_TextDocumentDidChange::notify &notify) {
        auto uri = notify.params.textDocument.uri.GetAbsolutePath();
        auto change = notify.params.contentChanges[0].text + "\n ";
        appstate.workspace.change_document(uri, change);
        std::string const document = appstate.workspace.documents()[uri];

        auto response = Notify_TextDocumentPublishDiagnostics::notify{};
        response.params.uri = AbsolutePath(uri);
        response.params.diagnostics = get_diagnostics(uri, change, appstate);
        server_point.send(response);
    });
    server_point.registerHandler([&](const td_completion::request &req, const CancelMonitor &monitor) -> lsp::ResponseOrError<td_completion::response> {
        HANDLE_MONITOR(monitor)
        td_completion::response rsp;
        rsp.id = req.id;
        auto uri = req.params.textDocument.uri.GetAbsolutePath();
        int const line = req.params.position.line;
        int const character = req.params.position.character;
        rsp.result = get_completions(uri, line, character, appstate);
        return rsp;
    });
    server_point.registerHandler([&](const td_hover::request &req, const CancelMonitor &monitor) -> lsp::ResponseOrError<td_hover::response> {
        HANDLE_MONITOR(monitor)
        td_hover::response rsp;
        rsp.id = req.id;
        auto uri = req.params.textDocument.uri.GetAbsolutePath();
        int const line = req.params.position.line;
        int const character = req.params.position.character;
        rsp.result = get_hover_info(uri, line, character, appstate);
        return rsp;
    });
    server_point.registerHandler([&](const td_definition::request &req, const CancelMonitor &monitor) -> lsp::ResponseOrError<td_definition::response> {
        // std::this_thread::sleep_for(std::chrono::seconds(8));
        HANDLE_MONITOR(monitor)
        td_definition::response rsp;
        rsp.id = req.id;
        auto uri = req.params.textDocument.uri.GetAbsolutePath();
        int const line = req.params.position.line;
        int const character = req.params.position.character;
        rsp.result = get_definition(uri, line, character, appstate);
        return rsp;
    });

    glslang::InitializeProcess();
    server->start();
    esc_event.wait();
    glslang::FinalizeProcess();
    server->stop();

    return 0;
}
