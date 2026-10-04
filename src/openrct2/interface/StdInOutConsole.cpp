/*****************************************************************************
 * Copyright (c) 2014-2026 OpenRCT2 developers
 *
 * For a complete list of all authors, please refer to contributors.md
 * Interested in contributing? Visit https://github.com/OpenRCT2/OpenRCT2
 *
 * OpenRCT2 is licensed under the GNU General Public License version 3.
 *****************************************************************************/

#include "StdInOutConsole.h"

#include "../Context.h"
#include "../Date.h"
#include "../PlatformEnvironment.h"
#include "../core/Path.hpp"
#include "../core/String.hpp"
#include "../localisation/FormatCodes.h"
#include "../platform/Platform.h"
#include "../scripting/ScriptEngine.h"

#include <linenoise.hpp>

#ifdef _WIN32
    // Win32 defines CreateDirectory as CreateDirectoryW/A. Do not let that
    // macro rewrite OpenRCT2::Path::CreateDirectory below.
    #ifdef CreateDirectory
        #undef CreateDirectory
    #endif
#endif

using namespace OpenRCT2;

// Ignore isatty warning on WIN32
#ifdef _MSC_VER
    #pragma warning(disable : 4996)
#endif

StdInOutConsole::~StdInOutConsole()
{
    StopLogging();
}

void StdInOutConsole::StartLogging()
{
    std::lock_guard<std::mutex> lock(_logMutex);
    if (_logFile != nullptr || _logStartAttempted)
        return;

    auto* context = GetContext();
    if (context == nullptr)
        return;

    _logStartAttempted = true;
    const auto logDirectory = Path::Combine(
        context->GetPlatformEnvironment().GetDirectoryPath(DirBase::user),
        u8"console");
    if (!Path::CreateDirectory(logDirectory))
        return;

    const auto date = Platform::GetDateLocal();
    const auto time = Platform::GetTimeLocal();
    const auto fileName = String::stdFormat(
        "openrct2-console-%04d-%02u-%02u_%02u-%02u-%02u.log",
        int(date.year), unsigned(date.month), unsigned(date.day),
        unsigned(time.hour), unsigned(time.minute), unsigned(time.second));
    _logPath = Path::Combine(logDirectory, fileName);

#ifdef _WIN32
    const auto pathW = String::toWideChar(_logPath);
    _logFile = _wfopen(pathW.c_str(), L"ab");
#else
    _logFile = std::fopen(_logPath.c_str(), "ab");
#endif
}

void StdInOutConsole::StopLogging()
{
    std::lock_guard<std::mutex> lock(_logMutex);
    if (_logFile == nullptr)
        return;

    std::fflush(_logFile);
    std::fclose(_logFile);
    _logFile = nullptr;
}

void StdInOutConsole::WriteLogLine(const std::string& s)
{
    std::lock_guard<std::mutex> lock(_logMutex);
    if (_logFile == nullptr)
        return;

    if (!s.empty())
        std::fwrite(s.data(), 1, s.size(), _logFile);
    std::fputc('\n', _logFile);
    std::fflush(_logFile);
}

void StdInOutConsole::Start()
{
    StartLogging();
    if (!_logPath.empty())
    {
        WriteLine(String::stdFormat(
            "Console output is being saved to: %s",
            _logPath.c_str()));
    }

    // Only start if stdin/stdout is a TTY
    if (!isatty(fileno(stdin)) || !isatty(fileno(stdout)))
    {
        return;
    }

    // Allow user to disable the console REPL. Setting this environment variable to any value will prevent REPL from starting.
    if (getenv("OPENRCT2_NO_REPL"))
    {
        return;
    }

    std::thread replThread([this]() -> void {
        linenoise::SetMultiLine(true);
        linenoise::SetHistoryMaxLen(32);

        std::string prompt = "\033[32mopenrct2 $\x1b[0m ";
        bool lastPromptQuit = false;
        while (true)
        {
            std::string line;
            std::string left = prompt;
            _isPromptShowing = true;
            auto quit = linenoise::Readline(left.c_str(), line);
            _isPromptShowing = false;
            if (quit)
            {
                if (lastPromptQuit)
                {
                    GetContext()->Finish();
                    break;
                }

                lastPromptQuit = true;
                std::puts("(To exit, press ^C again)");
            }
            else
            {
                lastPromptQuit = false;
                linenoise::AddHistory(line.c_str());
                Eval(line).wait();
            }
        }
    });
    replThread.detach();
}

std::future<void> StdInOutConsole::Eval(const std::string& s)
{
#ifdef ENABLE_SCRIPTING
    auto& scriptEngine = GetContext()->GetScriptEngine();
    return scriptEngine.Eval(s);
#else
    // Push on-demand evaluations onto a queue so that it can be processed deterministically
    // on the main thead at the right time.
    std::promise<void> barrier;
    auto future = barrier.get_future();
    _evalQueue.emplace(std::move(barrier), s);
    return future;
#endif
}

void StdInOutConsole::ProcessEvalQueue()
{
#ifndef ENABLE_SCRIPTING
    while (_evalQueue.size() > 0)
    {
        auto item = std::move(_evalQueue.front());
        _evalQueue.pop();
        auto promise = std::move(std::get<0>(item));
        auto command = std::move(std::get<1>(item));

        Execute(command);

        // Signal the promise so caller can continue
        promise.set_value();
    }
#endif
}

void StdInOutConsole::Clear()
{
    linenoise::linenoiseClearScreen();
}

void StdInOutConsole::Close()
{
    GetContext()->Finish();
}

void StdInOutConsole::WriteLine(const std::string& s, FormatToken colourFormat)
{
    StartLogging();
    WriteLogLine(s);

    std::string formatBegin;
    switch (colourFormat)
    {
        case FormatToken::colourRed:
            formatBegin = "\033[31m";
            break;
        case FormatToken::colourYellow:
            formatBegin = "\033[33m";
            break;
        default:
            break;
    }

    if (!Platform::IsColourTerminalSupported())
    {
        std::printf("%s\n", s.c_str());
        std::fflush(stdout);
    }
    else
    {
        if (_isPromptShowing)
        {
            auto* mainString = s.c_str();

            // If string contains \n, we need to replace with \r\n
            std::string newString;
            if (s.find('\n') != std::string::npos)
            {
                for (auto ch : s)
                {
                    if (ch == '\n')
                        newString += "\r\n";
                    else
                        newString += ch;
                }
                mainString = newString.c_str();
            }

            std::printf("\r%s%s\x1b[0m\x1b[0K\r\n", formatBegin.c_str(), mainString);
            std::fflush(stdout);
            linenoise::linenoiseEditRefreshLine();
        }
        else
        {
            std::printf("%s%s\x1b[0m\n", formatBegin.c_str(), s.c_str());
            std::fflush(stdout);
        }
    }
}
