#include <iostream>
#include <windows.h>
#include <vector>
#include <processthreadsapi.h>
#include <pathcch.h>

LPPROC_THREAD_ATTRIBUTE_LIST MakeAttributeList()
{
    LPPROC_THREAD_ATTRIBUTE_LIST attrbuteList;
    SIZE_T AttributeListSize{};
    InitializeProcThreadAttributeList(nullptr, 1, 0, &AttributeListSize);
    attrbuteList = (LPPROC_THREAD_ATTRIBUTE_LIST)HeapAlloc(
        GetProcessHeap(),
        0,
        AttributeListSize
    );

    DWORD error = 0;
    if (!InitializeProcThreadAttributeList(
        attrbuteList,
        1,
        0,
        &AttributeListSize))
    {
        error = GetLastError();
        throw error;
    }

    DWORD attribute = PROCESS_CREATION_DESKTOP_APP_BREAKAWAY_DISABLE_PROCESS_TREE;
    if (!UpdateProcThreadAttribute(
        attrbuteList,
        0,
        PROC_THREAD_ATTRIBUTE_DESKTOP_APP_POLICY,
        &attribute,
        sizeof(attribute),
        nullptr,
        nullptr))
    {
        error = GetLastError();
        throw error;
    }

    return attrbuteList;
}

int main(int argc, char* argv[]) try
{

    const auto ExecuteCommand = [](std::wstring commandline) -> unsigned long
    {
        STARTUPINFOEXW startup_info = {};
        PROCESS_INFORMATION process_info = {};
        startup_info.StartupInfo.cb = sizeof(startup_info);
        startup_info.lpAttributeList = MakeAttributeList();
        std::vector<wchar_t> chars(commandline.size() + 1);
        wmemcpy(&chars[0], commandline.data(), commandline.size());
        chars[commandline.size()] = L'\0';

        if (!::CreateProcess(
            nullptr,                // program to execute (nullptr = execute command line)
            chars.data(),            // command line to execute
            nullptr,                // process security attributes
            nullptr,                // thread security attributes
            false,                  // determines if handles from parent process are inherited
            EXTENDED_STARTUPINFO_PRESENT,                      // no creation flags
            nullptr,                // enviornment (nullptr = use parent's)
            nullptr,                // current directory (nullptr = use parent's)
            (LPSTARTUPINFO)&startup_info, // startup info
            &process_info           // process info
        ))
        {
            const auto error = ::GetLastError();
            switch (error)
            {
            case ERROR_FILE_NOT_FOUND:
            {
                std::wcerr << L"ERROR_FILE_NOT_FOUND" << std::endl;
                break;
            }
            default:
                std::wcerr << L"error: " << error << std::endl;
                break;
            }
            throw error;
        }
        ::WaitForSingleObject(process_info.hProcess, INFINITE);
        unsigned long exitCode = NO_ERROR;
        ::GetExitCodeProcess(process_info.hProcess, &exitCode);
        ::CloseHandle(process_info.hProcess);
        ::CloseHandle(process_info.hThread);

        return exitCode;
    };

    const auto GetPackageRoot = []() -> std::wstring
    {
        TCHAR moduleName[MAX_PATH] = {};
        ::GetModuleFileName(nullptr, moduleName, MAX_PATH);
        ::PathCchRemoveFileSpec(moduleName, MAX_PATH);
        return moduleName;
    };

    std::wstring commandLine = L"C:\\Windows\\System32\\cmd.exe /C \"";
    commandLine += GetPackageRoot();
    commandLine += L"\\Launcher.bat\"";

    return ExecuteCommand(commandLine.c_str());
}
catch (...)
{
    return 1;
}
