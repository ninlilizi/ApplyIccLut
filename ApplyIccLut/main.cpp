// ApplyIccLut - Applies LUTs from ICC profiles or .cube files to displays
//
// Workaround for Windows 11 not applying the LUT portion of ICC profiles at boot.
// Reads the VCGT (Video Card Gamma Table) or MHC2 RegammaLUT from the default ICC
// profile, or a 1D LUT from a .cube file, and loads it into the display adapter via
// SetDeviceGammaRamp.
//
// Supported input formats:
//   - ICC profiles with VCGT or MHC2 tags (.icc, .icm)
//   - 1D .cube LUT files (.cube) - up to 65536 entries, downsampled to 256
//
// Note on resolution: SetDeviceGammaRamp is limited to 256 entries per channel.
// For full 4096-entry precision, use the MHC2 ICC profile pipeline via Windows
// Color Management (which the GPU driver applies natively).
//
// Usage:
//   ApplyIccLut.exe                  GPU pipeline wake-up for all monitors
//   ApplyIccLut.exe -p file.icc     Apply ICC LUT via SetDeviceGammaRamp (256)
//   ApplyIccLut.exe -p file.cube    Apply .cube LUT via SetDeviceGammaRamp (256)
//   ApplyIccLut.exe -s Profile.icm  Set installed profile as GPU default (4096)
//   ApplyIccLut.exe -v              Verbose output
//   ApplyIccLut.exe -r              Reset to linear (identity) gamma ramp
//   ApplyIccLut.exe --probe         (hidden) Force re-download PDB and re-resolve offsets
//   ApplyIccLut.exe --probe -v      (hidden) Verbose PDB resolution output

#include <windows.h>
#include <icm.h>
#include <tlhelp32.h>
#include <aclapi.h>
#include <cstdio>
#include <cstdint>
#include <cmath>
#include <cstring>
#include <string>
#include <vector>
#include <algorithm>

#pragma comment(lib, "mscms.lib")
#pragma comment(lib, "version.lib")
#pragma comment(lib, "advapi32.lib")
#pragma comment(lib, "dbghelp.lib")
#pragma comment(lib, "urlmon.lib")

#include <DbgHelp.h>
#include <urlmon.h>

// ============================================================================
// Big-endian helpers (ICC profiles are big-endian)
// ============================================================================

static uint32_t ReadU32BE(const uint8_t* p)
{
    return (uint32_t(p[0]) << 24) | (uint32_t(p[1]) << 16) |
           (uint32_t(p[2]) << 8)  |  uint32_t(p[3]);
}

static uint16_t ReadU16BE(const uint8_t* p)
{
    return (uint16_t(p[0]) << 8) | p[1];
}

static int32_t ReadS32BE(const uint8_t* p)
{
    return static_cast<int32_t>(ReadU32BE(p));
}

static double S15F16ToDouble(int32_t v)
{
    return static_cast<double>(v) / 65536.0;
}

// ============================================================================
// ICC profile structures
// ============================================================================

struct IccTagEntry
{
    uint32_t signature;
    uint32_t offset;
    uint32_t size;
};

static constexpr uint32_t TAG_VCGT = 0x76636774; // 'vcgt'
static constexpr uint32_t TAG_MHC2 = 0x4D484332; // 'MHC2'

// ============================================================================
// File I/O
// ============================================================================

static std::vector<uint8_t> LoadFile(const std::wstring& path)
{
    std::vector<uint8_t> data;
    HANDLE hFile = CreateFileW(path.c_str(), GENERIC_READ, FILE_SHARE_READ,
                               nullptr, OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, nullptr);
    if (hFile == INVALID_HANDLE_VALUE)
        return data;

    DWORD fileSize = GetFileSize(hFile, nullptr);
    if (fileSize > 0 && fileSize != INVALID_FILE_SIZE)
    {
        data.resize(fileSize);
        DWORD bytesRead = 0;
        if (!ReadFile(hFile, data.data(), fileSize, &bytesRead, nullptr) ||
            bytesRead != fileSize)
        {
            data.clear();
        }
    }
    CloseHandle(hFile);
    return data;
}

// ============================================================================
// ICC tag table parsing
// ============================================================================

static std::vector<IccTagEntry> ParseIccTags(const std::vector<uint8_t>& icc)
{
    std::vector<IccTagEntry> tags;

    // Need at least: 128-byte header + 4-byte tag count
    if (icc.size() < 132)
        return tags;

    // Verify ICC signature "acsp" at offset 36
    if (icc[36] != 'a' || icc[37] != 'c' || icc[38] != 's' || icc[39] != 'p')
        return tags;

    uint32_t tagCount = ReadU32BE(&icc[128]);
    if (tagCount > 1000) // sanity
        return tags;

    for (uint32_t i = 0; i < tagCount; i++)
    {
        uint32_t base = 132 + i * 12;
        if (base + 12 > static_cast<uint32_t>(icc.size()))
            break;

        IccTagEntry t;
        t.signature = ReadU32BE(&icc[base]);
        t.offset    = ReadU32BE(&icc[base + 4]);
        t.size      = ReadU32BE(&icc[base + 8]);

        if (t.offset + t.size <= static_cast<uint32_t>(icc.size()))
            tags.push_back(t);
    }
    return tags;
}

static const IccTagEntry* FindTag(const std::vector<IccTagEntry>& tags, uint32_t sig)
{
    for (const auto& t : tags)
        if (t.signature == sig) return &t;
    return nullptr;
}

// ============================================================================
// VCGT tag parsing  (Video Card Gamma Table - standard ICC extension)
//
// Binary layout at tag offset:
//   [0-3]   Type signature 'vcgt'
//   [4-7]   Reserved (0)
//   [8-11]  Table type: 0 = lookup table, 1 = parametric formula
//
// Type 0 (table):
//   [12-13] Number of channels (3)
//   [14-15] Entries per channel
//   [16-17] Entry size in bytes (1 or 2)
//   [18+]   Interleaved channel data (ch0 entries, ch1 entries, ch2 entries)
//
// Type 1 (formula):
//   [12-15] Red gamma     (S15F16)
//   [16-19] Red min       (S15F16)
//   [20-23] Red max       (S15F16)
//   [24-35] Green gamma, min, max
//   [36-47] Blue gamma, min, max
// ============================================================================

static bool ParseVcgt(const std::vector<uint8_t>& icc, const IccTagEntry& tag,
                      WORD ramp[3][256])
{
    const uint8_t* d = &icc[tag.offset];
    uint32_t sz = tag.size;

    if (sz < 12) return false;

    uint32_t tableType = ReadU32BE(d + 8);

    if (tableType == 0)
    {
        // Lookup table
        if (sz < 18) return false;

        uint16_t nChannels = ReadU16BE(d + 12);
        uint16_t nEntries  = ReadU16BE(d + 14);
        uint16_t entrySize = ReadU16BE(d + 16);

        if (nChannels < 3 || nEntries == 0 || (entrySize != 1 && entrySize != 2))
            return false;

        uint32_t chBytes = static_cast<uint32_t>(nEntries) * entrySize;
        if (sz < 18 + 3 * chBytes) return false;

        const uint8_t* table = d + 18;

        for (int ch = 0; ch < 3; ch++)
        {
            const uint8_t* chData = table + ch * chBytes;

            for (int i = 0; i < 256; i++)
            {
                // Linearly interpolate from nEntries to 256
                double pos  = static_cast<double>(i) / 255.0 * (nEntries - 1);
                int    idx0 = static_cast<int>(pos);
                int    idx1 = std::min(idx0 + 1, static_cast<int>(nEntries) - 1);
                double frac = pos - idx0;

                double v0, v1;
                if (entrySize == 2)
                {
                    v0 = ReadU16BE(chData + idx0 * 2);
                    v1 = ReadU16BE(chData + idx1 * 2);
                }
                else
                {
                    v0 = chData[idx0] * 257.0; // scale 8-bit to 16-bit range
                    v1 = chData[idx1] * 257.0;
                }

                double val = v0 + frac * (v1 - v0);
                ramp[ch][i] = static_cast<WORD>(std::min(65535.0, std::max(0.0, val + 0.5)));
            }
        }
        return true;
    }
    else if (tableType == 1)
    {
        // Parametric formula:  output = min + (max - min) * pow(input, gamma)
        if (sz < 48) return false;

        for (int ch = 0; ch < 3; ch++)
        {
            double gamma  = S15F16ToDouble(ReadS32BE(d + 12 + ch * 12));
            double minVal = S15F16ToDouble(ReadS32BE(d + 16 + ch * 12));
            double maxVal = S15F16ToDouble(ReadS32BE(d + 20 + ch * 12));

            for (int i = 0; i < 256; i++)
            {
                double input  = static_cast<double>(i) / 255.0;
                double output = minVal + (maxVal - minVal) * pow(input, gamma);
                output = std::max(0.0, std::min(1.0, output));
                ramp[ch][i] = static_cast<WORD>(output * 65535.0 + 0.5);
            }
        }
        return true;
    }

    return false;
}

// ============================================================================
// MHC2 tag parsing  (ColorControl custom tag for HDR/SDR profiles)
//
// Binary layout at tag offset (from ColorControl source):
//   [0-3]   'MHC2' signature
//   [4-7]   Reserved (0)
//   [8-11]  LUT size (entries per channel)
//   [12-15] MinCLL  (S15F16)
//   [16-19] MaxCLL  (S15F16)
//   [20-23] Matrix offset  (always 36)
//   [24-27] LUT channel 0 offset
//   [28-31] LUT channel 1 offset
//   [32-35] LUT channel 2 offset
//   [36-83] 3x4 matrix (12 x S15F16, row-major)
//   Per-channel LUT:
//     [+0]  "sf32" (4 bytes) + padding (4 bytes) = 8 byte header
//     [+8]  N x S15F16 values (curve from 0.0 to 1.0)
// ============================================================================

static bool ParseMhc2Lut(const std::vector<uint8_t>& icc, const IccTagEntry& tag,
                         WORD ramp[3][256])
{
    const uint8_t* d = &icc[tag.offset];
    uint32_t sz = tag.size;

    if (sz < 36) return false;

    uint32_t lutSize    = ReadU32BE(d + 8);
    uint32_t lut0Offset = ReadU32BE(d + 24);
    uint32_t lut1Offset = ReadU32BE(d + 28);
    uint32_t lut2Offset = ReadU32BE(d + 32);

    if (lutSize == 0 || lutSize > 65536) return false;

    uint32_t lutOffsets[3] = { lut0Offset, lut1Offset, lut2Offset };
    uint32_t lutDataBytes  = 8 + lutSize * 4; // 8-byte header + entries

    for (int ch = 0; ch < 3; ch++)
    {
        uint32_t off = lutOffsets[ch];
        if (off + lutDataBytes > sz) return false;

        // Skip "sf32" + padding (8 bytes)
        const uint8_t* lutData = d + off + 8;

        for (int i = 0; i < 256; i++)
        {
            double pos  = static_cast<double>(i) / 255.0 * (lutSize - 1);
            int    idx0 = static_cast<int>(pos);
            int    idx1 = std::min(idx0 + 1, static_cast<int>(lutSize) - 1);
            double frac = pos - idx0;

            double v0 = S15F16ToDouble(ReadS32BE(lutData + idx0 * 4));
            double v1 = S15F16ToDouble(ReadS32BE(lutData + idx1 * 4));

            double val = v0 + frac * (v1 - v0);
            val = std::max(0.0, std::min(1.0, val));
            ramp[ch][i] = static_cast<WORD>(val * 65535.0 + 0.5);
        }
    }
    return true;
}

// ============================================================================
// .cube LUT file parsing (1D LUT format - IRIDAS/Adobe specification)
//
// Supports 1D LUTs (LUT_1D_SIZE) with up to 65536 entries.
// Data is three floating-point values per line: R G B
// Lines beginning with '#' are comments.
// Keywords: TITLE, LUT_1D_SIZE, LUT_3D_SIZE, DOMAIN_MIN, DOMAIN_MAX
//
// Note: SetDeviceGammaRamp only supports 256 entries, so high-resolution
// cube LUTs are downsampled via linear interpolation. The full 4096-entry
// resolution is only achievable through the MHC2 ICC profile pipeline.
// ============================================================================

static bool IsCubeFile(const std::wstring& path)
{
    if (path.size() < 5) return false;
    std::wstring ext = path.substr(path.size() - 5);
    for (auto& c : ext) c = static_cast<wchar_t>(towlower(c));
    return ext == L".cube";
}

static bool ParseCubeFile(const std::vector<uint8_t>& fileData,
                          WORD ramp[3][256], int& cubeLutSize, bool verbose)
{
    // Convert to string for line-by-line parsing
    std::string text(fileData.begin(), fileData.end());

    int lutSize = 0;
    bool is3D = false;
    double domainMin[3] = { 0.0, 0.0, 0.0 };
    double domainMax[3] = { 1.0, 1.0, 1.0 };
    std::vector<double> red, green, blue;
    std::string title;

    // Parse line by line
    size_t pos = 0;
    while (pos < text.size())
    {
        size_t eol = text.find('\n', pos);
        if (eol == std::string::npos) eol = text.size();

        std::string line = text.substr(pos, eol - pos);
        pos = eol + 1;

        // Trim trailing \r
        if (!line.empty() && line.back() == '\r')
            line.pop_back();

        // Skip empty lines and comments
        if (line.empty() || line[0] == '#')
            continue;

        // Parse keywords
        if (line.rfind("TITLE", 0) == 0)
        {
            // Extract title between quotes if present
            auto q1 = line.find('"');
            auto q2 = line.rfind('"');
            if (q1 != std::string::npos && q2 > q1)
                title = line.substr(q1 + 1, q2 - q1 - 1);
            continue;
        }

        if (line.rfind("LUT_3D_SIZE", 0) == 0)
        {
            is3D = true;
            wprintf(L"  ERROR: 3D .cube LUTs are not supported for gamma ramp application.\n");
            wprintf(L"         Only 1D LUTs (LUT_1D_SIZE) can be applied.\n");
            return false;
        }

        if (line.rfind("LUT_1D_SIZE", 0) == 0)
        {
            if (sscanf_s(line.c_str(), "LUT_1D_SIZE %d", &lutSize) != 1 || lutSize < 2)
            {
                wprintf(L"  ERROR: Invalid LUT_1D_SIZE in .cube file.\n");
                return false;
            }
            if (lutSize > 65536)
            {
                wprintf(L"  ERROR: LUT_1D_SIZE %d exceeds maximum of 65536.\n", lutSize);
                return false;
            }
            continue;
        }

        if (line.rfind("DOMAIN_MIN", 0) == 0)
        {
            sscanf_s(line.c_str(), "DOMAIN_MIN %lf %lf %lf",
                   &domainMin[0], &domainMin[1], &domainMin[2]);
            continue;
        }

        if (line.rfind("DOMAIN_MAX", 0) == 0)
        {
            sscanf_s(line.c_str(), "DOMAIN_MAX %lf %lf %lf",
                   &domainMax[0], &domainMax[1], &domainMax[2]);
            continue;
        }

        // Try parsing as data line: R G B
        double r, g, b;
        if (sscanf_s(line.c_str(), "%lf %lf %lf", &r, &g, &b) == 3)
        {
            red.push_back(r);
            green.push_back(g);
            blue.push_back(b);
        }
    }

    // If no LUT_1D_SIZE header, infer from data count
    if (lutSize == 0)
    {
        lutSize = static_cast<int>(red.size());
        if (verbose)
            wprintf(L"  NOTE: No LUT_1D_SIZE header; inferred %d from data lines.\n", lutSize);
    }

    if (lutSize < 2 || static_cast<int>(red.size()) < lutSize)
    {
        wprintf(L"  ERROR: .cube file has %zu data lines but needs at least %d.\n",
                red.size(), lutSize);
        return false;
    }

    cubeLutSize = lutSize;

    if (verbose)
    {
        if (!title.empty())
            wprintf(L"  Cube title: %S\n", title.c_str());
        wprintf(L"  Cube LUT size: %d entries per channel\n", lutSize);
        wprintf(L"  Domain: [%.4f..%.4f] [%.4f..%.4f] [%.4f..%.4f]\n",
                domainMin[0], domainMax[0], domainMin[1], domainMax[1],
                domainMin[2], domainMax[2]);
    }

    // Resample from lutSize entries to 256 for SetDeviceGammaRamp
    for (int ch = 0; ch < 3; ch++)
    {
        const std::vector<double>& src = (ch == 0) ? red : (ch == 1) ? green : blue;
        double dmin = domainMin[ch];
        double dmax = domainMax[ch];
        double range = dmax - dmin;
        if (range <= 0.0) range = 1.0;

        for (int i = 0; i < 256; i++)
        {
            double fpos = static_cast<double>(i) / 255.0 * (lutSize - 1);
            int    idx0 = static_cast<int>(fpos);
            int    idx1 = std::min(idx0 + 1, lutSize - 1);
            double frac = fpos - idx0;

            double v0 = src[idx0];
            double v1 = src[idx1];
            double val = v0 + frac * (v1 - v0);

            // Normalize from domain range to [0, 1]
            val = (val - dmin) / range;
            val = std::max(0.0, std::min(1.0, val));

            ramp[ch][i] = static_cast<WORD>(val * 65535.0 + 0.5);
        }
    }

    return true;
}

// ============================================================================
// Display enumeration via DisplayConfig API + EnumDisplayDevices
// ============================================================================

struct MonitorInfo
{
    std::wstring gdiDeviceName;   // e.g. \\.\DISPLAY1
    std::wstring friendlyName;    // e.g. "LG ULTRAGEAR"
    std::wstring devicePath;      // monitorDevicePath from DisplayConfig
    std::wstring eddDeviceId;     // DeviceID from EnumDisplayDevices
    LUID         adapterId;       // GPU adapter LUID (for ColorProfile APIs)
    UINT32       sourceId;        // Display source ID (for ColorProfile APIs)
};

static std::vector<MonitorInfo> EnumerateMonitors()
{
    std::vector<MonitorInfo> monitors;

    UINT32 pathCount = 0, modeCount = 0;
    if (GetDisplayConfigBufferSizes(QDC_ONLY_ACTIVE_PATHS, &pathCount, &modeCount) != ERROR_SUCCESS)
        return monitors;

    std::vector<DISPLAYCONFIG_PATH_INFO> paths(pathCount);
    std::vector<DISPLAYCONFIG_MODE_INFO> modes(modeCount);

    if (QueryDisplayConfig(QDC_ONLY_ACTIVE_PATHS, &pathCount, paths.data(),
                           &modeCount, modes.data(), nullptr) != ERROR_SUCCESS)
        return monitors;

    paths.resize(pathCount);

    for (const auto& path : paths)
    {
        MonitorInfo info;
        info.adapterId = {};
        info.sourceId  = 0;

        // Target (monitor) name and device path
        DISPLAYCONFIG_TARGET_DEVICE_NAME targetName = {};
        targetName.header.type      = DISPLAYCONFIG_DEVICE_INFO_GET_TARGET_NAME;
        targetName.header.size      = sizeof(targetName);
        targetName.header.adapterId = path.targetInfo.adapterId;
        targetName.header.id        = path.targetInfo.id;

        if (DisplayConfigGetDeviceInfo(&targetName.header) == ERROR_SUCCESS)
        {
            info.friendlyName = targetName.monitorFriendlyDeviceName;
            info.devicePath   = targetName.monitorDevicePath;
        }

        // Source (GDI adapter) device name - needed for CreateDC
        DISPLAYCONFIG_SOURCE_DEVICE_NAME sourceName = {};
        sourceName.header.type      = DISPLAYCONFIG_DEVICE_INFO_GET_SOURCE_NAME;
        sourceName.header.size      = sizeof(sourceName);
        sourceName.header.adapterId = path.sourceInfo.adapterId;
        sourceName.header.id        = path.sourceInfo.id;

        if (DisplayConfigGetDeviceInfo(&sourceName.header) == ERROR_SUCCESS)
            info.gdiDeviceName = sourceName.viewGdiDeviceName;

        // Store adapter/source IDs for ColorProfile APIs
        info.adapterId = path.sourceInfo.adapterId;
        info.sourceId  = path.sourceInfo.id;

        if (info.gdiDeviceName.empty())
            continue;

        // Deduplicate (clone modes can produce duplicates)
        bool dup = false;
        for (const auto& m : monitors)
            if (m.gdiDeviceName == info.gdiDeviceName) { dup = true; break; }
        if (dup) continue;

        // Also get device ID from EnumDisplayDevices (used by some WCS APIs)
        DISPLAY_DEVICEW dd = {};
        dd.cb = sizeof(dd);
        if (EnumDisplayDevicesW(info.gdiDeviceName.c_str(), 0, &dd,
                                EDD_GET_DEVICE_INTERFACE_NAME))
        {
            info.eddDeviceId = dd.DeviceID;
        }

        monitors.push_back(info);
    }

    return monitors;
}

// ============================================================================
// Default ICC profile resolution (multiple fallback strategies)
// ============================================================================

static std::wstring GetColorDirectory()
{
    DWORD bufSize = MAX_PATH;
    wchar_t buf[MAX_PATH] = {};
#pragma warning(suppress: 4996) // GetColorDirectoryW deprecation
    if (GetColorDirectoryW(nullptr, buf, &bufSize))
        return buf;
    return L"C:\\Windows\\System32\\spool\\drivers\\color";
}

// Try WcsGetDefaultColorProfile with a specific device name and scope
static std::wstring TryWcsGetDefault(const wchar_t* deviceName,
                                     WCS_PROFILE_MANAGEMENT_SCOPE scope)
{
    if (!deviceName || !deviceName[0]) return L"";

    wchar_t name[MAX_PATH] = {};
    DWORD cbName = sizeof(name);

    if (WcsGetDefaultColorProfile(scope, deviceName,
                                  CPT_ICC, CPST_NONE, 0,
                                  cbName, name) && name[0] != 0)
    {
        return name;
    }
    return L"";
}

// Try EnumColorProfilesW to find profiles associated with a device
static std::wstring TryEnumProfiles(const wchar_t* deviceName)
{
    if (!deviceName || !deviceName[0]) return L"";

    ENUMTYPEW et = {};
    et.dwSize    = sizeof(et);
    et.dwVersion = ENUM_TYPE_VERSION;
    et.dwFields  = ET_DEVICENAME;
    et.pDeviceName = const_cast<LPWSTR>(deviceName);

    // First call to get required buffer size
    DWORD bufSize = 0;
    DWORD nProfiles = 0;
    EnumColorProfilesW(nullptr, &et, nullptr, &bufSize, &nProfiles);

    if (bufSize == 0 || nProfiles == 0) return L"";

    std::vector<BYTE> buf(bufSize);
    nProfiles = 0;
    if (!EnumColorProfilesW(nullptr, &et, buf.data(), &bufSize, &nProfiles))
        return L"";

    if (nProfiles == 0) return L"";

    // Return the first profile name (multi-string buffer)
    return reinterpret_cast<const wchar_t*>(buf.data());
}

// Try GetICMProfile on a device context
static std::wstring TryGetICMProfile(const wchar_t* gdiDeviceName)
{
    if (!gdiDeviceName || !gdiDeviceName[0]) return L"";

    HDC hDC = CreateDCW(L"DISPLAY", gdiDeviceName, nullptr, nullptr);
    if (!hDC) return L"";

    wchar_t profilePath[MAX_PATH] = {};
    DWORD cbPath = MAX_PATH;

    std::wstring result;
    if (GetICMProfileW(hDC, &cbPath, profilePath) && profilePath[0] != 0)
        result = profilePath;

    DeleteDC(hDC);
    return result;
}

// Main profile resolution: tries multiple strategies
static std::wstring GetDefaultProfile(const MonitorInfo& mon, bool verbose)
{
    std::wstring result;

    // Candidate device names to try with WCS APIs (in priority order)
    const std::wstring* candidates[] = { &mon.devicePath, &mon.eddDeviceId };

    // Strategy 1: WcsGetDefaultColorProfile (user scope, then system scope)
    for (const auto* cand : candidates)
    {
        if (cand->empty()) continue;

        result = TryWcsGetDefault(cand->c_str(), WCS_PROFILE_MANAGEMENT_SCOPE_CURRENT_USER);
        if (!result.empty())
        {
            if (verbose) wprintf(L"  (found via WcsGetDefault user scope)\n");
            return result;
        }

        result = TryWcsGetDefault(cand->c_str(), WCS_PROFILE_MANAGEMENT_SCOPE_SYSTEM_WIDE);
        if (!result.empty())
        {
            if (verbose) wprintf(L"  (found via WcsGetDefault system scope)\n");
            return result;
        }
    }

    // Strategy 2: EnumColorProfiles for each device name candidate
    for (const auto* cand : candidates)
    {
        if (cand->empty()) continue;

        result = TryEnumProfiles(cand->c_str());
        if (!result.empty())
        {
            if (verbose) wprintf(L"  (found via EnumColorProfiles)\n");
            return result;
        }
    }

    // Strategy 3: GetICMProfile on the display DC
    result = TryGetICMProfile(mon.gdiDeviceName.c_str());
    if (!result.empty())
    {
        if (verbose) wprintf(L"  (found via GetICMProfile)\n");
        return result;
    }

    return L"";
}

static std::wstring ResolveProfilePath(const std::wstring& profileName)
{
    if (profileName.empty()) return L"";

    // Already a full path?
    if (profileName.size() > 2 &&
        (profileName[1] == L':' ||
         (profileName[0] == L'\\' && profileName[1] == L'\\')))
    {
        return profileName;
    }

    return GetColorDirectory() + L"\\" + profileName;
}

// ============================================================================
// GPU color profile management (Windows 10 1903+ ColorProfile* APIs)
//
// These APIs are loaded dynamically because they may not exist in older SDKs.
// They control the ICC profile that the GPU driver applies (MHC2 pipeline).
// ============================================================================

// ColorProfile API subtypes for SDR vs HDR
static constexpr COLORPROFILESUBTYPE CPST_STANDARD_DISPLAY = static_cast<COLORPROFILESUBTYPE>(7);
static constexpr COLORPROFILESUBTYPE CPST_EXTENDED_DISPLAY  = static_cast<COLORPROFILESUBTYPE>(8);

// Function pointer types for dynamically loaded APIs
using PFN_ColorProfileGetDisplayDefault = LONG(WINAPI*)(
    WCS_PROFILE_MANAGEMENT_SCOPE scope,
    LUID adapterId, UINT32 sourceId,
    COLORPROFILETYPE type, COLORPROFILESUBTYPE subtype,
    PWSTR* ppProfileName);

using PFN_ColorProfileRemoveDisplayAssociation = LONG(WINAPI*)(
    WCS_PROFILE_MANAGEMENT_SCOPE scope,
    LPCWSTR profileName,
    LUID adapterId, UINT32 sourceId,
    BOOL advancedColor);

using PFN_ColorProfileAddDisplayAssociation = LONG(WINAPI*)(
    WCS_PROFILE_MANAGEMENT_SCOPE scope,
    LPCWSTR profileName,
    LUID adapterId, UINT32 sourceId,
    BOOL setAsDefault,
    BOOL advancedColor);

// Dedicated "set as default" API - different code path from Remove+Add.
// This is likely what the Color Management "Set as Default" button calls.
using PFN_ColorProfileSetDisplayDefaultAssociation = HRESULT(WINAPI*)(
    WCS_PROFILE_MANAGEMENT_SCOPE scope,
    PCWSTR profileName,
    COLORPROFILETYPE type, COLORPROFILESUBTYPE subType,
    LUID adapterId, UINT32 sourceId);

// Undocumented mscms.dll export that forces the calibration subsystem to
// reload all display pipelines.
using PFN_InternalRefreshCalibration = BOOL(WINAPI*)(void);

struct GpuColorProfileApi
{
    PFN_ColorProfileGetDisplayDefault      GetDefault;
    PFN_ColorProfileRemoveDisplayAssociation Remove;
    PFN_ColorProfileAddDisplayAssociation  Add;
    PFN_ColorProfileSetDisplayDefaultAssociation SetDefault;
    PFN_InternalRefreshCalibration         RefreshCalibration;
    bool loaded;
};

static GpuColorProfileApi LoadGpuColorProfileApi()
{
    GpuColorProfileApi api = {};
    HMODULE hMscms = GetModuleHandleW(L"mscms.dll");
    if (!hMscms)
        hMscms = LoadLibraryW(L"mscms.dll");
    if (!hMscms)
        return api;

    api.GetDefault = reinterpret_cast<PFN_ColorProfileGetDisplayDefault>(
        GetProcAddress(hMscms, "ColorProfileGetDisplayDefault"));
    api.Remove = reinterpret_cast<PFN_ColorProfileRemoveDisplayAssociation>(
        GetProcAddress(hMscms, "ColorProfileRemoveDisplayAssociation"));
    api.Add = reinterpret_cast<PFN_ColorProfileAddDisplayAssociation>(
        GetProcAddress(hMscms, "ColorProfileAddDisplayAssociation"));
    api.SetDefault = reinterpret_cast<PFN_ColorProfileSetDisplayDefaultAssociation>(
        GetProcAddress(hMscms, "ColorProfileSetDisplayDefaultAssociation"));
    api.RefreshCalibration = reinterpret_cast<PFN_InternalRefreshCalibration>(
        GetProcAddress(hMscms, "InternalRefreshCalibration"));

    api.loaded = (api.GetDefault && api.Remove && api.Add);
    return api;
}

// Get the current default ICC profile name for a display (SDR or HDR)
static std::wstring GetGpuDefaultProfile(const GpuColorProfileApi& api,
                                         const MonitorInfo& mon,
                                         bool hdr, bool verbose)
{
    if (!api.loaded) return L"";

    auto subtype = hdr ? CPST_EXTENDED_DISPLAY : CPST_STANDARD_DISPLAY;
    PWSTR pName = nullptr;

    LONG err = api.GetDefault(
        WCS_PROFILE_MANAGEMENT_SCOPE_CURRENT_USER,
        mon.adapterId, mon.sourceId,
        CPT_ICC, subtype, &pName);

    if (err != ERROR_SUCCESS || !pName)
        return L"";

    std::wstring name(pName);
    LocalFree(pName);
    return name;
}

// Remove the ICC profile as default for the GPU pipeline, but keep it associated.
// This stops the GPU driver from applying the MHC2 transform.
static bool UnsetGpuProfile(const GpuColorProfileApi& api,
                            const MonitorInfo& mon,
                            const std::wstring& profileName,
                            bool hdr, bool verbose)
{
    if (!api.loaded || profileName.empty()) return false;

    BOOL advColor = hdr ? TRUE : FALSE;

    // Remove the association (un-defaults it)
    LONG err = api.Remove(
        WCS_PROFILE_MANAGEMENT_SCOPE_CURRENT_USER,
        profileName.c_str(),
        mon.adapterId, mon.sourceId,
        advColor);

    if (err != ERROR_SUCCESS)
    {
        if (verbose) wprintf(L"    Remove failed (err=%ld)\n", err);
        return false;
    }

    // Re-add WITHOUT setting as default (keeps it in the list)
    err = api.Add(
        WCS_PROFILE_MANAGEMENT_SCOPE_CURRENT_USER,
        profileName.c_str(),
        mon.adapterId, mon.sourceId,
        FALSE,      // setAsDefault = false
        advColor);

    if (err != ERROR_SUCCESS)
    {
        if (verbose) wprintf(L"    Re-add failed (err=%ld)\n", err);
        return false;
    }

    return true;
}

// Restore a profile as default for the GPU pipeline.
static bool RestoreGpuProfile(const GpuColorProfileApi& api,
                              const MonitorInfo& mon,
                              const std::wstring& profileName,
                              bool hdr, bool verbose)
{
    if (!api.loaded || profileName.empty()) return false;

    // Primary: use ColorProfileSetDisplayDefaultAssociation
    if (api.SetDefault)
    {
        auto subtype = hdr ? CPST_EXTENDED_DISPLAY : CPST_STANDARD_DISPLAY;
        HRESULT hr = api.SetDefault(
            WCS_PROFILE_MANAGEMENT_SCOPE_CURRENT_USER,
            profileName.c_str(),
            CPT_ICC, subtype,
            mon.adapterId, mon.sourceId);
        if (SUCCEEDED(hr))
            return true;
        if (verbose)
            wprintf(L"    SetDefault failed (hr=0x%08lX), trying Remove+Add\n", hr);
    }

    // Fallback: Remove + Add
    BOOL advColor = hdr ? TRUE : FALSE;

    api.Remove(
        WCS_PROFILE_MANAGEMENT_SCOPE_CURRENT_USER,
        profileName.c_str(),
        mon.adapterId, mon.sourceId,
        advColor);

    Sleep(200);

    LONG err = api.Add(
        WCS_PROFILE_MANAGEMENT_SCOPE_CURRENT_USER,
        profileName.c_str(),
        mon.adapterId, mon.sourceId,
        TRUE,
        advColor);

    return err == ERROR_SUCCESS;
}

// ============================================================================
// Gamma ramp utilities
// ============================================================================

static void MakeIdentityRamp(WORD ramp[3][256])
{
    for (int ch = 0; ch < 3; ch++)
        for (int i = 0; i < 256; i++)
            ramp[ch][i] = static_cast<WORD>(i * 257); // 0->0, 255->65535
}

static bool IsIdentityRamp(const WORD ramp[3][256])
{
    for (int ch = 0; ch < 3; ch++)
        for (int i = 0; i < 256; i++)
            if (std::abs(static_cast<int>(ramp[ch][i]) - i * 257) > 2)
                return false;
    return true;
}

// Compare two ramps; returns true if they match within tolerance
static bool RampsEqual(const WORD a[3][256], const WORD b[3][256], int tolerance = 2)
{
    for (int ch = 0; ch < 3; ch++)
        for (int i = 0; i < 256; i++)
            if (std::abs(static_cast<int>(a[ch][i]) - static_cast<int>(b[ch][i])) > tolerance)
                return false;
    return true;
}

// ============================================================================
// Process a single monitor
// ============================================================================

static bool ProcessMonitor(const MonitorInfo& mon, const std::wstring& manualProfile,
                           bool resetMode, bool restoreMode,
                           bool setGpuMode, const std::wstring& setGpuProfile,
                           const std::vector<bool>& hdrModes,
                           bool forceApply, bool verbose, const GpuColorProfileApi& gpuApi)
{
    wprintf(L"--- %s (%s) ---\n",
            mon.friendlyName.empty() ? L"(unknown)" : mon.friendlyName.c_str(),
            mon.gdiDeviceName.c_str());

    // Create device context for this display
    HDC hDC = CreateDCW(L"DISPLAY", mon.gdiDeviceName.c_str(), nullptr, nullptr);
    if (!hDC)
    {
        wprintf(L"  ERROR: CreateDC failed (err=%lu)\n\n", GetLastError());
        return false;
    }

    // Reset mode: unset GPU profile + load identity gamma ramp
    if (resetMode)
    {
        // 1) Reset the gamma ramp to identity
        WORD identity[3][256];
        MakeIdentityRamp(identity);
        bool rampOk = SetDeviceGammaRamp(hDC, identity) != 0;
        DeleteDC(hDC);

        if (rampOk)
            wprintf(L"  Gamma ramp reset to identity.\n");
        else
            wprintf(L"  WARNING: SetDeviceGammaRamp failed (err=%lu)\n", GetLastError());

        // 2) Unset the GPU-level ICC profile (stops MHC2 pipeline)
        if (gpuApi.loaded)
        {
            bool gpuReset = false;
            for (bool hdr : hdrModes)
            {
                std::wstring prof = GetGpuDefaultProfile(gpuApi, mon, hdr, verbose);
                if (!prof.empty())
                {
                    wprintf(L"  GPU %s profile: %s\n", hdr ? L"HDR" : L"SDR", prof.c_str());
                    if (UnsetGpuProfile(gpuApi, mon, prof, hdr, verbose))
                    {
                        wprintf(L"  GPU %s profile unset (removed as default).\n",
                                hdr ? L"HDR" : L"SDR");
                        gpuReset = true;
                    }
                    else
                        wprintf(L"  WARNING: Could not unset GPU %s profile.\n",
                                hdr ? L"HDR" : L"SDR");
                }
            }
            if (!gpuReset)
                wprintf(L"  No GPU color profile was active.\n");
        }
        else
        {
            wprintf(L"  NOTE: ColorProfile APIs not available (Win10 1903+ required)\n");
            wprintf(L"        GPU color pipeline was not reset.\n");
        }

        wprintf(L"\n");
        return rampOk;
    }

    // Restore mode: re-enable GPU color profiles as default
    if (restoreMode)
    {
        DeleteDC(hDC);

        if (!gpuApi.loaded)
        {
            wprintf(L"  ERROR: ColorProfile APIs not available (Win10 1903+ required)\n\n");
            return false;
        }

        bool anyRestored = false;
        for (bool hdr : hdrModes)
        {
            std::wstring prof = GetGpuDefaultProfile(gpuApi, mon, hdr, verbose);
            if (!prof.empty())
            {
                if (RestoreGpuProfile(gpuApi, mon, prof, hdr, verbose))
                {
                    wprintf(L"  GPU %s profile restored: %s\n",
                            hdr ? L"HDR" : L"SDR", prof.c_str());
                    anyRestored = true;
                }
                else
                    wprintf(L"  WARNING: Could not restore GPU %s profile: %s\n",
                            hdr ? L"HDR" : L"SDR", prof.c_str());
            }
        }

        if (!anyRestored)
            wprintf(L"  No GPU color profiles found to restore.\n");

        if (anyRestored)
        {
            BOOL prevState = FALSE;
            if (WcsGetCalibrationManagementState(&prevState))
            {
                WcsSetCalibrationManagementState(FALSE);
                Sleep(100);
                WcsSetCalibrationManagementState(TRUE);
            }
            if (gpuApi.RefreshCalibration)
                gpuApi.RefreshCalibration();
        }

        wprintf(L"\n");
        return anyRestored;
    }

    // Set GPU profile mode: set a named profile as HDR default, or kick both SDR+HDR
    if (setGpuMode)
    {
        DeleteDC(hDC);

        if (!gpuApi.loaded)
        {
            wprintf(L"  ERROR: ColorProfile APIs not available (Win10 1903+ required)\n\n");
            return false;
        }

        // Step 1: Ensure calibration management is ON *before* profile operations.
        // The MHC2 docs say the pipeline loader must be explicitly turned on.
        BOOL calState = FALSE;
        WcsGetCalibrationManagementState(&calState);
        if (!calState)
        {
            BOOL setOk = WcsSetCalibrationManagementState(TRUE);
            if (setOk)
                wprintf(L"  Calibration management: enabled (was OFF).\n");
            else
                wprintf(L"  WARNING: WcsSetCalibrationManagementState(TRUE) failed (err=%lu)\n",
                        GetLastError());
            Sleep(200);
        }
        else if (verbose)
            wprintf(L"  Calibration management: already ON.\n");

        bool anyOk = false;

        for (bool hdr : hdrModes)
        {
            // Resolve the profile name for this pipeline
            std::wstring prof = setGpuProfile;
            if (prof.empty())
            {
                prof = GetGpuDefaultProfile(gpuApi, mon, hdr, verbose);
                if (prof.empty())
                {
                    if (verbose)
                        wprintf(L"  No GPU %s profile set.\n", hdr ? L"HDR" : L"SDR");
                    continue;
                }
            }

            const wchar_t* pipeLabel = hdr ? L"HDR" : L"SDR";
            const wchar_t* actionLabel = setGpuProfile.empty() ? L"re-applied" : L"set to";
            auto subtype = hdr ? CPST_EXTENDED_DISPLAY : CPST_STANDARD_DISPLAY;
            bool ok = false;

            // Method A: WcsSetDefaultColorProfile (the old colorcpl.exe API).
            // Uses device name, completely different code path from newer APIs.
            if (!ok && !mon.eddDeviceId.empty())
            {
                BOOL wcsOk = WcsSetDefaultColorProfile(
                    WCS_PROFILE_MANAGEMENT_SCOPE_CURRENT_USER,
                    mon.eddDeviceId.c_str(),
                    CPT_ICC, subtype,
                    0,  // dwProfileID
                    prof.c_str());

                if (wcsOk)
                {
                    wprintf(L"  GPU %s profile %s: %s  (WcsSetDefault)\n",
                            pipeLabel, actionLabel, prof.c_str());
                    ok = true;
                }
                else if (verbose)
                    wprintf(L"  WcsSetDefaultColorProfile %s failed (err=%lu)\n",
                            pipeLabel, GetLastError());
            }

            // Method B: ColorProfileSetDisplayDefaultAssociation (newer API)
            if (!ok && gpuApi.SetDefault)
            {
                HRESULT hr = gpuApi.SetDefault(
                    WCS_PROFILE_MANAGEMENT_SCOPE_CURRENT_USER,
                    prof.c_str(),
                    CPT_ICC, subtype,
                    mon.adapterId, mon.sourceId);

                if (SUCCEEDED(hr))
                {
                    wprintf(L"  GPU %s profile %s: %s  (SetDefault)\n",
                            pipeLabel, actionLabel, prof.c_str());
                    ok = true;
                }
                else if (verbose)
                    wprintf(L"  ColorProfileSetDisplayDefaultAssociation %s failed (hr=0x%08lX)\n",
                            pipeLabel, hr);
            }

            // Method C: Remove + Add with setAsDefault=true
            if (!ok)
            {
                BOOL advColor = hdr ? TRUE : FALSE;

                gpuApi.Remove(
                    WCS_PROFILE_MANAGEMENT_SCOPE_CURRENT_USER,
                    prof.c_str(),
                    mon.adapterId, mon.sourceId,
                    advColor);

                Sleep(200);

                LONG err = gpuApi.Add(
                    WCS_PROFILE_MANAGEMENT_SCOPE_CURRENT_USER,
                    prof.c_str(),
                    mon.adapterId, mon.sourceId,
                    TRUE,
                    advColor);

                if (err == ERROR_SUCCESS)
                {
                    wprintf(L"  GPU %s profile %s: %s  (Remove+Add)\n",
                            pipeLabel, actionLabel, prof.c_str());
                    ok = true;
                }
                else
                {
                    wprintf(L"  ERROR: Failed to set GPU %s profile (err=%ld)\n",
                            pipeLabel, err);
                    if (err == ERROR_FILE_NOT_FOUND || err == 2)
                        wprintf(L"  HINT: Is the profile installed? Check spool\\drivers\\color\n");
                }
            }

            if (ok) anyOk = true;
        }

        if (!anyOk && setGpuProfile.empty())
        {
            wprintf(L"  ERROR: No GPU profiles found to re-apply.\n");
            wprintf(L"  Use -s <name> to specify a profile.\n");
        }

        // Post-profile activation: trigger calibration reload via all known mechanisms
        if (anyOk)
        {
            wprintf(L"  Activating calibration pipeline (please wait)...\n");

            // A) Toggle calibration management OFF/ON to force re-evaluation
            WcsSetCalibrationManagementState(FALSE);
            Sleep(100);
            BOOL setOk = WcsSetCalibrationManagementState(TRUE);
            if (verbose)
                wprintf(L"  Calibration management toggle: %s\n",
                        setOk ? L"OK" : L"failed");

            // B) InternalRefreshCalibration (undocumented reload trigger)
            if (gpuApi.RefreshCalibration)
            {
                gpuApi.RefreshCalibration();
                if (verbose)
                    wprintf(L"  InternalRefreshCalibration called.\n");
            }

            // C) Broadcast WM_SETTINGCHANGE with "ImmersiveColorSet" -
            //    the system message Windows sends when HDR/color settings change.
            DWORD_PTR result = 0;
            SendMessageTimeoutW(HWND_BROADCAST, WM_SETTINGCHANGE, 0,
                                (LPARAM)L"ImmersiveColorSet",
                                SMTO_ABORTIFHUNG | SMTO_NOTIMEOUTIFNOTHUNG,
                                5000, &result);
            if (verbose)
                wprintf(L"  WM_SETTINGCHANGE ImmersiveColorSet broadcast sent.\n");

            // D) Re-apply display config (triggers WM_DISPLAYCHANGE)
            UINT32 pathCount = 0, modeCount = 0;
            if (GetDisplayConfigBufferSizes(QDC_ONLY_ACTIVE_PATHS,
                                            &pathCount, &modeCount) == ERROR_SUCCESS)
            {
                std::vector<DISPLAYCONFIG_PATH_INFO> paths(pathCount);
                std::vector<DISPLAYCONFIG_MODE_INFO> modes(modeCount);
                if (QueryDisplayConfig(QDC_ONLY_ACTIVE_PATHS,
                                       &pathCount, paths.data(),
                                       &modeCount, modes.data(),
                                       nullptr) == ERROR_SUCCESS)
                {
                    paths.resize(pathCount);
                    modes.resize(modeCount);
                    LONG dcErr = SetDisplayConfig(
                        (UINT32)paths.size(), paths.data(),
                        (UINT32)modes.size(), modes.data(),
                        SDC_APPLY | SDC_USE_SUPPLIED_DISPLAY_CONFIG | SDC_ALLOW_CHANGES);
                    if (verbose)
                        wprintf(L"  SetDisplayConfig refresh: %s (err=%ld)\n",
                                dcErr == ERROR_SUCCESS ? L"OK" : L"failed", dcErr);
                }
            }

            // E) Run the Windows Calibration Loader scheduled task directly.
            //    This is the official Windows mechanism for loading calibrations.
            wprintf(L"  Triggering Calibration Loader task...\n");
            STARTUPINFOW si = {};
            si.cb = sizeof(si);
            si.dwFlags = STARTF_USESHOWWINDOW;
            si.wShowWindow = SW_HIDE;
            PROCESS_INFORMATION pi = {};
            wchar_t cmdLine[] = L"schtasks.exe /run /tn \"\\Microsoft\\Windows\\WindowsColorSystem\\Calibration Loader\"";
            if (CreateProcessW(nullptr, cmdLine, nullptr, nullptr, FALSE,
                               CREATE_NO_WINDOW, nullptr, nullptr, &si, &pi))
            {
                WaitForSingleObject(pi.hProcess, 5000);
                DWORD exitCode = 0;
                GetExitCodeProcess(pi.hProcess, &exitCode);
                CloseHandle(pi.hProcess);
                CloseHandle(pi.hThread);
                if (verbose)
                    wprintf(L"  Calibration Loader task: exit code %lu\n", exitCode);
            }
            else if (verbose)
                wprintf(L"  Calibration Loader task: CreateProcess failed (err=%lu)\n",
                        GetLastError());
        }

        wprintf(L"\n");
        return anyOk;
    }

    // Resolve ICC profile path
    std::wstring profilePath;
    if (!manualProfile.empty())
    {
        profilePath = manualProfile;
    }
    else
    {
        std::wstring profileName = GetDefaultProfile(mon, verbose);
        if (profileName.empty())
        {
            wprintf(L"  No default ICC profile found for this display.\n");
            if (verbose)
            {
                wprintf(L"    devicePath: %s\n",
                        mon.devicePath.empty() ? L"(empty)" : mon.devicePath.c_str());
                wprintf(L"    eddDeviceId: %s\n",
                        mon.eddDeviceId.empty() ? L"(empty)" : mon.eddDeviceId.c_str());
            }
            wprintf(L"  Use -p <path> to specify a profile manually.\n\n");
            DeleteDC(hDC);
            return true; // not an error, just no profile
        }
        profilePath = ResolveProfilePath(profileName);
    }

    wprintf(L"  Input: %s\n", profilePath.c_str());

    // Load file data
    auto fileData = LoadFile(profilePath);
    if (fileData.empty())
    {
        wprintf(L"  ERROR: Could not read file.\n\n");
        DeleteDC(hDC);
        return false;
    }

    WORD gammaRamp[3][256] = {};
    bool lutFound = false;
    const char* lutSource = nullptr;

    if (IsCubeFile(profilePath))
    {
        // ---- .cube LUT file path ----
        int cubeLutSize = 0;
        if (ParseCubeFile(fileData, gammaRamp, cubeLutSize, verbose))
        {
            lutFound  = true;
            lutSource = "cube";
            wprintf(L"  LUT source: .cube 1D LUT (%d entries", cubeLutSize);
            if (cubeLutSize > 256)
                wprintf(L" -> downsampled to 256 for SetDeviceGammaRamp");
            wprintf(L")\n");

            if (cubeLutSize > 256)
            {
                wprintf(L"  NOTE: SetDeviceGammaRamp is limited to 256 entries per channel.\n");
                wprintf(L"        For full %d-entry precision, use the MHC2 ICC profile\n", cubeLutSize);
                wprintf(L"        pipeline via Windows Color Management.\n");
            }
        }
        else
        {
            wprintf(L"  ERROR: Failed to parse .cube file.\n\n");
            DeleteDC(hDC);
            return false;
        }
    }
    else
    {
        // ---- ICC profile path ----
        if (verbose)
            wprintf(L"  ICC file size: %zu bytes\n", fileData.size());

        // Parse tag table
        auto tags = ParseIccTags(fileData);
        if (tags.empty())
        {
            wprintf(L"  ERROR: Invalid ICC profile (bad header or no tags).\n\n");
            DeleteDC(hDC);
            return false;
        }

        if (verbose)
        {
            wprintf(L"  ICC tags (%zu):", tags.size());
            for (const auto& t : tags)
            {
                char sig[5] = {};
                sig[0] = static_cast<char>((t.signature >> 24) & 0xFF);
                sig[1] = static_cast<char>((t.signature >> 16) & 0xFF);
                sig[2] = static_cast<char>((t.signature >>  8) & 0xFF);
                sig[3] = static_cast<char>( t.signature        & 0xFF);
                wprintf(L" %S", sig);
            }
            wprintf(L"\n");
        }

        // 1) Try VCGT (standard approach for monitor calibration profiles)
        const IccTagEntry* vcgtTag = FindTag(tags, TAG_VCGT);
        if (vcgtTag)
        {
            if (verbose)
                wprintf(L"  Found vcgt tag (offset=%u, size=%u)\n",
                        vcgtTag->offset, vcgtTag->size);

            if (ParseVcgt(fileData, *vcgtTag, gammaRamp))
            {
                lutFound  = true;
                lutSource = "vcgt";
            }
            else
                wprintf(L"  WARNING: vcgt tag present but failed to parse.\n");
        }

        // 2) Fallback: MHC2 RegammaLUT (ColorControl profiles)
        if (!lutFound)
        {
            const IccTagEntry* mhc2Tag = FindTag(tags, TAG_MHC2);
            if (mhc2Tag)
            {
                if (verbose)
                    wprintf(L"  Found MHC2 tag (offset=%u, size=%u)\n",
                            mhc2Tag->offset, mhc2Tag->size);

                if (ParseMhc2Lut(fileData, *mhc2Tag, gammaRamp))
                {
                    lutFound  = true;
                    lutSource = "MHC2";
                }
                else
                    wprintf(L"  WARNING: MHC2 tag present but failed to parse.\n");
            }
        }

        if (!lutFound)
        {
            wprintf(L"  ERROR: No vcgt or MHC2 LUT found in profile.\n\n");
            DeleteDC(hDC);
            return false;
        }

        wprintf(L"  LUT source: %S\n", lutSource);
    }

    if (IsIdentityRamp(gammaRamp))
        wprintf(L"  NOTE: LUT is an identity (no-op) ramp.\n");

    if (verbose)
    {
        wprintf(L"  Ramp samples [index] = (R, G, B):\n");
        const int samples[] = { 0, 1, 32, 64, 128, 192, 254, 255 };
        for (int idx : samples)
        {
            wprintf(L"    [%3d] = (%5u, %5u, %5u)  expect ~%5d\n",
                    idx, gammaRamp[0][idx], gammaRamp[1][idx], gammaRamp[2][idx],
                    idx * 257);
        }
    }

    // Read current gamma ramp to detect if LUT is already loaded
    WORD currentRamp[3][256] = {};
    if (GetDeviceGammaRamp(hDC, currentRamp))
    {
        bool currentIsIdentity = IsIdentityRamp(currentRamp);
        bool alreadyApplied    = RampsEqual(currentRamp, gammaRamp);

        if (verbose)
        {
            wprintf(L"  Current ramp: %s\n",
                    currentIsIdentity ? L"identity (linear)" : L"non-identity (LUT active)");
        }

        if (alreadyApplied)
        {
            wprintf(L"  LUT is already loaded. Nothing to do.\n\n");
            DeleteDC(hDC);
            return true;
        }

        if (!currentIsIdentity && !forceApply)
        {
            wprintf(L"  WARNING: A non-identity gamma ramp is already active.\n");
            wprintf(L"  The GPU driver or Windows may have already applied the LUT.\n");
            wprintf(L"  Applying again would double the correction.\n");
            wprintf(L"  Use -f to force, or -r to reset first.\n\n");
            DeleteDC(hDC);
            return true; // not a failure, just skipped
        }

        if (!currentIsIdentity && forceApply)
            wprintf(L"  NOTE: Overwriting existing non-identity ramp (--force).\n");
    }

    // Apply
    if (SetDeviceGammaRamp(hDC, gammaRamp))
    {
        wprintf(L"  Applied gamma ramp successfully.\n\n");
        DeleteDC(hDC);
        return true;
    }
    else
    {
        DWORD err = GetLastError();
        wprintf(L"  ERROR: SetDeviceGammaRamp failed (err=%lu)\n", err);
        if (err == ERROR_ACCESS_DENIED)
            wprintf(L"  HINT: Try running as Administrator, or check GPO restrictions.\n");
        wprintf(L"\n");
        DeleteDC(hDC);
        return false;
    }
}

// ============================================================================
// DWM Dithering DLL injection
// ============================================================================

static constexpr const wchar_t* DITHER_DLL_NAME = L"ApplyIccLut_Dither.dll";
static constexpr const wchar_t* DITHER_FLAG_FILE = L"ApplyIccLut_dither.flag";
static constexpr const wchar_t* DITHER_CFG_FILE = L"ApplyIccLut_dither.cfg";
static constexpr const wchar_t* DITHER_CACHE_FILE = L"ApplyIccLut_dither_cache.dat";

// ============================================================================
// True Windows version detection via RtlGetVersion (ntdll.dll)
//
// VerifyVersionInfo lies unless the exe has a supportedOS GUID in its manifest.
// RtlGetVersion always returns the real version regardless of manifest.
// ============================================================================

struct WindowsVersion { DWORD major, minor, build; };
static WindowsVersion GetTrueWindowsVersion()
{
    typedef LONG(WINAPI* RtlGetVersion_t)(OSVERSIONINFOW*);
    auto fn = (RtlGetVersion_t)GetProcAddress(GetModuleHandleW(L"ntdll.dll"), "RtlGetVersion");
    OSVERSIONINFOW osvi = {};
    osvi.dwOSVersionInfoSize = sizeof(osvi);
    if (fn && fn(&osvi) == 0)
        return { osvi.dwMajorVersion, osvi.dwMinorVersion, osvi.dwBuildNumber };
    return {};
}

static std::wstring GetSystemTempPath()
{
    wchar_t sysRoot[MAX_PATH];
    GetEnvironmentVariableW(L"SYSTEMROOT", sysRoot, MAX_PATH);
    return std::wstring(sysRoot) + L"\\Temp\\";
}

// ============================================================================
// Remote diagnostic scanning — reads dwmcore.dll from dwm.exe to diagnose
// pattern matching when DLL injection fails
// ============================================================================

// AOB pattern matching (returns false on match, true on mismatch — same as DLL)
// IOverlaySwapChain offsets (version-specific, passed to DLL via config)
// These are struct member offsets that PDB symbols don't provide.
static const int OVERLAY_SWAPCHAIN_OFFSET_W10     = -0x118;
static const int OVERLAY_SWAPCHAIN_OFFSET_W11     = 0xE0;
static const int OVERLAY_SWAPCHAIN_OFFSET_W11_24H2 = 0x108; // 24H2+: direct offset on overlaySwapChain
static const int OVERLAY_HWPROT_OFFSET_W10     = -0xbc;
static const int OVERLAY_HWPROT_OFFSET_W11     = -0x144;
static const int OVERLAY_HWPROT_OFFSET_W11_24H2 = 0x64;   // 24H2+: direct offset on overlaySwapChain

// Binary config struct shared with DitherDll (must match exactly)
#pragma pack(push, 1)
struct DitherConfig {
    UINT32 magic;              // 'DITH' (0x48544944)
    UINT32 version;            // struct version (2)
    INT64  presentOffset;      // COverlayContext::Present offset from dwmcore base
    INT64  directFlipOffset;   // IsCandidateDirectFlipCompatible offset from dwmcore base
    INT64  overlaysOffset;     // OverlaysEnabled offset from dwmcore base
    INT32  hwProtOffset;       // IOverlaySwapChain HardwareProtected offset
    INT32  swapChainOffset;    // IOverlaySwapChain IDXGISwapChain offset
    UINT32 isWindows11;        // 1 if Win11+ (affects overlay pointer resolution)
    UINT32 ditherBits;         // 0 = auto, otherwise forced bit depth
    UINT32 isWindows11_24h2;   // 1 if Win11 24H2+ (different function signatures & access pattern)
};
#pragma pack(pop)

static const UINT32 DITHER_CONFIG_MAGIC = 0x48544944;
static const UINT32 DITHER_CONFIG_VERSION = 2;

// ============================================================================
// Offset cache — caches discovered offsets keyed by dwmcore.dll file version
// so subsequent runs don't need to re-scan dwm.exe memory.
// Auto-invalidates when dwmcore.dll is updated (e.g. Windows Update).
// ============================================================================

static const UINT32 OFFSET_CACHE_MAGIC = 0x4843434F; // 'OCCH'
static const UINT32 OFFSET_CACHE_VERSION = 1;

#pragma pack(push, 1)
struct OffsetCacheEntry {
    UINT32 magic;               // 'OCCH' (0x4843434F)
    UINT32 version;             // 1
    UINT32 dwmcoreVersionMS;    // from VS_FIXEDFILEINFO
    UINT32 dwmcoreVersionLS;
    UINT32 dwmcoreSizeBytes;    // module size (extra validation)
    UINT32 osBuild;
    INT64  presentOffset;
    INT64  directFlipOffset;
    INT64  overlaysOffset;
    INT32  hwProtOffset;
    INT32  swapChainOffset;
    UINT32 isWindows11;
    UINT32 sourceFlags;         // 1=PDB-resolved
};
#pragma pack(pop)

static bool GetDwmcoreFileVersion(UINT32* versionMS, UINT32* versionLS, UINT32* fileSize)
{
    wchar_t sysDir[MAX_PATH];
    GetSystemDirectoryW(sysDir, MAX_PATH);
    std::wstring path = std::wstring(sysDir) + L"\\dwmcore.dll";

    // Get file size
    WIN32_FILE_ATTRIBUTE_DATA fad = {};
    if (GetFileAttributesExW(path.c_str(), GetFileExInfoStandard, &fad))
        *fileSize = fad.nFileSizeLow;
    else
        *fileSize = 0;

    // Get version info
    DWORD dummy = 0;
    DWORD verSize = GetFileVersionInfoSizeW(path.c_str(), &dummy);
    if (verSize == 0) return false;

    std::vector<BYTE> verData(verSize);
    if (!GetFileVersionInfoW(path.c_str(), 0, verSize, verData.data()))
        return false;

    VS_FIXEDFILEINFO* ffi = nullptr;
    UINT ffiLen = 0;
    if (!VerQueryValueW(verData.data(), L"\\", (void**)&ffi, &ffiLen) || !ffi)
        return false;

    *versionMS = ffi->dwFileVersionMS;
    *versionLS = ffi->dwFileVersionLS;
    return true;
}

static bool LoadOffsetCache(OffsetCacheEntry* entry)
{
    std::wstring cachePath = GetSystemTempPath() + DITHER_CACHE_FILE;
    HANDLE hFile = CreateFileW(cachePath.c_str(), GENERIC_READ, FILE_SHARE_READ,
                               NULL, OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, NULL);
    if (hFile == INVALID_HANDLE_VALUE) return false;

    DWORD bytesRead = 0;
    BOOL ok = ReadFile(hFile, entry, sizeof(*entry), &bytesRead, NULL);
    CloseHandle(hFile);

    if (!ok || bytesRead != sizeof(*entry)) return false;
    if (entry->magic != OFFSET_CACHE_MAGIC || entry->version != OFFSET_CACHE_VERSION)
        return false;

    // Validate against current dwmcore.dll version
    UINT32 curMS = 0, curLS = 0, curSize = 0;
    if (!GetDwmcoreFileVersion(&curMS, &curLS, &curSize)) return false;

    if (entry->dwmcoreVersionMS != curMS || entry->dwmcoreVersionLS != curLS ||
        entry->dwmcoreSizeBytes != curSize)
        return false;

    return true;
}

static bool SaveOffsetCache(const OffsetCacheEntry* entry)
{
    std::wstring cachePath = GetSystemTempPath() + DITHER_CACHE_FILE;
    HANDLE hFile = CreateFileW(cachePath.c_str(), GENERIC_WRITE, 0, NULL,
                               CREATE_ALWAYS, FILE_ATTRIBUTE_NORMAL, NULL);
    if (hFile == INVALID_HANDLE_VALUE) return false;

    DWORD written = 0;
    BOOL ok = WriteFile(hFile, entry, sizeof(*entry), &written, NULL);
    CloseHandle(hFile);
    return ok && written == sizeof(*entry);
}

// ============================================================================
// PDB symbol resolution — download dwmcore.pdb from Microsoft Symbol Server
// and resolve COverlayContext function RVAs by name.
// ============================================================================

struct PdbResolvedOffsets {
    INT64 presentRva;
    INT64 directFlipRva;
    INT64 overlaysRva;
    bool presentFound;
    bool directFlipFound;
    bool overlaysFound;
};

struct SymEnumCtx {
    PdbResolvedOffsets* offsets;
    DWORD64 moduleBase;
    bool verbose;
    int totalCount;
};

static BOOL CALLBACK PdbSymEnumCallback(PSYMBOL_INFO pSymInfo, ULONG SymbolSize, PVOID UserContext)
{
    auto ctx = (SymEnumCtx*)UserContext;
    DWORD64 rva = pSymInfo->Address - ctx->moduleBase;
    ctx->totalCount++;

    if (ctx->verbose)
        wprintf(L"    PDB sym: %S @ RVA 0x%llx\n", pSymInfo->Name, (UINT64)rva);

    // Match exactly "::Present" — the method name must end there (no PresentOverlay, etc.)
    {
        const char* match = strstr(pSymInfo->Name, "::Present");
        if (match)
        {
            char after = match[9]; // character after "::Present"
            if (after == '\0' || after == '(' || after == '@' || after == ' ')
            {
                wprintf(L"    PDB: >> Present candidate @ RVA 0x%llx (%S)\n", (UINT64)rva, pSymInfo->Name);
                if (!ctx->offsets->presentFound) {
                    ctx->offsets->presentRva = (INT64)rva;
                    ctx->offsets->presentFound = true;
                }
            }
        }
    }

    // Match "::IsCandidateDirectFlipCompat" — require :: prefix for exact method name
    if (strstr(pSymInfo->Name, "::IsCandidateDirectFlipCompat"))
    {
        wprintf(L"    PDB: >> DirectFlip candidate @ RVA 0x%llx (%S)\n", (UINT64)rva, pSymInfo->Name);
        if (!ctx->offsets->directFlipFound) {
            ctx->offsets->directFlipRva = (INT64)rva;
            ctx->offsets->directFlipFound = true;
        }
    }

    // Match exactly "::OverlaysEnabled" — exclude RGBOverlaysEnabled etc.
    {
        const char* match = strstr(pSymInfo->Name, "OverlaysEnabled");
        if (match && match >= pSymInfo->Name + 2 &&
            match[-1] == ':' && match[-2] == ':')
        {
            wprintf(L"    PDB: >> OverlaysEnabled candidate @ RVA 0x%llx (%S)\n", (UINT64)rva, pSymInfo->Name);
            if (!ctx->offsets->overlaysFound) {
                ctx->offsets->overlaysRva = (INT64)rva;
                ctx->offsets->overlaysFound = true;
            }
        }
    }

    return TRUE;
}

// ============================================================================
// PDB download via HTTP — parse PE debug directory for GUID, download from
// Microsoft Symbol Server, then load locally with system dbghelp.dll.
// No SDK dependencies required.
// ============================================================================

// Extract PDB GUID and age from a PE file's debug directory
struct PdbInfo {
    GUID guid;
    DWORD age;
    char pdbName[MAX_PATH];
    bool valid;
};

static PdbInfo GetPdbInfoFromDll(const wchar_t* dllPath)
{
    PdbInfo info = {};

    HANDLE hFile = CreateFileW(dllPath, GENERIC_READ, FILE_SHARE_READ, NULL, OPEN_EXISTING, 0, NULL);
    if (hFile == INVALID_HANDLE_VALUE) return info;

    HANDLE hMap = CreateFileMappingW(hFile, NULL, PAGE_READONLY, 0, 0, NULL);
    if (!hMap) { CloseHandle(hFile); return info; }

    const unsigned char* base = (const unsigned char*)MapViewOfFile(hMap, FILE_MAP_READ, 0, 0, 0);
    if (!base) { CloseHandle(hMap); CloseHandle(hFile); return info; }

    // Parse PE headers
    const IMAGE_DOS_HEADER* dosHdr = (const IMAGE_DOS_HEADER*)base;
    if (dosHdr->e_magic != IMAGE_DOS_SIGNATURE) goto done;

    {
        const IMAGE_NT_HEADERS64* ntHdr = (const IMAGE_NT_HEADERS64*)(base + dosHdr->e_lfanew);
        if (ntHdr->Signature != IMAGE_NT_SIGNATURE) goto done;

        DWORD debugRva = ntHdr->OptionalHeader.DataDirectory[IMAGE_DIRECTORY_ENTRY_DEBUG].VirtualAddress;
        DWORD debugSize = ntHdr->OptionalHeader.DataDirectory[IMAGE_DIRECTORY_ENTRY_DEBUG].Size;
        if (!debugRva || !debugSize) goto done;

        // Convert RVA to file offset using section headers
        const IMAGE_SECTION_HEADER* sections = IMAGE_FIRST_SECTION(ntHdr);
        DWORD debugFileOffset = 0;
        for (WORD i = 0; i < ntHdr->FileHeader.NumberOfSections; i++)
        {
            if (debugRva >= sections[i].VirtualAddress &&
                debugRva < sections[i].VirtualAddress + sections[i].SizeOfRawData)
            {
                debugFileOffset = debugRva - sections[i].VirtualAddress + sections[i].PointerToRawData;
                break;
            }
        }
        if (!debugFileOffset) goto done;

        int numEntries = debugSize / sizeof(IMAGE_DEBUG_DIRECTORY);
        auto debugDir = (const IMAGE_DEBUG_DIRECTORY*)(base + debugFileOffset);

        for (int i = 0; i < numEntries; i++)
        {
            if (debugDir[i].Type == IMAGE_DEBUG_TYPE_CODEVIEW)
            {
                const unsigned char* cvData = base + debugDir[i].PointerToRawData;
                // CV_INFO_PDB70: signature "RSDS" + GUID(16) + age(4) + filename
                if (cvData[0] == 'R' && cvData[1] == 'S' && cvData[2] == 'D' && cvData[3] == 'S')
                {
                    memcpy(&info.guid, cvData + 4, sizeof(GUID));
                    info.age = *(const DWORD*)(cvData + 20);
                    lstrcpynA(info.pdbName, (const char*)(cvData + 24), MAX_PATH);
                    info.valid = true;
                }
                break;
            }
        }
    } // ntHdr scope

done:
    UnmapViewOfFile(base);
    CloseHandle(hMap);
    CloseHandle(hFile);
    return info;
}

static bool DownloadPdbFromSymbolServer(const PdbInfo& pdbInfo, const char* localDir, char* outPath)
{
    // Format GUID as uppercase hex without dashes (32 chars)
    char guidStr[64];
    wsprintfA(guidStr, "%08X%04X%04X%02X%02X%02X%02X%02X%02X%02X%02X%d",
              pdbInfo.guid.Data1, pdbInfo.guid.Data2, pdbInfo.guid.Data3,
              pdbInfo.guid.Data4[0], pdbInfo.guid.Data4[1],
              pdbInfo.guid.Data4[2], pdbInfo.guid.Data4[3],
              pdbInfo.guid.Data4[4], pdbInfo.guid.Data4[5],
              pdbInfo.guid.Data4[6], pdbInfo.guid.Data4[7],
              pdbInfo.age);

    // Build local path: <cache>/<pdbname>/<guidStr>/<pdbname>
    char subDir[MAX_PATH];
    wsprintfA(subDir, "%s\\%s", localDir, pdbInfo.pdbName);
    CreateDirectoryA(subDir, NULL);
    wsprintfA(subDir, "%s\\%s\\%s", localDir, pdbInfo.pdbName, guidStr);
    CreateDirectoryA(subDir, NULL);

    wsprintfA(outPath, "%s\\%s", subDir, pdbInfo.pdbName);

    // Check if already cached
    if (GetFileAttributesA(outPath) != INVALID_FILE_ATTRIBUTES)
    {
        wprintf(L"    PDB: Using cached %S\n", outPath);
        return true;
    }

    // Download from symbol server
    char url[1024];
    wsprintfA(url, "https://msdl.microsoft.com/download/symbols/%s/%s/%s",
              pdbInfo.pdbName, guidStr, pdbInfo.pdbName);

    wprintf(L"    PDB: Downloading %S\n", url);
    HRESULT hr = URLDownloadToFileA(NULL, url, outPath, 0, NULL);
    if (FAILED(hr))
    {
        wprintf(L"    PDB: Download failed (hr=0x%08X)\n", hr);
        DeleteFileA(outPath);
        return false;
    }

    wprintf(L"    PDB: Downloaded to %S\n", outPath);
    return true;
}

static PdbResolvedOffsets ResolveDwmcorePdb(bool verbose)
{
    PdbResolvedOffsets result = {};

    wchar_t sysDir[MAX_PATH];
    GetSystemDirectoryW(sysDir, MAX_PATH);
    std::wstring dllPath = std::wstring(sysDir) + L"\\dwmcore.dll";

    wprintf(L"    PDB: Resolving symbols for %s\n", dllPath.c_str());

    // Step 1: Extract PDB GUID from dwmcore.dll PE headers
    PdbInfo pdbInfo = GetPdbInfoFromDll(dllPath.c_str());
    if (!pdbInfo.valid)
    {
        wprintf(L"    PDB: Could not extract PDB info from PE debug directory\n");
        return result;
    }
    wprintf(L"    PDB: %S GUID={%08X-%04X-%04X-%02X%02X%02X%02X%02X%02X%02X%02X} age=%u\n",
            pdbInfo.pdbName,
            pdbInfo.guid.Data1, pdbInfo.guid.Data2, pdbInfo.guid.Data3,
            pdbInfo.guid.Data4[0], pdbInfo.guid.Data4[1],
            pdbInfo.guid.Data4[2], pdbInfo.guid.Data4[3],
            pdbInfo.guid.Data4[4], pdbInfo.guid.Data4[5],
            pdbInfo.guid.Data4[6], pdbInfo.guid.Data4[7],
            pdbInfo.age);

    // Step 2: Download PDB from symbol server
    char localCache[MAX_PATH];
    ExpandEnvironmentStringsA("%LOCALAPPDATA%\\Temp\\SymbolCache", localCache, MAX_PATH);
    CreateDirectoryA(localCache, NULL);

    char pdbPath[MAX_PATH];
    if (!DownloadPdbFromSymbolServer(pdbInfo, localCache, pdbPath))
        return result;

    // Step 3: Load symbols from the local PDB with system dbghelp.dll
    DWORD opts = SymGetOptions();
    SymSetOptions((opts | SYMOPT_UNDNAME | SYMOPT_AUTO_PUBLICS) & ~SYMOPT_DEFERRED_LOADS);

    HANDLE hSym = (HANDLE)(uintptr_t)0xDEAD0001;

    // Point symbol path at the directory containing the downloaded PDB
    char pdbDir[MAX_PATH];
    lstrcpynA(pdbDir, pdbPath, MAX_PATH);
    char* lastSlash = strrchr(pdbDir, '\\');
    if (lastSlash) *lastSlash = '\0';

    if (!SymInitialize(hSym, pdbDir, FALSE))
    {
        wprintf(L"    PDB: SymInitialize failed (err=%lu)\n", GetLastError());
        SymSetOptions(opts);
        return result;
    }

    char dllPathA[MAX_PATH];
    WideCharToMultiByte(CP_ACP, 0, dllPath.c_str(), -1, dllPathA, MAX_PATH, NULL, NULL);

    DWORD64 baseAddr = SymLoadModuleEx(hSym, NULL, dllPathA, NULL, 0x10000000, 0, NULL, 0);
    if (!baseAddr)
    {
        wprintf(L"    PDB: SymLoadModuleEx failed (err=%lu)\n", GetLastError());
        SymCleanup(hSym);
        SymSetOptions(opts);
        return result;
    }

    // Verify we got PDB symbols (not just exports)
    IMAGEHLP_MODULE64 modInfo = {};
    modInfo.SizeOfStruct = sizeof(modInfo);
    if (SymGetModuleInfo64(hSym, baseAddr, &modInfo))
    {
        wprintf(L"    PDB: SymType=%d (%S)\n",
                modInfo.SymType,
                modInfo.SymType == SymPdb ? "PDB" :
                modInfo.SymType == SymExport ? "Export-only" :
                modInfo.SymType == SymNone ? "None" : "Other");
    }

    // Count total symbols
    int totalSymCount = 0;
    SymEnumSymbols(hSym, baseAddr, "*", [](PSYMBOL_INFO, ULONG, PVOID ctx) -> BOOL {
        (*(int*)ctx)++; return TRUE;
    }, &totalSymCount);
    wprintf(L"    PDB: Total symbols: %d\n", totalSymCount);

    if (totalSymCount <= 10)
    {
        wprintf(L"    PDB: Too few symbols -- PDB may not have loaded correctly\n");
        SymUnloadModule64(hSym, baseAddr);
        SymCleanup(hSym);
        SymSetOptions(opts);
        return result;
    }

    // Search for COverlayContext symbols
    SymEnumCtx ctx = { &result, baseAddr, verbose, 0 };
    SymEnumSymbols(hSym, baseAddr, "*COverlayContext*", PdbSymEnumCallback, &ctx);
    wprintf(L"    PDB: Matched %d COverlayContext symbols\n", ctx.totalCount);

    // Retry with decorated names if needed
    if (ctx.totalCount == 0)
    {
        SymSetOptions((opts | SYMOPT_AUTO_PUBLICS) & ~(SYMOPT_DEFERRED_LOADS | SYMOPT_UNDNAME));
        SymUnloadModule64(hSym, baseAddr);
        baseAddr = SymLoadModuleEx(hSym, NULL, dllPathA, NULL, 0x10000000, 0, NULL, 0);
        if (baseAddr)
        {
            ctx.totalCount = 0;
            SymEnumSymbols(hSym, baseAddr, "*COverlayContext*", PdbSymEnumCallback, &ctx);
            wprintf(L"    PDB: Matched %d symbols (decorated)\n", ctx.totalCount);
        }
    }

    SymUnloadModule64(hSym, baseAddr);
    SymCleanup(hSym);
    SymSetOptions(opts);

    wprintf(L"    PDB: Present=%s DirectFlip=%s Overlays=%s\n",
            result.presentFound ? L"FOUND" : L"NOT FOUND",
            result.directFlipFound ? L"FOUND" : L"NOT FOUND",
            result.overlaysFound ? L"FOUND" : L"NOT FOUND");

    return result;
}

// Scan dwmcore.dll in a remote dwm.exe process for hook target patterns.
// Returns true if all 3 patterns found, filling cfg with offsets and runtime params.
static bool ScanDwmcorePatterns(DWORD dwmPid, bool isWin11, bool is24h2, bool is25h2,
                                int ditherBits, DitherConfig* cfg)
{
    memset(cfg, 0, sizeof(*cfg));
    cfg->magic = DITHER_CONFIG_MAGIC;
    cfg->version = DITHER_CONFIG_VERSION;
    cfg->ditherBits = (UINT32)ditherBits;
    cfg->isWindows11 = isWin11 ? 1 : 0;
    cfg->isWindows11_24h2 = is24h2 ? 1 : 0; // is24h2 is true for both 24H2 and 25H2

    // Select runtime offsets for this OS version
    if (is24h2) // includes 25H2
    {
        cfg->hwProtOffset = OVERLAY_HWPROT_OFFSET_W11_24H2;
        cfg->swapChainOffset = OVERLAY_SWAPCHAIN_OFFSET_W11_24H2;
    }
    else if (isWin11)
    {
        cfg->hwProtOffset = OVERLAY_HWPROT_OFFSET_W11;
        cfg->swapChainOffset = OVERLAY_SWAPCHAIN_OFFSET_W11;
    }
    else
    {
        cfg->hwProtOffset = OVERLAY_HWPROT_OFFSET_W10;
        cfg->swapChainOffset = OVERLAY_SWAPCHAIN_OFFSET_W10;
    }

    // Resolve function offsets via PDB symbols from Microsoft Symbol Server
    wprintf(L"    --- PDB Symbol Resolution ---\n");
    PdbResolvedOffsets pdb = ResolveDwmcorePdb(false);

    if (pdb.presentFound && pdb.directFlipFound && pdb.overlaysFound)
    {
        cfg->presentOffset = pdb.presentRva;
        cfg->directFlipOffset = pdb.directFlipRva;
        cfg->overlaysOffset = pdb.overlaysRva;
        wprintf(L"    PDB: Present=0x%llx DirectFlip=0x%llx Overlays=0x%llx\n",
                (UINT64)pdb.presentRva, (UINT64)pdb.directFlipRva, (UINT64)pdb.overlaysRva);
        return true;
    }

    wprintf(L"    PDB: incomplete (%s%s%s)\n",
            pdb.presentFound ? L"" : L"Present ",
            pdb.directFlipFound ? L"" : L"DirectFlip ",
            pdb.overlaysFound ? L"" : L"Overlays ");
    wprintf(L"    ERROR: PDB symbol resolution failed to find all required functions.\n");
    wprintf(L"    TIP: Run 'ApplyIccLut.exe --probe' to force re-download the PDB.\n");
    return false;
}


// ============================================================================
// Probe mode — force PDB re-download and re-resolve symbols
// ============================================================================

static void RunProbeMode(bool verbose)
{
    wprintf(L"\n=== Probe Mode: Force PDB Re-query ===\n\n");

    // Detect version
    WindowsVersion wv = GetTrueWindowsVersion();
    const wchar_t* verName = L"Windows 10";
    bool isWin11 = (wv.build >= 22000);
    bool is24h2 = (wv.build >= 26100);
    if (isWin11) verName = L"Windows 11";
    if (is24h2) verName = L"Windows 11 24H2+";
    if (wv.build >= 26200) verName = L"Windows 11 25H2+";
    wprintf(L"  Windows: %s (build %lu)\n", verName, wv.build);

    // Get dwmcore.dll file version
    UINT32 verMS = 0, verLS = 0, fileSz = 0;
    if (GetDwmcoreFileVersion(&verMS, &verLS, &fileSz))
    {
        wprintf(L"  dwmcore.dll: %u.%u.%u.%u  size: %u\n",
                (verMS >> 16), (verMS & 0xFFFF), (verLS >> 16), (verLS & 0xFFFF), fileSz);
    }

    // Delete cached PDB to force re-download
    wprintf(L"\n  Deleting cached PDB to force re-download...\n");
    {
        wchar_t localAppData[MAX_PATH];
        if (GetEnvironmentVariableW(L"LOCALAPPDATA", localAppData, MAX_PATH))
        {
            std::wstring cacheDir = std::wstring(localAppData) + L"\\Temp\\SymbolCache\\";
            // Delete dwmcore.pdb files from cache
            WIN32_FIND_DATAW fd;
            std::wstring searchPath = cacheDir + L"dwmcore.pdb\\*";
            HANDLE hFind = FindFirstFileW(searchPath.c_str(), &fd);
            if (hFind != INVALID_HANDLE_VALUE)
            {
                do {
                    if (fd.dwFileAttributes & FILE_ATTRIBUTE_DIRECTORY)
                    {
                        std::wstring subDir = cacheDir + L"dwmcore.pdb\\" + fd.cFileName;
                        std::wstring pdbFile = subDir + L"\\dwmcore.pdb";
                        if (DeleteFileW(pdbFile.c_str()))
                            wprintf(L"    Deleted: %s\n", pdbFile.c_str());
                        RemoveDirectoryW(subDir.c_str());
                    }
                } while (FindNextFileW(hFind, &fd));
                FindClose(hFind);
            }
        }
    }

    // Re-resolve via PDB
    wprintf(L"\n  Re-downloading and resolving PDB symbols...\n");
    PdbResolvedOffsets pdb = ResolveDwmcorePdb(true);

    if (!pdb.presentFound || !pdb.directFlipFound || !pdb.overlaysFound)
    {
        wprintf(L"\n  FAILED: PDB resolution incomplete.\n");
        wprintf(L"    Present:    %s\n", pdb.presentFound ? L"found" : L"NOT FOUND");
        wprintf(L"    DirectFlip: %s\n", pdb.directFlipFound ? L"found" : L"NOT FOUND");
        wprintf(L"    Overlays:   %s\n", pdb.overlaysFound ? L"found" : L"NOT FOUND");
        return;
    }

    wprintf(L"\n  === PDB Results ===\n");
    wprintf(L"  Present:         RVA 0x%llx\n", (UINT64)pdb.presentRva);
    wprintf(L"  DirectFlip:      RVA 0x%llx\n", (UINT64)pdb.directFlipRva);
    wprintf(L"  OverlaysEnabled: RVA 0x%llx\n", (UINT64)pdb.overlaysRva);

    // Select runtime struct offsets for this OS version
    INT32 hwProtOff, swapChainOff;
    if (is24h2)
    {
        hwProtOff = OVERLAY_HWPROT_OFFSET_W11_24H2;
        swapChainOff = OVERLAY_SWAPCHAIN_OFFSET_W11_24H2;
    }
    else if (isWin11)
    {
        hwProtOff = OVERLAY_HWPROT_OFFSET_W11;
        swapChainOff = OVERLAY_SWAPCHAIN_OFFSET_W11;
    }
    else
    {
        hwProtOff = OVERLAY_HWPROT_OFFSET_W10;
        swapChainOff = OVERLAY_SWAPCHAIN_OFFSET_W10;
    }

    // Save to offset cache
    OffsetCacheEntry cache = {};
    cache.magic = OFFSET_CACHE_MAGIC;
    cache.version = OFFSET_CACHE_VERSION;
    cache.dwmcoreVersionMS = verMS;
    cache.dwmcoreVersionLS = verLS;
    cache.dwmcoreSizeBytes = fileSz;
    cache.osBuild = wv.build;
    cache.presentOffset = pdb.presentRva;
    cache.directFlipOffset = pdb.directFlipRva;
    cache.overlaysOffset = pdb.overlaysRva;
    cache.hwProtOffset = hwProtOff;
    cache.swapChainOffset = swapChainOff;
    cache.isWindows11 = isWin11 ? 1 : 0;
    cache.sourceFlags = 1; // PDB

    if (SaveOffsetCache(&cache))
        wprintf(L"\n  Saved to cache. Use --dither to apply.\n");
    else
        wprintf(L"\n  WARNING: Could not save cache file.\n");
}

static DWORD FindProcessByName(const wchar_t* name)
{
    HANDLE snap = CreateToolhelp32Snapshot(TH32CS_SNAPPROCESS, 0);
    if (snap == INVALID_HANDLE_VALUE)
        return 0;

    PROCESSENTRY32W pe = {};
    pe.dwSize = sizeof(pe);
    DWORD pid = 0;

    if (Process32FirstW(snap, &pe))
    {
        do {
            if (_wcsicmp(pe.szExeFile, name) == 0)
            {
                pid = pe.th32ProcessID;
                break;
            }
        } while (Process32NextW(snap, &pe));
    }
    CloseHandle(snap);
    return pid;
}

static bool EnableDebugPrivilege()
{
    HANDLE hToken;
    if (!OpenProcessToken(GetCurrentProcess(), TOKEN_ADJUST_PRIVILEGES | TOKEN_QUERY, &hToken))
        return false;

    TOKEN_PRIVILEGES tp = {};
    tp.PrivilegeCount = 1;
    tp.Privileges[0].Attributes = SE_PRIVILEGE_ENABLED;
    LookupPrivilegeValueW(NULL, SE_DEBUG_NAME, &tp.Privileges[0].Luid);
    BOOL ok = AdjustTokenPrivileges(hToken, FALSE, &tp, sizeof(tp), NULL, NULL);
    DWORD err = GetLastError();
    CloseHandle(hToken);
    return ok && err == ERROR_SUCCESS;
}

static bool ImpersonateSystem(bool verbose)
{
    DWORD lsassPid = FindProcessByName(L"lsass.exe");
    if (!lsassPid)
    {
        if (verbose) wprintf(L"  Could not find lsass.exe\n");
        return false;
    }

    HANDLE hProcess = OpenProcess(PROCESS_QUERY_LIMITED_INFORMATION, FALSE, lsassPid);
    if (!hProcess)
    {
        if (verbose) wprintf(L"  Could not open lsass process (err=%lu)\n", GetLastError());
        return false;
    }

    HANDLE hToken;
    if (!OpenProcessToken(hProcess, TOKEN_DUPLICATE | TOKEN_QUERY, &hToken))
    {
        if (verbose) wprintf(L"  Could not open lsass token (err=%lu)\n", GetLastError());
        CloseHandle(hProcess);
        return false;
    }
    CloseHandle(hProcess);

    HANDLE hDupToken;
    if (!DuplicateTokenEx(hToken, MAXIMUM_ALLOWED, NULL, SecurityImpersonation, TokenImpersonation, &hDupToken))
    {
        if (verbose) wprintf(L"  Could not duplicate token (err=%lu)\n", GetLastError());
        CloseHandle(hToken);
        return false;
    }
    CloseHandle(hToken);

    if (!ImpersonateLoggedOnUser(hDupToken))
    {
        if (verbose) wprintf(L"  ImpersonateLoggedOnUser failed (err=%lu)\n", GetLastError());
        CloseHandle(hDupToken);
        return false;
    }
    CloseHandle(hDupToken);

    if (verbose) wprintf(L"  Impersonated SYSTEM token.\n");
    return true;
}

// Forward declarations
static bool UninjectDll(DWORD pid, const wchar_t* dllName, bool verbose);

// Check if a named DLL is loaded in a remote process
static bool IsModuleLoadedInProcess(DWORD pid, const wchar_t* dllName)
{
    HANDLE snap = CreateToolhelp32Snapshot(TH32CS_SNAPMODULE | TH32CS_SNAPMODULE32, pid);
    if (snap == INVALID_HANDLE_VALUE)
        return false;

    MODULEENTRY32W me = {};
    me.dwSize = sizeof(me);
    bool found = false;
    if (Module32FirstW(snap, &me))
    {
        do {
            if (_wcsicmp(me.szModule, dllName) == 0)
            {
                found = true;
                break;
            }
        } while (Module32NextW(snap, &me));
    }
    CloseHandle(snap);
    return found;
}

// Set a NULL DACL on a file so that all users (including DWM's restricted account)
// can read it.  Without this, LoadLibraryW inside dwm.exe fails because the DWM
// service account (e.g. UMFD-0) lacks read access to files in %SYSTEMROOT%\Temp.
static void ClearFileDacl(const wchar_t* path)
{
    HANDLE hFile = CreateFileW(path,
                               READ_CONTROL | WRITE_DAC,
                               0, NULL, OPEN_EXISTING,
                               FILE_ATTRIBUTE_NORMAL | FILE_FLAG_BACKUP_SEMANTICS,
                               NULL);
    if (hFile == INVALID_HANDLE_VALUE)
        return;
    SetSecurityInfo(hFile, SE_FILE_OBJECT, DACL_SECURITY_INFORMATION,
                    NULL, NULL, NULL, NULL);
    CloseHandle(hFile);
}

// Classic DLL injection using CreateRemoteThread + LoadLibraryW.
// This avoids custom shellcode which would be blocked by HVCI/ACG on protected
// processes like dwm.exe.  We write only the DLL path (data) into remote memory
// and use kernel32!LoadLibraryW — already mapped and executable — as the thread
// entry point.  The thread exit code is the HMODULE (truncated to DWORD on x64,
// but we verify loading via module enumeration anyway).

static bool InjectDll(DWORD pid, const wchar_t* dllPath, const wchar_t* dllName, bool verbose)
{
    // Check if the DLL is already loaded in the target
    if (IsModuleLoadedInProcess(pid, dllName))
    {
        wprintf(L"    DLL already loaded -- uninjecting stale copy first.\n");
        UninjectDll(pid, dllName, verbose);
        Sleep(500);
    }

    HANDLE hProcess = OpenProcess(PROCESS_ALL_ACCESS, FALSE, pid);
    if (!hProcess)
    {
        wprintf(L"    OpenProcess failed (err=%lu)\n", GetLastError());
        return false;
    }

    // Check process signature policy (Code Integrity) in verbose mode
    if (verbose)
    {
        typedef BOOL (WINAPI *GetProcessMitigationPolicy_t)(HANDLE, int, PVOID, SIZE_T);
        auto pGetPolicy = (GetProcessMitigationPolicy_t)GetProcAddress(
            GetModuleHandleW(L"kernel32.dll"), "GetProcessMitigationPolicy");
        if (pGetPolicy)
        {
            // PROCESS_MITIGATION_BINARY_SIGNATURE_POLICY (policy id = 8)
            struct { DWORD Flags; } sigPolicy = {};
            if (pGetPolicy(hProcess, 8, &sigPolicy, sizeof(sigPolicy)))
            {
                bool microsoftOnly = (sigPolicy.Flags & 0x1) != 0;
                bool storeOnly     = (sigPolicy.Flags & 0x2) != 0;
                if (microsoftOnly || storeOnly)
                {
                    wprintf(L"    WARNING: Process has signature policy (flags=0x%lx)\n", sigPolicy.Flags);
                    if (microsoftOnly)
                        wprintf(L"      MicrosoftSignedOnly -- unsigned DLLs will be REJECTED.\n");
                    if (storeOnly)
                        wprintf(L"      StoreSignedOnly -- unsigned DLLs will be REJECTED.\n");
                }
                else
                {
                    wprintf(L"    Process signature policy: no restrictions (flags=0x%lx)\n", sigPolicy.Flags);
                }
            }
            else
            {
                wprintf(L"    Could not query signature policy (err=%lu)\n", GetLastError());
            }

            // PROCESS_MITIGATION_DYNAMIC_CODE_POLICY (policy id = 2)
            struct { DWORD Flags; } dynPolicy = {};
            if (pGetPolicy(hProcess, 2, &dynPolicy, sizeof(dynPolicy)))
            {
                bool prohibitDynCode = (dynPolicy.Flags & 0x1) != 0;
                if (prohibitDynCode)
                    wprintf(L"    WARNING: Process prohibits dynamic code (ACG enabled).\n");
                else
                    wprintf(L"    Dynamic code policy: allowed (flags=0x%lx)\n", dynPolicy.Flags);
            }
        }
    }

    // Classic injection: write DLL path string into remote process, then
    // CreateRemoteThread with kernel32!LoadLibraryW as the start address.
    // No executable shellcode is allocated — only data (the path string).
    HMODULE hKernel32 = GetModuleHandleW(L"kernel32.dll");
    FARPROC pLoadLibraryW = GetProcAddress(hKernel32, "LoadLibraryW");

    size_t pathBytes = (wcslen(dllPath) + 1) * sizeof(wchar_t);

    // Allocate ReadWrite memory in target for the DLL path (NOT executable)
    void* remotePath = VirtualAllocEx(hProcess, NULL, pathBytes,
                                      MEM_RESERVE | MEM_COMMIT, PAGE_READWRITE);
    if (!remotePath)
    {
        wprintf(L"    VirtualAllocEx failed (err=%lu)\n", GetLastError());
        CloseHandle(hProcess);
        return false;
    }

    // Write DLL path string
    if (!WriteProcessMemory(hProcess, remotePath, dllPath, pathBytes, NULL))
    {
        wprintf(L"    WriteProcessMemory failed (err=%lu)\n", GetLastError());
        VirtualFreeEx(hProcess, remotePath, 0, MEM_RELEASE);
        CloseHandle(hProcess);
        return false;
    }

    if (verbose)
        wprintf(L"    Remote path at %p (%zu bytes), LoadLibraryW at %p\n",
                remotePath, pathBytes, (void*)pLoadLibraryW);

    // Create remote thread that calls LoadLibraryW(remotePath)
    // LoadLibraryW is in kernel32.dll which is mapped at the same address in all processes.
    HANDLE hThread = CreateRemoteThread(hProcess, NULL, 0,
                                        (LPTHREAD_START_ROUTINE)pLoadLibraryW,
                                        remotePath, 0, NULL);
    if (!hThread)
    {
        DWORD err = GetLastError();
        wprintf(L"    CreateRemoteThread failed (err=%lu)\n", err);
        if (err == ERROR_ACCESS_DENIED)
            wprintf(L"    Process may be protected (PPL/CIG) and rejecting remote threads.\n");
        VirtualFreeEx(hProcess, remotePath, 0, MEM_RELEASE);
        CloseHandle(hProcess);
        return false;
    }

    DWORD waitResult = WaitForSingleObject(hThread, 15000);

    // Get thread exit code (this is the HMODULE returned by LoadLibraryW,
    // truncated to DWORD on x64 — but 0 means failure)
    DWORD threadExitCode = 0;
    GetExitCodeThread(hThread, &threadExitCode);
    CloseHandle(hThread);

    if (waitResult == WAIT_TIMEOUT)
    {
        wprintf(L"    WARNING: Remote thread timed out (15s).\n");
        VirtualFreeEx(hProcess, remotePath, 0, MEM_RELEASE);
        CloseHandle(hProcess);
        return false;
    }

    // Check if the target process is still alive (DWM may have crashed during LoadLibraryW)
    DWORD exitCode = STILL_ACTIVE;
    GetExitCodeProcess(hProcess, &exitCode);
    if (exitCode != STILL_ACTIVE)
    {
        wprintf(L"  ERROR: Target process (PID %lu) CRASHED during DLL loading (exit code %lu).\n", pid, exitCode);
        wprintf(L"    The DLL's import resolution or DllMain likely caused the crash.\n");
        VirtualFreeEx(hProcess, remotePath, 0, MEM_RELEASE);
        CloseHandle(hProcess);
        wprintf(L"    Diag log (if DllMain ran): %%SYSTEMROOT%%\\Temp\\ApplyIccLut_dither_diag.log\n");
        return false;
    }

    VirtualFreeEx(hProcess, remotePath, 0, MEM_RELEASE);
    CloseHandle(hProcess);

    if (verbose)
        wprintf(L"    LoadLibraryW thread exit code: 0x%lx\n", threadExitCode);

    // Verify with module enumeration (authoritative check)
    Sleep(200);
    bool loaded = IsModuleLoadedInProcess(pid, dllName);

    if (loaded)
    {
        wprintf(L"  DLL injected successfully into PID %lu.\n", pid);
        return true;
    }

    // DLL did not load — provide diagnostics
    wprintf(L"  ERROR: DLL failed to load in PID %lu.\n", pid);
    if (threadExitCode == 0)
    {
        wprintf(L"    LoadLibraryW returned NULL (thread exit code = 0).\n");
        wprintf(L"    Possible causes:\n");
        wprintf(L"      - DllMain returned FALSE (check diag log)\n");
        wprintf(L"      - Missing dependency DLL\n");
        wprintf(L"      - Code Integrity / WDAC policy blocked the load\n");
        wprintf(L"      - DLL file not accessible from target process context\n");
    }
    else
    {
        wprintf(L"    LoadLibraryW thread exit code 0x%lx but module not found in process.\n", threadExitCode);
        wprintf(L"    DllMain may have returned FALSE, causing immediate unload.\n");
    }
    wprintf(L"    Diag log (if DllMain ran): %%SYSTEMROOT%%\\Temp\\ApplyIccLut_dither_diag.log\n");
    return false;
}

static bool UninjectDll(DWORD pid, const wchar_t* dllName, bool verbose)
{
    HANDLE snap = CreateToolhelp32Snapshot(TH32CS_SNAPMODULE | TH32CS_SNAPMODULE32, pid);
    if (snap == INVALID_HANDLE_VALUE)
    {
        if (verbose) wprintf(L"  Could not snapshot modules for PID %lu (err=%lu)\n", pid, GetLastError());
        return false;
    }

    MODULEENTRY32W me = {};
    me.dwSize = sizeof(me);
    HMODULE hRemoteDll = NULL;

    if (Module32FirstW(snap, &me))
    {
        do {
            if (_wcsicmp(me.szModule, dllName) == 0)
            {
                hRemoteDll = me.hModule;
                break;
            }
        } while (Module32NextW(snap, &me));
    }
    CloseHandle(snap);

    if (!hRemoteDll)
    {
        if (verbose) wprintf(L"  DLL not found in process %lu.\n", pid);
        return false;
    }

    HANDLE hProcess = OpenProcess(PROCESS_ALL_ACCESS, FALSE, pid);
    if (!hProcess)
    {
        if (verbose) wprintf(L"  Could not open process %lu (err=%lu)\n", pid, GetLastError());
        return false;
    }

    HMODULE hKernel32 = GetModuleHandleW(L"kernel32.dll");
    FARPROC pFreeLibrary = GetProcAddress(hKernel32, "FreeLibrary");

    HANDLE hThread = CreateRemoteThread(hProcess, NULL, 0, (LPTHREAD_START_ROUTINE)pFreeLibrary, hRemoteDll, 0, NULL);
    if (!hThread)
    {
        if (verbose) wprintf(L"  CreateRemoteThread(FreeLibrary) failed (err=%lu)\n", GetLastError());
        CloseHandle(hProcess);
        return false;
    }

    WaitForSingleObject(hThread, 10000);
    CloseHandle(hThread);
    CloseHandle(hProcess);

    if (verbose) wprintf(L"  DLL uninjected from process %lu.\n", pid);
    return true;
}

// ============================================================================
// NvAPI hardware dithering (NVIDIA GPUs only)
// Uses undocumented NvAPI_GPU_SetDitherControl to enable GPU-level dithering
// at the display output stage — operates AFTER ICC/color management pipeline.
// ============================================================================

typedef int NvAPI_Status;
typedef void* NvPhysicalGpuHandle;
typedef void* NvDisplayHandle;
typedef unsigned int NvU32;

static constexpr NvAPI_Status NVAPI_OK = 0;
static constexpr NvU32 NVAPI_MAX_PHYSICAL_GPUS = 64;

// Dither control struct (v1, 24 bytes) — used by GetDitherControl
struct NV_GPU_DITHER_CONTROL_V1 {
    NvU32 version;    // sizeof | (1 << 16) = 0x10018
    NvU32 state;      // 0=Default, 1=Enabled, 2=Disabled
    NvU32 bits;       // 0=6-bit, 1=8-bit, 2=10-bit
    NvU32 mode;       // 0=SpatialDynamic, 1=SpatialStatic, 2=SpatialDynamic2x2, 3=SpatialStatic2x2, 4=Temporal
    NvU32 bitsCaps;   // bitmask of supported bit depths
    NvU32 modeCaps;   // bitmask of supported modes
};

static constexpr NvU32 NV_GPU_DITHER_CONTROL_VER1 = (1 << 16) | sizeof(NV_GPU_DITHER_CONTROL_V1);

// NvAPI function pointer types
typedef void* (*NvAPI_QueryInterface_t)(NvU32 id);
typedef NvAPI_Status (*NvAPI_Initialize_t)();
typedef NvAPI_Status (*NvAPI_Unload_t)();
typedef NvAPI_Status (*NvAPI_EnumPhysicalGPUs_t)(NvPhysicalGpuHandle handles[], NvU32* count);
typedef NvAPI_Status (*NvAPI_EnumNvidiaDisplayHandle_t)(NvU32 thisEnum, NvDisplayHandle* handle);
typedef NvAPI_Status (*NvAPI_GetPhysicalGPUsFromDisplay_t)(NvDisplayHandle display, NvPhysicalGpuHandle gpus[], NvU32* count);
typedef NvAPI_Status (*NvAPI_GetAssociatedDisplayOutputId_t)(NvDisplayHandle display, NvU32* outputId);
// SetDitherControl takes individual params: (gpu, outputId, state, bits, mode)
typedef NvAPI_Status (*NvAPI_GPU_SetDitherControl_t)(NvPhysicalGpuHandle gpu, NvU32 outputId, NvU32 state, NvU32 bits, NvU32 mode);
// GetDitherControl takes a struct pointer
typedef NvAPI_Status (*NvAPI_GPU_GetDitherControl_t)(NvPhysicalGpuHandle gpu, NvU32 outputId, NV_GPU_DITHER_CONTROL_V1* pControl);
typedef NvAPI_Status (*NvAPI_GetErrorMessage_t)(NvAPI_Status nr, char szDesc[64]);

// NvAPI function IDs (resolved via nvapi_QueryInterface)
static constexpr NvU32 NVFUNC_INITIALIZE                  = 0x0150E828;
static constexpr NvU32 NVFUNC_UNLOAD                      = 0xD22BDD7E;
static constexpr NvU32 NVFUNC_ENUM_PHYSICAL_GPUS          = 0xE5AC921F;
static constexpr NvU32 NVFUNC_ENUM_DISPLAY_HANDLE         = 0x9ABDD40D;
static constexpr NvU32 NVFUNC_GET_GPUS_FROM_DISPLAY       = 0x34EF9506;
static constexpr NvU32 NVFUNC_GET_ASSOC_DISPLAY_OUTPUT_ID = 0xD995937E;
static constexpr NvU32 NVFUNC_SET_DITHER_CONTROL          = 0xDF0DFCDD;
static constexpr NvU32 NVFUNC_GET_DITHER_CONTROL          = 0x932AC8FB;
static constexpr NvU32 NVFUNC_GET_ERROR_MESSAGE           = 0x6C2D048C;

struct NvApiContext {
    HMODULE hDll;
    NvAPI_Initialize_t Initialize;
    NvAPI_Unload_t Unload;
    NvAPI_EnumPhysicalGPUs_t EnumPhysicalGPUs;
    NvAPI_EnumNvidiaDisplayHandle_t EnumDisplayHandle;
    NvAPI_GetPhysicalGPUsFromDisplay_t GetGPUsFromDisplay;
    NvAPI_GetAssociatedDisplayOutputId_t GetAssocOutputId;
    NvAPI_GPU_SetDitherControl_t SetDitherControl;
    NvAPI_GPU_GetDitherControl_t GetDitherControl;
    NvAPI_GetErrorMessage_t GetErrorMessage;
};

static bool LoadNvApi(NvApiContext& ctx, bool verbose)
{
    memset(&ctx, 0, sizeof(ctx));

    ctx.hDll = LoadLibraryW(L"nvapi64.dll");
    if (!ctx.hDll)
    {
        if (verbose) wprintf(L"  NvAPI: nvapi64.dll not found (not an NVIDIA system?)\n");
        return false;
    }

    auto qi = (NvAPI_QueryInterface_t)GetProcAddress(ctx.hDll, "nvapi_QueryInterface");
    if (!qi)
    {
        if (verbose) wprintf(L"  NvAPI: nvapi_QueryInterface not found\n");
        FreeLibrary(ctx.hDll);
        ctx.hDll = NULL;
        return false;
    }

    ctx.Initialize        = (NvAPI_Initialize_t)qi(NVFUNC_INITIALIZE);
    ctx.Unload            = (NvAPI_Unload_t)qi(NVFUNC_UNLOAD);
    ctx.EnumPhysicalGPUs  = (NvAPI_EnumPhysicalGPUs_t)qi(NVFUNC_ENUM_PHYSICAL_GPUS);
    ctx.EnumDisplayHandle = (NvAPI_EnumNvidiaDisplayHandle_t)qi(NVFUNC_ENUM_DISPLAY_HANDLE);
    ctx.GetGPUsFromDisplay = (NvAPI_GetPhysicalGPUsFromDisplay_t)qi(NVFUNC_GET_GPUS_FROM_DISPLAY);
    ctx.GetAssocOutputId  = (NvAPI_GetAssociatedDisplayOutputId_t)qi(NVFUNC_GET_ASSOC_DISPLAY_OUTPUT_ID);
    ctx.SetDitherControl  = (NvAPI_GPU_SetDitherControl_t)qi(NVFUNC_SET_DITHER_CONTROL);
    ctx.GetDitherControl  = (NvAPI_GPU_GetDitherControl_t)qi(NVFUNC_GET_DITHER_CONTROL);
    ctx.GetErrorMessage   = (NvAPI_GetErrorMessage_t)qi(NVFUNC_GET_ERROR_MESSAGE);

    if (!ctx.Initialize || !ctx.EnumDisplayHandle || !ctx.GetGPUsFromDisplay ||
        !ctx.GetAssocOutputId || !ctx.SetDitherControl)
    {
        if (verbose) wprintf(L"  NvAPI: failed to resolve required functions\n");
        FreeLibrary(ctx.hDll);
        ctx.hDll = NULL;
        return false;
    }

    NvAPI_Status st = ctx.Initialize();
    if (st != NVAPI_OK)
    {
        if (verbose) wprintf(L"  NvAPI: Initialize failed (status=%d)\n", st);
        FreeLibrary(ctx.hDll);
        ctx.hDll = NULL;
        return false;
    }

    return true;
}

static const char* NvApiErrorStr(NvApiContext& ctx, NvAPI_Status st)
{
    static char buf[64] = {};
    if (ctx.GetErrorMessage)
    {
        ctx.GetErrorMessage(st, buf);
        return buf;
    }
    wsprintfA(buf, "status %d", st);
    return buf;
}

// ditherMode: 0=SpatialDynamic, 1=SpatialStatic, 2=SpatialDynamic2x2, 3=SpatialStatic2x2, 4=Temporal
static bool EnableNvApiDithering(int ditherBits, int ditherMode, bool verbose)
{
    NvApiContext nv;
    if (!LoadNvApi(nv, verbose))
        return false;

    // Map user bit depth to NvAPI bits enum: 0=6-bit, 1=8-bit, 2=10-bit
    NvU32 nvBits;
    const wchar_t* bitsName;
    if (ditherBits == 6)        { nvBits = 0; bitsName = L"6-bit"; }
    else if (ditherBits == 10)  { nvBits = 2; bitsName = L"10-bit"; }
    else                        { nvBits = 1; bitsName = L"8-bit"; } // default for 0 (auto), 8, or other

    if (ditherBits != 0 && ditherBits != 6 && ditherBits != 8 && ditherBits != 10)
        wprintf(L"  NvAPI: --dither-bits %d not directly supported, using %s\n", ditherBits, bitsName);

    static const wchar_t* modeNames[] = {
        L"spatial-dynamic", L"spatial-static", L"spatial-dynamic-2x2",
        L"spatial-static-2x2", L"temporal"
    };
    NvU32 nvMode = (ditherMode >= 0 && ditherMode <= 4) ? (NvU32)ditherMode : 4;
    wprintf(L"  NvAPI: enabling %s %s dithering on all displays...\n", bitsName, modeNames[nvMode]);

    int displayCount = 0;
    int successCount = 0;

    for (NvU32 i = 0; i < 64; i++)
    {
        NvDisplayHandle display = nullptr;
        NvAPI_Status st = nv.EnumDisplayHandle(i, &display);
        if (st != NVAPI_OK)
            break; // no more displays

        displayCount++;

        // Get GPU handle for this display
        NvPhysicalGpuHandle gpus[NVAPI_MAX_PHYSICAL_GPUS] = {};
        NvU32 gpuCount = 0;
        st = nv.GetGPUsFromDisplay(display, gpus, &gpuCount);
        if (st != NVAPI_OK || gpuCount == 0)
        {
            if (verbose) wprintf(L"    Display %u: GetGPUsFromDisplay failed (%hs)\n", i, NvApiErrorStr(nv, st));
            continue;
        }

        // Get output ID for this display
        NvU32 outputId = 0;
        st = nv.GetAssocOutputId(display, &outputId);
        if (st != NVAPI_OK)
        {
            if (verbose) wprintf(L"    Display %u: GetAssociatedDisplayOutputId failed (%hs)\n", i, NvApiErrorStr(nv, st));
            continue;
        }

        // Log current dither state (verbose only)
        if (verbose && nv.GetDitherControl)
        {
            NV_GPU_DITHER_CONTROL_V1 cur = {};
            cur.version = NV_GPU_DITHER_CONTROL_VER1;
            st = nv.GetDitherControl(gpus[0], outputId, &cur);
            if (st == NVAPI_OK)
            {
                wprintf(L"    Display %u: current state=%u bits=%u mode=%u bitsCaps=0x%X modeCaps=0x%X\n",
                    i, cur.state, cur.bits, cur.mode, cur.bitsCaps, cur.modeCaps);
            }
        }

        // Set dither: state=Enabled(1), bits=target, mode=user-selected
        st = nv.SetDitherControl(gpus[0], outputId, 1, nvBits, nvMode);
        if (st == NVAPI_OK)
        {
            successCount++;
            if (verbose) wprintf(L"    Display %u (outputId=0x%X): dithering enabled\n", i, outputId);
        }
        else
        {
            wprintf(L"    Display %u: SetDitherControl failed (%hs)\n", i, NvApiErrorStr(nv, st));
        }
    }

    if (nv.Unload) nv.Unload();

    if (displayCount == 0)
    {
        wprintf(L"  NvAPI: no NVIDIA displays found.\n");
        return false;
    }

    wprintf(L"  NvAPI: dithering enabled on %d of %d display(s).\n", successCount, displayCount);
    return successCount > 0;
}

static bool DisableNvApiDithering(bool verbose)
{
    NvApiContext nv;
    if (!LoadNvApi(nv, verbose))
        return false;

    if (verbose) wprintf(L"  NvAPI: resetting dither control to driver defaults...\n");

    int displayCount = 0;
    int successCount = 0;

    for (NvU32 i = 0; i < 64; i++)
    {
        NvDisplayHandle display = nullptr;
        NvAPI_Status st = nv.EnumDisplayHandle(i, &display);
        if (st != NVAPI_OK)
            break;

        displayCount++;

        NvPhysicalGpuHandle gpus[NVAPI_MAX_PHYSICAL_GPUS] = {};
        NvU32 gpuCount = 0;
        st = nv.GetGPUsFromDisplay(display, gpus, &gpuCount);
        if (st != NVAPI_OK || gpuCount == 0) continue;

        NvU32 outputId = 0;
        st = nv.GetAssocOutputId(display, &outputId);
        if (st != NVAPI_OK) continue;

        // Reset to Default state (0), bits and mode ignored when state=Default
        st = nv.SetDitherControl(gpus[0], outputId, 0, 0, 0);
        if (st == NVAPI_OK)
        {
            successCount++;
            if (verbose) wprintf(L"    Display %u (outputId=0x%X): dither reset to default\n", i, outputId);
        }
        else
        {
            if (verbose) wprintf(L"    Display %u: SetDitherControl(Default) failed (%hs)\n", i, NvApiErrorStr(nv, st));
        }
    }

    if (nv.Unload) nv.Unload();

    if (displayCount > 0)
        wprintf(L"  NvAPI: reset dithering on %d of %d display(s).\n", successCount, displayCount);

    return successCount > 0;
}

// ============================================================================
// DWM injection dithering (blue-noise shader post-process)
// ============================================================================

static bool DeployAndInjectDither(bool verbose, int ditherBits = 0)
{
    std::wstring tempDir = GetSystemTempPath();
    std::wstring dllDest = tempDir + DITHER_DLL_NAME;
    std::wstring flagPath = tempDir + DITHER_FLAG_FILE;
    std::wstring cfgPath = tempDir + DITHER_CFG_FILE;

    // Detect Windows version using RtlGetVersion (always returns true version)
    bool diagIsWin11 = false, diagIs24h2 = false, diagIs25h2 = false;
    DWORD osBuild = 0;
    {
        WindowsVersion wv = GetTrueWindowsVersion();
        osBuild = wv.build;
        const wchar_t* verName = L"Windows 10";
        if (wv.build >= 22000) { diagIsWin11 = true; verName = L"Windows 11"; }
        if (wv.build >= 26100) { diagIs24h2 = true;  verName = L"Windows 11 24H2+"; }
        if (wv.build >= 26200) { diagIs25h2 = true;  verName = L"Windows 11 25H2+"; }
        wprintf(L"  Detected: %s (build %lu)\n", verName, wv.build);
    }

    // Get the path to our own exe, the DLL should be next to it
    wchar_t exePath[MAX_PATH];
    GetModuleFileNameW(NULL, exePath, MAX_PATH);
    std::wstring exeDir(exePath);
    exeDir = exeDir.substr(0, exeDir.find_last_of(L'\\') + 1);
    std::wstring dllSource = exeDir + DITHER_DLL_NAME;

    wprintf(L"  Deploying dithering DLL...\n");
    if (verbose)
    {
        wprintf(L"    Source: %s\n", dllSource.c_str());
        wprintf(L"    Dest:   %s\n", dllDest.c_str());
    }

    // Verify source DLL exists and report size
    {
        WIN32_FILE_ATTRIBUTE_DATA fad = {};
        if (!GetFileAttributesExW(dllSource.c_str(), GetFileExInfoStandard, &fad))
        {
            wprintf(L"  ERROR: Source DLL not found: %s\n", dllSource.c_str());
            wprintf(L"  HINT: Ensure %s is next to ApplyIccLut.exe\n", DITHER_DLL_NAME);
            return false;
        }
        if (verbose)
            wprintf(L"    Source DLL size: %lu bytes\n", fad.nFileSizeLow);
    }

    if (!EnableDebugPrivilege())
    {
        wprintf(L"  WARNING: Could not enable SE_DEBUG_PRIVILEGE.\n");
    }
    if (verbose) wprintf(L"  Debug privilege enabled.\n");

    if (!ImpersonateSystem(verbose))
    {
        wprintf(L"  ERROR: Could not impersonate SYSTEM. Are you running as Administrator?\n");
        return false;
    }

    // Uninject any stale DLL before copying new one (old file may be locked)
    {
        DWORD currentSession = 0;
        ProcessIdToSessionId(GetCurrentProcessId(), &currentSession);
        HANDLE preSnap = CreateToolhelp32Snapshot(TH32CS_SNAPPROCESS, 0);
        if (preSnap != INVALID_HANDLE_VALUE)
        {
            PROCESSENTRY32W pe = {};
            pe.dwSize = sizeof(pe);
            if (Process32FirstW(preSnap, &pe))
            {
                do {
                    if (_wcsicmp(pe.szExeFile, L"dwm.exe") == 0)
                    {
                        DWORD sid = 0;
                        ProcessIdToSessionId(pe.th32ProcessID, &sid);
                        if (sid != currentSession) continue;
                        if (IsModuleLoadedInProcess(pe.th32ProcessID, DITHER_DLL_NAME))
                        {
                            wprintf(L"  Uninjecting stale DLL from dwm.exe (PID %lu)...\n", pe.th32ProcessID);
                            UninjectDll(pe.th32ProcessID, DITHER_DLL_NAME, verbose);
                        }
                    }
                } while (Process32NextW(preSnap, &pe));
            }
            CloseHandle(preSnap);
            Sleep(500); // Give DWM time to unload the DLL and release the file
        }
    }

    // Now copy the DLL (file should no longer be locked)
    if (!CopyFileW(dllSource.c_str(), dllDest.c_str(), FALSE))
    {
        DWORD err = GetLastError();
        wprintf(L"  ERROR: Could not copy DLL (err=%lu)\n", err);
        if (err == ERROR_SHARING_VIOLATION)
            wprintf(L"  HINT: Old DLL may still be locked by dwm.exe. Try --no-dither first.\n");
        return false;
    }

    // Clear DACL so DWM's restricted account can read the DLL
    ClearFileDacl(dllDest.c_str());

    // Verify destination file
    {
        WIN32_FILE_ATTRIBUTE_DATA fad = {};
        if (GetFileAttributesExW(dllDest.c_str(), GetFileExInfoStandard, &fad))
        {
            wprintf(L"    Deployed DLL: %lu bytes\n", fad.nFileSizeLow);
        }
        else
        {
            wprintf(L"  ERROR: Deployed DLL not found at %s\n", dllDest.c_str());
            return false;
        }
    }

    // Find DWM processes in our session
    DWORD currentSessionId = 0;
    ProcessIdToSessionId(GetCurrentProcessId(), &currentSessionId);

    std::vector<DWORD> dwmPids;
    {
        HANDLE snap = CreateToolhelp32Snapshot(TH32CS_SNAPPROCESS, 0);
        if (snap != INVALID_HANDLE_VALUE)
        {
            PROCESSENTRY32W pe = {};
            pe.dwSize = sizeof(pe);
            if (Process32FirstW(snap, &pe))
            {
                do {
                    if (_wcsicmp(pe.szExeFile, L"dwm.exe") == 0)
                    {
                        DWORD sid = 0;
                        ProcessIdToSessionId(pe.th32ProcessID, &sid);
                        if (sid == currentSessionId)
                            dwmPids.push_back(pe.th32ProcessID);
                    }
                } while (Process32NextW(snap, &pe));
            }
            CloseHandle(snap);
        }
    }

    if (dwmPids.empty())
    {
        wprintf(L"  ERROR: No dwm.exe found in session %lu.\n", currentSessionId);
        RevertToSelf();
        return false;
    }

    // Try offset cache first, then scan dwmcore.dll remotely
    DitherConfig cfg = {};
    bool fromCache = false;
    {
        OffsetCacheEntry cached = {};
        if (LoadOffsetCache(&cached))
        {
            wprintf(L"  Using cached offsets (source: %s).\n",
                    cached.sourceFlags == 1 ? L"PDB-resolved" : L"cached");
            cfg.magic = DITHER_CONFIG_MAGIC;
            cfg.version = DITHER_CONFIG_VERSION;
            cfg.presentOffset = cached.presentOffset;
            cfg.directFlipOffset = cached.directFlipOffset;
            cfg.overlaysOffset = cached.overlaysOffset;
            cfg.hwProtOffset = cached.hwProtOffset;
            cfg.swapChainOffset = cached.swapChainOffset;
            cfg.isWindows11 = cached.isWindows11;
            cfg.ditherBits = (UINT32)ditherBits;
            fromCache = true;
            if (verbose)
                wprintf(L"    Cached: present=0x%llx directflip=0x%llx overlays=0x%llx\n",
                        (UINT64)cfg.presentOffset, (UINT64)cfg.directFlipOffset, (UINT64)cfg.overlaysOffset);
        }
    }

    if (!fromCache)
    {
        wprintf(L"  Scanning dwmcore.dll for hook targets (PID %lu)...\n", dwmPids[0]);
        if (!ScanDwmcorePatterns(dwmPids[0], diagIsWin11, diagIs24h2, diagIs25h2, ditherBits, &cfg))
        {
            wprintf(L"  ERROR: PDB symbol resolution failed — cannot inject dithering.\n");
            wprintf(L"  TIP: Run 'ApplyIccLut.exe --probe' to force re-download the PDB.\n");
            RevertToSelf();
            return false;
        }

        // Save successful scan to cache for future runs
        UINT32 cacheVerMS = 0, cacheVerLS = 0, cacheFileSz = 0;
        if (GetDwmcoreFileVersion(&cacheVerMS, &cacheVerLS, &cacheFileSz))
        {
            OffsetCacheEntry newCache = {};
            newCache.magic = OFFSET_CACHE_MAGIC;
            newCache.version = OFFSET_CACHE_VERSION;
            newCache.dwmcoreVersionMS = cacheVerMS;
            newCache.dwmcoreVersionLS = cacheVerLS;
            newCache.dwmcoreSizeBytes = cacheFileSz;
            newCache.osBuild = osBuild;
            newCache.presentOffset = cfg.presentOffset;
            newCache.directFlipOffset = cfg.directFlipOffset;
            newCache.overlaysOffset = cfg.overlaysOffset;
            newCache.hwProtOffset = cfg.hwProtOffset;
            newCache.swapChainOffset = cfg.swapChainOffset;
            newCache.isWindows11 = cfg.isWindows11;
            newCache.sourceFlags = 1; // PDB-resolved
            SaveOffsetCache(&newCache);
            if (verbose) wprintf(L"    Offsets saved to cache.\n");
        }
    }

    // Write binary config file (DLL reads this at load time)
    {
        HANDLE hCfg = CreateFileW(cfgPath.c_str(), GENERIC_WRITE, 0, NULL, CREATE_ALWAYS, FILE_ATTRIBUTE_NORMAL, NULL);
        if (hCfg == INVALID_HANDLE_VALUE)
        {
            wprintf(L"  ERROR: Could not write config file (err=%lu)\n", GetLastError());
            RevertToSelf();
            return false;
        }
        DWORD written;
        WriteFile(hCfg, &cfg, sizeof(cfg), &written, NULL);
        CloseHandle(hCfg);
        ClearFileDacl(cfgPath.c_str());  // DWM needs to read this too
        if (verbose)
            wprintf(L"    Config written: present=0x%llx directflip=0x%llx overlays=0x%llx\n",
                    (UINT64)cfg.presentOffset, (UINT64)cfg.directFlipOffset, (UINT64)cfg.overlaysOffset);
    }

    // Pre-create diag log with cleared DACL so the DLL can write to it from DWM's context
    {
        std::wstring diagPath = tempDir + L"ApplyIccLut_dither_diag.log";
        HANDLE hDiag = CreateFileW(diagPath.c_str(), GENERIC_WRITE, FILE_SHARE_READ | FILE_SHARE_WRITE,
                                   NULL, OPEN_ALWAYS, FILE_ATTRIBUTE_NORMAL, NULL);
        if (hDiag != INVALID_HANDLE_VALUE)
            CloseHandle(hDiag);
        ClearFileDacl(diagPath.c_str());
    }

    // Inject into all DWM processes in our session
    bool anySuccess = false;
    for (DWORD dwmPid : dwmPids)
    {
        wprintf(L"  Injecting into dwm.exe (PID %lu, session %lu)...\n", dwmPid, currentSessionId);
        if (InjectDll(dwmPid, dllDest.c_str(), DITHER_DLL_NAME, verbose))
            anySuccess = true;
    }

    RevertToSelf();

    if (anySuccess)
    {
        // Write flag file for boot persistence (contains dither bits for re-injection)
        HANDLE hFlag = CreateFileW(flagPath.c_str(), GENERIC_WRITE, 0, NULL, CREATE_ALWAYS, FILE_ATTRIBUTE_NORMAL, NULL);
        if (hFlag != INVALID_HANDLE_VALUE)
        {
            if (ditherBits > 0)
            {
                char buf[16];
                int len = wsprintfA(buf, "%d", ditherBits);
                DWORD written;
                WriteFile(hFlag, buf, len, &written, NULL);
            }
            CloseHandle(hFlag);
        }

        wprintf(L"  Dithering active (%s). Flag file written for boot persistence.\n",
                ditherBits > 0 ? L"forced bit depth" : L"auto bit depth");
    }
    else
    {
        wprintf(L"  ERROR: Could not inject into any dwm.exe process.\n");
    }

    return anySuccess;
}

static bool UninjectAndRemoveDither(bool verbose)
{
    std::wstring tempDir = GetSystemTempPath();
    std::wstring dllPath = tempDir + DITHER_DLL_NAME;
    std::wstring flagPath = tempDir + DITHER_FLAG_FILE;
    std::wstring cfgPath = tempDir + DITHER_CFG_FILE;

    if (!EnableDebugPrivilege())
    {
        wprintf(L"  WARNING: Could not enable SE_DEBUG_PRIVILEGE.\n");
    }

    if (!ImpersonateSystem(verbose))
    {
        wprintf(L"  ERROR: Could not impersonate SYSTEM. Are you running as Administrator?\n");
        return false;
    }

    // Uninject from all DWM processes in our session
    DWORD currentSessionId = 0;
    ProcessIdToSessionId(GetCurrentProcessId(), &currentSessionId);

    HANDLE snap = CreateToolhelp32Snapshot(TH32CS_SNAPPROCESS, 0);
    if (snap != INVALID_HANDLE_VALUE)
    {
        PROCESSENTRY32W pe = {};
        pe.dwSize = sizeof(pe);
        if (Process32FirstW(snap, &pe))
        {
            do {
                if (_wcsicmp(pe.szExeFile, L"dwm.exe") == 0)
                {
                    DWORD dwmSessionId = 0;
                    ProcessIdToSessionId(pe.th32ProcessID, &dwmSessionId);
                    if (dwmSessionId != currentSessionId)
                        continue;
                    wprintf(L"  Uninjecting from dwm.exe (PID %lu)...\n", pe.th32ProcessID);
                    UninjectDll(pe.th32ProcessID, DITHER_DLL_NAME, verbose);
                }
            } while (Process32NextW(snap, &pe));
        }
        CloseHandle(snap);
    }

    RevertToSelf();

    // Remove flag file and config file
    DeleteFileW(flagPath.c_str());
    DeleteFileW(cfgPath.c_str());

    // Remove DLL from temp
    DeleteFileW(dllPath.c_str());

    wprintf(L"  Dithering removed.\n");
    return true;
}

static bool IsDitherFlagSet()
{
    std::wstring flagPath = GetSystemTempPath() + DITHER_FLAG_FILE;
    DWORD attrib = GetFileAttributesW(flagPath.c_str());
    return (attrib != INVALID_FILE_ATTRIBUTES);
}

static bool IsDitherAlreadyInjected()
{
    if (!EnableDebugPrivilege())
        return false;

    if (!ImpersonateSystem(false))
        return false;

    bool found = false;
    DWORD dwmPid = FindProcessByName(L"dwm.exe");
    if (dwmPid)
    {
        HANDLE snap = CreateToolhelp32Snapshot(TH32CS_SNAPMODULE | TH32CS_SNAPMODULE32, dwmPid);
        if (snap != INVALID_HANDLE_VALUE)
        {
            MODULEENTRY32W me = {};
            me.dwSize = sizeof(me);
            if (Module32FirstW(snap, &me))
            {
                do {
                    if (_wcsicmp(me.szModule, DITHER_DLL_NAME) == 0)
                    {
                        found = true;
                        break;
                    }
                } while (Module32NextW(snap, &me));
            }
            CloseHandle(snap);
        }
    }

    RevertToSelf();
    return found;
}

// ============================================================================
// Entry point
// ============================================================================

int wmain(int argc, wchar_t* argv[])
{
    wprintf(L"ApplyIccLut - ICC/Cube LUT Loader for Windows 11\n");
    wprintf(L"=================================================\n\n");

    // Parse arguments
    std::wstring manualProfile;
    std::wstring setGpuProfile;
    bool setGpuMode  = false;
    bool verbose     = false;
    bool resetMode   = false;
    bool restoreMode = false;
    bool forceApply  = false;
    bool ditherMode  = false;
    bool noDitherMode = false;
    bool probeMode   = false;
    int  ditherBits  = 0; // 0 = auto (SDR=8, HDR=10)
    int  ditherModeVal = 4; // NvAPI dither mode: 0-4, default=4 (Temporal)
    bool targetSdr   = false;
    bool targetHdr   = false;
    bool monitorExplicit = false; // true if -m was given on the command line
    int  targetMonitor = 1; // 1-based index, 0 = all monitors

    for (int i = 1; i < argc; i++)
    {
        if (wcscmp(argv[i], L"-v") == 0 || wcscmp(argv[i], L"--verbose") == 0)
        {
            verbose = true;
        }
        else if (wcscmp(argv[i], L"-r") == 0 || wcscmp(argv[i], L"--reset") == 0)
        {
            resetMode = true;
        }
        else if (wcscmp(argv[i], L"-f") == 0 || wcscmp(argv[i], L"--force") == 0)
        {
            forceApply = true;
        }
        else if (wcscmp(argv[i], L"-R") == 0 || wcscmp(argv[i], L"--restore") == 0)
        {
            restoreMode = true;
        }
        else if (wcscmp(argv[i], L"--sdr") == 0)
        {
            targetSdr = true;
        }
        else if (wcscmp(argv[i], L"--hdr") == 0)
        {
            targetHdr = true;
        }
        else if (wcscmp(argv[i], L"-d") == 0 || wcscmp(argv[i], L"--dither") == 0)
        {
            ditherMode = true;
        }
        else if (wcscmp(argv[i], L"--no-dither") == 0)
        {
            noDitherMode = true;
        }
        else if (wcscmp(argv[i], L"--probe") == 0)
        {
            probeMode = true;
        }
        else if (wcscmp(argv[i], L"--dither-bits") == 0 && i + 1 < argc)
        {
            ditherBits = _wtoi(argv[++i]);
            if (ditherBits < 1 || ditherBits > 16)
            {
                wprintf(L"Invalid dither bit depth: %s  (valid range: 1-16)\n", argv[i]);
                return 1;
            }
        }
        else if (wcscmp(argv[i], L"--dither-mode") == 0 && i + 1 < argc)
        {
            const wchar_t* modeArg = argv[++i];
            if (_wcsicmp(modeArg, L"spatial") == 0 || _wcsicmp(modeArg, L"spatial-dynamic") == 0)
                ditherModeVal = 0;
            else if (_wcsicmp(modeArg, L"spatial-static") == 0)
                ditherModeVal = 1;
            else if (_wcsicmp(modeArg, L"spatial2x2") == 0 || _wcsicmp(modeArg, L"spatial-dynamic-2x2") == 0)
                ditherModeVal = 2;
            else if (_wcsicmp(modeArg, L"spatial-static-2x2") == 0)
                ditherModeVal = 3;
            else if (_wcsicmp(modeArg, L"temporal") == 0)
                ditherModeVal = 4;
            else
            {
                wprintf(L"Invalid dither mode: %s\n", modeArg);
                wprintf(L"  Valid modes: temporal (default), spatial, spatial-static,\n");
                wprintf(L"              spatial2x2, spatial-static-2x2\n");
                return 1;
            }
        }
        else if ((wcscmp(argv[i], L"-p") == 0 || wcscmp(argv[i], L"--profile") == 0)
                 && i + 1 < argc)
        {
            manualProfile = argv[++i];
        }
        else if (wcscmp(argv[i], L"-s") == 0 || wcscmp(argv[i], L"--set-gpu-profile") == 0)
        {
            setGpuMode = true;
            // Optional profile name: consume next arg if it doesn't look like a flag
            if (i + 1 < argc && argv[i + 1][0] != L'-')
                setGpuProfile = argv[++i];
        }
        else if ((wcscmp(argv[i], L"-m") == 0 || wcscmp(argv[i], L"--monitor") == 0)
                 && i + 1 < argc)
        {
            monitorExplicit = true;
            targetMonitor = _wtoi(argv[++i]);
            if (targetMonitor < 0)
            {
                wprintf(L"Invalid monitor number: %s  (use 0 for all, or 1-%s)\n",
                        argv[i], L"N");
                return 1;
            }
        }
        else if (wcscmp(argv[i], L"-h") == 0 || wcscmp(argv[i], L"--help") == 0)
        {
            wprintf(L"Usage: ApplyIccLut.exe [options]\n\n");
            wprintf(L"Options:\n");
            wprintf(L"  -m <num>    Target a single monitor by number (default: 1)\n");
            wprintf(L"              Use 0 to apply to all monitors\n");
            wprintf(L"  -p <path>   Apply LUT via SetDeviceGammaRamp (256 entries, ICC or .cube)\n");
            wprintf(L"  -s [name]   Set GPU default profile (full MHC2 pipeline, up to 4096 LUT)\n");
            wprintf(L"              With name: sets the specified profile as GPU default\n");
            wprintf(L"              Without name: re-applies the current default (wake-up)\n");
            wprintf(L"  -f          Force apply even if a LUT is already active\n");
            wprintf(L"  -r          Reset gamma ramp AND unset GPU color profile\n");
            wprintf(L"  -R          Restore: re-enable GPU color profile as default\n");
            wprintf(L"  --sdr       Target SDR pipeline only (for -s, -r, -R)\n");
            wprintf(L"  --hdr       Target HDR pipeline only (for -s, -r, -R)\n");
            wprintf(L"              Default: both SDR and HDR when neither is specified\n");
            wprintf(L"  -d          Enable dithering (NvAPI hardware on NVIDIA, DWM fallback)\n");
            wprintf(L"  --dither-bits <N>  Override dither bit depth (NvAPI: 6/8/10, default: 8)\n");
            wprintf(L"  --dither-mode <M>  NvAPI dither mode (default: temporal)\n");
            wprintf(L"              Modes: temporal, spatial, spatial-static,\n");
            wprintf(L"                     spatial2x2, spatial-static-2x2\n");
            wprintf(L"  --no-dither Disable dithering (resets NvAPI + removes DWM injection)\n");
            wprintf(L"  -v          Verbose output\n");
            wprintf(L"  -h          Show this help\n\n");
            wprintf(L"Supported formats:\n");
            wprintf(L"  ICC profiles (.icc, .icm)  - reads VCGT or MHC2 LUT tags\n");
            wprintf(L"  Cube LUT files (.cube)     - reads 1D LUTs (LUT_1D_SIZE)\n\n");
            wprintf(L"GPU pipeline (-s):\n");
            wprintf(L"  Sets an ICC profile as the GPU default via ColorProfile APIs.\n");
            wprintf(L"  The GPU driver applies the full MHC2 pipeline (matrix + LUT) at\n");
            wprintf(L"  up to 4096 entries per channel - no downsampling.\n");
            wprintf(L"  Without a name, re-sets the current default (useful to wake up\n");
            wprintf(L"  a pipeline that failed to apply at boot).\n");
            wprintf(L"  Profile must exist in: %%WINDIR%%\\system32\\spool\\drivers\\color\n\n");
            wprintf(L"Dithering (-d / --no-dither):\n");
            wprintf(L"  On NVIDIA GPUs, uses NvAPI hardware dithering at the display\n");
            wprintf(L"  output stage (after ICC color management).\n");
            wprintf(L"  Bit depths: 6, 8, 10 (default: 8). Modes: temporal (default),\n");
            wprintf(L"  spatial, spatial-static, spatial2x2, spatial-static-2x2.\n");
            wprintf(L"  Falls back to DWM injection (blue-noise shader) on non-NVIDIA\n");
            wprintf(L"  or older systems. Use --no-dither to disable. Automatically\n");
            wprintf(L"  re-enabled at boot with the GPU wake-up (Task Scheduler).\n\n");
            wprintf(L"With no arguments, performs a GPU pipeline wake-up kick\n");
            wprintf(L"on all monitors (equivalent to -s -m 0), and re-enables\n");
            wprintf(L"dithering if previously enabled with --dither.\n\n");
            wprintf(L"Tip: Add to Task Scheduler (trigger: At log on) to fix\n");
            wprintf(L"Windows 11 not loading the LUT at boot.\n");
            return 0;
        }
        else
        {
            wprintf(L"Unknown option: %s  (use -h for help)\n", argv[i]);
            return 1;
        }
    }

    // Handle probe mode (independent, exits after completion)
    if (probeMode)
    {
        RunProbeMode(verbose);
        return 0;
    }

    // Handle dithering modes (these are independent of the LUT pipeline)
    if (ditherMode)
    {
        // Try NvAPI hardware dithering first (NVIDIA GPUs)
        wprintf(L"Dithering: attempting NvAPI hardware dithering...\n");
        if (EnableNvApiDithering(ditherBits, ditherModeVal, verbose))
        {
            // Write flag file for boot persistence with "nvapi:" prefix
            // Format: "nvapi:<bits>,<mode>"
            std::wstring flagPath = GetSystemTempPath() + DITHER_FLAG_FILE;
            HANDLE hFlag = CreateFileW(flagPath.c_str(), GENERIC_WRITE, 0, NULL, CREATE_ALWAYS, FILE_ATTRIBUTE_NORMAL, NULL);
            if (hFlag != INVALID_HANDLE_VALUE)
            {
                char buf[32];
                int len = wsprintfA(buf, "nvapi:%d,%d", ditherBits, ditherModeVal);
                DWORD written;
                WriteFile(hFlag, buf, len, &written, NULL);
                CloseHandle(hFlag);
            }
            wprintf(L"  Flag file written for boot persistence.\n\n");
        }
        else
        {
            // Fall back to DWM injection (pre-25H2 or non-NVIDIA)
            wprintf(L"Dithering: NvAPI not available, falling back to DWM injection...\n");
            if (!DeployAndInjectDither(verbose, ditherBits))
                return 1;
            wprintf(L"\n");
        }
    }

    if (noDitherMode)
    {
        // Disable NvAPI hardware dithering (safe even if not NVIDIA)
        wprintf(L"Dithering: disabling...\n");
        DisableNvApiDithering(verbose);

        // Also remove DWM-injected dithering and flag files
        UninjectAndRemoveDither(verbose);
        wprintf(L"\n");
    }

    // If only dither/no-dither was requested, exit early
    if ((ditherMode || noDitherMode) && !setGpuMode && !resetMode && !restoreMode && manualProfile.empty())
        return 0;

    // Default behaviour: no mode selected → GPU pipeline wake-up on all monitors
    if (!setGpuMode && !resetMode && !restoreMode && manualProfile.empty())
    {
        setGpuMode = true;  // bare -s behaviour (wake-up kick)
        if (!monitorExplicit)
            targetMonitor = 0; // all monitors

        // Auto-re-enable dithering if flag file exists
        if (IsDitherFlagSet())
        {
            // Read stored dither settings from flag file
            // Format: "nvapi:<bits>,<mode>" or just "<bits>" for DWM
            int storedBits = 0;
            int storedMode = 4; // default: Temporal
            bool storedNvApi = false;
            {
                std::wstring flagPath = GetSystemTempPath() + DITHER_FLAG_FILE;
                HANDLE hFlag = CreateFileW(flagPath.c_str(), GENERIC_READ, FILE_SHARE_READ, NULL, OPEN_EXISTING, 0, NULL);
                if (hFlag != INVALID_HANDLE_VALUE)
                {
                    char buf[32] = {};
                    DWORD bytesRead = 0;
                    ReadFile(hFlag, buf, sizeof(buf) - 1, &bytesRead, NULL);
                    CloseHandle(hFlag);

                    if (bytesRead >= 6 && memcmp(buf, "nvapi:", 6) == 0)
                    {
                        storedNvApi = true;
                        // Parse bits
                        int val = 0;
                        DWORD j = 6;
                        for (; j < bytesRead && buf[j] >= '0' && buf[j] <= '9'; j++)
                            val = val * 10 + (buf[j] - '0');
                        if (val > 0 && val <= 16)
                            storedBits = val;
                        // Parse mode after comma
                        if (j < bytesRead && buf[j] == ',')
                        {
                            j++;
                            int mval = 0;
                            for (; j < bytesRead && buf[j] >= '0' && buf[j] <= '9'; j++)
                                mval = mval * 10 + (buf[j] - '0');
                            if (mval >= 0 && mval <= 4)
                                storedMode = mval;
                        }
                    }
                    else
                    {
                        int val = 0;
                        for (DWORD j = 0; j < bytesRead && buf[j] >= '0' && buf[j] <= '9'; j++)
                            val = val * 10 + (buf[j] - '0');
                        if (val > 0 && val <= 16)
                            storedBits = val;
                    }
                }
            }

            if (storedNvApi)
            {
                wprintf(L"Dithering: re-enabling NvAPI hardware dithering (flag file present)...\n");
                EnableNvApiDithering(storedBits, storedMode, verbose);
                wprintf(L"\n");
            }
            else if (!IsDitherAlreadyInjected())
            {
                wprintf(L"Dithering: re-injecting DWM (flag file present from previous --dither)...\n");
                DeployAndInjectDither(verbose, storedBits);
                wprintf(L"\n");
            }
        }
    }

    // Default: target both SDR and HDR if neither specified
    if (!targetSdr && !targetHdr)
    {
        targetSdr = true;
        targetHdr = true;
    }

    // Build the list of pipelines to operate on
    std::vector<bool> hdrModes;
    if (targetSdr) hdrModes.push_back(false); // SDR first
    if (targetHdr) hdrModes.push_back(true);  // then HDR

    // Enumerate displays
    auto monitors = EnumerateMonitors();
    if (monitors.empty())
    {
        wprintf(L"ERROR: No active displays found.\n");
        return 1;
    }

    wprintf(L"Active displays: %zu\n", monitors.size());
    for (size_t i = 0; i < monitors.size(); i++)
    {
        wprintf(L"  %zu. %s  (%s)\n", i + 1,
                monitors[i].friendlyName.empty() ? L"(unknown)" : monitors[i].friendlyName.c_str(),
                monitors[i].gdiDeviceName.c_str());
    }

    // Validate monitor selection
    if (targetMonitor > static_cast<int>(monitors.size()))
    {
        wprintf(L"\nERROR: Monitor %d does not exist (max: %zu).\n",
                targetMonitor, monitors.size());
        return 1;
    }

    if (targetMonitor > 0)
        wprintf(L"\nTarget: monitor %d\n\n", targetMonitor);
    else
        wprintf(L"\nTarget: all monitors\n\n");

    // Load GPU ColorProfile APIs (Win10 1903+, used for -r)
    auto gpuApi = LoadGpuColorProfileApi();
    if (verbose)
        wprintf(L"GPU ColorProfile APIs: %s\n\n",
                gpuApi.loaded ? L"available" : L"not available");

    // Process selected monitor(s)
    int okCount   = 0;
    int failCount = 0;

    for (size_t i = 0; i < monitors.size(); i++)
    {
        // Skip monitors that aren't selected (0 = all)
        if (targetMonitor > 0 && static_cast<int>(i + 1) != targetMonitor)
            continue;

        if (ProcessMonitor(monitors[i], manualProfile, resetMode, restoreMode,
                           setGpuMode, setGpuProfile, hdrModes,
                           forceApply, verbose, gpuApi))
            okCount++;
        else
            failCount++;
    }

    wprintf(L"Done. %d succeeded, %d failed.\n", okCount, failCount);
    return (failCount > 0) ? 1 : 0;
}
