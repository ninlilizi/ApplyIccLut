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
//   ApplyIccLut.exe --probe         Discover dithering offsets for this dwmcore.dll
//   ApplyIccLut.exe --probe -v      Verbose offset discovery with all candidates

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
static bool aob_match_inverse(const unsigned char* buf, const unsigned char* mask, int len)
{
    for (int i = 0; i < len; ++i)
    {
        if (buf[i] != mask[i] && mask[i] != '?')
            return true;
    }
    return false;
}

// All AOB patterns
// Windows 10
static const unsigned char pat_present_w10[] = {
    0x48, 0x89, 0x5c, 0x24, 0x08, 0x48, 0x89, 0x74, 0x24, 0x10, 0x57, 0x48, 0x83, 0xec, 0x40, 0x48, 0x8b, 0xb1, 0x20,
    0x2c, 0x00, 0x00, 0x45, 0x8b, 0xd0, 0x48, 0x8b, 0xfa, 0x48, 0x8b, 0xd9, 0x48, 0x85, 0xf6, 0x0f, 0x85
};
static const unsigned char pat_directflip_w10[] = {
    0x48, 0x89, 0x7c, 0x24, 0x20, 0x55, 0x41, 0x54, 0x41, 0x55, 0x41, 0x56, 0x41, 0x57, 0x48, 0x8b, 0xec, 0x48, 0x83,
    0xec, 0x40
};
static const unsigned char pat_overlays_w10[] = {
    0x75, 0x04, 0x32, 0xc0, 0xc3, 0xcc, 0x83, 0x79, 0x30, 0x01, 0x0f, 0x97, 0xc0, 0xc3
};

// Windows 11 (pre-24H2)
static const unsigned char pat_present_w11[] = {
    0x40, 0x53, 0x55, 0x56, 0x57, 0x41, 0x56, 0x41, 0x57, 0x48, 0x81, 0xEC, 0x88, 0x00, 0x00, 0x00, 0x48, 0x8B, 0x05,
    '?', '?', '?', '?', 0x48, 0x33, 0xC4, 0x48, 0x89, 0x44, 0x24, 0x78, 0x48
};
static const unsigned char pat_directflip_w11[] = {
    0x40, 0x55, 0x53, 0x56, 0x57, 0x41, 0x54, 0x41, 0x55, 0x41, 0x56, 0x41, 0x57, 0x48, 0x8B, 0xEC, 0x48, 0x83, 0xEC,
    0x68, 0x48,
};
static const unsigned char pat_overlays_w11[] = {
    0x83, 0x3D, '?', '?', '?', '?', '?', 0x75, 0x04
};

// Windows 11 24H2 (build >= 26100)
static const unsigned char pat_present_w11_24h2[] = {
    0x40, 0x53, 0x55, 0x56, 0x57, 0x41, 0x56, 0x41, 0x57, 0x48, 0x81, 0xEC, 0x90, 0x00, 0x00, 0x00, 0x48, 0x8B, 0x05,
    '?', '?', '?', '?', 0x48, 0x33, 0xC4, 0x48, 0x89, 0x44, 0x24, 0x80, 0x48
};
static const unsigned char pat_directflip_w11_24h2[] = {
    0x40, 0x55, 0x53, 0x56, 0x57, 0x41, 0x54, 0x41, 0x55, 0x41, 0x56, 0x41, 0x57, 0x48, 0x8B, 0xEC, 0x48, 0x83, 0xEC,
    0x70, 0x48,
};
static const unsigned char pat_overlays_w11_24h2[] = {
    0x83, 0x3D, '?', '?', '?', '?', '?', 0x74, 0x04
};

// Windows 11 25H2 (build >= 26200)
static const unsigned char pat_present_w11_25h2[] = {
    0x40, 0x53, 0x55, 0x56, 0x57, 0x41, 0x56, 0x41, 0x57, 0x48, 0x83, 0xEC, 0x68, 0x48, 0x8B, 0x05,
    '?', '?', '?', '?', 0x48, 0x33, 0xC4, 0x48, 0x89, 0x44, 0x24, 0x50
};
static const unsigned char pat_directflip_w11_25h2[] = {
    0x40, 0x55, 0x53, 0x56, 0x57, 0x41, 0x54, 0x41, 0x55, 0x41, 0x56, 0x41, 0x57, 0x48, 0x8B, 0xEC, 0x48, 0x83, 0xEC,
    0x78, 0x48,
};
static const unsigned char pat_overlays_w11_25h2[] = {
    0x48, 0x8B, 0x41, 0x38, 0x48, 0x85, 0xC0, 0x75, 0x04, 0x32, 0xC0, 0xC3
};

// IOverlaySwapChain offsets (version-specific, passed to DLL via config)
static const int OVERLAY_SWAPCHAIN_OFFSET_W10     = -0x118;
static const int OVERLAY_SWAPCHAIN_OFFSET_W11     = 0xE0;
static const int OVERLAY_SWAPCHAIN_OFFSET_W11_24H2 = 0xE8;
static const int OVERLAY_HWPROT_OFFSET_W10     = -0xbc;
static const int OVERLAY_HWPROT_OFFSET_W11     = -0x144;
static const int OVERLAY_HWPROT_OFFSET_W11_24H2 = -0x14C;

// ============================================================================
// Cross-version pattern sets for --probe mode
// ============================================================================

struct PatternSet {
    const char* label;
    const unsigned char *present, *directFlip, *overlays;
    int presentLen, directFlipLen, overlaysLen;
    bool isWin11;
    int hwProtOffset, swapChainOffset;
};

static const PatternSet ALL_PATTERNS[] = {
    {
        "Win10",
        pat_present_w10,      pat_directflip_w10,      pat_overlays_w10,
        sizeof(pat_present_w10), sizeof(pat_directflip_w10), sizeof(pat_overlays_w10),
        false, OVERLAY_HWPROT_OFFSET_W10, OVERLAY_SWAPCHAIN_OFFSET_W10
    },
    {
        "Win11 (pre-24H2)",
        pat_present_w11,      pat_directflip_w11,      pat_overlays_w11,
        sizeof(pat_present_w11), sizeof(pat_directflip_w11), sizeof(pat_overlays_w11),
        true, OVERLAY_HWPROT_OFFSET_W11, OVERLAY_SWAPCHAIN_OFFSET_W11
    },
    {
        "Win11 24H2",
        pat_present_w11_24h2, pat_directflip_w11_24h2, pat_overlays_w11_24h2,
        sizeof(pat_present_w11_24h2), sizeof(pat_directflip_w11_24h2), sizeof(pat_overlays_w11_24h2),
        true, OVERLAY_HWPROT_OFFSET_W11_24H2, OVERLAY_SWAPCHAIN_OFFSET_W11_24H2
    },
    {
        "Win11 25H2",
        pat_present_w11_25h2, pat_directflip_w11_25h2, pat_overlays_w11_25h2,
        sizeof(pat_present_w11_25h2), sizeof(pat_directflip_w11_25h2), sizeof(pat_overlays_w11_25h2),
        true, OVERLAY_HWPROT_OFFSET_W11_24H2, OVERLAY_SWAPCHAIN_OFFSET_W11_24H2
    },
};
static const int NUM_PATTERN_SETS = sizeof(ALL_PATTERNS) / sizeof(ALL_PATTERNS[0]);

// Binary config struct shared with DitherDll (must match exactly)
#pragma pack(push, 1)
struct DitherConfig {
    UINT32 magic;              // 'DITH' (0x48544944)
    UINT32 version;            // struct version (1)
    INT64  presentOffset;      // COverlayContext::Present offset from dwmcore base
    INT64  directFlipOffset;   // IsCandidateDirectFlipCompatible offset from dwmcore base
    INT64  overlaysOffset;     // OverlaysEnabled offset from dwmcore base
    INT32  hwProtOffset;       // IOverlaySwapChain HardwareProtected offset
    INT32  swapChainOffset;    // IOverlaySwapChain IDXGISwapChain offset
    UINT32 isWindows11;        // 1 if Win11+ (affects overlay pointer resolution)
    UINT32 ditherBits;         // 0 = auto, otherwise forced bit depth
};
#pragma pack(pop)

static const UINT32 DITHER_CONFIG_MAGIC = 0x48544944;
static const UINT32 DITHER_CONFIG_VERSION = 1;

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
    UINT32 sourceFlags;         // 1=formal AOB, 2=probe-discovered
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

    // Open dwm.exe and find dwmcore.dll
    HANDLE hProcess = OpenProcess(PROCESS_QUERY_INFORMATION | PROCESS_VM_READ, FALSE, dwmPid);
    if (!hProcess)
    {
        wprintf(L"    Could not open dwm.exe for scanning (err=%lu)\n", GetLastError());
        return false;
    }

    HMODULE dwmcoreBase = NULL;
    DWORD dwmcoreSize = 0;
    {
        HANDLE snap = CreateToolhelp32Snapshot(TH32CS_SNAPMODULE | TH32CS_SNAPMODULE32, dwmPid);
        if (snap != INVALID_HANDLE_VALUE)
        {
            MODULEENTRY32W me = {};
            me.dwSize = sizeof(me);
            if (Module32FirstW(snap, &me))
            {
                do {
                    if (_wcsicmp(me.szModule, L"dwmcore.dll") == 0)
                    {
                        dwmcoreBase = me.hModule;
                        dwmcoreSize = me.modBaseSize;
                        break;
                    }
                } while (Module32NextW(snap, &me));
            }
            CloseHandle(snap);
        }
    }

    if (!dwmcoreBase || dwmcoreSize == 0)
    {
        wprintf(L"    dwmcore.dll not found in dwm.exe\n");
        CloseHandle(hProcess);
        return false;
    }

    wprintf(L"    dwmcore.dll: base=0x%llx size=%lu\n", (UINT64)dwmcoreBase, dwmcoreSize);

    // Read the module image
    std::vector<unsigned char> image(dwmcoreSize);
    SIZE_T bytesRead = 0;
    ReadProcessMemory(hProcess, dwmcoreBase, image.data(), dwmcoreSize, &bytesRead);
    CloseHandle(hProcess);

    if (bytesRead == 0)
    {
        wprintf(L"    Could not read dwmcore.dll memory\n");
        return false;
    }

    size_t imageSize = bytesRead;

    // Select patterns for version
    const unsigned char* presentPat;   int presentLen;
    const unsigned char* directFlipPat; int directFlipLen;
    const unsigned char* overlaysPat;  int overlaysLen;

    if (is25h2)
    {
        presentPat = pat_present_w11_25h2;     presentLen = sizeof(pat_present_w11_25h2);
        directFlipPat = pat_directflip_w11_25h2; directFlipLen = sizeof(pat_directflip_w11_25h2);
        overlaysPat = pat_overlays_w11_25h2;   overlaysLen = sizeof(pat_overlays_w11_25h2);
    }
    else if (is24h2)
    {
        presentPat = pat_present_w11_24h2;     presentLen = sizeof(pat_present_w11_24h2);
        directFlipPat = pat_directflip_w11_24h2; directFlipLen = sizeof(pat_directflip_w11_24h2);
        overlaysPat = pat_overlays_w11_24h2;   overlaysLen = sizeof(pat_overlays_w11_24h2);
    }
    else if (isWin11)
    {
        presentPat = pat_present_w11;          presentLen = sizeof(pat_present_w11);
        directFlipPat = pat_directflip_w11;    directFlipLen = sizeof(pat_directflip_w11);
        overlaysPat = pat_overlays_w11;        overlaysLen = sizeof(pat_overlays_w11);
    }
    else
    {
        presentPat = pat_present_w10;          presentLen = sizeof(pat_present_w10);
        directFlipPat = pat_directflip_w10;    directFlipLen = sizeof(pat_directflip_w10);
        overlaysPat = pat_overlays_w10;        overlaysLen = sizeof(pat_overlays_w10);
    }

    // Scan for patterns
    bool foundPresent = false, foundDirectFlip = false, foundOverlays = false;

    for (size_t i = 0; i < imageSize; i++)
    {
        if (!foundPresent && i + presentLen <= imageSize &&
            !aob_match_inverse(image.data() + i, presentPat, presentLen))
        {
            foundPresent = true;
            cfg->presentOffset = (INT64)i;
            wprintf(L"    Found Present at offset 0x%llx\n", (UINT64)i);
        }
        else if (!foundDirectFlip && i + directFlipLen <= imageSize &&
                 !aob_match_inverse(image.data() + i, directFlipPat, directFlipLen))
        {
            foundDirectFlip = true;
            cfg->directFlipOffset = (INT64)i;
            wprintf(L"    Found DirectFlip at offset 0x%llx\n", (UINT64)i);
        }
        else if (!foundOverlays && i + overlaysLen <= imageSize &&
                 !aob_match_inverse(image.data() + i, overlaysPat, overlaysLen))
        {
            foundOverlays = true;
            cfg->overlaysOffset = (INT64)i;
            wprintf(L"    Found OverlaysEnabled at offset 0x%llx\n", (UINT64)i);
        }

        if (foundPresent && foundDirectFlip && foundOverlays)
            break;
    }

    if (!foundPresent)   wprintf(L"    ERROR: Present pattern not found\n");
    if (!foundDirectFlip) wprintf(L"    ERROR: DirectFlip pattern not found\n");
    if (!foundOverlays)  wprintf(L"    ERROR: OverlaysEnabled pattern not found\n");

    return foundPresent && foundDirectFlip && foundOverlays;
}

static void DiagLogPrintf(FILE* f, const char* fmt, ...)
{
    SYSTEMTIME st;
    GetLocalTime(&st);
    fprintf(f, "[%02d:%02d:%02d.%03d] ", st.wHour, st.wMinute, st.wSecond, st.wMilliseconds);
    va_list ap;
    va_start(ap, fmt);
    vfprintf(f, fmt, ap);
    va_end(ap);
    fprintf(f, "\n");
}

static void DiagLogBytes(FILE* f, const char* prefix, const unsigned char* bytes, size_t len)
{
    fprintf(f, "  %s", prefix);
    for (size_t i = 0; i < len && i < 48; i++)
        fprintf(f, "%02x ", bytes[i]);
    if (len > 48) fprintf(f, "...");
    fprintf(f, "\n");
}

static void RunRemoteDiagnostics(DWORD dwmPid, bool isWin11, bool is24h2, bool is25h2)
{
    std::wstring logPath = GetSystemTempPath() + L"ApplyIccLut_dither_diag.log";
    FILE* f = NULL;
    if (_wfopen_s(&f, logPath.c_str(), L"a") != 0 || !f)
    {
        wprintf(L"  Could not open diagnostic log for writing.\n");
        return;
    }

    WindowsVersion diagVer = GetTrueWindowsVersion();
    DiagLogPrintf(f, "=== Remote Diagnostic Scan (from main app) ===");
    DiagLogPrintf(f, "Target: dwm.exe PID %lu", dwmPid);
    DiagLogPrintf(f, "OS: %s (build %lu)",
        is25h2 ? "Windows 11 25H2+" :
        is24h2 ? "Windows 11 24H2" :
        isWin11 ? "Windows 11 (pre-24H2)" : "Windows 10",
        diagVer.build);

    // Open dwm.exe and find dwmcore.dll
    HANDLE hProcess = OpenProcess(PROCESS_QUERY_INFORMATION | PROCESS_VM_READ, FALSE, dwmPid);
    if (!hProcess)
    {
        DiagLogPrintf(f, "ERROR: Could not open dwm.exe for reading (err=%lu)", GetLastError());
        fclose(f);
        return;
    }

    // Enumerate modules to find dwmcore.dll
    HMODULE dwmcoreBase = NULL;
    DWORD dwmcoreSize = 0;
    {
        HANDLE snap = CreateToolhelp32Snapshot(TH32CS_SNAPMODULE | TH32CS_SNAPMODULE32, dwmPid);
        if (snap != INVALID_HANDLE_VALUE)
        {
            MODULEENTRY32W me = {};
            me.dwSize = sizeof(me);
            if (Module32FirstW(snap, &me))
            {
                do {
                    if (_wcsicmp(me.szModule, L"dwmcore.dll") == 0)
                    {
                        dwmcoreBase = me.hModule;
                        dwmcoreSize = me.modBaseSize;
                        break;
                    }
                } while (Module32NextW(snap, &me));
            }
            CloseHandle(snap);
        }
    }

    if (!dwmcoreBase || dwmcoreSize == 0)
    {
        DiagLogPrintf(f, "ERROR: dwmcore.dll not found in dwm.exe");
        CloseHandle(hProcess);
        fclose(f);
        return;
    }

    DiagLogPrintf(f, "dwmcore.dll base: 0x%llx  size: %lu bytes", (UINT64)dwmcoreBase, dwmcoreSize);

    // Read the entire dwmcore.dll image from remote process
    std::vector<unsigned char> image(dwmcoreSize);
    SIZE_T bytesRead = 0;
    if (!ReadProcessMemory(hProcess, dwmcoreBase, image.data(), dwmcoreSize, &bytesRead))
    {
        DiagLogPrintf(f, "ERROR: ReadProcessMemory failed (err=%lu, read %zu of %lu)",
                      GetLastError(), bytesRead, dwmcoreSize);
        // Try partial read — may have gaps due to guard pages
        if (bytesRead == 0)
        {
            CloseHandle(hProcess);
            fclose(f);
            return;
        }
    }
    DiagLogPrintf(f, "Read %zu bytes from dwmcore.dll", bytesRead);
    CloseHandle(hProcess);

    size_t imageSize = bytesRead;

    // Select patterns for version
    const unsigned char* presentPat;   int presentLen;
    const unsigned char* directFlipPat; int directFlipLen;
    const unsigned char* overlaysPat;  int overlaysLen;
    const char* verLabel;

    if (is25h2)
    {
        presentPat = pat_present_w11_25h2;     presentLen = sizeof(pat_present_w11_25h2);
        directFlipPat = pat_directflip_w11_25h2; directFlipLen = sizeof(pat_directflip_w11_25h2);
        overlaysPat = pat_overlays_w11_25h2;   overlaysLen = sizeof(pat_overlays_w11_25h2);
        verLabel = "25H2";
    }
    else if (is24h2)
    {
        presentPat = pat_present_w11_24h2;     presentLen = sizeof(pat_present_w11_24h2);
        directFlipPat = pat_directflip_w11_24h2; directFlipLen = sizeof(pat_directflip_w11_24h2);
        overlaysPat = pat_overlays_w11_24h2;   overlaysLen = sizeof(pat_overlays_w11_24h2);
        verLabel = "24H2";
    }
    else if (isWin11)
    {
        presentPat = pat_present_w11;          presentLen = sizeof(pat_present_w11);
        directFlipPat = pat_directflip_w11;    directFlipLen = sizeof(pat_directflip_w11);
        overlaysPat = pat_overlays_w11;        overlaysLen = sizeof(pat_overlays_w11);
        verLabel = "Win11";
    }
    else
    {
        presentPat = pat_present_w10;          presentLen = sizeof(pat_present_w10);
        directFlipPat = pat_directflip_w10;    directFlipLen = sizeof(pat_directflip_w10);
        overlaysPat = pat_overlays_w10;        overlaysLen = sizeof(pat_overlays_w10);
        verLabel = "Win10";
    }

    DiagLogPrintf(f, "Scanning with %s patterns...", verLabel);
    DiagLogBytes(f, "Present pattern: ", presentPat, presentLen);
    DiagLogBytes(f, "DirectFlip pattern: ", directFlipPat, directFlipLen);
    DiagLogBytes(f, "OverlaysEnabled pattern: ", overlaysPat, overlaysLen);

    // Scan for patterns
    bool foundPresent = false, foundDirectFlip = false, foundOverlays = false;
    size_t presentOff = 0, directFlipOff = 0, overlaysOff = 0;

    for (size_t i = 0; i < imageSize; i++)
    {
        if (!foundPresent && i + presentLen <= imageSize &&
            !aob_match_inverse(image.data() + i, presentPat, presentLen))
        {
            foundPresent = true;
            presentOff = i;
            DiagLogPrintf(f, "FOUND Present at offset 0x%zx", i);
            DiagLogBytes(f, "Bytes: ", image.data() + i, 48);
        }
        else if (!foundDirectFlip && i + directFlipLen <= imageSize &&
                 !aob_match_inverse(image.data() + i, directFlipPat, directFlipLen))
        {
            foundDirectFlip = true;
            directFlipOff = i;
            DiagLogPrintf(f, "FOUND DirectFlip at offset 0x%zx", i);
            DiagLogBytes(f, "Bytes: ", image.data() + i, 48);
        }
        else if (!foundOverlays && i + overlaysLen <= imageSize &&
                 !aob_match_inverse(image.data() + i, overlaysPat, overlaysLen))
        {
            foundOverlays = true;
            overlaysOff = i;
            DiagLogPrintf(f, "FOUND OverlaysEnabled at offset 0x%zx", i);
            DiagLogBytes(f, "Bytes: ", image.data() + i, 48);
        }

        if (foundPresent && foundDirectFlip && foundOverlays)
            break;
    }

    if (!foundPresent)   DiagLogPrintf(f, "NOT FOUND: Present pattern");
    if (!foundDirectFlip) DiagLogPrintf(f, "NOT FOUND: DirectFlip pattern");
    if (!foundOverlays)  DiagLogPrintf(f, "NOT FOUND: OverlaysEnabled pattern");

    // If any pattern was not found, run discovery (broad prologue search)
    if (!foundPresent || !foundDirectFlip || !foundOverlays)
    {
        DiagLogPrintf(f, "=== Pattern Discovery ===");

        if (!foundPresent)
        {
            DiagLogPrintf(f, "--- Present candidates (prologue search) ---");
            int count = 0;
            // Type1: 40 53 55 56 57 41 56 41 57 48 81 EC
            const unsigned char p1[] = { 0x40, 0x53, 0x55, 0x56, 0x57, 0x41, 0x56, 0x41, 0x57, 0x48, 0x81, 0xEC };
            for (size_t i = 0; i < imageSize - 48 && count < 5; i++)
            {
                if (memcmp(image.data() + i, p1, sizeof(p1)) == 0)
                {
                    DiagLogPrintf(f, "  [Type1-BigStack] offset 0x%zx", i);
                    DiagLogBytes(f, "", image.data() + i, 48);
                    count++;
                }
            }
            // Type2: 40 53 55 56 57 41 56 41 57 48 83 EC
            const unsigned char p2[] = { 0x40, 0x53, 0x55, 0x56, 0x57, 0x41, 0x56, 0x41, 0x57, 0x48, 0x83, 0xEC };
            for (size_t i = 0; i < imageSize - 48 && count < 10; i++)
            {
                if (memcmp(image.data() + i, p2, sizeof(p2)) == 0)
                {
                    DiagLogPrintf(f, "  [Type2-SmallStack] offset 0x%zx", i);
                    DiagLogBytes(f, "", image.data() + i, 48);
                    count++;
                }
            }
            if (count == 0) DiagLogPrintf(f, "  No Present candidates found");
        }

        if (!foundDirectFlip)
        {
            DiagLogPrintf(f, "--- DirectFlip candidates (prologue search) ---");
            int count = 0;
            const unsigned char p[] = { 0x40, 0x55, 0x53, 0x56, 0x57, 0x41, 0x54, 0x41, 0x55, 0x41, 0x56, 0x41, 0x57, 0x48, 0x8B, 0xEC, 0x48, 0x83, 0xEC };
            for (size_t i = 0; i < imageSize - 48 && count < 10; i++)
            {
                if (memcmp(image.data() + i, p, sizeof(p)) == 0)
                {
                    DiagLogPrintf(f, "  offset 0x%zx (stack_alloc=0x%02x)", i, image[i + sizeof(p)]);
                    DiagLogBytes(f, "", image.data() + i, 48);
                    count++;
                }
            }
            if (count == 0) DiagLogPrintf(f, "  No DirectFlip candidates found");
        }

        if (!foundOverlays)
        {
            DiagLogPrintf(f, "--- OverlaysEnabled candidates ---");
            int count = 0;
            // Type1: cmp dword [rip+XX], imm  /  jz/jnz +4
            for (size_t i = 0; i < imageSize - 16 && count < 10; i++)
            {
                if (image[i] == 0x83 && image[i + 1] == 0x3D &&
                    (image[i + 7] == 0x74 || image[i + 7] == 0x75) &&
                    image[i + 8] == 0x04)
                {
                    DiagLogPrintf(f, "  [cmp+jcc] offset 0x%zx", i);
                    DiagLogBytes(f, "", image.data() + i, 16);
                    count++;
                }
            }
            // Type2: mov rax,[rcx+XX] / test rax,rax / jnz +4 / xor al,al / ret
            for (size_t i = 0; i < imageSize - 16 && count < 15; i++)
            {
                if (image[i] == 0x48 && image[i + 1] == 0x8B && image[i + 2] == 0x41 &&
                    image[i + 4] == 0x48 && image[i + 5] == 0x85 && image[i + 6] == 0xC0 &&
                    image[i + 7] == 0x75 && image[i + 8] == 0x04 &&
                    image[i + 9] == 0x32 && image[i + 10] == 0xC0 && image[i + 11] == 0xC3)
                {
                    DiagLogPrintf(f, "  [mov+test+jnz] offset 0x%zx", i);
                    DiagLogBytes(f, "", image.data() + i, 16);
                    count++;
                }
            }
            if (count == 0) DiagLogPrintf(f, "  No OverlaysEnabled candidates found");
        }
    }

    if (foundPresent && foundDirectFlip && foundOverlays)
        DiagLogPrintf(f, "RESULT: All 3 patterns found — DLL should match. Injection failure is NOT a pattern issue.");
    else
        DiagLogPrintf(f, "RESULT: Pattern scan incomplete — DLL's DllMain would return FALSE.");

    DiagLogPrintf(f, "=== End Remote Diagnostic Scan ===");
    fclose(f);

    wprintf(L"  Diagnostic scan written to: %s\n", logPath.c_str());
}

// ============================================================================
// Forward declarations for functions used by probe mode
// ============================================================================
static bool EnableDebugPrivilege();
static bool ImpersonateSystem(bool verbose);
static bool InjectDll(DWORD pid, const wchar_t* dllPath, const wchar_t* dllName, bool verbose);
static bool IsModuleLoadedInProcess(DWORD pid, const wchar_t* dllName);
static bool UninjectDll(DWORD pid, const wchar_t* dllName, bool verbose);

// ============================================================================
// Probe mode — cross-version pattern discovery and offset caching
// ============================================================================

struct ProbeCandidate {
    size_t offset;
    unsigned char bytes[64]; // first 64 bytes at this offset for scoring
};

// Reusable prologue discovery functions (refactored from RunRemoteDiagnostics)

static std::vector<ProbeCandidate> FindPresentCandidates(
    const unsigned char* image, size_t imageSize, int maxCandidates = 20)
{
    std::vector<ProbeCandidate> results;
    // Type1: BigStack — 40 53 55 56 57 41 56 41 57 48 81 EC
    const unsigned char p1[] = { 0x40, 0x53, 0x55, 0x56, 0x57, 0x41, 0x56, 0x41, 0x57, 0x48, 0x81, 0xEC };
    for (size_t i = 0; i < imageSize - 64 && (int)results.size() < maxCandidates; i++)
    {
        if (memcmp(image + i, p1, sizeof(p1)) == 0)
        {
            ProbeCandidate c;
            c.offset = i;
            memcpy(c.bytes, image + i, 64);
            results.push_back(c);
        }
    }
    // Type2: SmallStack — 40 53 55 56 57 41 56 41 57 48 83 EC
    const unsigned char p2[] = { 0x40, 0x53, 0x55, 0x56, 0x57, 0x41, 0x56, 0x41, 0x57, 0x48, 0x83, 0xEC };
    for (size_t i = 0; i < imageSize - 64 && (int)results.size() < maxCandidates; i++)
    {
        if (memcmp(image + i, p2, sizeof(p2)) == 0)
        {
            ProbeCandidate c;
            c.offset = i;
            memcpy(c.bytes, image + i, 64);
            results.push_back(c);
        }
    }
    return results;
}

static std::vector<ProbeCandidate> FindDirectFlipCandidates(
    const unsigned char* image, size_t imageSize, int maxCandidates = 20)
{
    std::vector<ProbeCandidate> results;
    // 9-register save prologue: 40 55 53 56 57 41 54 41 55 41 56 41 57 48 8B EC 48 83 EC
    const unsigned char p[] = { 0x40, 0x55, 0x53, 0x56, 0x57, 0x41, 0x54, 0x41, 0x55, 0x41, 0x56, 0x41, 0x57, 0x48, 0x8B, 0xEC, 0x48, 0x83, 0xEC };
    for (size_t i = 0; i < imageSize - 64 && (int)results.size() < maxCandidates; i++)
    {
        if (memcmp(image + i, p, sizeof(p)) == 0)
        {
            ProbeCandidate c;
            c.offset = i;
            memcpy(c.bytes, image + i, 64);
            results.push_back(c);
        }
    }
    // Also search for Win10-style DirectFlip: 48 89 7C 24 20 55 41 54 41 55 41 56 41 57 48 8B EC 48 83 EC
    const unsigned char p10[] = { 0x48, 0x89, 0x7C, 0x24, 0x20, 0x55, 0x41, 0x54, 0x41, 0x55, 0x41, 0x56, 0x41, 0x57, 0x48, 0x8B, 0xEC, 0x48, 0x83, 0xEC };
    for (size_t i = 0; i < imageSize - 64 && (int)results.size() < maxCandidates; i++)
    {
        if (memcmp(image + i, p10, sizeof(p10)) == 0)
        {
            ProbeCandidate c;
            c.offset = i;
            memcpy(c.bytes, image + i, 64);
            results.push_back(c);
        }
    }
    return results;
}

static std::vector<ProbeCandidate> FindOverlaysEnabledCandidates(
    const unsigned char* image, size_t imageSize, int maxCandidates = 20)
{
    std::vector<ProbeCandidate> results;
    // Type1: cmp dword [rip+XX], imm / jz|jnz +4
    for (size_t i = 0; i < imageSize - 16 && (int)results.size() < maxCandidates; i++)
    {
        if (image[i] == 0x83 && image[i + 1] == 0x3D &&
            (image[i + 7] == 0x74 || image[i + 7] == 0x75) &&
            image[i + 8] == 0x04)
        {
            ProbeCandidate c;
            c.offset = i;
            size_t copyLen = std::min((size_t)64, imageSize - i);
            memcpy(c.bytes, image + i, copyLen);
            results.push_back(c);
        }
    }
    // Type2: mov rax,[rcx+XX] / test rax,rax / jnz +4 / xor al,al / ret
    for (size_t i = 0; i < imageSize - 16 && (int)results.size() < maxCandidates; i++)
    {
        if (image[i] == 0x48 && image[i + 1] == 0x8B && image[i + 2] == 0x41 &&
            image[i + 4] == 0x48 && image[i + 5] == 0x85 && image[i + 6] == 0xC0 &&
            image[i + 7] == 0x75 && image[i + 8] == 0x04 &&
            image[i + 9] == 0x32 && image[i + 10] == 0xC0 && image[i + 11] == 0xC3)
        {
            ProbeCandidate c;
            c.offset = i;
            size_t copyLen = std::min((size_t)64, imageSize - i);
            memcpy(c.bytes, image + i, copyLen);
            results.push_back(c);
        }
    }
    // Type3: Win10-style — jnz +4 / xor al,al / ret / int3 / cmp [rcx+0x30], 1 / seta al / ret
    for (size_t i = 0; i < imageSize - 16 && (int)results.size() < maxCandidates; i++)
    {
        if (image[i] == 0x75 && image[i + 1] == 0x04 &&
            image[i + 2] == 0x32 && image[i + 3] == 0xC0 && image[i + 4] == 0xC3 &&
            image[i + 5] == 0xCC && image[i + 6] == 0x83 && image[i + 7] == 0x79)
        {
            ProbeCandidate c;
            c.offset = i;
            size_t copyLen = std::min((size_t)64, imageSize - i);
            memcpy(c.bytes, image + i, copyLen);
            results.push_back(c);
        }
    }
    return results;
}

// Compute similarity score: percentage of bytes that match (wildcards count as match)
static double PatternSimilarityScore(const unsigned char* candidate,
                                     const unsigned char* pattern, int patternLen)
{
    int matches = 0;
    for (int i = 0; i < patternLen; i++)
    {
        if (pattern[i] == '?' || candidate[i] == pattern[i])
            matches++;
    }
    return (double)matches / patternLen * 100.0;
}

// Read the dwmcore.dll image from a running dwm.exe process.
// Returns the image bytes and sets dwmcoreSize. Caller provides dwmPid.
static std::vector<unsigned char> ReadDwmcoreImage(DWORD dwmPid, DWORD* dwmcoreSize)
{
    *dwmcoreSize = 0;
    std::vector<unsigned char> image;

    HANDLE hProcess = OpenProcess(PROCESS_QUERY_INFORMATION | PROCESS_VM_READ, FALSE, dwmPid);
    if (!hProcess) return image;

    HMODULE dwmcoreBase = NULL;
    {
        HANDLE snap = CreateToolhelp32Snapshot(TH32CS_SNAPMODULE | TH32CS_SNAPMODULE32, dwmPid);
        if (snap != INVALID_HANDLE_VALUE)
        {
            MODULEENTRY32W me = {};
            me.dwSize = sizeof(me);
            if (Module32FirstW(snap, &me))
            {
                do {
                    if (_wcsicmp(me.szModule, L"dwmcore.dll") == 0)
                    {
                        dwmcoreBase = me.hModule;
                        *dwmcoreSize = me.modBaseSize;
                        break;
                    }
                } while (Module32NextW(snap, &me));
            }
            CloseHandle(snap);
        }
    }

    if (!dwmcoreBase || *dwmcoreSize == 0)
    {
        CloseHandle(hProcess);
        return image;
    }

    image.resize(*dwmcoreSize);
    SIZE_T bytesRead = 0;
    ReadProcessMemory(hProcess, dwmcoreBase, image.data(), *dwmcoreSize, &bytesRead);
    CloseHandle(hProcess);

    if (bytesRead == 0)
    {
        image.clear();
        return image;
    }
    image.resize(bytesRead);
    return image;
}

// Phase A: Try ALL known pattern sets against dwmcore image.
// Returns index into ALL_PATTERNS on success, or -1 on failure.
static int CrossVersionPatternScan(const unsigned char* image, size_t imageSize,
                                   INT64* outPresent, INT64* outDirectFlip, INT64* outOverlays,
                                   bool verbose)
{
    for (int ps = 0; ps < NUM_PATTERN_SETS; ps++)
    {
        const PatternSet& p = ALL_PATTERNS[ps];
        bool foundPresent = false, foundDirectFlip = false, foundOverlays = false;
        INT64 presOff = 0, dfOff = 0, ovOff = 0;

        for (size_t i = 0; i < imageSize; i++)
        {
            if (!foundPresent && i + p.presentLen <= imageSize &&
                !aob_match_inverse(image + i, p.present, p.presentLen))
            {
                foundPresent = true;
                presOff = (INT64)i;
            }
            else if (!foundDirectFlip && i + p.directFlipLen <= imageSize &&
                     !aob_match_inverse(image + i, p.directFlip, p.directFlipLen))
            {
                foundDirectFlip = true;
                dfOff = (INT64)i;
            }
            else if (!foundOverlays && i + p.overlaysLen <= imageSize &&
                     !aob_match_inverse(image + i, p.overlays, p.overlaysLen))
            {
                foundOverlays = true;
                ovOff = (INT64)i;
            }
            if (foundPresent && foundDirectFlip && foundOverlays)
                break;
        }

        if (verbose)
        {
            wprintf(L"    [%S] Present=%s DirectFlip=%s Overlays=%s\n",
                    p.label,
                    foundPresent ? L"YES" : L"no",
                    foundDirectFlip ? L"YES" : L"no",
                    foundOverlays ? L"YES" : L"no");
        }

        if (foundPresent && foundDirectFlip && foundOverlays)
        {
            *outPresent = presOff;
            *outDirectFlip = dfOff;
            *outOverlays = ovOff;
            return ps;
        }
    }
    return -1;
}

// Scored combination for Phase C
struct ScoredCombo {
    size_t presentIdx, directFlipIdx, overlaysIdx;
    int patternSetIdx; // which formal pattern set scored best
    double totalScore; // sum of 3 similarity percentages
};

static void RunProbeMode(bool verbose)
{
    wprintf(L"\n=== Probe Mode: Offset Discovery ===\n\n");

    // Detect version
    WindowsVersion wv = GetTrueWindowsVersion();
    const wchar_t* verName = L"Windows 10";
    if (wv.build >= 22000) verName = L"Windows 11";
    if (wv.build >= 26100) verName = L"Windows 11 24H2+";
    if (wv.build >= 26200) verName = L"Windows 11 25H2+";
    wprintf(L"  Windows: %s (build %lu)\n", verName, wv.build);

    // Get dwmcore.dll file version
    UINT32 verMS = 0, verLS = 0, fileSz = 0;
    if (GetDwmcoreFileVersion(&verMS, &verLS, &fileSz))
    {
        wprintf(L"  dwmcore.dll: %u.%u.%u.%u  size: %u\n",
                (verMS >> 16), (verMS & 0xFFFF), (verLS >> 16), (verLS & 0xFFFF), fileSz);
    }

    // Enable privileges for reading dwm.exe
    EnableDebugPrivilege();
    ImpersonateSystem(false);

    // Find dwm.exe
    DWORD currentSession = 0;
    ProcessIdToSessionId(GetCurrentProcessId(), &currentSession);
    DWORD dwmPid = 0;
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
                        if (sid == currentSession) { dwmPid = pe.th32ProcessID; break; }
                    }
                } while (Process32NextW(snap, &pe));
            }
            CloseHandle(snap);
        }
    }

    if (dwmPid == 0)
    {
        wprintf(L"  ERROR: dwm.exe not found in session %lu.\n", currentSession);
        RevertToSelf();
        return;
    }
    wprintf(L"  dwm.exe PID: %lu\n\n", dwmPid);

    // Read dwmcore.dll image
    DWORD dwmcoreSize = 0;
    auto image = ReadDwmcoreImage(dwmPid, &dwmcoreSize);
    if (image.empty())
    {
        wprintf(L"  ERROR: Could not read dwmcore.dll from dwm.exe.\n");
        RevertToSelf();
        return;
    }
    wprintf(L"  Read %zu bytes from dwmcore.dll\n\n", image.size());

    // ===== Phase A: Cross-version formal pattern scan =====
    wprintf(L"  Phase A: Cross-version formal pattern scan...\n");
    INT64 presOff = 0, dfOff = 0, ovOff = 0;
    int matchedSet = CrossVersionPatternScan(image.data(), image.size(),
                                             &presOff, &dfOff, &ovOff, verbose);

    const char* method = nullptr;
    int usedPatternSet = -1;

    if (matchedSet >= 0)
    {
        method = "Cross-version AOB match";
        usedPatternSet = matchedSet;
        wprintf(L"\n  SUCCESS: Matched %S patterns!\n", ALL_PATTERNS[matchedSet].label);
        wprintf(L"    Present:         offset 0x%llx\n", (UINT64)presOff);
        wprintf(L"    DirectFlip:      offset 0x%llx\n", (UINT64)dfOff);
        wprintf(L"    OverlaysEnabled: offset 0x%llx\n", (UINT64)ovOff);
    }
    else
    {
        // ===== Phase B: Broad prologue discovery =====
        wprintf(L"\n  Phase B: Broad prologue discovery...\n");
        auto presentCands = FindPresentCandidates(image.data(), image.size());
        auto dfCands = FindDirectFlipCandidates(image.data(), image.size());
        auto ovCands = FindOverlaysEnabledCandidates(image.data(), image.size());

        wprintf(L"    Present candidates:    %zu\n", presentCands.size());
        wprintf(L"    DirectFlip candidates: %zu\n", dfCands.size());
        wprintf(L"    Overlays candidates:   %zu\n", ovCands.size());

        if (verbose)
        {
            for (size_t i = 0; i < presentCands.size(); i++)
                wprintf(L"      Present[%zu]: 0x%zx\n", i, presentCands[i].offset);
            for (size_t i = 0; i < dfCands.size(); i++)
                wprintf(L"      DirectFlip[%zu]: 0x%zx\n", i, dfCands[i].offset);
            for (size_t i = 0; i < ovCands.size(); i++)
                wprintf(L"      Overlays[%zu]: 0x%zx\n", i, ovCands[i].offset);
        }

        if (presentCands.empty() || dfCands.empty() || ovCands.empty())
        {
            wprintf(L"\n  ERROR: Not enough candidates found for discovery.\n");
            wprintf(L"  This dwmcore.dll may have a significantly different structure.\n");
            RevertToSelf();
            return;
        }

        // ===== Phase C: Score candidates against all formal patterns =====
        wprintf(L"\n  Phase C: Scoring candidates against known patterns...\n");
        std::vector<ScoredCombo> combos;

        for (size_t pi = 0; pi < presentCands.size(); pi++)
        {
            for (size_t di = 0; di < dfCands.size(); di++)
            {
                for (size_t oi = 0; oi < ovCands.size(); oi++)
                {
                    // Score against each pattern set
                    double bestScore = 0;
                    int bestPS = 0;
                    for (int ps = 0; ps < NUM_PATTERN_SETS; ps++)
                    {
                        const PatternSet& p = ALL_PATTERNS[ps];
                        double s1 = PatternSimilarityScore(presentCands[pi].bytes, p.present, p.presentLen);
                        double s2 = PatternSimilarityScore(dfCands[di].bytes, p.directFlip, p.directFlipLen);
                        double s3 = PatternSimilarityScore(ovCands[oi].bytes, p.overlays, p.overlaysLen);
                        double total = s1 + s2 + s3;
                        if (total > bestScore)
                        {
                            bestScore = total;
                            bestPS = ps;
                        }
                    }
                    ScoredCombo sc;
                    sc.presentIdx = pi;
                    sc.directFlipIdx = di;
                    sc.overlaysIdx = oi;
                    sc.patternSetIdx = bestPS;
                    sc.totalScore = bestScore;
                    combos.push_back(sc);
                }
            }
        }

        // Sort by score descending
        std::sort(combos.begin(), combos.end(),
                  [](const ScoredCombo& a, const ScoredCombo& b) { return a.totalScore > b.totalScore; });

        // Limit to top 10
        if (combos.size() > 10) combos.resize(10);

        wprintf(L"    Top combinations:\n");
        for (size_t i = 0; i < combos.size(); i++)
        {
            const auto& c = combos[i];
            wprintf(L"      #%zu: score=%.1f%% Present=0x%zx DirectFlip=0x%zx Overlays=0x%zx [%S]\n",
                    i + 1, c.totalScore / 3.0,
                    presentCands[c.presentIdx].offset,
                    dfCands[c.directFlipIdx].offset,
                    ovCands[c.overlaysIdx].offset,
                    ALL_PATTERNS[c.patternSetIdx].label);
        }

        if (combos.empty() || combos[0].totalScore < 150.0) // < 50% average
        {
            wprintf(L"\n  WARNING: Best score is below 50%% — results may be unreliable.\n");
        }

        // ===== Phase D: Live testing =====
        if (combos.size() > 1 && combos[0].totalScore - combos[1].totalScore < 10.0)
        {
            wprintf(L"\n  Phase D: Live testing (multiple high-scoring combinations)...\n");
            wprintf(L"  Testing top combinations by injecting into DWM...\n\n");

            std::wstring tempDir = GetSystemTempPath();
            std::wstring cfgPath = tempDir + DITHER_CFG_FILE;
            std::wstring dllDest = tempDir + DITHER_DLL_NAME;

            // Get source DLL path
            wchar_t exePath[MAX_PATH];
            GetModuleFileNameW(NULL, exePath, MAX_PATH);
            std::wstring exeDir(exePath);
            exeDir = exeDir.substr(0, exeDir.find_last_of(L'\\') + 1);
            std::wstring dllSource = exeDir + DITHER_DLL_NAME;

            // Ensure DLL is deployed
            CopyFileW(dllSource.c_str(), dllDest.c_str(), FALSE);

            for (size_t ci = 0; ci < combos.size() && ci < 10; ci++)
            {
                const auto& c = combos[ci];
                const PatternSet& ps = ALL_PATTERNS[c.patternSetIdx];
                size_t pOff = presentCands[c.presentIdx].offset;
                size_t dOff = dfCands[c.directFlipIdx].offset;
                size_t oOff = ovCands[c.overlaysIdx].offset;

                wprintf(L"  Testing combo #%zu (score=%.1f%%): P=0x%zx D=0x%zx O=0x%zx\n",
                        ci + 1, c.totalScore / 3.0, pOff, dOff, oOff);

                // Record DWM PID before injection
                DWORD prePid = 0;
                {
                    HANDLE snap2 = CreateToolhelp32Snapshot(TH32CS_SNAPPROCESS, 0);
                    if (snap2 != INVALID_HANDLE_VALUE)
                    {
                        PROCESSENTRY32W pe2 = {};
                        pe2.dwSize = sizeof(pe2);
                        if (Process32FirstW(snap2, &pe2))
                        {
                            do {
                                if (_wcsicmp(pe2.szExeFile, L"dwm.exe") == 0)
                                {
                                    DWORD sid2 = 0;
                                    ProcessIdToSessionId(pe2.th32ProcessID, &sid2);
                                    if (sid2 == currentSession) { prePid = pe2.th32ProcessID; break; }
                                }
                            } while (Process32NextW(snap2, &pe2));
                        }
                        CloseHandle(snap2);
                    }
                }

                // Write config
                DitherConfig cfg = {};
                cfg.magic = DITHER_CONFIG_MAGIC;
                cfg.version = DITHER_CONFIG_VERSION;
                cfg.presentOffset = (INT64)pOff;
                cfg.directFlipOffset = (INT64)dOff;
                cfg.overlaysOffset = (INT64)oOff;
                cfg.hwProtOffset = ps.hwProtOffset;
                cfg.swapChainOffset = ps.swapChainOffset;
                cfg.isWindows11 = ps.isWin11 ? 1 : 0;
                cfg.ditherBits = 0;

                HANDLE hCfg = CreateFileW(cfgPath.c_str(), GENERIC_WRITE, 0, NULL,
                                          CREATE_ALWAYS, FILE_ATTRIBUTE_NORMAL, NULL);
                if (hCfg != INVALID_HANDLE_VALUE)
                {
                    DWORD written;
                    WriteFile(hCfg, &cfg, sizeof(cfg), &written, NULL);
                    CloseHandle(hCfg);
                }

                // Inject
                bool injected = InjectDll(prePid, dllDest.c_str(), DITHER_DLL_NAME, false);
                if (!injected)
                {
                    wprintf(L"    Injection failed — skipping.\n");
                    continue;
                }

                // Wait and check stability
                Sleep(5000);

                // Check if DWM survived (same PID still running)
                DWORD postPid = 0;
                {
                    HANDLE snap3 = CreateToolhelp32Snapshot(TH32CS_SNAPPROCESS, 0);
                    if (snap3 != INVALID_HANDLE_VALUE)
                    {
                        PROCESSENTRY32W pe3 = {};
                        pe3.dwSize = sizeof(pe3);
                        if (Process32FirstW(snap3, &pe3))
                        {
                            do {
                                if (_wcsicmp(pe3.szExeFile, L"dwm.exe") == 0)
                                {
                                    DWORD sid3 = 0;
                                    ProcessIdToSessionId(pe3.th32ProcessID, &sid3);
                                    if (sid3 == currentSession) { postPid = pe3.th32ProcessID; break; }
                                }
                            } while (Process32NextW(snap3, &pe3));
                        }
                        CloseHandle(snap3);
                    }
                }

                if (postPid != prePid)
                {
                    wprintf(L"    DWM crashed and restarted (PID %lu -> %lu) — bad combo.\n", prePid, postPid);
                    // Update dwmPid for subsequent tests
                    Sleep(3000); // Wait for DWM to stabilize
                    continue;
                }

                // DWM survived — check if DLL is loaded
                bool dllLoaded = IsModuleLoadedInProcess(postPid, DITHER_DLL_NAME);
                if (!dllLoaded)
                {
                    wprintf(L"    DLL not loaded (DllMain returned FALSE) — pattern mismatch.\n");
                    continue;
                }

                // Ask user
                wprintf(L"    DWM stable, DLL loaded. Is display rendering normally? [Y/n] ");
                fflush(stdout);
                wchar_t response[16] = {};
                if (fgetws(response, 16, stdin) && (response[0] == L'n' || response[0] == L'N'))
                {
                    wprintf(L"    User rejected — uninjecting and trying next.\n");
                    UninjectDll(postPid, DITHER_DLL_NAME, false);
                    Sleep(2000);
                    continue;
                }

                // User accepted!
                wprintf(L"    Accepted! Using this combination.\n");
                UninjectDll(postPid, DITHER_DLL_NAME, false);
                presOff = (INT64)pOff;
                dfOff = (INT64)dOff;
                ovOff = (INT64)oOff;
                usedPatternSet = c.patternSetIdx;
                method = "Live-tested probe";
                goto probe_done;
            }
        }

        // Use the top-scoring combo if we didn't do live testing or live testing failed
        if (method == nullptr && !combos.empty())
        {
            const auto& best = combos[0];
            presOff = (INT64)presentCands[best.presentIdx].offset;
            dfOff = (INT64)dfCands[best.directFlipIdx].offset;
            ovOff = (INT64)ovCands[best.overlaysIdx].offset;
            usedPatternSet = best.patternSetIdx;
            method = "Probe best-score";
            wprintf(L"\n  Using top-scoring combination (%.1f%% confidence).\n",
                    best.totalScore / 3.0);
        }
    }

probe_done:
    if (method == nullptr)
    {
        wprintf(L"\n  FAILED: Could not determine offsets.\n");
        RevertToSelf();
        return;
    }

    // ===== Phase E: Save results =====
    wprintf(L"\n  === Probe Results ===\n");
    wprintf(L"  Windows: %s (build %lu)\n", verName, wv.build);
    if (GetDwmcoreFileVersion(&verMS, &verLS, &fileSz))
    {
        wprintf(L"  dwmcore.dll: %u.%u.%u.%u  size: %u\n",
                (verMS >> 16), (verMS & 0xFFFF), (verLS >> 16), (verLS & 0xFFFF), fileSz);
    }
    wprintf(L"\n  Method: %S", method);
    if (usedPatternSet >= 0)
        wprintf(L" (%S patterns)", ALL_PATTERNS[usedPatternSet].label);
    wprintf(L"\n\n");

    wprintf(L"  Present:         offset 0x%llx\n", (UINT64)presOff);
    wprintf(L"  DirectFlip:      offset 0x%llx\n", (UINT64)dfOff);
    wprintf(L"  OverlaysEnabled: offset 0x%llx\n\n", (UINT64)ovOff);

    // Print bytes at discovered offsets
    if (presOff < (INT64)image.size())
    {
        wprintf(L"  Present bytes:   ");
        size_t n = std::min((size_t)28, image.size() - (size_t)presOff);
        for (size_t i = 0; i < n; i++) wprintf(L"%02X ", image[(size_t)presOff + i]);
        wprintf(L"\n");
    }
    if (dfOff < (INT64)image.size())
    {
        wprintf(L"  DirectFlip bytes:");
        size_t n = std::min((size_t)21, image.size() - (size_t)dfOff);
        for (size_t i = 0; i < n; i++) wprintf(L" %02X", image[(size_t)dfOff + i]);
        wprintf(L"\n");
    }
    if (ovOff < (INT64)image.size())
    {
        wprintf(L"  Overlays bytes:  ");
        size_t n = std::min((size_t)14, image.size() - (size_t)ovOff);
        for (size_t i = 0; i < n; i++) wprintf(L"%02X ", image[(size_t)ovOff + i]);
        wprintf(L"\n");
    }

    // Suggested C++ patterns
    wprintf(L"\n  Suggested C++ patterns:\n");
    auto printCPattern = [&](const wchar_t* name, INT64 off, int len) {
        wprintf(L"    static const unsigned char %s[] = {\n        ", name);
        size_t start = (size_t)off;
        for (int i = 0; i < len && start + i < image.size(); i++)
        {
            if (i > 0 && i % 13 == 0) wprintf(L"\n        ");
            wprintf(L"0x%02X, ", image[start + i]);
        }
        wprintf(L"\n    };\n");
    };
    printCPattern(L"pat_present_XXXXX", presOff, 28);
    printCPattern(L"pat_directflip_XXXXX", dfOff, 21);
    printCPattern(L"pat_overlays_XXXXX", ovOff, 14);

    // Save to cache
    OffsetCacheEntry cache = {};
    cache.magic = OFFSET_CACHE_MAGIC;
    cache.version = OFFSET_CACHE_VERSION;
    cache.dwmcoreVersionMS = verMS;
    cache.dwmcoreVersionLS = verLS;
    cache.dwmcoreSizeBytes = fileSz;
    cache.osBuild = wv.build;
    cache.presentOffset = presOff;
    cache.directFlipOffset = dfOff;
    cache.overlaysOffset = ovOff;
    if (usedPatternSet >= 0)
    {
        cache.hwProtOffset = ALL_PATTERNS[usedPatternSet].hwProtOffset;
        cache.swapChainOffset = ALL_PATTERNS[usedPatternSet].swapChainOffset;
        cache.isWindows11 = ALL_PATTERNS[usedPatternSet].isWin11 ? 1 : 0;
    }
    else
    {
        // Fallback: use version-based defaults
        cache.isWindows11 = (wv.build >= 22000) ? 1 : 0;
        if (wv.build >= 26100) {
            cache.hwProtOffset = OVERLAY_HWPROT_OFFSET_W11_24H2;
            cache.swapChainOffset = OVERLAY_SWAPCHAIN_OFFSET_W11_24H2;
        } else if (wv.build >= 22000) {
            cache.hwProtOffset = OVERLAY_HWPROT_OFFSET_W11;
            cache.swapChainOffset = OVERLAY_SWAPCHAIN_OFFSET_W11;
        } else {
            cache.hwProtOffset = OVERLAY_HWPROT_OFFSET_W10;
            cache.swapChainOffset = OVERLAY_SWAPCHAIN_OFFSET_W10;
        }
    }
    cache.sourceFlags = (matchedSet >= 0) ? 1 : 2; // 1=formal AOB, 2=probe

    if (SaveOffsetCache(&cache))
        wprintf(L"\n  Saved to cache. Use --dither to apply immediately.\n");
    else
        wprintf(L"\n  WARNING: Could not save cache file.\n");

    // Write human-readable report
    std::wstring reportPath = GetSystemTempPath() + L"ApplyIccLut_probe_report.txt";
    FILE* reportFile = NULL;
    if (_wfopen_s(&reportFile, reportPath.c_str(), L"w") == 0 && reportFile)
    {
        fprintf(reportFile, "=== Probe Results ===\n");
        fprintf(reportFile, "Windows: build %lu\n", wv.build);
        fprintf(reportFile, "dwmcore.dll: %u.%u.%u.%u  size: %u\n",
                (verMS >> 16), (verMS & 0xFFFF), (verLS >> 16), (verLS & 0xFFFF), fileSz);
        fprintf(reportFile, "Method: %s", method);
        if (usedPatternSet >= 0)
            fprintf(reportFile, " (%s patterns)", ALL_PATTERNS[usedPatternSet].label);
        fprintf(reportFile, "\n\n");
        fprintf(reportFile, "Present:         offset 0x%llx\n", (UINT64)presOff);
        fprintf(reportFile, "DirectFlip:      offset 0x%llx\n", (UINT64)dfOff);
        fprintf(reportFile, "OverlaysEnabled: offset 0x%llx\n", (UINT64)ovOff);

        fprintf(reportFile, "\nPresent bytes:   ");
        size_t n = std::min((size_t)28, image.size() - (size_t)presOff);
        for (size_t i = 0; i < n; i++) fprintf(reportFile, "%02X ", image[(size_t)presOff + i]);
        fprintf(reportFile, "\nDirectFlip bytes:");
        n = std::min((size_t)21, image.size() - (size_t)dfOff);
        for (size_t i = 0; i < n; i++) fprintf(reportFile, " %02X", image[(size_t)dfOff + i]);
        fprintf(reportFile, "\nOverlays bytes:  ");
        n = std::min((size_t)14, image.size() - (size_t)ovOff);
        for (size_t i = 0; i < n; i++) fprintf(reportFile, "%02X ", image[(size_t)ovOff + i]);
        fprintf(reportFile, "\n");

        fclose(reportFile);
        wprintf(L"  Report written to: %s\n", reportPath.c_str());
    }

    RevertToSelf();
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
                    cached.sourceFlags == 1 ? L"formal AOB" : L"probe-discovered");
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
            wprintf(L"  ERROR: Pattern scan failed — cannot inject dithering.\n");
            wprintf(L"  TIP: Run 'ApplyIccLut.exe --probe' to discover offsets for this dwmcore.dll version.\n");
            // Run extended diagnostics and write to log file
            wprintf(L"  Running diagnostic discovery for new patterns...\n");
            RunRemoteDiagnostics(dwmPids[0], diagIsWin11, diagIs24h2, diagIs25h2);
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
            newCache.sourceFlags = 1; // formal AOB
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
            wprintf(L"  -d          Inject blue-noise dithering DLL into DWM\n");
            wprintf(L"  --dither-bits <N>  Override dither bit depth (default: auto 8/10)\n");
            wprintf(L"              Use 8 for monitors with a bad 8-bit scaler on HDR\n");
            wprintf(L"  --no-dither Remove dithering DLL from DWM\n");
            wprintf(L"  --probe     Discover dithering offsets for this dwmcore.dll version\n");
            wprintf(L"              Tries all known patterns, then broad prologue search\n");
            wprintf(L"              Results cached for use by --dither. Use -v for detail.\n");
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
            wprintf(L"Blue-noise dithering (-d / --no-dither):\n");
            wprintf(L"  Injects a DLL into DWM that applies spatial blue-noise dithering\n");
            wprintf(L"  as a GPU post-process. Reduces banding in gradients caused by\n");
            wprintf(L"  quantization to 8-bit (SDR) or 10-bit (HDR). The DLL stays\n");
            wprintf(L"  resident in DWM after injection; ApplyIccLut.exe can exit.\n");
            wprintf(L"  Use --no-dither to remove. Automatically re-injected at boot\n");
            wprintf(L"  when combined with the default GPU wake-up (Task Scheduler).\n");
            wprintf(L"  --dither-bits overrides the auto bit depth; use 8 if your HDR\n");
            wprintf(L"  monitor has a bad internal scaler that truncates to 8-bit.\n");
            wprintf(L"  HDR dithering is PQ-aware (ST 2084) for perceptual accuracy.\n\n");
            wprintf(L"With no arguments, performs a GPU pipeline wake-up kick\n");
            wprintf(L"on all monitors (equivalent to -s -m 0), and re-injects\n");
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
        wprintf(L"Dithering: injecting blue-noise dithering DLL into DWM...\n");
        if (!DeployAndInjectDither(verbose, ditherBits))
            return 1;
        wprintf(L"\n");
    }

    if (noDitherMode)
    {
        wprintf(L"Dithering: removing blue-noise dithering DLL from DWM...\n");
        if (!UninjectAndRemoveDither(verbose))
            return 1;
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

        // Auto-re-inject dithering if flag file exists and not already injected
        if (IsDitherFlagSet() && !IsDitherAlreadyInjected())
        {
            // Read stored bit depth from flag file (if any)
            int storedBits = 0;
            {
                std::wstring flagPath = GetSystemTempPath() + DITHER_FLAG_FILE;
                HANDLE hFlag = CreateFileW(flagPath.c_str(), GENERIC_READ, FILE_SHARE_READ, NULL, OPEN_EXISTING, 0, NULL);
                if (hFlag != INVALID_HANDLE_VALUE)
                {
                    char buf[32] = {};
                    DWORD bytesRead = 0;
                    ReadFile(hFlag, buf, sizeof(buf) - 1, &bytesRead, NULL);
                    CloseHandle(hFlag);
                    int val = 0;
                    for (DWORD j = 0; j < bytesRead && buf[j] >= '0' && buf[j] <= '9'; j++)
                        val = val * 10 + (buf[j] - '0');
                    if (val > 0 && val <= 16)
                        storedBits = val;
                }
            }
            wprintf(L"Dithering: re-injecting (flag file present from previous --dither)...\n");
            DeployAndInjectDither(verbose, storedBits);
            wprintf(L"\n");
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
