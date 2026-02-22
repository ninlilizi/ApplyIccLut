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

#include <windows.h>
#include <icm.h>
#include <cstdio>
#include <cstdint>
#include <cmath>
#include <cstring>
#include <string>
#include <vector>
#include <algorithm>

#pragma comment(lib, "mscms.lib")

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

struct GpuColorProfileApi
{
    PFN_ColorProfileGetDisplayDefault      GetDefault;
    PFN_ColorProfileRemoveDisplayAssociation Remove;
    PFN_ColorProfileAddDisplayAssociation  Add;
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

    BOOL advColor = hdr ? TRUE : FALSE;

    // Remove first to ensure clean state
    api.Remove(
        WCS_PROFILE_MANAGEMENT_SCOPE_CURRENT_USER,
        profileName.c_str(),
        mon.adapterId, mon.sourceId,
        advColor);

    // Add with setAsDefault = true
    LONG err = api.Add(
        WCS_PROFILE_MANAGEMENT_SCOPE_CURRENT_USER,
        profileName.c_str(),
        mon.adapterId, mon.sourceId,
        TRUE,       // setAsDefault = true
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

        bool anyOk = false;

        if (!setGpuProfile.empty())
        {
            // Explicit name: set as default for the selected pipeline(s)
            for (bool hdr : hdrModes)
            {
                BOOL advColor = hdr ? TRUE : FALSE;

                gpuApi.Remove(
                    WCS_PROFILE_MANAGEMENT_SCOPE_CURRENT_USER,
                    setGpuProfile.c_str(),
                    mon.adapterId, mon.sourceId,
                    advColor);

                LONG err = gpuApi.Add(
                    WCS_PROFILE_MANAGEMENT_SCOPE_CURRENT_USER,
                    setGpuProfile.c_str(),
                    mon.adapterId, mon.sourceId,
                    TRUE,       // setAsDefault = true
                    advColor);

                if (err == ERROR_SUCCESS)
                {
                    wprintf(L"  GPU %s profile set to: %s\n",
                            hdr ? L"HDR" : L"SDR", setGpuProfile.c_str());
                    anyOk = true;
                }
                else
                {
                    wprintf(L"  ERROR: Failed to set GPU %s profile (err=%ld)\n",
                            hdr ? L"HDR" : L"SDR", err);
                    if (err == ERROR_FILE_NOT_FOUND || err == 2)
                        wprintf(L"  HINT: Is the profile installed? Check spool\\drivers\\color\n");
                }
            }
        }
        else
        {
            // No name: re-apply current defaults for selected pipeline(s) (wake-up kick)
            for (bool hdr : hdrModes)
            {
                std::wstring prof = GetGpuDefaultProfile(gpuApi, mon, hdr, verbose);
                if (prof.empty())
                {
                    if (verbose)
                        wprintf(L"  No GPU %s profile set.\n", hdr ? L"HDR" : L"SDR");
                    continue;
                }

                BOOL advColor = hdr ? TRUE : FALSE;

                gpuApi.Remove(
                    WCS_PROFILE_MANAGEMENT_SCOPE_CURRENT_USER,
                    prof.c_str(),
                    mon.adapterId, mon.sourceId,
                    advColor);

                LONG err = gpuApi.Add(
                    WCS_PROFILE_MANAGEMENT_SCOPE_CURRENT_USER,
                    prof.c_str(),
                    mon.adapterId, mon.sourceId,
                    TRUE,       // setAsDefault = true
                    advColor);

                if (err == ERROR_SUCCESS)
                {
                    wprintf(L"  GPU %s profile re-applied: %s\n",
                            hdr ? L"HDR" : L"SDR", prof.c_str());
                    anyOk = true;
                }
                else
                    wprintf(L"  WARNING: Failed to re-apply GPU %s profile (err=%ld)\n",
                            hdr ? L"HDR" : L"SDR", err);
            }

            if (!anyOk)
            {
                wprintf(L"  ERROR: No GPU profiles found to re-apply.\n");
                wprintf(L"  Use -s <name> to specify a profile.\n");
            }
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
            wprintf(L"With no arguments, performs a GPU pipeline wake-up kick\n");
            wprintf(L"on all monitors (equivalent to -s -m 0).\n\n");
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

    // Default behaviour: no mode selected → GPU pipeline wake-up on all monitors
    if (!setGpuMode && !resetMode && !restoreMode && manualProfile.empty())
    {
        setGpuMode = true;  // bare -s behaviour (wake-up kick)
        if (!monitorExplicit)
            targetMonitor = 0; // all monitors
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
