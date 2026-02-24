# ApplyIccLut

A command-line tool for applying display calibration LUTs on Windows 11. Works around the Windows 11 bug where ICC profile LUTs (especially HDR) are not applied at boot.  

This tool specifically supports Windows 11 24H2. Existing similar tools stopped working on 24H2 and havn't been updated.

## The Problem

Windows 11 frequently fails to load the LUT (Look-Up Table) portion of ICC display profiles at startup. This means your carefully calibrated monitor reverts to uncalibrated output every time you log on. The issue is particularly prevalent with HDR color pipelines, where the GPU-level MHC2 correction profile silently stops being applied after a reboot or wake from sleep.

## How It Works

ApplyIccLut provides two methods for applying display calibration, plus an optional blue-noise dithering post-process:

### 1. SetDeviceGammaRamp (Legacy GDI Path)

Reads the LUT from an ICC profile (VCGT or MHC2 tag) or a `.cube` file and loads it into the display adapter via the Win32 `SetDeviceGammaRamp` API. This path is limited to **256 entries per channel** (16-bit values) by the Windows API.

Before applying, the tool performs a **pre-flight check**: it reads the current gamma ramp to detect whether a LUT is already active. If the ramp is non-identity, it warns that applying again would double the correction and skips unless `--force` is used. If the desired ramp is already loaded, it exits with no changes.

### 2. GPU Color Pipeline (ColorProfile APIs)

Sets an installed ICC profile as the GPU default via `ColorProfileAddDisplayAssociation` / `ColorProfileRemoveDisplayAssociation` (Windows 10 1903+). The GPU driver applies the full **MHC2 pipeline** (3x4 color matrix + 1D LUT) natively at up to **4096 entries per channel** with S15Fixed16 precision -- no downsampling.

This is the same mechanism used by Windows Color Management in Settings. The `-s` flag triggers a remove-then-add cycle that forces the driver to re-read and apply the profile, effectively "waking up" a pipeline that failed to activate at boot.

### 3. Blue-Noise Dithering (DWM Injection)

Injects a lightweight DLL into the Desktop Window Manager (`dwm.exe`) that applies spatial blue-noise dithering as a GPU pixel shader post-process. This reduces visible banding artifacts in gradients caused by quantization when the output bit depth is limited:

- **SDR**: Dithers to 8-bit (255 levels) using gamma 2.2 perceptual threshold
- **HDR**: Dithers to 10-bit (1023 levels) by default, PQ-aware (ST 2084) — quantizes in PQ space where the actual bit truncation occurs, with BT.709↔BT.2020 color space conversion for scRGB input
- **Override**: Use `--dither-bits 8` to force 8-bit dithering on HDR, useful for monitors with a bad internal 8-bit scaler that truncates the 10-bit signal

The dithering DLL hooks `COverlayContext::Present` in `dwmcore.dll` using MinHook and applies ordered dithering with a 64x64 blue-noise texture. The DLL stays resident in DWM after injection, so `ApplyIccLut.exe` does not need to remain running. DirectFlip and MPO (Multi-Plane Overlay) optimizations are disabled while dithering is active to ensure the shader runs on every frame.

This feature is adapted from [dwm_lut](https://github.com/lauralex/dwm_lut) with all 3D LUT code removed.

## Supported Formats

| Format | Extensions | LUT Source |
|---|---|---|
| ICC Profile | `.icc`, `.icm` | VCGT tag (Video Card Gamma Table) or MHC2 RegammaLUT |
| Cube LUT | `.cube` | 1D LUT data only (`LUT_1D_SIZE`, up to 65536 entries) |

- **ICC profiles**: The tool first looks for an MHC2 tag (preferred, higher precision). If not found, it falls back to VCGT. Both parametric (formula-based) and table-based VCGT formats are supported.
- **Cube files**: The `.cube` format (IRIDAS/Adobe) is primarily a 3D LUT format, but also supports 1D LUT data via `LUT_1D_SIZE`. This tool reads only the 1D LUT data; files containing only a 3D LUT (`LUT_3D_SIZE`) are rejected, since `SetDeviceGammaRamp` can only apply per-channel 1D curves. Entries are downsampled to 256 via linear interpolation. Supports `DOMAIN_MIN`, `DOMAIN_MAX`, `TITLE`, and comment lines.

## Usage

```
ApplyIccLut.exe [options]
```

With no arguments, performs a **GPU pipeline wake-up kick on all monitors** — re-applying both SDR and HDR default profiles via the full-precision ColorProfile API (equivalent to `-s -m 0`). If dithering was previously enabled with `--dither`, it is automatically re-injected.

### Options

| Flag | Description |
|---|---|
| `-m <num>` | Target monitor by number (default: 1). Use `-m 0` to apply to all monitors. |
| `-p <path>` | Apply a specific ICC or `.cube` LUT via `SetDeviceGammaRamp` (256 entries). |
| `-s [name]` | GPU pipeline mode. With a profile name: sets it as the GPU default. Without a name: re-applies the current default (wake-up kick). |
| `-f` | Force apply even if a non-identity gamma ramp is already active. |
| `-r` | Reset: load an identity (linear) gamma ramp and unset the GPU color profile. |
| `-R` | Restore: re-enable the GPU color profile as default. |
| `--sdr` | Target SDR pipeline only (applies to `-s`, `-r`, `-R`). |
| `--hdr` | Target HDR pipeline only (applies to `-s`, `-r`, `-R`). |
| `-d` / `--dither` | Inject blue-noise dithering DLL into DWM. |
| `--dither-bits <N>` | Override dither bit depth (default: auto 8 for SDR, 10 for HDR). Use 8 for HDR monitors with a bad 8-bit internal scaler. |
| `--no-dither` | Remove dithering DLL from DWM. |
| `-v` | Verbose output (ramp samples, API details). |
| `-h` | Show help. |

When neither `--sdr` nor `--hdr` is specified, GPU operations target **both** pipelines.

### Examples

```bash
# Wake up GPU pipelines on all monitors (default behaviour)
ApplyIccLut.exe

# Apply a specific .cube LUT to monitor 2
ApplyIccLut.exe -p "D:\Calibration\MyDisplay.cube" -m 2

# Wake up the HDR GPU pipeline (re-apply current default profile)
ApplyIccLut.exe -s -m 1 --hdr

# Set a specific profile as GPU default for both SDR and HDR
ApplyIccLut.exe -s MyProfile_HDR_Correction.icm -m 1

# Reset everything (identity ramp + unset GPU profiles) on all monitors
ApplyIccLut.exe -r -m 0

# Restore GPU color profiles after a reset (SDR only)
ApplyIccLut.exe -R --sdr

# Force-apply even if a LUT is already loaded
ApplyIccLut.exe -f

# Enable blue-noise dithering (injected into DWM)
ApplyIccLut.exe --dither

# Disable dithering
ApplyIccLut.exe --no-dither

# Enable dithering alongside GPU pipeline wake-up
ApplyIccLut.exe --dither -v

# Enable dithering forced to 8-bit (for HDR monitors with bad 8-bit scaler)
ApplyIccLut.exe --dither --dither-bits 8
```

## Administrator Privileges

The tool requires **administrator privileges** to enable the Windows calibration management subsystem (`WcsSetCalibrationManagementState`), which is typically OFF after a fresh boot. The executable embeds a `requireAdministrator` manifest, so a UAC prompt will appear automatically when launched.

## Persistence Across Reboots

When the tool sets or re-applies a GPU color profile, it also enables the Windows automatic colour management machinery. This means the applied profiles should **survive subsequent reboots** without needing to run the tool again. You generally only need to run it once after initial setup or after changing profiles.

If you find that profiles are still not being applied after a reboot, you can use Task Scheduler as a fallback to run the tool automatically at logon.

## Task Scheduler (Fallback)

If profiles are not persisting across reboots, you can create a Task Scheduler task as a fallback:

1. Open **Task Scheduler** and select **Create Task** (not "Create Basic Task").
2. **General** tab:
   - Check **"Run with highest privileges"** (required for calibration management APIs).
   - Ensure **"Run only when user is logged on"** is selected (the tool needs the user's display session).
3. **Trigger** tab: Add a trigger for **"At log on"** (for your user account).
   - Optionally add a delay (e.g. 10 seconds) to allow the display driver to initialize.
4. **Action** tab: Add **"Start a program"**.
   - **Program**: Full path to `ApplyIccLut.exe`
   - **Arguments**: No arguments needed (defaults to all monitors). Or use `-m 1` to target a specific monitor, `--hdr` for HDR only, etc.

## Building

### Requirements

- **Visual Studio 2022** (v143 toolset)
- **Windows SDK** (for `icm.h` and `mscms.lib`)
- **C++17** or later
- **Platform**: x64

### Build from command line

```bash
MSBuild.exe ApplyIccLut.sln -p:Configuration=Release -p:Platform=x64
```

Output: `x64\Release\ApplyIccLut.exe` and `x64\Release\ApplyIccLut_Dither.dll`

Place both files in the same directory. The DLL is deployed to `%SYSTEMROOT%\Temp\` at injection time.

### Dependencies

The main executable links against `mscms.lib` (Windows Color Management) and uses only Win32 APIs.

The dithering DLL (`DitherDll` project) bundles [MinHook](https://github.com/TsudaKageworker/minhook) source directly (no vcpkg required) and links against `d3d11.lib`, `d3dcompiler.lib`, `dxgi.lib`, and `dxguid.lib`.

The GPU ColorProfile APIs (`ColorProfileAddDisplayAssociation`, `ColorProfileRemoveDisplayAssociation`, `ColorProfileGetDisplayDefault`) are loaded dynamically at runtime from `mscms.dll` and require Windows 10 version 1903 or later. The tool gracefully degrades if these APIs are unavailable.

## Technical Notes

- **SetDeviceGammaRamp** is hard-limited to `WORD[3][256]` (256 entries per channel, 16-bit). This is a Windows API limitation that cannot be increased.
- **MHC2 ICC pipeline** supports up to 4096 LUT entries per channel at S15Fixed16 precision, plus a 3x4 color matrix for cross-channel correction. This is applied by the GPU driver, not by this tool directly.
- The `-s` wake-up kick works by removing and re-adding the profile as the default, which forces the GPU driver to re-read and apply it.
- The pre-flight ramp check compares the current gamma ramp against both identity and the target LUT to avoid double-application.
- `.cube` is primarily a 3D LUT format (IRIDAS/Adobe), but also carries optional 1D LUT data. This tool reads only the 1D portion (`LUT_1D_SIZE`); 3D-only files are rejected since `SetDeviceGammaRamp` can only apply per-channel 1D curves.
- **Blue-noise dithering** uses a 64x64 pre-computed blue-noise texture (from [momentsingraphics.de](http://momentsingraphics.de/BlueNoise.html)) to distribute quantization error. SDR dithering uses gamma 2.2 perceptual thresholding. HDR dithering is PQ-aware (SMPTE ST 2084): it converts scRGB to BT.2020 linear, encodes to PQ, quantizes in PQ space (where the display interface truncates), and converts back — thresholds are compared in linear light for perceptual uniformity. The `--dither-bits` flag overrides the auto bit depth, stored in a config file (`%SYSTEMROOT%\Temp\ApplyIccLut_dither.cfg`) that the DLL reads at injection time.
- **DWM hooking** uses AOB (Array of Bytes) pattern scanning to locate `COverlayContext::Present`, `IsCandidateDirectFlipCompatible`, and `OverlaysEnabled` in `dwmcore.dll`. These patterns may change with Windows updates.
- The dithering DLL disables DirectFlip and MPO when active, which adds a small amount of GPU overhead for DWM composition. This is necessary to ensure the dithering shader runs on every frame.
- Boot persistence for dithering uses a flag file (`%SYSTEMROOT%\Temp\ApplyIccLut_dither.flag`). When the tool runs with no arguments (default GPU wake-up mode) and this flag exists, the dithering DLL is automatically re-injected.

## License

MIT
