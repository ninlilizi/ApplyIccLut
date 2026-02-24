#pragma once

#include <windows.h>
#include <intrin.h>
#include <psapi.h>
#include <d3d11.h>
#include <d3dcompiler.h>
#include <dxgi.h>
#include <MinHook.h>

// Import libraries are specified in the vcxproj AdditionalDependencies.
// d3dcompiler_47.dll is delay-loaded to avoid crashing DWM during injection.
