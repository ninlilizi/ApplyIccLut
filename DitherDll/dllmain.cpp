// DitherDll - Blue-noise dithering injected into DWM's rendering pipeline
//
// Hooks COverlayContext::Present in dwmcore.dll and applies ordered dithering
// using a 64x64 blue-noise texture. This reduces banding artifacts in gradients
// when the output bit depth is limited (8-bit SDR, 10-bit HDR).
//
// Adapted from dwm_lut (https://github.com/lauralex/dwm_lut) with all 3D LUT
// code removed. Only blue-noise dithering remains.

#include "framework.h"
#include "noise.h"

#include <cstring>

#define RELEASE_IF_NOT_NULL(x) { if (x != NULL) { x->Release(); } }

// ============================================================================
// Diagnostic logging — writes to %SYSTEMROOT%\Temp\ApplyIccLut_dither_diag.log
// Uses raw Win32 API only (no CRT) so it works even if CRT isn't initialized.
// ============================================================================

static HANDLE g_diagHandle = INVALID_HANDLE_VALUE;

static void DiagOpen()
{
    if (g_diagHandle != INVALID_HANDLE_VALUE) return;
    char path[MAX_PATH];
    ExpandEnvironmentStringsA("%SYSTEMROOT%\\Temp\\ApplyIccLut_dither_diag.log", path, MAX_PATH);
    g_diagHandle = CreateFileA(path, FILE_APPEND_DATA, FILE_SHARE_READ | FILE_SHARE_WRITE,
                               NULL, OPEN_ALWAYS, FILE_ATTRIBUTE_NORMAL, NULL);
}

static void DiagClose()
{
    if (g_diagHandle != INVALID_HANDLE_VALUE) { CloseHandle(g_diagHandle); g_diagHandle = INVALID_HANDLE_VALUE; }
}

static void DiagLog(const char* fmt, ...)
{
    if (g_diagHandle == INVALID_HANDLE_VALUE) return;
    char buf[512];
    SYSTEMTIME st;
    GetLocalTime(&st);
    int pos = wsprintfA(buf, "[%02d:%02d:%02d.%03d] ", st.wHour, st.wMinute, st.wSecond, st.wMilliseconds);
    va_list ap;
    va_start(ap, fmt);
    pos += wvsprintfA(buf + pos, fmt, ap);
    va_end(ap);
    buf[pos++] = '\r'; buf[pos++] = '\n';
    DWORD written;
    WriteFile(g_diagHandle, buf, pos, &written, NULL);
    FlushFileBuffers(g_diagHandle);
}

// ============================================================================
// Config file format — written by the main application, read by DLL at load
// Contains pre-resolved function offsets and runtime parameters so the DLL
// does not need version detection, pattern scanning, or any setup logic.
// ============================================================================

#pragma pack(push, 1)
struct DitherConfig {
    UINT32 magic;              // 'DITH' (0x48544944)
    UINT32 version;            // struct version (2)
    INT64  presentOffset;      // COverlayContext::Present offset from dwmcore base
    INT64  directFlipOffset;   // IsCandidateDirectFlipCompatible offset from dwmcore base
    INT64  overlaysOffset;     // OverlaysEnabled offset from dwmcore base
    INT32  hwProtOffset;       // IOverlaySwapChain HardwareProtected offset (pre-resolved for OS version)
    INT32  swapChainOffset;    // IOverlaySwapChain IDXGISwapChain offset (pre-resolved for OS version)
    UINT32 isWindows11;        // 1 if Win11+ (affects overlay pointer resolution in Present hook)
    UINT32 ditherBits;         // 0 = auto (SDR=8, HDR=10), otherwise forced bit depth
    UINT32 isWindows11_24h2;   // 1 if Win11 24H2+ (different function signatures & access pattern)
};
#pragma pack(pop)

static const UINT32 DITHER_CONFIG_MAGIC = 0x48544944; // 'DITH'
static const UINT32 DITHER_CONFIG_VERSION = 2;

// Runtime values read from config (used by hooks)
static bool g_isWindows11 = false;
static bool g_isWindows11_24h2 = false;
static int  g_hwProtOffset = 0;
static int  g_swapChainOffset = 0;

// ============================================================================
// HLSL shader: dithering-only (no 3D LUT)
// ============================================================================

static const char shaderCode[] = R"(
struct VS_INPUT {
    float2 pos : POSITION;
    float2 tex : TEXCOORD;
};

struct VS_OUTPUT {
    float4 pos : SV_POSITION;
    float2 tex : TEXCOORD;
};

Texture2D backBufferTex : register(t0);
SamplerState smp : register(s0);

Texture2D noiseTex : register(t1);
SamplerState noiseSmp : register(s1);

cbuffer DitherParams : register(b0) {
    int isHdr;
    float levels;   // e.g. 255.0 for 8-bit, 1023.0 for 10-bit
    int pad1;
    int pad2;
};

// ---- PQ (SMPTE ST 2084) transfer functions ----
static const float pq_m1 = 0.1593017578125;     // 1305.0 / 8192.0
static const float pq_m2 = 78.84375;            // 2523.0 / 32.0
static const float pq_c1 = 0.8359375;           // 107.0 / 128.0
static const float pq_c2 = 18.8515625;          // 2413.0 / 128.0
static const float pq_c3 = 18.6875;             // 2392.0 / 128.0

float3 LinearToPQ(float3 L) {
    L = max(L, 0.0);
    float3 Lm = pow(L, pq_m1);
    return pow((pq_c1 + pq_c2 * Lm) / (1.0 + pq_c3 * Lm), pq_m2);
}

float3 PQToLinear(float3 N) {
    float3 Np = pow(max(N, 0.0), 1.0 / pq_m2);
    return pow(max(Np - pq_c1, 0.0) / (pq_c2 - pq_c3 * Np), 1.0 / pq_m1);
}

// ---- Color space matrices (BT.709/scRGB <-> BT.2020) ----
// scRGB uses BT.709 primaries; PQ uses BT.2020.
static const float3x3 bt709_to_bt2020 = {
    0.627404, 0.329283, 0.043313,
    0.069097, 0.919541, 0.011362,
    0.016392, 0.088013, 0.895595
};

static const float3x3 bt2020_to_bt709 = {
     1.660491, -0.587641, -0.072850,
    -0.124551,  1.132900, -0.008349,
    -0.018151, -0.100579,  1.118730
};

// ---- Fast gamma 2.2 approximation (cubic polynomial) ----
// From dwm_lut_fps_boost: replaces expensive pow(x, 2.2) with 3 MAD ops.
// Max error ~0.003 on [0,1] which is well within 8-bit precision (1/255).
float3 FastPow22(float3 x) {
    return x * (x * (x * 0.305306011 + 0.682171111) + 0.012522878);
}

// ---- SDR dithering (gamma-encoded content) ----
float3 OrderedDitherSDR(float3 rgb, float2 pos) {
    float3 low  = floor(rgb * levels) / levels;
    float3 high = low + 1.0 / levels;

    // Linearize with fast gamma 2.2 approximation for threshold comparison
    float3 rgb_lin  = FastPow22(saturate(rgb));
    float3 low_lin  = FastPow22(saturate(low));
    float3 high_lin = FastPow22(saturate(high));

    float noise = noiseTex.Sample(noiseSmp, pos / 64.0).x;
    float3 threshold = lerp(low_lin, high_lin, noise);

    return lerp(low, high, rgb_lin > threshold);
}

// ---- HDR dithering (PQ-aware, scRGB input) ----
// The quantization bottleneck is in PQ space (after Windows converts scRGB->PQ
// for the 10-bit HDR output).  PQ (ST 2084) is perceptually uniform by design,
// so we threshold directly in PQ space -- no need to decode to linear first.
// This halves the pow() calls vs the linear-threshold approach.
float3 OrderedDitherHDR(float3 scrgb, float2 pos) {
    // scRGB -> BT.2020 linear, normalized for PQ (1.0 = 10000 nits, scRGB 1.0 = 80 nits)
    float3 bt2020_lin = mul(bt709_to_bt2020, scrgb) * (80.0 / 10000.0);
    bt2020_lin = max(bt2020_lin, 0.0);

    // Encode to PQ (this is what the display interface does)
    float3 pq = LinearToPQ(bt2020_lin);

    // Quantize in PQ space (where the actual bit truncation occurs)
    float3 low  = floor(pq * levels) / levels;
    float3 high = low + 1.0 / levels;

    // Threshold in PQ space (perceptually uniform, so noise distributes evenly)
    float noise = noiseTex.Sample(noiseSmp, pos / 64.0).x;
    float3 result_pq = lerp(low, high, pq > lerp(low, high, noise));

    // Decode back: PQ -> BT.2020 linear -> scRGB
    float3 result_lin = PQToLinear(result_pq) * (10000.0 / 80.0);
    return mul(bt2020_to_bt709, result_lin);
}

VS_OUTPUT VS(VS_INPUT input) {
    VS_OUTPUT output;
    output.pos = float4(input.pos, 0, 1);
    output.tex = input.tex;
    return output;
}

float4 PS(VS_OUTPUT input) : SV_TARGET {
    float3 sample_color = backBufferTex.Sample(smp, input.tex).rgb;
    float3 result;
    if (isHdr)
        result = OrderedDitherHDR(sample_color, input.pos.xy);
    else
        result = OrderedDitherSDR(sample_color, input.pos.xy);
    return float4(result, 1);
}
)";

// ============================================================================
// D3D11 state
// ============================================================================

static ID3D11Device* device;
static ID3D11DeviceContext* deviceContext;
static ID3D11VertexShader* vertexShader;
static ID3D11PixelShader* pixelShader;
static ID3D11InputLayout* inputLayout;

static ID3D11Buffer* vertexBuffer;
static UINT numVerts;
static UINT stride;
static UINT vbOffset;

static D3D11_TEXTURE2D_DESC backBufferDesc;
static D3D11_TEXTURE2D_DESC textureDesc[2];

static ID3D11SamplerState* samplerState;
static ID3D11Texture2D* texture[2];
static ID3D11ShaderResourceView* textureView[2];

static ID3D11SamplerState* noiseSamplerState;
static ID3D11ShaderResourceView* noiseTextureView;

static ID3D11Buffer* constantBuffer;

// ============================================================================
// Configuration (read from binary config file written by main application)
// ============================================================================

static int g_configBits = 0; // 0 = auto (SDR=8, HDR=10), otherwise forced bit depth

// Read the DitherConfig struct from the config file.
// Returns true if valid config was loaded, with hook addresses resolved.
static bool ReadConfig(DitherConfig* out)
{
    char path[MAX_PATH];
    ExpandEnvironmentStringsA("%SYSTEMROOT%\\Temp\\ApplyIccLut_dither.cfg", path, MAX_PATH);
    HANDLE hFile = CreateFileA(path, GENERIC_READ, FILE_SHARE_READ, NULL, OPEN_EXISTING, 0, NULL);
    if (hFile == INVALID_HANDLE_VALUE)
        return false;

    DitherConfig cfg = {};
    DWORD bytesRead = 0;
    ReadFile(hFile, &cfg, sizeof(cfg), &bytesRead, NULL);
    CloseHandle(hFile);

    if (bytesRead != sizeof(cfg) || cfg.magic != DITHER_CONFIG_MAGIC || cfg.version != DITHER_CONFIG_VERSION)
        return false;

    *out = cfg;
    return true;
}

// ============================================================================
// Track which overlay contexts have dithering active (for DirectFlip disable)
// ============================================================================

static int numDitherTargets;
static void** ditherTargets;

static bool IsDitherActive(void* target)
{
    for (int i = 0; i < numDitherTargets; i++)
        if (ditherTargets[i] == target)
            return true;
    return false;
}

static void SetDitherActive(void* target)
{
    if (!IsDitherActive(target))
    {
        ditherTargets = (void**)realloc(ditherTargets, (numDitherTargets + 1) * sizeof(*ditherTargets));
        ditherTargets[numDitherTargets++] = target;
    }
}

static void UnsetDitherActive(void* target)
{
    for (int i = 0; i < numDitherTargets; i++)
    {
        if (ditherTargets[i] == target)
        {
            ditherTargets[i] = ditherTargets[--numDitherTargets];
            ditherTargets = (void**)realloc(ditherTargets, numDitherTargets * sizeof(*ditherTargets));
            return;
        }
    }
}

// ============================================================================
// Draw helper
// ============================================================================

static void DrawRectangle(struct tagRECT* rect, int index)
{
    float width = (float)backBufferDesc.Width;
    float height = (float)backBufferDesc.Height;

    float screenLeft   = rect->left / width;
    float screenTop    = rect->top / height;
    float screenRight  = rect->right / width;
    float screenBottom = rect->bottom / height;

    float left   = screenLeft * 2 - 1;
    float top    = screenTop * -2 + 1;
    float right  = screenRight * 2 - 1;
    float bottom = screenBottom * -2 + 1;

    float texWidth  = (float)textureDesc[index].Width;
    float texHeight = (float)textureDesc[index].Height;
    float texLeft   = rect->left / texWidth;
    float texTop    = rect->top / texHeight;
    float texRight  = rect->right / texWidth;
    float texBottom = rect->bottom / texHeight;

    float vertexData[] = {
        left, bottom, texLeft, texBottom,
        left, top,    texLeft, texTop,
        right, bottom, texRight, texBottom,
        right, top,    texRight, texTop
    };

    D3D11_MAPPED_SUBRESOURCE resource;
    deviceContext->Map(vertexBuffer, 0, D3D11_MAP_WRITE_DISCARD, 0, &resource);
    memcpy(resource.pData, vertexData, stride * numVerts);
    deviceContext->Unmap(vertexBuffer, 0);

    deviceContext->IASetVertexBuffers(0, 1, &vertexBuffer, &stride, &vbOffset);
    deviceContext->Draw(numVerts, 0);
}

// ============================================================================
// D3D11 initialization
// ============================================================================

static void InitializeStuff(IDXGISwapChain* swapChain)
{
    swapChain->GetDevice(IID_ID3D11Device, (void**)&device);
    device->GetImmediateContext(&deviceContext);

    // Compile vertex shader
    {
        ID3DBlob* vsBlob;
        ID3DBlob* errBlob;
        HRESULT hr = D3DCompile(shaderCode, sizeof(shaderCode), NULL, NULL, NULL,
                                "VS", "vs_5_0", 0, 0, &vsBlob, &errBlob);
        if (FAILED(hr)) { if (errBlob) errBlob->Release(); return; }

        device->CreateVertexShader(vsBlob->GetBufferPointer(), vsBlob->GetBufferSize(), NULL, &vertexShader);

        D3D11_INPUT_ELEMENT_DESC inputDesc[] = {
            {"POSITION", 0, DXGI_FORMAT_R32G32_FLOAT, 0, 0, D3D11_INPUT_PER_VERTEX_DATA, 0},
            {"TEXCOORD", 0, DXGI_FORMAT_R32G32_FLOAT, 0, D3D11_APPEND_ALIGNED_ELEMENT, D3D11_INPUT_PER_VERTEX_DATA, 0}
        };
        device->CreateInputLayout(inputDesc, 2, vsBlob->GetBufferPointer(), vsBlob->GetBufferSize(), &inputLayout);
        vsBlob->Release();
    }

    // Compile pixel shader
    {
        ID3DBlob* psBlob;
        ID3DBlob* errBlob;
        HRESULT hr = D3DCompile(shaderCode, sizeof(shaderCode), NULL, NULL, NULL,
                                "PS", "ps_5_0", 0, 0, &psBlob, &errBlob);
        if (FAILED(hr)) { if (errBlob) errBlob->Release(); return; }

        device->CreatePixelShader(psBlob->GetBufferPointer(), psBlob->GetBufferSize(), NULL, &pixelShader);
        psBlob->Release();
    }

    // Vertex buffer
    {
        stride = 4 * sizeof(float);
        numVerts = 4;
        vbOffset = 0;

        D3D11_BUFFER_DESC vbDesc = {};
        vbDesc.ByteWidth = stride * numVerts;
        vbDesc.Usage = D3D11_USAGE_DYNAMIC;
        vbDesc.BindFlags = D3D11_BIND_VERTEX_BUFFER;
        vbDesc.CPUAccessFlags = D3D11_CPU_ACCESS_WRITE;
        device->CreateBuffer(&vbDesc, NULL, &vertexBuffer);
    }

    // Sampler for back buffer (point sampling)
    {
        D3D11_SAMPLER_DESC sd = {};
        sd.Filter = D3D11_FILTER_MIN_MAG_MIP_POINT;
        sd.AddressU = sd.AddressV = sd.AddressW = D3D11_TEXTURE_ADDRESS_CLAMP;
        sd.ComparisonFunc = D3D11_COMPARISON_NEVER;
        device->CreateSamplerState(&sd, &samplerState);
    }

    // Noise sampler (wrapping)
    {
        D3D11_SAMPLER_DESC sd = {};
        sd.Filter = D3D11_FILTER_MIN_MAG_MIP_POINT;
        sd.AddressU = sd.AddressV = sd.AddressW = D3D11_TEXTURE_ADDRESS_WRAP;
        sd.ComparisonFunc = D3D11_COMPARISON_NEVER;
        device->CreateSamplerState(&sd, &noiseSamplerState);
    }

    // Blue noise texture (64x64 R32_FLOAT)
    {
        D3D11_TEXTURE2D_DESC desc = {};
        desc.Width = NOISE_SIZE;
        desc.Height = NOISE_SIZE;
        desc.MipLevels = 1;
        desc.ArraySize = 1;
        desc.Format = DXGI_FORMAT_R32_FLOAT;
        desc.SampleDesc.Count = 1;
        desc.Usage = D3D11_USAGE_IMMUTABLE;
        desc.BindFlags = D3D11_BIND_SHADER_RESOURCE;

        float noise[NOISE_SIZE][NOISE_SIZE];
        for (int i = 0; i < NOISE_SIZE; i++)
            for (int j = 0; j < NOISE_SIZE; j++)
                noise[i][j] = (noiseBytes[i][j] + 0.5f) / 256.0f;

        D3D11_SUBRESOURCE_DATA initData;
        initData.pSysMem = noise;
        initData.SysMemPitch = sizeof(noise[0]);
        initData.SysMemSlicePitch = 0;

        ID3D11Texture2D* tex;
        device->CreateTexture2D(&desc, &initData, &tex);
        device->CreateShaderResourceView((ID3D11Resource*)tex, NULL, &noiseTextureView);
        tex->Release();
    }

    // Constant buffer (isHdr flag)
    {
        D3D11_BUFFER_DESC cbDesc = {};
        cbDesc.BindFlags = D3D11_BIND_CONSTANT_BUFFER;
        cbDesc.ByteWidth = 16;
        cbDesc.Usage = D3D11_USAGE_DYNAMIC;
        cbDesc.CPUAccessFlags = D3D11_CPU_ACCESS_WRITE;
        device->CreateBuffer(&cbDesc, NULL, &constantBuffer);
    }
}

static void UninitializeStuff()
{
    RELEASE_IF_NOT_NULL(device)
    RELEASE_IF_NOT_NULL(deviceContext)
    RELEASE_IF_NOT_NULL(vertexShader)
    RELEASE_IF_NOT_NULL(pixelShader)
    RELEASE_IF_NOT_NULL(inputLayout)
    RELEASE_IF_NOT_NULL(vertexBuffer)
    RELEASE_IF_NOT_NULL(samplerState)
    for (int i = 0; i < 2; i++)
    {
        RELEASE_IF_NOT_NULL(texture[i])
        RELEASE_IF_NOT_NULL(textureView[i])
    }
    RELEASE_IF_NOT_NULL(noiseSamplerState)
    RELEASE_IF_NOT_NULL(noiseTextureView)
    RELEASE_IF_NOT_NULL(constantBuffer)
    free(ditherTargets);
}

// ============================================================================
// Apply dithering to the back buffer
// ============================================================================

static bool ApplyDither(IDXGISwapChain* swapChain, struct tagRECT* rects, int numRects)
{
    if (!device)
        InitializeStuff(swapChain);

    if (!device || !pixelShader)
        return false;

    ID3D11Texture2D* backBuffer;
    ID3D11RenderTargetView* renderTargetView;

    if (FAILED(swapChain->GetBuffer(0, IID_ID3D11Texture2D, (void**)&backBuffer)))
        return false;

    D3D11_TEXTURE2D_DESC newBackBufferDesc;
    backBuffer->GetDesc(&newBackBufferDesc);

    // Determine SDR vs HDR from back buffer format
    int index = -1;
    if (newBackBufferDesc.Format == DXGI_FORMAT_B8G8R8A8_UNORM)
        index = 0; // SDR
    else if (newBackBufferDesc.Format == DXGI_FORMAT_R16G16B16A16_FLOAT)
        index = 1; // HDR

    if (index == -1)
    {
        backBuffer->Release();
        return false;
    }

    // Resize copy texture if needed
    D3D11_TEXTURE2D_DESC oldTexDesc = textureDesc[index];
    if (newBackBufferDesc.Width > oldTexDesc.Width || newBackBufferDesc.Height > oldTexDesc.Height)
    {
        if (texture[index] != NULL)
        {
            texture[index]->Release();
            textureView[index]->Release();
        }

        UINT newWidth  = (newBackBufferDesc.Width > oldTexDesc.Width) ? newBackBufferDesc.Width : oldTexDesc.Width;
        UINT newHeight = (newBackBufferDesc.Height > oldTexDesc.Height) ? newBackBufferDesc.Height : oldTexDesc.Height;

        D3D11_TEXTURE2D_DESC newTexDesc = newBackBufferDesc;
        newTexDesc.Width = newWidth;
        newTexDesc.Height = newHeight;
        newTexDesc.Usage = D3D11_USAGE_DEFAULT;
        newTexDesc.BindFlags = D3D11_BIND_SHADER_RESOURCE;
        newTexDesc.CPUAccessFlags = 0;
        newTexDesc.MiscFlags = 0;

        textureDesc[index] = newTexDesc;

        device->CreateTexture2D(&textureDesc[index], NULL, &texture[index]);
        device->CreateShaderResourceView((ID3D11Resource*)texture[index], NULL, &textureView[index]);
    }

    backBufferDesc = newBackBufferDesc;

    if (FAILED(device->CreateRenderTargetView((ID3D11Resource*)backBuffer, NULL, &renderTargetView)))
    {
        backBuffer->Release();
        return false;
    }

    const D3D11_VIEWPORT viewport = {0, 0, (float)backBufferDesc.Width, (float)backBufferDesc.Height, 0.0f, 1.0f};
    deviceContext->RSSetViewports(1, &viewport);
    deviceContext->OMSetRenderTargets(1, &renderTargetView, NULL);
    renderTargetView->Release();

    deviceContext->IASetPrimitiveTopology(D3D11_PRIMITIVE_TOPOLOGY_TRIANGLESTRIP);
    deviceContext->IASetInputLayout(inputLayout);

    deviceContext->VSSetShader(vertexShader, NULL, 0);
    deviceContext->PSSetShader(pixelShader, NULL, 0);

    deviceContext->PSSetShaderResources(0, 1, &textureView[index]);
    deviceContext->PSSetSamplers(0, 1, &samplerState);

    deviceContext->PSSetShaderResources(1, 1, &noiseTextureView);
    deviceContext->PSSetSamplers(1, 1, &noiseSamplerState);

    // Set constant buffer: isHdr flag + levels
    int bits = g_configBits;
    if (bits == 0) bits = (index == 1) ? 10 : 8; // auto: SDR=8-bit, HDR=10-bit
    float levelsVal = (float)((1 << bits) - 1);   // 255 for 8, 1023 for 10

    struct { int isHdr; float levels; int pad1; int pad2; } constantData = {index == 1, levelsVal, 0, 0};
    D3D11_MAPPED_SUBRESOURCE resource;
    deviceContext->Map((ID3D11Resource*)constantBuffer, 0, D3D11_MAP_WRITE_DISCARD, 0, &resource);
    memcpy(resource.pData, &constantData, sizeof(constantData));
    deviceContext->Unmap((ID3D11Resource*)constantBuffer, 0);
    deviceContext->PSSetConstantBuffers(0, 1, &constantBuffer);

    for (int i = 0; i < numRects; i++)
    {
        D3D11_BOX sourceRegion;
        sourceRegion.left = rects[i].left;
        sourceRegion.right = rects[i].right;
        sourceRegion.top = rects[i].top;
        sourceRegion.bottom = rects[i].bottom;
        sourceRegion.front = 0;
        sourceRegion.back = 1;

        deviceContext->CopySubresourceRegion((ID3D11Resource*)texture[index], 0,
                                             rects[i].left, rects[i].top, 0,
                                             (ID3D11Resource*)backBuffer, 0, &sourceRegion);
        DrawRectangle(&rects[i], index);
    }

    backBuffer->Release();
    return true;
}

// ============================================================================
// Hook: COverlayContext::Present
// ============================================================================

typedef struct rectVec {
    struct tagRECT* start;
    struct tagRECT* end;
    struct tagRECT* cap;
} rectVec;

// ============================================================================
// Hook: COverlayContext::Present — pre-24H2 (6 params, returns long)
// ============================================================================

typedef long (COverlayContext_Present_t)(void*, void*, unsigned int, rectVec*, unsigned int, bool);

static COverlayContext_Present_t* COverlayContext_Present_orig = NULL;
static COverlayContext_Present_t* COverlayContext_Present_real_orig = NULL;

// ============================================================================
// Hook: COverlayContext::Present — 24H2+ (7 params, returns long long)
// ============================================================================

typedef long long (COverlayContext_Present_24h2_t)(void*, void*, unsigned int, rectVec*, int, void*, bool);

static COverlayContext_Present_24h2_t* COverlayContext_Present_orig_24h2 = NULL;
static COverlayContext_Present_24h2_t* COverlayContext_Present_real_orig_24h2 = NULL;

// Shared state
static volatile LONG g_hookCallCount = 0;
static volatile LONG g_hookDitherCount = 0;

// Common Present logic — called from both hook variants
static void PresentHookCommon(void* self, void* overlaySwapChain, rectVec* rectVec)
{
    if (*((bool*)overlaySwapChain + g_hwProtOffset))
    {
        UnsetDitherActive(self);
    }
    else
    {
        IDXGISwapChain* swapChain;
        if (g_isWindows11_24h2)
        {
            // 24H2+: direct offset on overlaySwapChain
            swapChain = *(IDXGISwapChain**)((unsigned char*)overlaySwapChain + g_swapChainOffset);
        }
        else if (g_isWindows11)
        {
            // Pre-24H2 Win11: indirect through legacy overlay pointer
            int sub = *(int*)((unsigned char*)overlaySwapChain - 4);
            void* realOverlay = (unsigned char*)overlaySwapChain - sub - 0x1b0;
            swapChain = *(IDXGISwapChain**)((unsigned char*)realOverlay + g_swapChainOffset);
        }
        else
        {
            // Win10: direct offset
            swapChain = *(IDXGISwapChain**)((unsigned char*)overlaySwapChain + g_swapChainOffset);
        }

        if (ApplyDither(swapChain, rectVec->start, (int)(rectVec->end - rectVec->start)))
        {
            InterlockedIncrement(&g_hookDitherCount);
            SetDitherActive(self);
        }
        else
            UnsetDitherActive(self);
    }
}

static void PresentHookFirstCall(void* self, void* overlaySwapChain)
{
    DiagOpen();
    DiagLog("=== HOOK CALLED! First Present hook invocation ===");
    DiagLog("  self=%p overlaySwapChain=%p is24h2=%d", self, overlaySwapChain, (int)g_isWindows11_24h2);
    DiagClose();
}

static long COverlayContext_Present_hook(void* self, void* overlaySwapChain, unsigned int a3,
                                         rectVec* rectVec, unsigned int a5, bool a6)
{
    LONG count = InterlockedIncrement(&g_hookCallCount);
    if (count == 1) PresentHookFirstCall(self, overlaySwapChain);

    if (g_isWindows11 || _ReturnAddress() < (void*)COverlayContext_Present_real_orig)
        PresentHookCommon(self, overlaySwapChain, rectVec);

    return COverlayContext_Present_orig(self, overlaySwapChain, a3, rectVec, a5, a6);
}

static long long COverlayContext_Present_hook_24h2(void* self, void* overlaySwapChain, unsigned int a3,
                                                    rectVec* rectVec, int a5, void* a6, bool a7)
{
    LONG count = InterlockedIncrement(&g_hookCallCount);
    if (count == 1) PresentHookFirstCall(self, overlaySwapChain);

    // 24H2+: always enter (no _ReturnAddress guard needed)
    PresentHookCommon(self, overlaySwapChain, rectVec);

    return COverlayContext_Present_orig_24h2(self, overlaySwapChain, a3, rectVec, a5, a6, a7);
}

// ============================================================================
// Hook: COverlayContext::IsCandidateDirectFlipCompatible — pre-24H2 (8 params)
// ============================================================================

typedef bool (COverlayContext_IsCandidateDirectFlipCompatbile_t)(void*, void*, void*, void*, int, unsigned int, bool, bool);

static COverlayContext_IsCandidateDirectFlipCompatbile_t* COverlayContext_IsCandidateDirectFlipCompatbile_orig = NULL;

static bool COverlayContext_IsCandidateDirectFlipCompatbile_hook(void* self, void* a2, void* a3, void* a4,
                                                                  int a5, unsigned int a6, bool a7, bool a8)
{
    if (IsDitherActive(self))
        return false;
    return COverlayContext_IsCandidateDirectFlipCompatbile_orig(self, a2, a3, a4, a5, a6, a7, a8);
}

// ============================================================================
// Hook: COverlayContext::IsCandidateDirectFlipCompatible — 24H2+ (6 params)
// ============================================================================

typedef bool (COverlayContext_IsCandidateDirectFlipCompatbile_24h2_t)(void*, void*, void*, void*, unsigned int, bool);

static COverlayContext_IsCandidateDirectFlipCompatbile_24h2_t* COverlayContext_IsCandidateDirectFlipCompatbile_orig_24h2 = NULL;

static bool COverlayContext_IsCandidateDirectFlipCompatbile_hook_24h2(void* self, void* a2, void* a3, void* a4,
                                                                       unsigned int a5, bool a6)
{
    if (IsDitherActive(self))
        return false;
    return COverlayContext_IsCandidateDirectFlipCompatbile_orig_24h2(self, a2, a3, a4, a5, a6);
}

// ============================================================================
// Hook: COverlayContext::OverlaysEnabled (same signature all versions)
// ============================================================================

typedef bool (COverlayContext_OverlaysEnabled_t)(void*);
static COverlayContext_OverlaysEnabled_t* COverlayContext_OverlaysEnabled_orig = NULL;

static bool COverlayContext_OverlaysEnabled_hook(void* self)
{
    if (IsDitherActive(self))
        return false;
    return COverlayContext_OverlaysEnabled_orig(self);
}

// ============================================================================
// DLL entry point
// ============================================================================

BOOL APIENTRY DllMain(HMODULE hModule, DWORD fdwReason, LPVOID lpReserved)
{
    switch (fdwReason)
    {
    case DLL_PROCESS_ATTACH:
    {
        DiagOpen();
        DiagLog("=== DllMain DLL_PROCESS_ATTACH ===");
        DiagLog("DLL hModule: %p", hModule);

        // Read config written by the main application (contains pre-resolved offsets)
        DitherConfig cfg = {};
        if (!ReadConfig(&cfg))
        {
            DiagLog("FAIL: ReadConfig returned false");
            // Try to provide more detail about why
            char cfgPath[MAX_PATH];
            ExpandEnvironmentStringsA("%SYSTEMROOT%\\Temp\\ApplyIccLut_dither.cfg", cfgPath, MAX_PATH);
            HANDLE hTest = CreateFileA(cfgPath, GENERIC_READ, FILE_SHARE_READ, NULL, OPEN_EXISTING, 0, NULL);
            if (hTest == INVALID_HANDLE_VALUE)
            {
                DiagLog("  Config file not accessible: %s (err=%lu)", cfgPath, GetLastError());
            }
            else
            {
                DWORD fileSize = GetFileSize(hTest, NULL);
                DiagLog("  Config file exists, size=%lu (expected %lu)", fileSize, (unsigned long)sizeof(DitherConfig));
                if (fileSize >= sizeof(DitherConfig))
                {
                    DitherConfig raw = {};
                    DWORD br = 0;
                    ReadFile(hTest, &raw, sizeof(raw), &br, NULL);
                    DiagLog("  Read %lu bytes, magic=0x%08X (expect 0x%08X), version=%u (expect %u)",
                            br, raw.magic, DITHER_CONFIG_MAGIC, raw.version, DITHER_CONFIG_VERSION);
                }
                CloseHandle(hTest);
            }
            DiagClose();
            return FALSE;
        }
        DiagLog("ReadConfig OK: present=0x%lx directflip=0x%lx overlays=0x%lx",
                (unsigned long)cfg.presentOffset, (unsigned long)cfg.directFlipOffset,
                (unsigned long)cfg.overlaysOffset);
        DiagLog("  hwProt=%d swapChain=%d isWin11=%u is24h2=%u bits=%u",
                cfg.hwProtOffset, cfg.swapChainOffset, cfg.isWindows11, cfg.isWindows11_24h2, cfg.ditherBits);

        // Get dwmcore.dll base in our process to resolve offsets to absolute addresses
        HMODULE dwmcore = GetModuleHandle(L"dwmcore.dll");
        if (!dwmcore)
        {
            DiagLog("FAIL: GetModuleHandle(dwmcore.dll) returned NULL (err=%lu)", GetLastError());
            DiagClose();
            return FALSE;
        }
        DiagLog("dwmcore.dll base: %p", dwmcore);

        unsigned char* base = (unsigned char*)dwmcore;

        void* presentAddr = (void*)(base + cfg.presentOffset);
        void* directFlipAddr = (void*)(base + cfg.directFlipOffset);
        void* overlaysAddr = (void*)(base + cfg.overlaysOffset);

        if (cfg.isWindows11_24h2) {
            COverlayContext_Present_orig_24h2 = (COverlayContext_Present_24h2_t*)presentAddr;
            COverlayContext_Present_real_orig_24h2 = COverlayContext_Present_orig_24h2;
            COverlayContext_IsCandidateDirectFlipCompatbile_orig_24h2 =
                (COverlayContext_IsCandidateDirectFlipCompatbile_24h2_t*)directFlipAddr;
        } else {
            COverlayContext_Present_orig = (COverlayContext_Present_t*)presentAddr;
            COverlayContext_Present_real_orig = COverlayContext_Present_orig;
            COverlayContext_IsCandidateDirectFlipCompatbile_orig =
                (COverlayContext_IsCandidateDirectFlipCompatbile_t*)directFlipAddr;
        }
        COverlayContext_OverlaysEnabled_orig = (COverlayContext_OverlaysEnabled_t*)overlaysAddr;

        DiagLog("Hook targets: Present=%p DirectFlip=%p Overlays=%p (24h2=%d)",
                presentAddr, directFlipAddr, overlaysAddr, (int)cfg.isWindows11_24h2);

        // Dump prologue BEFORE hooking to verify we're at a real function
        {
            unsigned char* p = (unsigned char*)presentAddr;
            DiagLog("Present BEFORE hook: %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X",
                    p[0], p[1], p[2], p[3], p[4], p[5], p[6], p[7],
                    p[8], p[9], p[10], p[11], p[12], p[13], p[14], p[15]);
        }

        // Store runtime parameters from config
        g_isWindows11 = (cfg.isWindows11 != 0);
        g_isWindows11_24h2 = (cfg.isWindows11_24h2 != 0);
        g_hwProtOffset = cfg.hwProtOffset;
        g_swapChainOffset = cfg.swapChainOffset;
        g_configBits = (int)cfg.ditherBits;

        // Install hooks via MinHook
        MH_STATUS mhStatus = MH_Initialize();
        DiagLog("MH_Initialize: %d (%s)", mhStatus, MH_StatusToString(mhStatus));
        if (mhStatus != MH_OK)
        {
            DiagLog("FAIL: MH_Initialize failed");
            DiagClose();
            return FALSE;
        }

        if (g_isWindows11_24h2) {
            mhStatus = MH_CreateHook(presentAddr, (PVOID)COverlayContext_Present_hook_24h2,
                                     (PVOID*)&COverlayContext_Present_orig_24h2);
            DiagLog("MH_CreateHook(Present_24h2): %d (%s)", mhStatus, MH_StatusToString(mhStatus));

            mhStatus = MH_CreateHook(directFlipAddr, (PVOID)COverlayContext_IsCandidateDirectFlipCompatbile_hook_24h2,
                                     (PVOID*)&COverlayContext_IsCandidateDirectFlipCompatbile_orig_24h2);
            DiagLog("MH_CreateHook(DirectFlip_24h2): %d (%s)", mhStatus, MH_StatusToString(mhStatus));
        } else {
            mhStatus = MH_CreateHook(presentAddr, (PVOID)COverlayContext_Present_hook,
                                     (PVOID*)&COverlayContext_Present_orig);
            DiagLog("MH_CreateHook(Present): %d (%s)", mhStatus, MH_StatusToString(mhStatus));

            mhStatus = MH_CreateHook(directFlipAddr, (PVOID)COverlayContext_IsCandidateDirectFlipCompatbile_hook,
                                     (PVOID*)&COverlayContext_IsCandidateDirectFlipCompatbile_orig);
            DiagLog("MH_CreateHook(DirectFlip): %d (%s)", mhStatus, MH_StatusToString(mhStatus));
        }

        mhStatus = MH_CreateHook(overlaysAddr, (PVOID)COverlayContext_OverlaysEnabled_hook,
                                 (PVOID*)&COverlayContext_OverlaysEnabled_orig);
        DiagLog("MH_CreateHook(Overlays): %d (%s)", mhStatus, MH_StatusToString(mhStatus));

        mhStatus = MH_EnableHook(MH_ALL_HOOKS);
        DiagLog("MH_EnableHook(ALL): %d (%s)", mhStatus, MH_StatusToString(mhStatus));

        if (mhStatus != MH_OK)
        {
            DiagLog("FAIL: MH_EnableHook failed");
            DiagClose();
            return FALSE;
        }

        // Full JMP chain verification
        {
            void* realOrig = g_isWindows11_24h2 ?
                (void*)COverlayContext_Present_real_orig_24h2 :
                (void*)COverlayContext_Present_real_orig;
            void* trampoline = g_isWindows11_24h2 ?
                (void*)COverlayContext_Present_orig_24h2 :
                (void*)COverlayContext_Present_orig;
            void* hookFn = g_isWindows11_24h2 ?
                (void*)COverlayContext_Present_hook_24h2 :
                (void*)COverlayContext_Present_hook;

            unsigned char* p = (unsigned char*)realOrig;
            DiagLog("Present @%p AFTER hook: %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X",
                    p, p[0], p[1], p[2], p[3], p[4], p[5], p[6], p[7], p[8], p[9], p[10], p[11], p[12], p[13]);
            DiagLog("Hook fn: %p, trampoline: %p", hookFn, trampoline);

            if (p[0] == 0xE9) {
                INT32 rel = *(INT32*)(p + 1);
                unsigned char* jmpTarget = p + 5 + rel;
                DiagLog("  E9 JMP -> %p (rel32=0x%08X)", jmpTarget, (unsigned)rel);
                DiagLog("  target bytes: %02X %02X %02X %02X %02X %02X %02X %02X",
                        jmpTarget[0], jmpTarget[1], jmpTarget[2], jmpTarget[3],
                        jmpTarget[4], jmpTarget[5], jmpTarget[6], jmpTarget[7]);

                // Decode FF 25 indirect JMP to verify full chain reaches our hook
                if (jmpTarget[0] == 0xFF && jmpTarget[1] == 0x25) {
                    INT32 indRel = *(INT32*)(jmpTarget + 2);
                    void** pFinalAddr = (void**)(jmpTarget + 6 + indRel);
                    void* finalAddr = *pFinalAddr;
                    DiagLog("  FF25 -> [%p] = %p", pFinalAddr, finalAddr);
                    DiagLog("  Chain targets hook fn: %s",
                            (finalAddr == hookFn) ? "YES" : "NO <<<< MISMATCH!");
                }

                // Check page protections along the chain
                MEMORY_BASIC_INFORMATION mbi = {};
                if (VirtualQuery(jmpTarget, &mbi, sizeof(mbi)))
                    DiagLog("  rbCodeIn page: protect=0x%lx state=0x%lx type=0x%lx",
                            mbi.Protect, mbi.State, mbi.Type);
            } else {
                DiagLog("  WARNING: byte[0]=0x%02X, NOT E9 JMP!", p[0]);
            }

            MEMORY_BASIC_INFORMATION mbi2 = {};
            if (VirtualQuery(trampoline, &mbi2, sizeof(mbi2)))
                DiagLog("  Trampoline page: protect=0x%lx type=0x%lx", mbi2.Protect, mbi2.Type);
            MEMORY_BASIC_INFORMATION mbi3 = {};
            if (VirtualQuery(hookFn, &mbi3, sizeof(mbi3)))
                DiagLog("  Hook fn page: protect=0x%lx type=0x%lx", mbi3.Protect, mbi3.Type);

            // Check the PATCHED page protection (the dwmcore.dll code page we modified)
            MEMORY_BASIC_INFORMATION mbi4 = {};
            if (VirtualQuery(p, &mbi4, sizeof(mbi4)))
                DiagLog("  Patched code page: protect=0x%lx type=0x%lx", mbi4.Protect, mbi4.Type);
        }

        // Check process mitigation policies
        {
            typedef BOOL (WINAPI *GetProcessMitigationPolicy_t)(HANDLE, int, PVOID, SIZE_T);
            auto pGetPolicy = (GetProcessMitigationPolicy_t)GetProcAddress(
                GetModuleHandleW(L"kernel32.dll"), "GetProcessMitigationPolicy");
            if (pGetPolicy) {
                struct { DWORD Flags; } dynPolicy = {};
                if (pGetPolicy(GetCurrentProcess(), 2, &dynPolicy, sizeof(dynPolicy)))
                    DiagLog("ACG flags: 0x%lx", dynPolicy.Flags);
                struct { DWORD Flags; } sigPolicy = {};
                if (pGetPolicy(GetCurrentProcess(), 8, &sigPolicy, sizeof(sigPolicy)))
                    DiagLog("Signature flags: 0x%lx", sigPolicy.Flags);
            }
        }

        // Check if VBS/HVCI might be silently active (NtQuerySystemInformation)
        {
            typedef LONG (NTAPI *NtQuerySystemInformation_t)(ULONG, PVOID, ULONG, PULONG);
            auto pNtQSI = (NtQuerySystemInformation_t)GetProcAddress(
                GetModuleHandleW(L"ntdll.dll"), "NtQuerySystemInformation");
            if (pNtQSI) {
                // SystemCodeIntegrityInformation (class 103)
                struct { ULONG Length; ULONG CodeIntegrityOptions; } ciInfo = { sizeof(ciInfo), 0 };
                LONG status = pNtQSI(103, &ciInfo, sizeof(ciInfo), NULL);
                DiagLog("CodeIntegrity: status=0x%lx options=0x%lx (bit0=HVCI)", status, ciInfo.CodeIntegrityOptions);
            }
        }

        DiagLog("=== DllMain DLL_PROCESS_ATTACH complete (success) ===");
        DiagClose();
        break;
    }
    case DLL_PROCESS_DETACH:
        DiagOpen();
        DiagLog("=== DllMain DLL_PROCESS_DETACH (hook calls: %ld, dither: %ld) ===",
                g_hookCallCount, g_hookDitherCount);
        MH_DisableHook(MH_ALL_HOOKS);
        MH_Uninitialize();
        Sleep(100);
        UninitializeStuff();
        DiagLog("=== DllMain DLL_PROCESS_DETACH complete ===");
        DiagClose();
        break;
    default:
        break;
    }
    return TRUE;
}
