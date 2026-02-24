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
    UINT32 version;            // struct version (1)
    INT64  presentOffset;      // COverlayContext::Present offset from dwmcore base
    INT64  directFlipOffset;   // IsCandidateDirectFlipCompatible offset from dwmcore base
    INT64  overlaysOffset;     // OverlaysEnabled offset from dwmcore base
    INT32  hwProtOffset;       // IOverlaySwapChain HardwareProtected offset (pre-resolved for OS version)
    INT32  swapChainOffset;    // IOverlaySwapChain IDXGISwapChain offset (pre-resolved for OS version)
    UINT32 isWindows11;        // 1 if Win11+ (affects overlay pointer resolution in Present hook)
    UINT32 ditherBits;         // 0 = auto (SDR=8, HDR=10), otherwise forced bit depth
};
#pragma pack(pop)

static const UINT32 DITHER_CONFIG_MAGIC = 0x48544944; // 'DITH'
static const UINT32 DITHER_CONFIG_VERSION = 1;

// Runtime values read from config (used by hooks)
static bool g_isWindows11 = false;
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

// ---- SDR dithering (gamma-encoded content) ----
float3 OrderedDitherSDR(float3 rgb, float2 pos) {
    float3 low  = floor(rgb * levels) / levels;
    float3 high = low + 1.0 / levels;

    // Linearize with sRGB-approximate gamma for threshold comparison
    float3 rgb_lin  = pow(saturate(rgb), 2.2);
    float3 low_lin  = pow(saturate(low), 2.2);
    float3 high_lin = pow(saturate(high), 2.2);

    float noise = noiseTex.Sample(noiseSmp, pos / 64.0).x;
    float3 threshold = lerp(low_lin, high_lin, noise);

    return lerp(low, high, rgb_lin > threshold);
}

// ---- HDR dithering (PQ-aware, scRGB input) ----
// The quantization bottleneck is in PQ space (after Windows converts scRGB->PQ
// for the 10-bit HDR output). A bad scaler may further truncate to 8-bit in PQ.
// We dither in PQ space so that after quantization the error is spatially
// distributed with blue-noise, and we compare thresholds in linear light
// for perceptual uniformity.
float3 OrderedDitherHDR(float3 scrgb, float2 pos) {
    // scRGB -> BT.2020 linear, normalized for PQ (1.0 = 10000 nits, scRGB 1.0 = 80 nits)
    float3 bt2020_lin = mul(bt709_to_bt2020, scrgb) * (80.0 / 10000.0);
    bt2020_lin = max(bt2020_lin, 0.0);

    // Encode to PQ (this is what the display interface does)
    float3 pq = LinearToPQ(bt2020_lin);

    // Quantize in PQ space (where the actual bit truncation occurs)
    float3 low  = floor(pq * levels) / levels;
    float3 high = low + 1.0 / levels;

    // Decode quantization boundaries back to linear for threshold comparison
    float3 low_lin  = PQToLinear(low);
    float3 high_lin = PQToLinear(high);

    float noise = noiseTex.Sample(noiseSmp, pos / 64.0).x;
    float3 threshold = lerp(low_lin, high_lin, noise);

    // Choose low or high PQ value based on where the linear signal falls
    float3 result_pq = lerp(low, high, bt2020_lin > threshold);

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

typedef long (COverlayContext_Present_t)(void*, void*, unsigned int, rectVec*, unsigned int, bool);

static COverlayContext_Present_t* COverlayContext_Present_orig = NULL;
static COverlayContext_Present_t* COverlayContext_Present_real_orig = NULL;
static bool g_firstHookCallLogged = false;

static long COverlayContext_Present_hook(void* self, void* overlaySwapChain, unsigned int a3,
                                         rectVec* rectVec, unsigned int a5, bool a6)
{
    if (!g_firstHookCallLogged)
    {
        g_firstHookCallLogged = true;
        DiagOpen();
        DiagLog("=== HOOK CALLED! First Present hook invocation ===");
        DiagLog("  self=%p overlaySwapChain=%p retAddr=%p realOrig=%p",
                self, overlaySwapChain, _ReturnAddress(), COverlayContext_Present_real_orig);
        DiagClose();
    }

    if (_ReturnAddress() < (void*)COverlayContext_Present_real_orig)
    {
        if (*((bool*)overlaySwapChain + g_hwProtOffset))
        {
            UnsetDitherActive(self);
        }
        else
        {
            IDXGISwapChain* swapChain;
            if (g_isWindows11)
            {
                int sub = *(int*)((unsigned char*)overlaySwapChain - 4);
                void* realOverlay = (unsigned char*)overlaySwapChain - sub - 0x1b0;
                swapChain = *(IDXGISwapChain**)((unsigned char*)realOverlay + g_swapChainOffset);
            }
            else
            {
                swapChain = *(IDXGISwapChain**)((unsigned char*)overlaySwapChain + g_swapChainOffset);
            }

            if (ApplyDither(swapChain, rectVec->start, (int)(rectVec->end - rectVec->start)))
                SetDitherActive(self);
            else
                UnsetDitherActive(self);
        }
    }

    return COverlayContext_Present_orig(self, overlaySwapChain, a3, rectVec, a5, a6);
}

// ============================================================================
// Hook: COverlayContext::IsCandidateDirectFlipCompatible (disable DirectFlip)
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
// Hook: COverlayContext::OverlaysEnabled (disable MPO)
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
                DiagLog("  Config file exists, size=%lu (expected %zu)", fileSize, sizeof(DitherConfig));
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
        DiagLog("ReadConfig OK: present=0x%llx directflip=0x%llx overlays=0x%llx",
                (unsigned long long)cfg.presentOffset, (unsigned long long)cfg.directFlipOffset,
                (unsigned long long)cfg.overlaysOffset);
        DiagLog("  hwProt=%d swapChain=%d isWin11=%u bits=%u",
                cfg.hwProtOffset, cfg.swapChainOffset, cfg.isWindows11, cfg.ditherBits);

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

        COverlayContext_Present_orig = (COverlayContext_Present_t*)(base + cfg.presentOffset);
        COverlayContext_Present_real_orig = COverlayContext_Present_orig;
        COverlayContext_IsCandidateDirectFlipCompatbile_orig =
            (COverlayContext_IsCandidateDirectFlipCompatbile_t*)(base + cfg.directFlipOffset);
        COverlayContext_OverlaysEnabled_orig =
            (COverlayContext_OverlaysEnabled_t*)(base + cfg.overlaysOffset);

        DiagLog("Hook targets: Present=%p DirectFlip=%p Overlays=%p",
                COverlayContext_Present_orig,
                COverlayContext_IsCandidateDirectFlipCompatbile_orig,
                COverlayContext_OverlaysEnabled_orig);

        // Store runtime parameters from config
        g_isWindows11 = (cfg.isWindows11 != 0);
        g_hwProtOffset = cfg.hwProtOffset;
        g_swapChainOffset = cfg.swapChainOffset;
        g_configBits = (int)cfg.ditherBits;

        // Install hooks (check return values)
        MH_STATUS mhStatus;

        mhStatus = MH_Initialize();
        DiagLog("MH_Initialize: %d", (int)mhStatus);
        if (mhStatus != MH_OK)
        {
            DiagLog("FAIL: MH_Initialize failed");
            DiagClose();
            return FALSE;
        }

        mhStatus = MH_CreateHook((PVOID)COverlayContext_Present_orig,
                       (PVOID)COverlayContext_Present_hook,
                       (PVOID*)&COverlayContext_Present_orig);
        DiagLog("MH_CreateHook(Present): %d", (int)mhStatus);

        mhStatus = MH_CreateHook((PVOID)COverlayContext_IsCandidateDirectFlipCompatbile_orig,
                       (PVOID)COverlayContext_IsCandidateDirectFlipCompatbile_hook,
                       (PVOID*)&COverlayContext_IsCandidateDirectFlipCompatbile_orig);
        DiagLog("MH_CreateHook(DirectFlip): %d", (int)mhStatus);

        mhStatus = MH_CreateHook((PVOID)COverlayContext_OverlaysEnabled_orig,
                       (PVOID)COverlayContext_OverlaysEnabled_hook,
                       (PVOID*)&COverlayContext_OverlaysEnabled_orig);
        DiagLog("MH_CreateHook(Overlays): %d", (int)mhStatus);

        mhStatus = MH_EnableHook(MH_ALL_HOOKS);
        DiagLog("MH_EnableHook(ALL): %d", (int)mhStatus);

        DiagLog("=== DllMain DLL_PROCESS_ATTACH complete (success) ===");
        DiagClose();
        break;
    }
    case DLL_PROCESS_DETACH:
        DiagOpen();
        DiagLog("=== DllMain DLL_PROCESS_DETACH ===");
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
