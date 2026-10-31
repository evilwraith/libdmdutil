#include <algorithm>
#include <chrono>
#include <cstring>
#include <thread>

#include "DMDInternal.h"
#include "DMDUtil/Config.h"
#include "DMDUtil/Logger.h"
#include "FrameUtil.h"
#include "OutputFilters.h"
#include "ZeDMD.h"

namespace DMDUtil
{

namespace
{
// Config::GetScalerMode() for a ZeDMD panel twice the frame's size: 1 doubles each pixel, 2 is
// Scale2x (EPX), which keeps diagonal edges from turning into 2x2 steps.
template <typename Pixel, typename Equal, typename Put>
void Scale2xGeneric(const Pixel& at, Equal eq, Put put, uint16_t w, uint16_t h)
{
  auto px = [&](int x, int y)
  {
    x = std::clamp(x, 0, (int)w - 1);
    y = std::clamp(y, 0, (int)h - 1);
    return at(x, y);
  };
  for (int y = 0; y < (int)h; ++y)
  {
    for (int x = 0; x < (int)w; ++x)
    {
      const auto P = px(x, y);
      const auto A = px(x, y - 1);
      const auto B = px(x + 1, y);
      const auto C = px(x, y + 1);
      const auto D = px(x - 1, y);
      put(x * 2, y * 2, (eq(D, A) && !eq(D, C) && !eq(A, B)) ? A : P);
      put(x * 2 + 1, y * 2, (eq(A, B) && !eq(A, D) && !eq(B, C)) ? B : P);
      put(x * 2, y * 2 + 1, (eq(C, D) && !eq(C, B) && !eq(D, A)) ? D : P);
      put(x * 2 + 1, y * 2 + 1, (eq(B, C) && !eq(B, A) && !eq(C, D)) ? C : P);
    }
  }
}

void ScaleRgb24x2(uint32_t mode, const uint8_t* src, uint8_t* dst, uint16_t w, uint16_t h)
{
  const size_t dstW = (size_t)w * 2u;
  auto put = [&](int x, int y, const uint8_t* p) { memcpy(dst + ((size_t)y * dstW + (size_t)x) * 3u, p, 3); };
  if (mode == 2)
  {
    auto at = [&](int x, int y) { return src + ((size_t)y * w + (size_t)x) * 3u; };
    auto eq = [](const uint8_t* a, const uint8_t* b) { return a[0] == b[0] && a[1] == b[1] && a[2] == b[2]; };
    Scale2xGeneric(at, eq, put, w, h);
    return;
  }
  for (int y = 0; y < (int)h; ++y)
    for (int x = 0; x < (int)w; ++x)
    {
      const uint8_t* p = src + ((size_t)y * w + (size_t)x) * 3u;
      put(x * 2, y * 2, p);
      put(x * 2 + 1, y * 2, p);
      put(x * 2, y * 2 + 1, p);
      put(x * 2 + 1, y * 2 + 1, p);
    }
}

void ScaleRgb565x2(uint32_t mode, const uint16_t* src, uint16_t* dst, uint16_t w, uint16_t h)
{
  const size_t dstW = (size_t)w * 2u;
  auto put = [&](int x, int y, uint16_t v) { dst[(size_t)y * dstW + (size_t)x] = v; };
  if (mode == 2)
  {
    auto at = [&](int x, int y) { return src[(size_t)y * w + (size_t)x]; };
    auto eq = [](uint16_t a, uint16_t b) { return a == b; };
    Scale2xGeneric(at, eq, put, w, h);
    return;
  }
  for (int y = 0; y < (int)h; ++y)
    for (int x = 0; x < (int)w; ++x)
    {
      const uint16_t v = src[(size_t)y * w + (size_t)x];
      put(x * 2, y * 2, v);
      put(x * 2 + 1, y * 2, v);
      put(x * 2, y * 2 + 1, v);
      put(x * 2 + 1, y * 2 + 1, v);
    }
}
}  // namespace

void DMD::ZeDMDThread()
{
  auto decoder = std::make_unique<OutputDecoder>();
  Config* const pConfig = Config::GetInstance();
  decoder->showNotColorized = pConfig->IsShowNotColorizedFrames();
  decoder->excludeColorized = pConfig->IsExcludeColorizedFramesForZeDMD();
  decoder->targetWidth = m_pZeDMD->GetWidth();
  const int roundedCorners = pConfig->GetRoundedCorners();
  uint16_t width = 0;
  uint16_t height = 0;
  std::vector<uint16_t> rgb565(DMDUTIL_MAX_FRAME_PIXELS);
  const uint32_t scalerMode = pConfig->GetScalerMode();
  const uint16_t panelWidth = (uint16_t)m_pZeDMD->GetWidth();
  const uint16_t panelHeight = (uint16_t)m_pZeDMD->GetHeight();
  std::vector<uint8_t> scaledRgb24;
  bool scaling = false;

  OutputLoop(false, 0, nullptr,
             [&](uint8_t slot, uint16_t position, uint16_t skipped)
             {
               if (skipped > 0)
                 Log(DMDUtil_LogLevel_INFO, "ZeDMD: Skipping %d frame(s) before position %d", skipped, position);

               const Update& update = *m_pUpdateBufferQueue[slot];
               if (DecodeUpdate(update, *decoder) == DecodedKind::None) return;

               if (decoder->width != width || decoder->height != height)
               {
                 Log(DMDUtil_LogLevel_INFO, "ZeDMD: Change frame size from %dx%d to %dx%d", width, height,
                     decoder->width, decoder->height);
                 width = decoder->width;
                 height = decoder->height;
                 // A scaler fills a panel twice the frame's size itself instead of leaving it to ZeDMD
                 scaling = scalerMode != 0 && width > 0 && height > 0 && panelWidth == width * 2 &&
                           panelHeight == height * 2;
                 if (scaling) scaledRgb24.resize((size_t)panelWidth * panelHeight * 3);
                 m_pZeDMD->SetFrameSize(scaling ? panelWidth : width, scaling ? panelHeight : height);
               }

               Log(DMDUtil_LogLevel_DEBUG, "ZeDMD: Render frame buffer position %d at real buffer position %d",
                   position, slot);

               if (scaling)
               {
                 if (decoder->rgb565Source)
                 {
                   ScaleRgb565x2(scalerMode, update.segData, rgb565.data(), width, height);
                   ApplyRoundedCornersRGB565(rgb565.data(), panelWidth, panelHeight, roundedCorners);
                   m_pZeDMD->RenderRgb565(rgb565.data());
                 }
                 else
                 {
                   ScaleRgb24x2(scalerMode, decoder->rgb24, scaledRgb24.data(), width, height);
                   ApplyRoundedCornersRGB24(scaledRgb24.data(), panelWidth, panelHeight, roundedCorners);
                   m_pZeDMD->RenderRgb888(scaledRgb24.data());
                 }
               }
               else if (decoder->rgb565Source)
               {
                 memcpy(rgb565.data(), update.segData, (size_t)width * height * sizeof(uint16_t));
                 ApplyRoundedCornersRGB565(rgb565.data(), width, height, roundedCorners);
                 m_pZeDMD->RenderRgb565(rgb565.data());
               }
               else
               {
                 ApplyRoundedCornersRGB24(decoder->rgb24, width, height, roundedCorners);
                 m_pZeDMD->RenderRgb888(decoder->rgb24);
               }
             });
}

#ifdef DMDUTIL_WITH_PIN2DMD
void DMD::PIN2DMDThread()
{
  auto decoder = std::make_unique<OutputDecoder>();
  Config* const pConfig = Config::GetInstance();
  decoder->showNotColorized = pConfig->IsShowNotColorizedFrames();
  decoder->excludeColorized = pConfig->IsExcludeColorizedFramesForPIN2DMD();
  decoder->targetWidth = m_PIN2DMDWidth;
  const int roundedCorners = pConfig->GetRoundedCorners();
  const uint16_t targetWidth = m_PIN2DMDWidth;
  const uint16_t targetHeight = m_PIN2DMDHeight;
  std::vector<uint8_t> scaled((size_t)targetWidth * targetHeight * 3);
  std::vector<uint8_t> temp;

  OutputLoop(false, 0, nullptr,
             [&](uint8_t slot, uint16_t, uint16_t)
             {
               if (DecodeUpdate(*m_pUpdateBufferQueue[slot], *decoder) == DecodedKind::None) return;
               if (!ScaleFrameToTarget(decoder->rgb24, decoder->width, decoder->height, scaled.data(), targetWidth,
                                       targetHeight, 24, temp))
                 return;
               ApplyRoundedCornersRGB24(scaled.data(), targetWidth, targetHeight, roundedCorners);
               PIN2DMDRenderRaw(targetWidth, targetHeight, scaled.data(), 1);
             });
}
#endif

#ifdef DMDUTIL_WITH_PINDMD3
void DMD::PinDMD3Thread()
{
  auto decoder = std::make_unique<OutputDecoder>();
  Config* const pConfig = Config::GetInstance();
  decoder->showNotColorized = pConfig->IsShowNotColorizedFrames();
  decoder->excludeColorized = pConfig->IsExcludeColorizedFramesForPinDMD3();
  decoder->targetWidth = (uint16_t)m_pPinDMD3->GetWidth();
  const int roundedCorners = pConfig->GetRoundedCorners();
  const uint16_t targetWidth = (uint16_t)m_pPinDMD3->GetWidth();
  const uint16_t targetHeight = (uint16_t)m_pPinDMD3->GetHeight();
  std::vector<uint8_t> scaled((size_t)targetWidth * targetHeight * 3);
  std::vector<uint8_t> temp;

  OutputLoop(false, 0, nullptr,
             [&](uint8_t slot, uint16_t, uint16_t)
             {
               if (DecodeUpdate(*m_pUpdateBufferQueue[slot], *decoder) == DecodedKind::None) return;
               if (!ScaleFrameToTarget(decoder->rgb24, decoder->width, decoder->height, scaled.data(), targetWidth,
                                       targetHeight, 24, temp))
                 return;
               ApplyRoundedCornersRGB24(scaled.data(), targetWidth, targetHeight, roundedCorners);
               m_pPinDMD3->UpdateRGB24(scaled.data());
             });
}
#endif

#ifdef DMDUTIL_WITH_PIXELCADE
void DMD::PixelcadeDMDThread()
{
  auto decoder = std::make_unique<OutputDecoder>();
  Config* const pConfig = Config::GetInstance();
  decoder->showNotColorized = pConfig->IsShowNotColorizedFrames();
  decoder->excludeColorized = pConfig->IsExcludeColorizedFramesForPixelcade();
  decoder->targetWidth = (uint16_t)m_pPixelcadeDMD->GetWidth();
  const int roundedCorners = pConfig->GetRoundedCorners();
  const uint16_t targetWidth = (uint16_t)m_pPixelcadeDMD->GetWidth();
  const uint16_t targetHeight = (uint16_t)m_pPixelcadeDMD->GetHeight();
  const size_t targetLength = (size_t)targetWidth * targetHeight;
  const bool isV2 = m_pPixelcadeDMD->GetIsV2();
  std::vector<uint8_t> scaled(targetLength * 3);
  std::vector<uint16_t> rgb565(targetLength);
  std::vector<uint8_t> temp;

  OutputLoop(false, 0, nullptr,
             [&](uint8_t slot, uint16_t, uint16_t)
             {
               const Update& update = *m_pUpdateBufferQueue[slot];
               if (DecodeUpdate(update, *decoder) == DecodedKind::None) return;

               if (decoder->rgb565Source)
               {
                 if (!ScaleFrameToTarget((const uint8_t*)update.segData, decoder->width, decoder->height,
                                         (uint8_t*)rgb565.data(), targetWidth, targetHeight, 16, temp))
                   return;
               }
               else
               {
                 if (!ScaleFrameToTarget(decoder->rgb24, decoder->width, decoder->height, scaled.data(), targetWidth,
                                         targetHeight, 24, temp))
                   return;
                 if (isV2)
                 {
                   ApplyRoundedCornersRGB24(scaled.data(), targetWidth, targetHeight, roundedCorners);
                   m_pPixelcadeDMD->UpdateRGB24(scaled.data());
                   return;
                 }
                 Rgb888ToRgb565(scaled.data(), rgb565.data(), targetLength);
               }

               ApplyRoundedCornersRGB565(rgb565.data(), targetWidth, targetHeight, roundedCorners);
               m_pPixelcadeDMD->Update(rgb565.data());
             });
}
#endif

#ifdef DMDUTIL_WITH_DMDEXTSTREAM
void DMD::DMDExtStreamThread()
{
  auto decoder = std::make_unique<OutputDecoder>();
  Config* const pConfig = Config::GetInstance();
  decoder->showNotColorized = pConfig->IsShowNotColorizedFrames();
  decoder->excludeColorized = pConfig->IsExcludeColorizedFramesForDMDExtStream();
  decoder->targetWidth = (uint16_t)m_pDMDExtStream->GetWidth();
  const int roundedCorners = pConfig->GetRoundedCorners();
  const uint16_t targetWidth = (uint16_t)m_pDMDExtStream->GetWidth();
  const uint16_t targetHeight = (uint16_t)m_pDMDExtStream->GetHeight();
  const size_t targetLength = (size_t)targetWidth * targetHeight;
  std::vector<uint8_t> scaled(targetLength * 3);
  std::vector<uint8_t> scaledIndexed(targetLength);
  std::vector<uint8_t> temp;
  uint8_t palette16[16 * 3];

  OutputLoop(
      false, m_pDMDExtStream->IsRetryEnabled() ? 1000 : 0,
      [&]()
      {
        if (!m_pDMDExtStream->IsConnected()) m_pDMDExtStream->EnsureConnected();
      },
      [&](uint8_t slot, uint16_t, uint16_t)
      {
        const DecodedKind kind = DecodeUpdate(*m_pUpdateBufferQueue[slot], *decoder);
        if (kind == DecodedKind::None) return;
        if (!m_pDMDExtStream->EnsureConnected()) return;

        m_pDMDExtStream->SetGameName(m_romName);

        if (kind == DecodedKind::Indexed && (decoder->depth == 2 || decoder->depth == 4))
        {
          if (!ScaleFrameToTarget(decoder->indexed, decoder->width, decoder->height, scaledIndexed.data(), targetWidth,
                                  targetHeight, 8, temp))
            return;

          if (decoder->mode == Mode::SerumV1 || decoder->mode == Mode::Vni)
          {
            memset(palette16, 0, sizeof(palette16));
            memcpy(palette16, decoder->palette, (size_t)(1 << decoder->depth) * 3);
            m_pDMDExtStream->SendColoredGray4(scaledIndexed.data(), palette16);
          }
          else
          {
            m_pDMDExtStream->SendColor(decoder->r, decoder->g, decoder->b);
            m_pDMDExtStream->SendGray(scaledIndexed.data(), decoder->depth);
          }
          return;
        }

        if (!ScaleFrameToTarget(decoder->rgb24, decoder->width, decoder->height, scaled.data(), targetWidth,
                                targetHeight, 24, temp))
          return;
        ApplyRoundedCornersRGB24(scaled.data(), targetWidth, targetHeight, roundedCorners);
        m_pDMDExtStream->SendRgb24(scaled.data());
      });
}
#endif

}  // namespace DMDUtil
