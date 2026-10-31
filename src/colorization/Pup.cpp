#include <algorithm>
#include <cmath>
#include <chrono>
#include <cstring>
#include <thread>

#include "DMDInternal.h"
#include "DMDUtil/Config.h"
#include "DMDUtil/Logger.h"
#include "FrameUtil.h"
#include "OutputFilters.h"
#include "pupdmd.h"

namespace DMDUtil
{

void PUPDMDCALLBACK PUPDMDLogCallback(const char* format, va_list args, const void* pUserData)
{
  char buffer[1024];
  vsnprintf(buffer, sizeof(buffer), format, args);

  Log(DMDUtil_LogLevel_INFO, "%s", buffer);
}

// ---------------------------------------------------------------------------------------------
// PuP capture for RGB frames (FlexDMD, UltraDMD and other script-driven DMDs).
//
// On Windows those frames reach PinUP through DmdExt, whose PinUP output only accepts 2/4-shade
// frames at a fixed 128x32. DmdExt therefore turns an RGB frame into 16 shades by HSL lightness,
// and when the frame is larger (FlexDMD's 256x64 "HQ" mode) scales it to 128x32 with WPF:
//   RGB24 -> ImageUtil.ConvertToGray(numColors 16)          per pixel: round(L * 15)
//         -> TransformGray: ConvertFromGray4(hue 0, sat 1)  gray -> red-hued Bgr32 bitmap
//                           TransformedBitmap(ScaleTransform 0.5)  2:1 area average (WIC Fant)
//                           ConvertToGray4(bmp)              per pixel: round(L * 15)
// PinUP then hashes that frame on a black-to-OrangeRed ramp, which is what every PupCapture
// bitmap authored this way contains (16 colours, red = 17 * shade). Reproducing the chain lets
// the indexed matcher, loaded at depth 4, match those captures. C# Math.Round rounds half to even
// and the doubles below follow the C# operation order exactly, so the exact .5 cases (common: L*15
// of an averaged block lands on x.5 whenever the four shades sum to 2 mod 4) resolve the same way.
// The one guess is WIC's rounding of a per-channel average that is not a whole number; it only
// matters in blocks that mix shades below and above 8, and round-half-up is used.
// ---------------------------------------------------------------------------------------------
static double DmdExtLightness(uint8_t red, uint8_t green, uint8_t blue)
{
  const double r = red / 255.0;
  const double g = green / 255.0;
  const double b = blue / 255.0;
  return (std::max(std::max(r, g), b) + std::min(std::min(r, g), b)) / 2.0;
}

// std::nearbyint rounds half to even in the default rounding mode, as C#'s Math.Round(double) does
static uint8_t DmdExtShade(double lightness) { return (uint8_t)std::nearbyint(lightness * 15.0); }

static double DmdExtColorCalc(double c, double t1, double t2)
{
  if (c < 0) c += 1.0;
  if (c > 1) c -= 1.0;
  if (6.0 * c < 1.0) return t1 + (t2 - t1) * 6.0 * c;
  if (2.0 * c < 1.0) return t2;
  if (3.0 * c < 2.0) return t1 + (t2 - t1) * (2.0 / 3.0 - c) * 6.0;
  return t1;
}

// ColorUtil.HslToRgb(hue 0, saturation 1, luminosity shade/15), as ConvertFromGray4 calls it.
static void DmdExtShadeToRgb(uint8_t shade, uint8_t& red, uint8_t& green, uint8_t& blue)
{
  const double luminosity = 1.0 * shade / 15;
  const double saturation = 1.0;
  const double t2 =
      (luminosity < 0.5) ? luminosity * (1.0 + saturation) : (luminosity + saturation) - (luminosity * saturation);
  const double t1 = 2.0 * luminosity - t2;
  const double th = 0.0 / 6.0;
  red = (uint8_t)std::nearbyint(DmdExtColorCalc(th + (1.0 / 3.0), t1, t2) * 255.0);
  green = (uint8_t)std::nearbyint(DmdExtColorCalc(th, t1, t2) * 255.0);
  blue = (uint8_t)std::nearbyint(DmdExtColorCalc(th - (1.0 / 3.0), t1, t2) * 255.0);
}

// RGB24 frame of 128x32 or 256x64 -> the 128x32 16-shade frame DmdExt hands PinUP. Returns false
// for any other size (DmdExt would letterbox those; no capture set seen so far needs it).
static bool DmdExtPinUpFrame(const uint8_t* rgb, uint16_t width, uint16_t height, uint8_t* out128x32)
{
  if (width == 128 && height == 32)
  {
    for (int i = 0; i < 128 * 32; i++)
      out128x32[i] = DmdExtShade(DmdExtLightness(rgb[i * 3], rgb[i * 3 + 1], rgb[i * 3 + 2]));
    return true;
  }
  if (width == 256 && height == 64)
  {
    uint8_t shadeRgb[16][3];
    for (uint8_t s = 0; s < 16; s++) DmdExtShadeToRgb(s, shadeRgb[s][0], shadeRgb[s][1], shadeRgb[s][2]);
    for (int y = 0; y < 32; y++)
    {
      for (int x = 0; x < 128; x++)
      {
        unsigned int sum[3] = {0, 0, 0};
        for (int dy = 0; dy < 2; dy++)
        {
          for (int dx = 0; dx < 2; dx++)
          {
            const int p = ((y * 2 + dy) * 256 + (x * 2 + dx)) * 3;
            const uint8_t shade = DmdExtShade(DmdExtLightness(rgb[p], rgb[p + 1], rgb[p + 2]));
            sum[0] += shadeRgb[shade][0];
            sum[1] += shadeRgb[shade][1];
            sum[2] += shadeRgb[shade][2];
          }
        }
        out128x32[y * 128 + x] = DmdExtShade(
            DmdExtLightness((uint8_t)((sum[0] + 2) / 4), (uint8_t)((sum[1] + 2) / 4), (uint8_t)((sum[2] + 2) / 4)));
      }
    }
    return true;
  }
  return false;
}

// A 64-row indexed frame (192x64 or 256x64) -> the 128x32 the PuPCapture trigger hashes are
// authored at, with the same 2x2 box sum and quantization as the original Windows capture tool
// (Freezy's DmdExt). 256x64 fills 128x32; 192x64 becomes 96x32 centred (ofsX 16). For a 4-level
// DMD the 0..12 box sum is quantized at 3/6/11, not round-to-nearest 2/6/10: anything else leaves
// glyph edges one level off and the hash never matches (Baywatch; vpinball PR #3527).
static void DmdExtScaleDown64(const uint8_t* src, uint16_t width, uint8_t depth, uint8_t* out128x32)
{
  memset(out128x32, 0, 128 * 32);
  const bool fourLevel = (depth == 2);
  const unsigned int ofsX = (128 - (width / 2)) / 2;
  for (unsigned int y = 0; y < 32; y++)
  {
    for (unsigned int x = 0; x < (unsigned int)(width / 2); x++)
    {
      const unsigned int px = x * 2;
      const unsigned int py = y * (2u * width);
      const uint32_t sum = (uint32_t)src[py + px] + src[py + px + 1] + src[py + px + width] + src[py + px + width + 1];
      uint8_t value;
      if (fourLevel)
        value = sum < 3 ? 0 : sum < 6 ? 1 : sum < 11 ? 2 : 3;
      else
        value = (uint8_t)((sum + 2) >> 2);
      out128x32[y * 128 + ofsX + x] = value;
    }
  }
}

void DMD::PupDMDThread()
{
  std::vector<uint8_t> renderBuffer(DMDUTIL_MAX_FRAME_PIXELS, 0);
  std::vector<uint8_t> rgbRenderBuffer;  // last RGB frame matched, so an unchanged one is not hashed again
  uint8_t palette[192] = {0};
  char name[DMDUTIL_MAX_NAME_SIZE] = {0};
  // Highest index this ROM has driven so far, and the depth the trigger set is loaded at. Both reset
  // when the ROM changes. See loadAt below for why the declared frame depth cannot be used directly.
  uint8_t observedMaxIndex = 0;
  uint8_t loadedDepth = 0;

  // The pack folder the captures live under: the host's PuP name when it set one (the table-side ROM
  // alias can differ from PinMAME's resolved ROM, VPMAlias.txt / cPuPPack), else the ROM name.
  auto pupName = [&]() -> const char* { return (m_pupName[0] != '\0') ? m_pupName : m_romName; };

  auto loadAt = [&](uint8_t depth)
  {
    delete m_pPUPDMD;
    m_pPUPDMD = nullptr;
    if (m_pupVideosPath[0] == '\0')
      snprintf(m_pupVideosPath, sizeof(m_pupVideosPath), "%s", Config::GetInstance()->GetPUPVideosPath());
    m_pPUPDMD = new PUPDMD::DMD();
    m_pPUPDMD->SetLogCallback(PUPDMDLogCallback, nullptr);
    if (!m_pPUPDMD->Load(m_pupVideosPath, name, depth))
    {
      delete m_pPUPDMD;
      m_pPUPDMD = nullptr;
    }
    loadedDepth = depth;
  };

  OutputLoop(true, 0, nullptr,
             [&](uint8_t slot, uint16_t, uint16_t)
             {
               const Update& update = *m_pUpdateBufferQueue[slot];

               if (strcmp(pupName(), name) != 0)
               {
                 snprintf(name, sizeof(name), "%s", pupName());
                 observedMaxIndex = 0;
                 loadedDepth = 0;
                 rgbRenderBuffer.clear();
                 delete m_pPUPDMD;
                 m_pPUPDMD = nullptr;
               }

               if (!Config::GetInstance()->IsPUPCapture() || name[0] == '\0' || !update.hasData) return;

               const uint16_t width = update.width;
               const uint16_t height = update.height;
               const size_t length = (size_t)width * height;
               if (length == 0 || length > DMDUTIL_MAX_FRAME_PIXELS) return;

               // RGB frames: see DmdExtPinUpFrame. Only true RGB (depth 24); a tinted gray RGB24 update
               // is a different Windows path (Render_4_Shades) and is not reproduced here.
               if (update.mode == Mode::RGB24 && update.depth == 24)
               {
                 if (!((width == 128 && height == 32) || (width == 256 && height == 64))) return;
                 // DmdExt always hands PinUP 16 shades for an RGB source, so the set is loaded at depth 4
                 if (loadedDepth != 4)
                 {
                   Log(DMDUtil_LogLevel_INFO, "Loading PuP capture triggers for %s at depth 4 for %dx%d RGB frames",
                       name, width, height);
                   loadAt(4);
                   rgbRenderBuffer.clear();
                 }
                 if (!m_pPUPDMD) return;

                 const size_t rgbLength = length * 3;
                 if (rgbRenderBuffer.size() == rgbLength && memcmp(rgbRenderBuffer.data(), update.data, rgbLength) == 0)
                   return;
                 rgbRenderBuffer.assign(update.data, update.data + rgbLength);

                 uint8_t pinUpFrame[128 * 32];
                 if (!DmdExtPinUpFrame(rgbRenderBuffer.data(), width, height, pinUpFrame)) return;
                 const uint16_t triggerID = m_pPUPDMD->MatchIndexed(pinUpFrame, 128, 32);
                 if (triggerID > 0) HandleTrigger(triggerID);
                 return;
               }

               if (update.mode != Mode::Data || update.depth == 24) return;

               // Load the trigger set against the index range the ROM actually drives, not the declared
               // depth. PUPDMD::Load thresholds each capture's red channel into indices at the depth it is
               // given, and MatchIndexed hashes those against the raw frame, so the depth must describe the
               // frame's content. layout.depth does not: libpinmame sets 4 for every GEN_GTS3 / GEN_SAM /
               // GEN_SPA / GEN_ALVG(_DMD2) ROM by generation, because those *can* drive 16 shades. A 4-shade
               // ROM there emits {0,1,2,3} while its captures load as {0,1,4,15}: every trigger loads and none
               // fires, silently (Barb Wire: 61 triggers, declared depth 4, frames only ever {0,1,2,3}).
               // Nothing loads until the DMD shows something (black says nothing about the scale), and a ROM
               // that later drives an index above 3 is reloaded at its declared depth.
               for (size_t i = 0; i < length; i++)
                 if (update.data[i] > observedMaxIndex) observedMaxIndex = update.data[i];
               const uint8_t declaredDepth = (uint8_t)update.depth;
               const uint8_t depth = (declaredDepth > 2 && observedMaxIndex <= 3) ? 2 : declaredDepth;
               if (observedMaxIndex > 0 && depth != loadedDepth)
               {
                 Log(DMDUtil_LogLevel_INFO,
                     "Loading PuP capture triggers for %s at depth %d (declared %d, highest index driven so far %d)",
                     name, depth, declaredDepth, observedMaxIndex);
                 loadAt(depth);
               }

               if (!m_pPUPDMD) return;
               if (memcmp(renderBuffer.data(), update.data, length) == 0) return;
               memcpy(renderBuffer.data(), update.data, length);

               // The trigger hashes are stored at 128x32, and MatchIndexed skips any trigger whose stored
               // size differs from the size passed, so every frame is normalized to 128x32 and matched as such.
               uint8_t scaledBuffer[128 * 32];
               if (width == 128 && height == 32)
                 memcpy(scaledBuffer, renderBuffer.data(), 128 * 32);
               else if (width == 128 && height == 16)
                 FrameUtil::Helper::CenterIndexed(scaledBuffer, 128, 32, renderBuffer.data(), 128, 16);
               else if (width <= 256 && height == 64)
                 DmdExtScaleDown64(renderBuffer.data(), width, depth, scaledBuffer);
               else
                 return;

               uint16_t triggerID = 0;
               if (Config::GetInstance()->IsPUPExactColorMatch())
               {
                 triggerID = m_pPUPDMD->MatchIndexed(scaledBuffer, 128, 32);
               }
               else
               {
                 UpdatePalette(palette, depth, 255, 69, 0);

                 std::vector<uint8_t> frame(128 * 32 * 3);
                 for (size_t i = 0; i < 128 * 32; i++) memcpy(&frame[i * 3], &palette[scaledBuffer[i] * 3], 3);
                 triggerID = m_pPUPDMD->Match(frame.data(), 128, 32, false);
               }

               if (triggerID > 0) HandleTrigger(triggerID);
             });
}

}  // namespace DMDUtil
