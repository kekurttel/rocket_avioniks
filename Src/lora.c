/* LoRa telemetry uplink driver. See Inc/lora.h for the module contract, the
 * wire format and the flight-safety rules. Pure C, no heap, no libc float
 * parsing, no variable-length anything.
 *
 * The file compiles TWICE, from the same text:
 *   - target  (LORA_WITH_HAL defined by CMakeLists.txt): adds the one bounded
 *             HAL_UART_Transmit() in lora_poll().
 *   - host    (tests/host/test_lora_host.c): the HAL block is compiled out and
 *             lora_poll() falls back to a simulated transmit, so cadence, seq,
 *             the SIT/SUT gate and the frame builder are all unit-tested with
 *             no board and no HAL.
 * Everything above the HAL block is byte-identical between the two. */
#include "lora.h"

#include <math.h> /* isfinite: NaN/inf daraltmadan ONCE yakalanir */

/* ---------------- explicit little-endian byte writers ----------------
 * One function per width, each taking the destination offset. Every write is
 * masked to 8 bits, so the wire image never depends on the host's int width,
 * endianness or struct padding. */

static void put_u8(uint8_t *o, size_t off, uint8_t v)
{
  o[off] = (uint8_t)v;
}

static void put_u16(uint8_t *o, size_t off, uint16_t v)
{
  o[off] = (uint8_t)(v & 0xFFu);
  o[off + 1u] = (uint8_t)((v >> 8) & 0xFFu);
}

static void put_i16(uint8_t *o, size_t off, int16_t v)
{
  /* Two's complement is mandated by C99 6.2.6.2, so the shift is arithmetic
   * on every compiler this project uses; the explicit cast documents intent
   * and avoids an implementation-defined conversion in the byte math. */
  uint16_t u = (uint16_t)(int32_t)v;
  put_u16(o, off, u);
}

static void put_u32(uint8_t *o, size_t off, uint32_t v)
{
  o[off] = (uint8_t)(v & 0xFFu);
  o[off + 1u] = (uint8_t)((v >> 8) & 0xFFu);
  o[off + 2u] = (uint8_t)((v >> 16) & 0xFFu);
  o[off + 3u] = (uint8_t)((v >> 24) & 0xFFu);
}

static void put_i32(uint8_t *o, size_t off, int32_t v)
{
  uint32_t u = (uint32_t)v;
  put_u32(o, off, u);
}

/* ---------------- link state (target and host alike) ---------------- */
static bool s_ready = false;
static uint16_t s_seq = 0;
static uint32_t s_next_ms = 0; /* next send deadline, unsigned-safe */
static uint32_t s_ok = 0;
static uint32_t s_drop = 0; /* SADECE build reddi (lora_build_air sigmazsa) */
static uint32_t s_txerr = 0; /* HAL_UART_Transmit timeout/hata (kablo/modul) */
static uint32_t s_txconsec = 0; /* ust uste TX hatasi (basarida sifirlanir) */
static uint32_t s_reinit = 0; /* huart1 re-init sayisi */
/* TX-oncesi AUX-atlama: vade dolmusken AUX dusuk gorulup paket havaya
 * verilmeden atlandiysa +1. drop (build) ve txerr (UART) toplamlarina
 * KATILMAZ; ayri durust sayac (getter: lora_airbusy_count). */
static uint32_t s_airbusy = 0;
/* Su anki link baud'u: her zaman FALLBACK (9600). 57600 yukseltme
 * KALDIRILDI; boot da re-init de huart1'i 9600'den ayirmaz. Re-init bu
 * baud'u kullanir; yazisiz degismez. */
static uint32_t s_link_baud = LORA_LINK_BAUD_FALLBACK;
static uint8_t s_tx[LORA_AIR_LEN]; /* internal fallback buffer */
#if !defined(LORA_WITH_HAL)
/* Host AUX-seviye enjeksiyonu (hedefte YOK): true = AUX dusuk (modul
 * mesgul). Hedefteki gercek pin okumayla ayni karari surer; varsayilan
 * false. Uretim yolunda kullanilmaz. */
static bool s_host_aux_low = false;
#endif

/* TX basarisi: ardisik-hata sayaci sifirlanir (re-init esigi kirilir).
 * Hedef + host ortak, saf muhasebe, bekleme yok. */
static void lora_txok_note(void)
{
  s_txconsec = 0u;
}

/* TX-hatasi muhasebesi (hedef + host ortak): s_txerr her hatada +1.
 * Ardisik sayac LORA_TXERR_REINIT_K'ye ulasinca sifirlanir ve true doner;
 * cagiran huart1'i yeniden baslatir (hedef) ya da re-init sayacini artirir
 * (host test). Saf karar, donanim yok, bekleme yok, vade/seq degismez. */
static bool lora_txerr_note(void)
{
  s_txerr++;
  s_txconsec++;
  if (s_txconsec >= LORA_TXERR_REINIT_K)
  {
    s_txconsec = 0u;
    return true;
  }
  return false;
}

uint8_t lora_crc(const uint8_t *d, size_t n)
{
  uint8_t s = 0;
  if (d == NULL)
  {
    return 0;
  }
  for (size_t i = 0; i < n; i++)
  {
    s = (uint8_t)(s + d[i]);
  }
  return s;
}

size_t lora_build(uint8_t *out, size_t cap, const lora_frame_t *in)
{
  if (out == NULL || in == NULL || cap < LORA_FRAME_LEN)
  {
    return 0; /* nothing written: the caller's buffer is untouched */
  }
  put_u8(out, LORA_OFF_MAGIC, LORA_MAGIC);
  put_u8(out, LORA_OFF_VER, LORA_VER);
  put_u16(out, LORA_OFF_SEQ, in->seq);
  put_u32(out, LORA_OFF_T_MS, in->t_ms);
  put_u16(out, LORA_OFF_FLIGHT_STATE, in->flight_state);
  put_u16(out, LORA_OFF_EK7_MODE, in->ek7_mode);
  put_i32(out, LORA_OFF_ALT_DM, in->alt_dm);
  put_i16(out, LORA_OFF_VEL_CMS, in->vel_cms);
  put_u16(out, LORA_OFF_SATS, in->sats);
  put_i32(out, LORA_OFF_LAT_E6, in->lat_e6);
  put_i32(out, LORA_OFF_LON_E6, in->lon_e6);
  put_u16(out, LORA_OFF_BATT_MV, in->batt_mv);
  put_u8(out, LORA_OFF_FLAGS, in->flags);
  put_u16(out, LORA_OFF_TILT, in->tilt_ddeg);
  put_i16(out, LORA_OFF_LEANX, in->lean_x);
  put_i16(out, LORA_OFF_LEANY, in->lean_y);
  put_u8(out, LORA_OFF_CRC, lora_crc(out, LORA_CRC_OFF));
  return LORA_FRAME_LEN;
}

#if LORA_FIXED_HDR
size_t lora_build_air(uint8_t *out, size_t cap, const lora_frame_t *in)
{
  if (out == NULL || in == NULL || cap < LORA_AIR_LEN)
  {
    return 0;
  }
  out[0] = LORA_FIXED_HDR_0;
  out[1] = LORA_FIXED_HDR_1;
  out[2] = LORA_FIXED_HDR_2;
  if (lora_build(out + 3u, cap - 3u, in) != LORA_FRAME_LEN)
  {
    return 0;
  }
  return LORA_AIR_LEN;
}
#else
size_t lora_build_air(uint8_t *out, size_t cap, const lora_frame_t *in)
{
  if (out == NULL || in == NULL || cap < LORA_FRAME_LEN)
  {
    return 0;
  }
  return lora_build(out, cap, in);
}
#endif

static uint16_t read_u16_le(const uint8_t *f, size_t off)
{
  return (uint16_t)((uint16_t)f[off] | ((uint16_t)f[off + 1u] << 8));
}

static uint32_t read_u32_le(const uint8_t *f, size_t off)
{
  return (uint32_t)f[off] | ((uint32_t)f[off + 1u] << 8) |
         ((uint32_t)f[off + 2u] << 16) | ((uint32_t)f[off + 3u] << 24);
}

bool lora_verify(const uint8_t *f, size_t n)
{
  size_t expected;
  if (f == NULL || n < 2u || f[0] != LORA_MAGIC)
  {
    return false;
  }
  switch (f[1])
  {
    case LORA_VER_V1: expected = LORA_V1_FRAME_LEN; break;
    case LORA_VER_V2: expected = LORA_V2_FRAME_LEN; break;
    case LORA_VER_V3: expected = LORA_V3_FRAME_LEN; break;
    default: return false;
  }
  if (n != expected)
  {
    return false;
  }
  /* Every supported version stores the additive CRC as its final byte. */
  return lora_crc(f, n - 1u) == f[n - 1u];
}

static int16_t dm_to_m_compat(int32_t dm)
{
  int64_t m;
  if (dm == LORA_ALT_DM_INVALID)
  {
    return LORA_ALT_INVALID;
  }
  m = (dm >= 0) ? ((int64_t)dm + 5) / 10 : ((int64_t)dm - 5) / 10;
  if (m < -32767 || m > 32767)
  {
    return LORA_ALT_INVALID; /* legacy whole-metre view cannot represent it */
  }
  return (int16_t)m;
}

bool lora_parse(const uint8_t *f, size_t n, lora_frame_t *out)
{
  uint8_t ver;
  int16_t legacy_alt;
  if (out == NULL || !lora_verify(f, n))
  {
    return false;
  }
  ver = f[1];
  out->seq = read_u16_le(f, 2u);
  out->t_ms = read_u32_le(f, 4u);
  out->flight_state = read_u16_le(f, 8u);
  out->ek7_mode = read_u16_le(f, 10u);

  if (ver == LORA_VER_V3)
  {
    out->alt_dm = (int32_t)read_u32_le(f, 12u);
    out->alt_m = dm_to_m_compat(out->alt_dm);
    out->vel_cms = (int16_t)read_u16_le(f, 16u);
    out->sats = read_u16_le(f, 18u);
    out->lat_e6 = (int32_t)read_u32_le(f, 20u);
    out->lon_e6 = (int32_t)read_u32_le(f, 24u);
    out->batt_mv = read_u16_le(f, 28u);
    out->flags = f[30u];
    out->tilt_ddeg = read_u16_le(f, 31u);
    out->lean_x = (int16_t)read_u16_le(f, 33u);
    out->lean_y = (int16_t)read_u16_le(f, 35u);
    return true;
  }

  /* Legacy V1/V2 retain their original i16 whole-metre wire field and offsets. */
  legacy_alt = (int16_t)read_u16_le(f, 12u);
  out->alt_m = legacy_alt;
  out->alt_dm = (legacy_alt == LORA_ALT_INVALID)
                   ? LORA_ALT_DM_INVALID : (int32_t)legacy_alt * 10;
  out->vel_cms = (int16_t)read_u16_le(f, 14u);
  out->sats = read_u16_le(f, 16u);
  out->lat_e6 = (int32_t)read_u32_le(f, 18u);
  out->lon_e6 = (int32_t)read_u32_le(f, 22u);
  out->batt_mv = read_u16_le(f, 26u);
  out->flags = f[28u];
  if (ver == LORA_VER_V2)
  {
    out->tilt_ddeg = read_u16_le(f, 29u);
    out->lean_x = (int16_t)read_u16_le(f, 31u);
    out->lean_y = (int16_t)read_u16_le(f, 33u);
  }
  else
  {
    out->tilt_ddeg = LORA_TILT_INVALID;
    out->lean_x = 0;
    out->lean_y = 0;
  }
  return true;
}

/* Metre -> tam metre (int16). Gecerli aralik -32767..+32767 m; bu aralikta
 * yarim-uzakliktan yuvarlama (eski davranis aynen). Gecersiz/NaN/inf ya da
 * yuvarlaninca hedef araliga SIGMAYAN her deger LORA_ALT_INVALID (0x8000)
 * verir; doyurma YOK (kelepcelenmis 32767 gercek okumadan ayirt edilemezdi).
 * Esik +-32767.5'tir: 32767.0 gecerli (32767 olur), 32767.5 tasan ilk deger
 * gozcuye duser (32768 sigmaz). Daraltma noktasinda deger her zaman int16
 * araliginda oldugu icin float->int donusumu tanimsiz davranis uretemez. */
static int16_t m_to_m_i16(float m, bool valid)
{
  if (!valid || !isfinite(m) || m >= 32767.5f || m <= -32767.5f)
  {
    return LORA_ALT_INVALID;
  }
  /* |m| < 32767.5 oldugu icin +-0.5 sonrasi kesme her zaman -32767..+32767
   * bandinda kalir: daraltma tanimli. */
  return (int16_t)((m >= 0.0f) ? (m + 0.5f) : (m - 0.5f));
}

/* Metres -> signed decimetres (V3). This carries launch-relative altitude to
 * 0.1 m while retaining an explicit invalid sentinel and the backend's 100 km
 * physical envelope. Rounding is symmetric (half away from zero). */
static int32_t m_to_dm_i32(float m, bool valid)
{
  float dm;
  if (!valid || !isfinite(m) || m > 100000.0f || m < -100000.0f)
  {
    return LORA_ALT_DM_INVALID;
  }
  dm = m * 10.0f;
  if (!isfinite(dm) || dm >= 1000000.5f || dm <= -1000000.5f)
  {
    return LORA_ALT_DM_INVALID;
  }
  return (int32_t)((dm >= 0.0f) ? (dm + 0.5f) : (dm - 0.5f));
}

/* m/s -> cm/s (int16), truncation preserved from V2. Out-of-range,
 * non-finite, and overflowed values produce the explicit invalid sentinel. */
static int16_t m_to_cm_i16(float m, bool valid)
{
  float cm;
  if (!valid || !isfinite(m))
  {
    return LORA_I16_INVALID;
  }
  cm = m * 100.0f;
  if (!isfinite(cm) || cm >= 32768.0f || cm <= -32768.0f)
  {
    return LORA_I16_INVALID;
  }
  return (int16_t)cm;
}

void lora_frame_from_tlm(lora_frame_t *out, uint16_t seq, uint32_t t_ms,
                         const lora_tlm_t *in)
{
  if (out == NULL || in == NULL)
  {
    return;
  }
  out->seq = seq;
  out->t_ms = t_ms;
  out->flight_state = in->flight_state;
  out->ek7_mode = in->ek7_mode;
  out->alt_m = m_to_m_i16(in->alt_m, in->alt_valid); /* legacy/test view */
  out->alt_dm = m_to_dm_i32(in->alt_m, in->alt_valid);
  out->vel_cms = m_to_cm_i16(in->vel_mps, in->vel_valid);
  /* Satellites are ALWAYS passed through, fix or no fix: the ground
   * station watches sats climb 0 -> 5 -> 8 while the antenna hunts the
   * sky, long before the first fix. 0xFFFF stays reserved (input-side
   * sentinel only). Position stays fix-gated: without a fix lat/lon read
   * 0/0 ("no position") so a stale last-known fix can never ride along. */
  out->sats = in->sats;
  out->lat_e6 = in->gps_fix ? in->lat_e6 : LORA_I32_NOPOS;
  out->lon_e6 = in->gps_fix ? in->lon_e6 : LORA_I32_NOPOS;
  out->batt_mv = in->batt_mv;
  out->flags = in->flags;
  /* Egiklik: derece -> decidegree yuvarla (aralikta eski davranis aynen).
   * Gecerli aralik 0..180 derece; gecersiz/NaN/inf/MENZIL-DISI okuma
   * 0xFFFF + sifir yon verir (doyurma YOK: bozuk 250 dereceyi 1800'e
   * kelepcelemek "tam yatik" gibi gosterirdi). Daraltma noktasinda dd her
   * zaman 0..1800 bandindadir, o yuzden (uint16_t) donusumu tanimlidir. */
  if (!in->tilt_valid || !isfinite(in->tilt_deg))
  {
    out->tilt_ddeg = LORA_TILT_INVALID;
    out->lean_x = 0;
    out->lean_y = 0;
  }
  else
  {
    float dd = in->tilt_deg * 10.0f;
    /* Egilme yonu: float x1000; NaN/inf/menzil-disi -> 0 (doyurma YOK).
     * Esik +-1000.5: 1000.4 yuvarlayinca 1000'e SIGAR, 1000.5 tasinca
     * gozcuye duser. Daraltma noktasinda deger her zaman +-1000 bandinda
     * oldugu icin (int16_t) donusumu tanimlidir. Yaw referansi YOK. */
    float lx;
    float ly;
    if (!isfinite(dd) || dd < 0.0f || dd > 1800.0f)
    {
      out->tilt_ddeg = LORA_TILT_INVALID;
      out->lean_x = 0;
      out->lean_y = 0;
    }
    else
    {
      out->tilt_ddeg = (uint16_t)(dd + 0.5f);
      lx = in->lean_x * 1000.0f;
      ly = in->lean_y * 1000.0f;
      if (!isfinite(lx) || lx > 1000.5f || lx < -1000.5f)
      {
        out->lean_x = 0;
      }
      else
      {
        out->lean_x = (int16_t)(lx >= 0.0f ? (lx + 0.5f) : (lx - 0.5f));
      }
      if (!isfinite(ly) || ly > 1000.5f || ly < -1000.5f)
      {
        out->lean_y = 0;
      }
      else
      {
        out->lean_y = (int16_t)(ly >= 0.0f ? (ly + 0.5f) : (ly - 0.5f));
      }
    }
  }
}

uint32_t lora_interval_ms(uint16_t flight_state)
{
  /* FLIGHT_WAIT_FOR_LAUNCH == 2 in Inc/flight.h. Restated as a constant here
   * because lora.h stays free of flight.h (host tests compile it alone);
   * tests/host/test_lora_host.c pins the value against the real enum. */
  const uint16_t flight_wait_for_launch = 2u;
  uint32_t ms = (flight_state == flight_wait_for_launch)
                    ? LORA_SEND_INTERVAL_WAIT_MS
                    : LORA_SEND_INTERVAL_MS;
  /* Fixed 100 ms schedule (10 Hz) for pad and flight. AUX is sampled before
   * transmit; a busy module consumes the due slot rather than burst-replaying. */
  if (ms < LORA_SEND_GAP_MS)
  {
    ms = LORA_SEND_GAP_MS;
  }
  return ms;
}

bool lora_link_allowed(uint16_t ek7_mode)
{
  if (!LORA_ALLOW_SIT_SUT &&
      (ek7_mode == LORA_EK7MODE_SIT || ek7_mode == LORA_EK7MODE_SUT))
  {
    /* SIT/SUT stream the Ek-7 binary frames on USART2; stay off the air. */
    return false;
  }
  return true;
}

/* Both module images are deliberately the same 9600-UART / 4.8-kbps RF
 * setting. Keep the legacy HI symbol for ABI/test compatibility; it is not
 * selected by the production boot path. */
const uint8_t LORA_CFG_EXPECTED[LORA_CFG_LEN] = {
  0x00u, 0x00u, LORA_REG0_9600, LORA_REG1_IMAGE, LORA_REG2_CH,
  LORA_REG3_TRANSPARENT_BITS
};
const uint8_t LORA_CFG_EXPECTED_HI[LORA_CFG_LEN] = {
  0x00u, 0x00u, LORA_REG0_57600, LORA_REG1_IMAGE, LORA_REG2_CH,
  LORA_REG3_TRANSPARENT_BITS
};
/* Canonical persistent/RAM image shared with the Uno receiver. REG0=0x63:
 * 9600 8N1 module UART + 4.8 kbps RF data rate. */
const uint8_t LORA_CFG_COMMON[LORA_CFG_LEN] = {
  0x00u, 0x00u, 0x63u, 0x00u, 0x17u, 0x00u
};

bool lora_cfg_common_img(uint8_t *out8, bool dialog)
{
  uint8_t i;
  if (!dialog || out8 == NULL)
  {
    return false;
  }
  for (i = 0u; i < LORA_CFG_LEN; i++)
  {
    out8[i] = LORA_CFG_COMMON[i];
  }
  out8[6u] = 0x00u;
  out8[7u] = 0x00u;
  return true;
}

size_t lora_cfg_write_frame(uint8_t command, uint8_t addr, uint8_t len,
                            const uint8_t *data, uint8_t *out, size_t cap)
{
  /* E220 manual §7.1: BOTH write commands carry a length byte.
   * C0 persistent = C0 addr len params; C2 temporary = C2 addr len params
   * (e.g. manual example "C2 00 03 12 34 62"). A C2 frame without the length
   * byte is answered FF FF FF and applies nothing. */
  if ((command != 0xC0u && command != 0xC2u) || data == NULL || out == NULL ||
      len == 0u || len > 8u)
  {
    return 0u;
  }
  {
    const size_t total = 3u + len;
    if (cap < total)
    {
      return 0u;
    }
    out[0] = command;
    out[1] = addr;
    out[2] = len;
    for (uint8_t i = 0u; i < len; i++)
    {
      out[3u + i] = data[i];
    }
    return total;
  }
}

/* Dogrulanmis 6 baytlik imajdan link baud karari (saf, HAL'siz): HI imajina
 * tam eslesme -> 57600; her baska durum (9600 imaji, bozuk imaj, NULL) ->
 * 9600 geri-donus. Hedef cfg motoru yaziyi bu kararla dogrular (eslesmezse
 * huart1'e dokunulmaz), host testi ayni karari kilitler. */
uint32_t lora_cfg_verified_baud(const uint8_t *v6)
{
  if (v6 == NULL)
  {
    return LORA_LINK_BAUD_FALLBACK;
  }
  for (uint8_t i = 0u; i < LORA_CFG_LEN; i++)
  {
    if (v6[i] != LORA_CFG_EXPECTED_HI[i])
    {
      return LORA_LINK_BAUD_FALLBACK;
    }
  }
  return LORA_LINK_BAUD;
}

/* Kesif-onarim karar cekirdegi (hedef + host ortak, saf, bekleme yok):
 * basliktaki OLAY notu baglami verir. */
static bool lora_cfg_img_eq(const uint8_t *a, const uint8_t *b)
{
  for (uint8_t i = 0u; i < LORA_CFG_LEN; i++)
  {
    if (a[i] != b[i])
    {
      return false;
    }
  }
  return true;
}

uint8_t lora_cfg_classify(const uint8_t *v6)
{
  if (v6 == NULL)
  {
    return LORA_CFG_IMG_NONE;
  }
  if (lora_cfg_img_eq(v6, LORA_CFG_EXPECTED_HI))
  {
    return LORA_CFG_IMG_HI;
  }
  if (lora_cfg_img_eq(v6, LORA_CFG_EXPECTED))
  {
    return LORA_CFG_IMG_LO;
  }
  return LORA_CFG_IMG_OTHER;
}

bool lora_cfg_baud_known(uint32_t b)
{
  switch (b)
  {
    case 9600u:
    case 115200u:
    case 57600u:
    case 38400u:
    case 19200u:
    case 4800u:
    case 2400u:
    case 1200u:
      return true;
    default:
      return false;
  }
}

bool lora_cfg_needs_repair(const uint8_t *v6)
{
  return lora_cfg_classify(v6) != LORA_CFG_IMG_HI;
}

uint32_t lora_cfg_found_link_baud(const uint8_t *img6, uint32_t found_baud)
{
  /* Kanitlanmis degisim kazanir: modul bu baud'da cevap verdi, demek ki
   * dinledigi baud budur (imaj UART bitleri baska soylese bile tele guven). */
  if (lora_cfg_baud_known(found_baud))
  {
    return found_baud;
  }
  /* Kanit yoksa imaj konusmasi: HI->57600, LO->9600, gerisi geri-donus. */
  if (lora_cfg_classify(img6) == LORA_CFG_IMG_HI)
  {
    return LORA_LINK_BAUD;
  }
  if (lora_cfg_classify(img6) == LORA_CFG_IMG_LO)
  {
    return LORA_LINK_BAUD_FALLBACK;
  }
  return LORA_LINK_BAUD_FALLBACK;
}

/* Tarama-tekrar karari (saf, HAL'siz, hedef+host ortak): hit=true ise tur
 * kazanilmistir, dur (false). hit=false iken bitmis tur sayisi done ROUNDS
 * altindaysa bir tur daha dene (true); ROUNDS dolmussa birak (false).
 * done ROUNDS ustune tasmissa da false (sinir disi giris guvenli). */
bool lora_cfg_scan_retry(bool hit, uint8_t done)
{
  if (hit)
  {
    return false;
  }
  return done < LORA_CFG_SCAN_ROUNDS;
}

/* C0-kalici lik adayi (saf, HAL'siz): ilk tarama okumasi 63-imajindan
 * FARKLIYSA true (modul irasamis); esitse false (flash omru: eslesen imaja
 * C0 YAZILMAZ); NULL ise false (kanit yok, yazilacak karar yok). */
bool lora_cfg_needs_persist(const uint8_t *first6)
{
  if (first6 == NULL)
  {
    return false;
  }
  return !lora_cfg_img_eq(first6, LORA_CFG_EXPECTED);
}

/* C0 kapisi (saf, HAL'siz): mismatch (ilk okuma farkli) VE c2_ok (C2-RAM
 * yaz+dog rula BASARILI) ise true; yoksa false. False ise sessizce C2-RAM'de
 * kalinir, link yolu aynidir (9600 + RAM 63-imaji). */
bool lora_cfg_persist_now(bool mismatch, bool c2_ok)
{
  return (mismatch && c2_ok) ? true : false;
}

/* TX-oncesi AUX karari (hedef + host ortak, saf okuma, bekleme yok):
 * hedefte gercek pin (dusuk = modul mesgul), hostta enjekte bayrak. */
static bool lora_air_busy(void)
{
#if defined(LORA_WITH_HAL)
  return HAL_GPIO_ReadPin(LORA_AUX_GPIO_Port, LORA_AUX_Pin) == GPIO_PIN_RESET;
#else
  return s_host_aux_low;
#endif
}

/* =================== target-only HAL block =================== */
#if defined(LORA_WITH_HAL)
/* The USART1 handle itself is DEFINED in Src/main.c, next to huart2/huart6
 * (the CubeMX convention), so main.c owns every UART handle in the firmware.
 * Only the declaration lives here. lora.c never initialises the handle and
 * never touches it before lora_poll(), which is unreachable while
 * lora_ready() is false (lora_init(HAL_ERROR) leaves the link closed). */
extern UART_HandleTypeDef huart1;

/* ---- E220 register config (Mode 3), target-only ----
 * Manual §6/§7: Mode 3 = M1=1,M0=1; configuration commands use fixed
 * 9600 8N1 regardless of REG0 (manual §7.1; the 8-baud scan below is a
 * fallback for unknown modules and finds 9600 first).
 * Read = C1 addr len -> C1 addr len params. Write = C2 addr len params
 * (RAM-only) -> C1/C0 addr len echo. Wrong format answers FF FF FF. Every
 * step is bounded; any failure returns the module to Mode 0 and reports
 * FAILED. Never a fault. huart1 baud'u HICBIR durumda DEGILMEZ (hep 9600):
 * the common 0x63 image (9600 UART + 4.8 kbps RF) is used; no live baud switch. */
/* NOT: LORA_CFG_EXPECTED / LORA_CFG_EXPECTED_HI artik HAL'siz ortak alanda
 * tanimli (hedef cfg motoru + host baud-karar testi ayni veriyi kullanir). */

/* Last completed register read-back + AUX level at boot-config exit,
 * for the one-line TUI diagnosis. Never touches flight state. */
static uint8_t s_last_read[LORA_CFG_LEN] = { 0u, 0u, 0u, 0u, 0u, 0u };
static bool s_have_read = false;
static uint8_t s_aux_exit = 2u; /* 0 = low, 1 = high, 2 = unknown */
/* Baud actually used for the successful Mode 3 exchange (0 = none). */
static uint32_t s_baud_used = 0u;
static uint8_t s_stage = 0u;
/* Write variant: production applies the common 0x63 image at 9600 UART.
 * Legacy variant codes remain for TUI compatibility only.
 * Boot TUI "W=" alani bu degeri aynen basar (imza/sira degismez). */
static uint8_t s_variant = 0xFFu;
/* AUX-low observations sampled right after a due transmit: a healthy
 * module pulls AUX low while the packet goes to air. ok climbing with
 * this stuck at 0 means bytes never enter the module (wire/mode/power),
 * no matter what HAL_UART_Transmit reported. Read-only GPIO, no timing
 * effect. */
static uint32_t s_aux_busy = 0u;

uint32_t lora_aux_busy(void)
{
  return s_aux_busy;
}

static uint8_t lora_aux_level(void)
{
  return (HAL_GPIO_ReadPin(LORA_AUX_GPIO_Port, LORA_AUX_Pin) ==
          GPIO_PIN_SET) ? 1u : 0u;
}

/* AUX-high bekleme (hedef, boot-fazi, sinirli): tmo_ms icinde AUX yuksege
 * cikarsa true, sure dolarsa false. SONUC YOLU KESMEZ: cagiran her durumda
 * C1'i YINE DENER (bag siz AUX guvenligi: bagli olmayan AUX pini hep dusuk
 * okunur ve yapilandirmayi takilip birakmamalidir). Kisa doner, kesme
 * kapatmaz, ucusa dokunmaz. */
static bool lora_aux_wait_high(uint32_t tmo_ms)
{
  uint32_t t0 = HAL_GetTick();
  if (tmo_ms == 0u)
  {
    return lora_aux_level() == 1u;
  }
  while ((HAL_GetTick() - t0) < tmo_ms)
  {
    if (lora_aux_level() == 1u)
    {
      return true;
    }
  }
  return lora_aux_level() == 1u;
}

void lora_cfg_diag(uint8_t *out6, bool *have, uint8_t *aux_level,
                   uint32_t *baud_used, uint8_t *stage, uint8_t *variant)
{
  uint8_t i;
  if (out6 == NULL || have == NULL || aux_level == NULL ||
      baud_used == NULL || stage == NULL || variant == NULL)
  {
    return;
  }
  for (i = 0u; i < LORA_CFG_LEN; i++)
  {
    out6[i] = s_last_read[i];
  }
  *have = s_have_read;
  *aux_level = s_aux_exit;
  *baud_used = s_baud_used;
  *stage = s_stage;
  *variant = s_variant;
}

#define LORA_CFG_TMO_MS (200u)
#define LORA_AUX_TMO_MS (300u)

static void lora_mode_set(uint8_t m0, uint8_t m1)
{
  HAL_GPIO_WritePin(LORA_M0_GPIO_Port, LORA_M0_Pin,
                    m0 ? GPIO_PIN_SET : GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LORA_M1_GPIO_Port, LORA_M1_Pin,
                    m1 ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

static void lora_mode_settle(void)
{
  /* Fixed settle after a mode switch. Table 6-1: 1 ms when idle; 100 ms
   * covers self-check/reconfig with margin. AUX is recorded for the TUI
   * but never gates progress: an unconnected AUX pin reads low forever
   * and must not wedge configuration. */
  HAL_Delay(100u);
  s_aux_exit = lora_aux_level();
}

/* Transmit tx_n bytes, then receive exactly rx_n bytes. True on full
 * exchange, false on any timeout/error. RX has no IRQ on huart1, so a
 * blocking receive is correct here (boot phase only). */
static bool lora_cfg_xfer(const uint8_t *tx, size_t tx_n, uint8_t *rx,
                          size_t rx_n, uint32_t tmo_ms)
{
  uint32_t t0;
  size_t i;
  if (tx == NULL || tx_n == 0u || tx_n > 16u || rx == NULL || rx_n == 0u ||
      rx_n > 16u || tmo_ms == 0u)
  {
    return false;
  }
  if (HAL_UART_Transmit(&huart1, (uint8_t *)tx, (uint16_t)tx_n,
                        LORA_TX_TMO_MS) != HAL_OK)
  {
    return false;
  }
  /* Byte-wise receive against one total deadline: simple, no HAL-version
   * partial-transfer bookkeeping. Flash commit can be slow, so callers
   * pass a generous budget for writes. */
  t0 = HAL_GetTick();
  for (i = 0u; i < rx_n; i++)
  {
    uint32_t el = HAL_GetTick() - t0;
    uint32_t left;
    if (el >= tmo_ms)
    {
      return false;
    }
    left = tmo_ms - el;
    if (left > 100u)
    {
      left = 100u;
    }
    if (HAL_UART_Receive(&huart1, &rx[i], 1u, left) != HAL_OK)
    {
      return false;
    }
  }
  return true;
}

static bool lora_cfg_read(uint8_t *params_out)
{
  const uint8_t cmd[3] = { 0xC1u, LORA_CFG_ADDR, LORA_CFG_LEN };
  uint8_t rsp[3u + LORA_CFG_LEN];
  if (params_out == NULL)
  {
    return false;
  }
  for (uint8_t i = 0u; i < 3u + LORA_CFG_LEN; i++)
  {
    rsp[i] = 0u;
  }
  if (!lora_cfg_xfer(cmd, sizeof cmd, rsp, sizeof rsp, LORA_CFG_TMO_MS))
  {
    return false;
  }
  if (rsp[0] != 0xC1u || rsp[1] != LORA_CFG_ADDR || rsp[2] != LORA_CFG_LEN)
  {
    return false; /* includes the FF FF FF wrong-format answer */
  }
  for (uint8_t i = 0u; i < LORA_CFG_LEN; i++)
  {
    params_out[i] = rsp[3u + i];
  }
  return true;
}

/* E220 manual §7.1 register write frames both carry a length byte:
 * C0 persistent = C0 addr len params;
 * C2 temporary  = C2 addr len params (manual example "C2 00 03 12 34 62").
 * Both reply C1 addr len params; a malformed frame is answered FF FF FF. */
static bool lora_cfg_write_cmd(uint8_t cmd_byte, uint8_t addr, uint8_t len,
                               const uint8_t *params, uint32_t tmo_ms)
{
  uint8_t cmd[3u + 8u];
  uint8_t rsp[3u + 8u];
  size_t tx_n = lora_cfg_write_frame(cmd_byte, addr, len, params, cmd, sizeof cmd);
  if (tx_n == 0u)
  {
    return false;
  }
  for (uint8_t i = 0u; i < (uint8_t)(3u + len); i++)
  {
    rsp[i] = 0u;
  }
  if (!lora_cfg_xfer(cmd, tx_n, rsp, (size_t)(3u + len), tmo_ms))
  {
    return false;
  }
  if (rsp[0] != 0xC1u || rsp[1] != addr || rsp[2] != len)
  {
    /* Some modules echo C0 instead of C1; accept either command byte. */
    if (rsp[0] != 0xC0u || rsp[1] != addr || rsp[2] != len)
    {
      return false;
    }
  }
  for (uint8_t i = 0u; i < len; i++)
  {
    if (rsp[3u + i] != params[i])
    {
      return false;
    }
  }
  return true;
}

/* 8-baud tarama listesi (Mode-3 diyalog kaniti; 9600 once). */
static const uint32_t LORA_CFG_SCAN_BAUDS[8] = {
  9600u, 115200u, 57600u, 38400u, 19200u, 4800u, 2400u, 1200u
};

/* 8-baud kesif taramasi, en fazla LORA_CFG_SCAN_ROUNDS tur (Mode-3 diyalog
 * kaniti). Her C1 denemesi oncesi AUX-high en fazla 300 ms beklenir, gelmezse
 * YINE DENENIR (bag siz AUX guvenligi: sonuc yolu kesmez). Tur arasi Mode-0'a
 * donulup 100 ms oturulur (marjinal Mode-3 girisi ikinci turda tutar). Ilk
 * tutan kazanir; huart bulunan baud'da kalir. Hicbiri tutmazsa false. Tur
 * karari saf yuva lora_cfg_scan_retry ile verilir (hedef+host ayni). */
static bool lora_cfg_scan(uint8_t *img_out, uint32_t *baud_out)
{
  uint8_t done = 0u;
  while (lora_cfg_scan_retry(false, done))
  {
    for (uint8_t b = 0u; b < 8u; b++)
    {
      uint8_t img[LORA_CFG_LEN];
      huart1.Init.BaudRate = LORA_CFG_SCAN_BAUDS[b];
      if (HAL_UART_Init(&huart1) != HAL_OK)
      {
        continue;
      }
      lora_mode_set(1u, 1u); /* Mode 3 = config */
      lora_mode_settle();
      /* AUX kapisi: en fazla 300 ms yuksek beklenir; dusuk kalirsa YINE
       * DENENIR (bagli AUX sarti YOK, takilma yok). */
      (void)lora_aux_wait_high(LORA_AUX_TMO_MS);
      if (lora_cfg_read(img))
      {
        if (img_out != NULL)
        {
          for (uint8_t i = 0u; i < LORA_CFG_LEN; i++)
          {
            img_out[i] = img[i];
          }
        }
        if (baud_out != NULL)
        {
          *baud_out = LORA_CFG_SCAN_BAUDS[b];
        }
        return true;
      }
    }
    done++;
    if (lora_cfg_scan_retry(false, done))
    {
      /* Tur arasi Mode-0 + 100 ms: marjinal Mode-3 girisini arindirir. */
      lora_mode_set(0u, 0u);
      lora_mode_settle();
    }
  }
  return false;
}

lora_cfg_result_t lora_cfg_boot(void)
{
  /* Boot sets the local module to the common 00 00 63 00 17 00 image: 9600 8N1
   * module UART + 4.8 kbps RF, max 22 dBm. Scan for Mode-3 dialogue, write
   * C2 with the datasheet frame (C2 addr len params), switch via Mode 0, then
   * verify. If the saved image differed and the new one verifies, write C0
   * once to persist the common image. The active UART remains 9600 throughout
   * normal telemetry; no flight-control or deployment thresholds change.
   * Dogrulama okumasi SONUCU DEGISTIRMEZ (RX olu olabilir): echo eslesirse
   * FIXED, eslesmezse FAILED doner; iki durumda da link yolu 9600 ve
   * yazilmis 63-imaji ile Mode 0'da birakilir. Ariza yok, ucus dongusune
   * dokunmaz, SIT/SUT kapisi aynen (lora_poll ici). TUI alanlari aynidir
   * (AUX/B/RD/ST/W); bu kol W=1 basar (63-yazim kolu). */
  uint8_t cur[LORA_CFG_LEN];
  uint8_t first[LORA_CFG_LEN];
  uint8_t urg8[8u];
  uint32_t found = 0u;
  bool echo = false;
  bool match = false;
  bool mismatch0 = false; /* ilk tarama okumasi 63'ten farkli mi (C0 adayi) */
  if (!s_ready)
  {
    return LORA_CFG_SKIPPED;
  }
  s_baud_used = 0u;
  s_link_baud = LORA_LINK_BAUD_FALLBACK;
  s_variant = 0xFFu;
  s_stage = 0u;
  /* ADIM 1: Mode-3 diyalog kaniti (8-baud tarama, 9600 once). Tutmazsa
   * modul dinlemiyor demektir: huart 9600'e alinir, Mode 0'a donulur,
   * FAILED raporlanir. Yazma YOK (sagir modul isitilmez). */
  if (!lora_cfg_scan(cur, &found))
  {
    s_stage = 1u;
    huart1.Init.BaudRate = LORA_LINK_BAUD_FALLBACK;
    (void)HAL_UART_Init(&huart1);
    lora_mode_set(0u, 0u);
    lora_mode_settle();
    return LORA_CFG_FAILED;
  }
  s_baud_used = found;
  s_have_read = true;
  for (uint8_t i = 0u; i < LORA_CFG_LEN; i++)
  {
    s_last_read[i] = cur[i];
    first[i] = cur[i]; /* C0 karari icin ilk okuma saklanir (degismez kopya) */
  }
  /* C0 adayi: ilk okuma 63-imajindan farkliysa modul irasamistir (guc
   * cevriminde C2-RAM ucmus, FLASH bilinmeyen imaja donmus olabilir). Eslesen
   * imaja C0 YAZILMAZ (flash omru). Karar saf yuvada verilir (hostla ayni). */
  mismatch0 = lora_cfg_needs_persist(first);
  /* ADIM 2: C2 temporary config at fixed 9600 8N1 (manual §7.1: config mode
   * speaks 9600 regardless of REG0). The config becomes active when Mode 3
   * is exited. */
  huart1.Init.BaudRate = LORA_LINK_BAUD_FALLBACK; /* config mode is fixed 9600 8N1 (manual §7.1), not the scanned UART */
  if (HAL_UART_Init(&huart1) == HAL_OK)
  {
    lora_mode_set(1u, 1u);
    lora_mode_settle();
    if (lora_cfg_common_img(urg8, true))
    {
      echo = lora_cfg_write_cmd(0xC2u, LORA_CFG_ADDR, 8u, urg8, 800u);
    }
  }
  /* ADIM 3: leaving Mode 3 applies the RAM image and its UART setting.
   * Switch the MCU UART only after the radio has exited configuration mode. */
  lora_mode_set(0u, 0u);
  lora_mode_settle();
  huart1.Init.BaudRate = LORA_LINK_BAUD_FALLBACK;
  (void)HAL_UART_Init(&huart1);
  lora_mode_set(1u, 1u);
  lora_mode_settle();
  lora_mode_settle();
  if (lora_cfg_read(cur))
  {
    s_have_read = true;
    for (uint8_t i = 0u; i < LORA_CFG_LEN; i++)
    {
      s_last_read[i] = cur[i];
      if (cur[i] != LORA_CFG_EXPECTED[i])
      {
        match = false;
        break;
      }
      match = true;
    }
  }
  if (!echo)
  {
    s_stage = 2u; /* echo yok: yazi tutmamis olabilir */
  }
  else if (!match)
  {
    s_stage = 3u; /* echo tuttu ama dogrulama tutmadi */
  }
  else
  {
    s_stage = 4u;
    s_variant = 1u; /* 63-imaji dogrulandi */
  }
  /* ADIM 4: kosullu C0-kalicilik (yalniz irasamis + C2-dogrulamali imajda).
   * Kapı saf yuvada: mismatch0 && echo && match. Ayni 63-imaji bir kez C0
   * 00 08 ile yazilip (flash kaydi yavas: 1500 ms butce) AUX-high 1500 ms
   * beklenip 500 ms oturulur, Mode 0 cikis + Mode 3 giris sonrasi tekrar
   * okunur. Basarisizlik SESSIZCE C2-RAM'e duser: sonuc/evre/link yolu
   * DEGILMEZ (RAM 63-imaji zaten kosuyor, huart 9600). Eslesen imaja bu blok
   * hic girmez (flash omru). */
  if (lora_cfg_persist_now(mismatch0, (echo && match)))
  {
    bool c0echo = false;
    bool c0match = false;
    uint8_t back[LORA_CFG_LEN];
    huart1.Init.BaudRate = LORA_LINK_BAUD_FALLBACK;
    if (HAL_UART_Init(&huart1) == HAL_OK)
    {
      lora_mode_set(1u, 1u);
      lora_mode_settle();
      if (lora_cfg_common_img(urg8, true))
      {
        c0echo = lora_cfg_write_cmd(0xC0u, LORA_CFG_ADDR, 8u, urg8, 1500u);
      }
    }
    /* Uzun settle: flash kaydi AUX-dusuk tutar; bag siz guvenlik aynen
     * (beklenemeyen AUX yolu kesmez, tekrar okuma denenir). */
    (void)lora_aux_wait_high(1500u);
    HAL_Delay(500u);
    lora_mode_set(0u, 0u);
    lora_mode_settle();
    lora_mode_set(1u, 1u);
    lora_mode_settle();
    if (c0echo && lora_cfg_read(back))
    {
      c0match = true;
      for (uint8_t i = 0u; i < LORA_CFG_LEN; i++)
      {
        if (back[i] != LORA_CFG_EXPECTED[i])
        {
          c0match = false;
          break;
        }
      }
      if (c0match)
      {
        s_have_read = true;
        for (uint8_t i = 0u; i < LORA_CFG_LEN; i++)
        {
          s_last_read[i] = back[i];
        }
      }
    }
    (void)c0echo;
    (void)c0match; /* sonuc sessiz: C2-RAM link yolu aynen korunur */
  }
  huart1.Init.BaudRate = LORA_LINK_BAUD_FALLBACK;
  (void)HAL_UART_Init(&huart1);
  lora_mode_set(0u, 0u);
  lora_mode_settle();
  return (echo && match) ? LORA_CFG_FIXED : LORA_CFG_FAILED;
}

const char *lora_cfg_str(lora_cfg_result_t r)
{
  switch (r)
  {
    case LORA_CFG_OK:
      return "OK";
    case LORA_CFG_FIXED:
      return "FIXED";
    case LORA_CFG_FAILED:
      return "FAILED";
    case LORA_CFG_SKIPPED:
    default:
      return "SKIPPED";
  }
}

/* Ardisik-K TX hatasinda huart1'i sinirli sekilde yeniden baslatir.
 * HAL_UART_Abort + DeInit + Init: yalniz yazmac yazmalari, bekleme yok,
 * kesme kapatma yok, ucus durumuna dokunma yok. HAL_UART_MspDeInit'te
 * USART1 dali olmadigi icin (Src/stm32f4xx_hal_msp.c: yalniz USART2 var)
 * DeInit pin/saat konfigurasyonunu bozmaz; Init MspInit uzerinden hepsini
 * geri kurar (saat + PB6/PB7 AF7). Baud her zaman 9600'dir (s_link_baud);
 * cfg yazisiz degistirmez, re-init de degistirmez. Tipik maliyet < 100 us @168 MHz; ucus dongusunu
 * 5 ms'den kisa tutar. */
static void lora_uart_reinit(void)
{
  (void)HAL_UART_Abort(&huart1);
  (void)HAL_UART_DeInit(&huart1);
  huart1.Init.BaudRate = s_link_baud;
  (void)HAL_UART_Init(&huart1);
  s_reinit++;
}
#endif

void lora_init(lora_status_t status)
{
  s_ready = false; /* never assume a link until init says so */
  s_seq = 0;
  s_next_ms = 0;
  s_ok = 0;
  s_drop = 0;
  s_txerr = 0;
  s_txconsec = 0u;
  s_reinit = 0u;
  s_airbusy = 0u;
  s_link_baud = LORA_LINK_BAUD_FALLBACK;
#if defined(LORA_WITH_HAL)
  s_ready = (status == HAL_OK);
#else
  /* Host: the same rule against the plain int status. */
  s_ready = (status == 0);
  s_host_aux_low = false;
#endif
}

bool lora_ready(void)
{
  return s_ready;
}

uint16_t lora_seq(void)
{
  return s_seq;
}

uint32_t lora_ok_count(void)
{
  return s_ok;
}

uint32_t lora_drop_count(void)
{
  /* Toplam kayip (build + TX): main.c TUI "drop=" alani eski anlamiyla
   * (kaybolan paket) calisir; ayrim lora_txerr_count() ile okunur.
   * s_airbusy (TX-oncesi AUX atlama) toplama KATILMAZ, ayri sayilir. */
  return s_drop + s_txerr;
}

uint32_t lora_txerr_count(void)
{
  return s_txerr;
}

uint32_t lora_reinit_count(void)
{
  return s_reinit;
}

uint32_t lora_airbusy_count(void)
{
  return s_airbusy;
}

uint32_t lora_link_baud(void)
{
  return s_link_baud;
}

bool lora_poll(uint32_t now_ms, const lora_tlm_t *t, uint8_t *tx, size_t cap)
{
  if (!s_ready || t == NULL)
  {
    return false;
  }
  if (!lora_link_allowed(t->ek7_mode))
  {
    return false; /* not a drop: the gate is doing its job, silently */
  }
  /* Unsigned-safe deadline test: a HAL tick wrap must not freeze the link.
   * "ready" starts true because s_next_ms == 0, so the first poll sends. */
  if ((int32_t)(now_ms - s_next_ms) < 0)
  {
    return false;
  }
  uint8_t *buf = (tx != NULL && cap >= LORA_AIR_LEN) ? tx : s_tx;
  size_t bufcap = (buf == tx) ? cap : sizeof s_tx;
  lora_frame_t f;
  lora_frame_from_tlm(&f, s_seq, now_ms, t);
  if (lora_build_air(buf, bufcap, &f) != LORA_AIR_LEN)
  {
    s_drop++;
    return false;
  }
  /* Birikimli vade: gonderim sonrasi vade bir periyot ileri alinir, o
   * yuzden geciken bir tur faz kaydirma yapmaz (kayma birikmez). Hala
   * vade gerisindeyse (uzun duraklama) vade simdiki zamana yeniden
   * kurulur: kacirilan kadans atlanir, birikmis patlama gonderilmez.
   * Tek paket/poll kurali aynen korunur. */
  uint32_t iv_ms = lora_interval_ms(t->flight_state);
  s_next_ms += iv_ms;
  if ((int32_t)(now_ms - s_next_ms) >= 0)
  {
    s_next_ms = now_ms + iv_ms;
  }
  /* TX-oncesi AUX bakarak atlama (hedef + host ortak): AUX dusukse modul
   * hâlâ havaya veriyordur; paketi gonderME, bekleme YOK, vade yukarida
   * aynen tuketildi (faz korunur, patlama yok). seq/ok/drop/txerr degismez;
   * yalniz s_airbusy +1 (drop toplami bozulmaz). Gonderim-sonrasi AUX
   * ornekleme (s_aux_busy) ayri korunur. */
  if (lora_air_busy())
  {
    s_airbusy++;
    return false;
  }
#if defined(LORA_WITH_HAL)
  if (HAL_UART_Transmit(&huart1, buf, (uint16_t)LORA_AIR_LEN,
                        LORA_TX_TMO_MS) == HAL_OK)
  {
    lora_txok_note(); /* ardisik-hata sayaci kirilir */
    /* AUX snapshot right AFTER the bytes left: LOW means the module took
     * them toward air (buffer busy / transmitting). Sampled, never
     * waited on. TX-oncesi atlama karari (s_airbusy) yukarida ayri verilir;
     * bu gonderim-sonrasi gozlem aynen korunur. */
    if (HAL_GPIO_ReadPin(LORA_AUX_GPIO_Port, LORA_AUX_Pin) == GPIO_PIN_RESET)
    {
      s_aux_busy++;
    }
    s_ok++;
    s_seq++; /* uint16 wraps naturally: 0xFFFF -> 0x0000 */
    return true;
  }
  /* Timeout/hata (orn. cikmis E220 kablosu): paket dustu, yeniden deneme
   * yok, bekleme yok, dongu LORA_TX_TMO_MS icinde ilerler. s_txerr artar
   * (lora_drop_count toplaminda gorunur, s_drop build'e ozel kalir);
   * ardisik K hatada UART sinirli re-init edilir. */
  if (lora_txerr_note())
  {
    lora_uart_reinit();
  }
  return false;
#else
  /* Host build: no radio to talk to, so treat the write as delivered. This is
   * what lets the cadence, seq and gate tests run with no board. */
  lora_txok_note();
  s_ok++;
  s_seq++;
  return true;
#endif
}

#if !defined(LORA_WITH_HAL)
/* Host ariza-enjeksiyonu: hedef TX-hatasiyla ayni muhasebe, donanim yok.
 * Kadans/vade/seq/ok degistirmez; yalniz s_txerr + esikte s_reinit. */
void lora_host_inject_txerr(void)
{
  if (lora_txerr_note())
  {
    s_reinit++;
  }
}

/* Host AUX-seviye enjeksiyonu: hedefteki gercek pin okumayla ayni karari
 * surer (poll-oncesi atlama). Donanima dokunmaz, sahte veri uretmez. */
void lora_host_set_aux_low(bool low)
{
  s_host_aux_low = low;
}
#endif