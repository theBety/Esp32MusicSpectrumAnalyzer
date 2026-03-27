#include <Arduino.h>
#include <SPI.h>
#include <SD.h>
#include <WiFi.h>
#include <WebServer.h>
#include <arduinoFFT.h>

HardwareSerial SpectrumSerial(2);

// ---------------- UART ----------------
static const int TXD2 = 17;
static const int RXD2 = 16;  // nepouzito

// ---------------- SD ----------------
static const int SD_SCK  = 18;
static const int SD_MISO = 19;
static const int SD_MOSI = 23;
static const int SD_CS   = 27;

// ---------------- WIFI ----------------
const char* AP_SSID = "ESP32-Music";
const char* AP_PASS = "BetyDostaneZaJedna";

WebServer server(80);

// ---------------- FFT ----------------
static const uint16_t FFT_SAMPLES = 512;
static const uint8_t NUM_BANDS = 12;

double vReal[FFT_SAMPLES];
double vImag[FFT_SAMPLES];
double samplingFrequency = 16000.0;

ArduinoFFT<double> FFT(vReal, vImag, FFT_SAMPLES, samplingFrequency);

// ---------------- WAV ----------------
struct WavInfo {
  uint16_t audioFormat = 0;
  uint16_t numChannels = 0;
  uint32_t sampleRate = 0;
  uint16_t bitsPerSample = 0;
  uint32_t dataOffset = 0;
  uint32_t dataSize = 0;
};

File wavFile;
WavInfo wav;

uint8_t bands[NUM_BANDS];
float bandSmooth[NUM_BANDS];

bool isPlaying = false;
bool playbackFinished = false;
String currentTrack = "";

static const int MAX_TRACKS = 64;
String trackList[MAX_TRACKS];
int trackCount = 0;
int currentTrackIndex = -1;

// ============================================================

uint16_t readLE16(File &f) {
  uint8_t b0 = f.read();
  uint8_t b1 = f.read();
  return (uint16_t)(b0 | (b1 << 8));
}

uint32_t readLE32(File &f) {
  uint8_t b0 = f.read();
  uint8_t b1 = f.read();
  uint8_t b2 = f.read();
  uint8_t b3 = f.read();
  return (uint32_t)(b0 | (b1 << 8) | (b2 << 16) | (b3 << 24));
}

bool parseWavHeader(File &f, WavInfo &info) {
  f.seek(0);

  char riff[5] = {0};
  f.read((uint8_t*)riff, 4);
  if (strncmp(riff, "RIFF", 4) != 0) return false;

  (void)readLE32(f);

  char wave[5] = {0};
  f.read((uint8_t*)wave, 4);
  if (strncmp(wave, "WAVE", 4) != 0) return false;

  bool foundFmt = false;
  bool foundData = false;

  while (f.available()) {
    char chunkId[5] = {0};
    if (f.read((uint8_t*)chunkId, 4) != 4) break;
    uint32_t chunkSize = readLE32(f);

    if (strncmp(chunkId, "fmt ", 4) == 0) {
      info.audioFormat   = readLE16(f);
      info.numChannels   = readLE16(f);
      info.sampleRate    = readLE32(f);
      (void)readLE32(f); // byteRate
      (void)readLE16(f); // blockAlign
      info.bitsPerSample = readLE16(f);

      if (chunkSize > 16) {
        f.seek(f.position() + (chunkSize - 16));
      }
      foundFmt = true;
    }
    else if (strncmp(chunkId, "data", 4) == 0) {
      info.dataOffset = f.position();
      info.dataSize = chunkSize;
      foundData = true;
      break;
    }
    else {
      f.seek(f.position() + chunkSize);
    }

    if (chunkSize & 1) {
      f.seek(f.position() + 1);
    }
  }

  if (!foundFmt || !foundData) return false;
  if (info.audioFormat != 1) return false;       // jen PCM
  if (info.bitsPerSample != 16) return false;    // jen 16 bit
  if (info.numChannels != 1 && info.numChannels != 2) return false;

  return true;
}

void sendFrame(const uint8_t *data, size_t len) {
  SpectrumSerial.write(0xAA);
  SpectrumSerial.write(data, len);
  SpectrumSerial.write(0x55);
}

void clearBands() {
  memset(bands, 0, sizeof(bands));
  for (int i = 0; i < NUM_BANDS; i++) bandSmooth[i] = 0.0f;
  sendFrame(bands, NUM_BANDS);
}

double readNextMonoSample(File &f, const WavInfo &info, bool &ok) {
  ok = false;
  if (!f.available()) return 0.0;

  if (info.numChannels == 1) {
    int16_t s = (int16_t)readLE16(f);
    ok = true;
    return (double)s;
  }

  int16_t l = (int16_t)readLE16(f);
  int16_t r = (int16_t)readLE16(f);
  ok = true;
  return ((double)l + (double)r) * 0.5;
}

bool fillFFTBuffer() {
  for (uint16_t i = 0; i < FFT_SAMPLES; i++) {
    bool ok = false;
    double s = readNextMonoSample(wavFile, wav, ok);

    if (!ok) {
      playbackFinished = true;
      return false;
    }

    vReal[i] = s / 32768.0;
    vImag[i] = 0.0;
  }
  return true;
}

void computeBandsFromFFT() {
  double binHz = samplingFrequency / FFT_SAMPLES;

  const double bandEdges[NUM_BANDS + 1] = {
    40, 80, 120, 180, 260, 380, 550, 800, 1200, 1800, 2700, 4000, 6000
  };

  for (int i = 0; i < NUM_BANDS; i++) {
    double sum = 0.0;
    int count = 0;

    int startBin = max(1, (int)(bandEdges[i] / binHz));
    int endBin = min((int)(FFT_SAMPLES / 2 - 1), (int)(bandEdges[i + 1] / binHz));

    for (int b = startBin; b <= endBin; b++) {
      sum += vReal[b];
      count++;
    }

    double avg = (count > 0) ? (sum / count) : 0.0;
    double level = log10(1.0 + avg * 20.0) * 120.0;

    if (i < 3) level *= 1.25;  // lehce zvýraznit basy

    if (level > 255.0) level = 255.0;
    if (level < 0.0) level = 0.0;

    bandSmooth[i] = bandSmooth[i] * 0.65f + (float)level * 0.35f;
    bands[i] = (uint8_t)bandSmooth[i];
  }
}

void processFFTFrame() {
  FFT = ArduinoFFT<double>(vReal, vImag, FFT_SAMPLES, samplingFrequency);
  FFT.windowing(FFTWindow::Hamming, FFTDirection::Forward);
  FFT.compute(FFTDirection::Forward);
  FFT.complexToMagnitude();
  computeBandsFromFFT();
  sendFrame(bands, NUM_BANDS);
}

void stopPlayback() {
  if (wavFile) wavFile.close();
  isPlaying = false;
  playbackFinished = false;
  currentTrack = "";
  currentTrackIndex = -1;
  clearBands();
  Serial.println("Stop");
}

bool playTrackByIndex(int idx) {
  if (idx < 0 || idx >= trackCount) return false;

  if (wavFile) wavFile.close();

  String name = trackList[idx];
  File f = SD.open("/" + name);
  if (!f) {
    Serial.println("Nepodarilo se otevrit soubor");
    return false;
  }

  WavInfo tmp;
  if (!parseWavHeader(f, tmp)) {
    Serial.println("Nepodporovany WAV");
    f.close();
    return false;
  }

  wavFile = f;
  wav = tmp;
  samplingFrequency = (double)wav.sampleRate;
  wavFile.seek(wav.dataOffset);

  currentTrack = name;
  currentTrackIndex = idx;
  isPlaying = true;
  playbackFinished = false;

  for (int i = 0; i < NUM_BANDS; i++) {
    bandSmooth[i] = 0.0f;
    bands[i] = 0;
  }

  Serial.print("Analyzuji: ");
  Serial.println(currentTrack);
  return true;
}

void playNext() {
  if (trackCount == 0) return;
  int nextIndex = currentTrackIndex;
  if (nextIndex < 0) nextIndex = 0;
  else nextIndex = (nextIndex + 1) % trackCount;
  playTrackByIndex(nextIndex);
}

void playPrev() {
  if (trackCount == 0) return;
  int prevIndex = currentTrackIndex;
  if (prevIndex < 0) prevIndex = 0;
  else prevIndex = (prevIndex - 1 + trackCount) % trackCount;
  playTrackByIndex(prevIndex);
}

void scanTracks() {
  trackCount = 0;

  File root = SD.open("/");
  if (!root || !root.isDirectory()) return;

  File entry = root.openNextFile();
  while (entry && trackCount < MAX_TRACKS) {
    if (!entry.isDirectory()) {
      String name = entry.name();
      String lower = name;
      lower.toLowerCase();
      if (lower.endsWith(".wav")) {
        if (name.startsWith("/")) name = name.substring(1);
        trackList[trackCount++] = name;
      }
    }
    entry.close();
    entry = root.openNextFile();
  }

  Serial.print("Nalezeno WAV souboru: ");
  Serial.println(trackCount);
}

String htmlHeader() {
  return String(
    "<!DOCTYPE html><html><head><meta charset='utf-8'>"
    "<meta name='viewport' content='width=device-width,initial-scale=1'>"
    "<title>ESP32 Music</title>"
    "<style>"
    "body{font-family:Arial;background:#fadad9;color:#eee;margin:20px;}"
    "button,a.btn{display:inline-block;margin:6px;padding:10px 16px;border:none;border-radius:8px;"
    "background:#ff8fab;color:white;text-decoration:none;font-size:16px;}"
    "a.stop{background:#c65c69;} a.nav{background:#e0959c;} a.playphone{background:#d78289;}"
    ".card{background:#e9abae;padding:14px;border-radius:12px;margin:10px 0;}"
    ".small{color:#bbb;font-size:14px;}"
    "audio{width:100%;margin-top:12px;}"
    "</style></head><body>"
    "<h2>ESP32 Music Analyzer</h2>");
}

String escapeHtml(const String& s) {
  String out = s;
  out.replace("&", "&amp;");
  out.replace("<", "&lt;");
  out.replace(">", "&gt;");
  out.replace("\"", "&quot;");
  out.replace("'", "&#39;");
  return out;
}

void handleRoot() {
  String html = htmlHeader();

  html += "<div class='card'>";
  html += "<div><b>Stav analyzy:</b> ";
  html += isPlaying ? "Bezi" : "Zastaveno";
  html += "</div>";

  html += "<div><b>Analyzovana skladba:</b> ";
  html += currentTrack.length() ? escapeHtml(currentTrack) : "-";
  html += "</div>";

  html += "<div style='margin-top:12px;'>";
  html += "<a class='btn nav' href='/prev'>Prev</a>";
  html += "<a class='btn nav' href='/next'>Next</a>";
  html += "<a class='btn stop' href='/stop'>Stop</a>";
  html += "</div>";

  html += "<audio id='player' controls></audio>";
  html += "</div>";

  html += "<h3>Skladby</h3>";

  if (trackCount == 0) {
    html += "<div class='card'>Na SD karte neni zadny WAV soubor.</div>";
  } else {
    for (int i = 0; i < trackCount; i++) {
      html += "<div class='card'>";
      html += "<div><b>" + String(i + 1) + ".</b> " + escapeHtml(trackList[i]) + "</div>";
      html += "<div style='margin-top:10px;'>";
      html += "<a class='btn playphone' href='#' onclick='playOnPhone(" + String(i) + ");return false;'>Play on phone + Analyze</a>";
      html += "<a class='btn' href='/play?id=" + String(i) + "'>Analyze only</a>";
      html += "</div></div>";
    }
  }

  html += R"HTML(
<script>
function playOnPhone(id){
  fetch('/play?id=' + id)
    .then(() => {
      const player = document.getElementById('player');
      player.src = '/stream?id=' + id + '&t=' + Date.now();
      player.play().catch(err => console.log(err));
    });
}
</script>
)HTML";

  html += "<div class='small'>WiFi: ESP32-Music / BetyDostaneZaJedna</div>";
  html += "</body></html>";

  server.send(200, "text/html; charset=utf-8", html);
}

void handlePlay() {
  if (!server.hasArg("id")) {
    server.send(400, "text/plain", "Chybi id");
    return;
  }

  int idx = server.arg("id").toInt();
  if (!playTrackByIndex(idx)) {
    server.send(500, "text/plain", "Soubor nelze prehrat");
    return;
  }

  server.send(200, "text/plain", "OK");
}

void handleStop() {
  stopPlayback();
  server.sendHeader("Location", "/");
  server.send(303);
}

void handleNext() {
  playNext();
  server.sendHeader("Location", "/");
  server.send(303);
}

void handlePrev() {
  playPrev();
  server.sendHeader("Location", "/");
  server.send(303);
}

void handleStream() {
  if (!server.hasArg("id")) {
    server.send(400, "text/plain", "Chybi id");
    return;
  }

  int idx = server.arg("id").toInt();
  if (idx < 0 || idx >= trackCount) {
    server.send(404, "text/plain", "Neplatne id");
    return;
  }

  String name = trackList[idx];
  File f = SD.open("/" + name);
  if (!f) {
    server.send(404, "text/plain", "Soubor nenalezen");
    return;
  }

  server.streamFile(f, "audio/wav");
  f.close();
}

void setupWeb() {
  WiFi.mode(WIFI_AP);
  WiFi.softAP(AP_SSID, AP_PASS);

  Serial.println();
  Serial.print("AP IP: ");
  Serial.println(WiFi.softAPIP());

  server.on("/", handleRoot);
  server.on("/play", handlePlay);
  server.on("/stop", handleStop);
  server.on("/next", handleNext);
  server.on("/prev", handlePrev);
  server.on("/stream", handleStream);

  server.begin();
  Serial.println("Web server spusten");
}

void setup() {
  Serial.begin(115200);
  delay(800);

  SpectrumSerial.begin(115200, SERIAL_8N1, RXD2, TXD2);

  SPI.begin(SD_SCK, SD_MISO, SD_MOSI, SD_CS);

  if (!SD.begin(SD_CS, SPI)) {
    Serial.println("SD init selhala");
    while (true) delay(1000);
  }

  Serial.println("SD OK");
  scanTracks();
  clearBands();
  setupWeb();
}

void loop() {
  server.handleClient();

  if (!isPlaying) {
    delay(2);
    return;
  }

  if (playbackFinished) {
    Serial.println("Konec souboru");
    clearBands();
    isPlaying = false;
    playbackFinished = false;
    delay(10);
    return;
  }

  if (!fillFFTBuffer()) {
    return;
  }

  processFFTFrame();

  uint32_t frameTimeMs = (uint32_t)((1000.0 * FFT_SAMPLES) / samplingFrequency);
  if (frameTimeMs < 1) frameTimeMs = 1;
  delay(frameTimeMs);
}