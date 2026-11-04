#include <Arduino.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include <SPI.h>
#include <TFT_eSPI.h>
#include <math.h>
#include <time.h>
#include <Preferences.h>

#include "secrets.h"

/*
 * Configuration and User Credentials.
 */
// [CUSTOMIZABLE] Target Wi-Fi network SSID name string.
const char* ssid = SECRET_SSID;
// [CUSTOMIZABLE] Target Wi-Fi network access password string.
const char* password = SECRET_PASS;

// [CUSTOMIZABLE] Telegram Bot HTTP API access token string.
const char* TELEGRAM_TOKEN = SECRET_TELEGRAM_TOKEN;

// [CUSTOMIZABLE] Target Telegram Chat ID string for receiving alerts.
const char* TELEGRAM_CHAT_ID = SECRET_TELEGRAM_CHAT_ID;

// [CUSTOMIZABLE] Receiver geographic latitude coordinate in decimal degrees.
const float MY_LAT = SECRET_MY_LAT;

// [CUSTOMIZABLE] Receiver geographic longitude coordinate in decimal degrees.
const float MY_LON = SECRET_MY_LON;

/*
 * System Parameters and Server Settings.
 */
// [CUSTOMIZABLE] Address of the NTP server for system clock synchronization.
const char* ntpServer = "pool.ntp.org";

// [CUSTOMIZABLE] Data API search radius in nautical miles (1 NM = 1.852 km).
float RADIUS_NM = 13.5;

// [CUSTOMIZABLE] Maximum physical display radar coverage radius in kilometers.
const float RADIUS_KM = 25.0;

// [CUSTOMIZABLE] GPIO input pin connected to physical push button.
const int BUTTON_PIN = 25;

// [CUSTOMIZABLE] GPIO output pin powering button module (provides 3.3V VCC).
const int BUTTON_VCC_PIN = 33;

// [CUSTOMIZABLE] HTTP polling interval in milliseconds for fetching aircraft data (default: 15000 ms).
const unsigned long FETCH_INTERVAL = 15000;

// [CUSTOMIZABLE] Manual UI interaction timeout in milliseconds before reverting to auto mode (default: 10000 ms).
const unsigned long AUTO_REVERT_TIME = 10000;

// [CUSTOMIZABLE] Angular velocity of radar sweep beam rotation in degrees per frame update.
const float SWEEP_SPEED = 2.0;

// [CUSTOMIZABLE] Number of trailing lines rendered behind radar sweep for visual fade effect.
const int TRAIL_LENGTH = 18;

// [CUSTOMIZABLE] Horizontal scroll velocity of footer marquee text in pixels per frame.
const float MARQUEE_SPEED = 5.0;

// [CUSTOMIZABLE] Maximum number of aircraft records stored simultaneously in memory array buffer.
const int MAX_AIRCRAFT = 15;

// [CUSTOMIZABLE] Maximum number of unique flight history entries retained in non-volatile storage.
const int MAX_HISTORY = 30;

/*
 * Global State and Data Structures.
 */
// Data structure containing comprehensive real-time flight state and telemetry data.
struct Aircraft {
  String flight;        // Flight callsign or identifier (e.g. "IBE3120")
  String desc;          // Aircraft model description (e.g. "Airbus A350-900")
  String category;      // ICAO aircraft category string
  String registration;  // Tail registration mark (e.g. "EC-LOV")
  String squawk;        // Transponder squawk code string (e.g. "7700")
  float baro_rate;      // Vertical speed climb/descent rate in feet per minute
  float lat, lon;       // Target latitude and longitude positions in decimal degrees
  float alt_m;          // Barometric altitude converted to meters
  float gs_kmh;         // Ground speed converted to kilometers per hour
  float track;          // True heading track angle in degrees (0 - 359)
  float distance_km;    // Radial distance from receiver location in kilometers
  float bearing_deg;    // Relative azimuth bearing angle from receiver in degrees
  String hex;           // ICAO 24-bit address hex identifier code
  String typeCode;      // ICAO 4-character aircraft type code (e.g. "A359")
  float alt_geom;       // Geometric/GPS altitude converted to meters
  bool autopilotOn;     // Autopilot engagement status flag
  float seenPos;        // Seconds since position update was last received
};

// Data structure pairing flight callsigns with cumulative sighting counts.
struct FlightRecord {
  String flight;        // Flight callsign string
  int count;            // Cumulative sighting detection counter
};

// State machine enumeration specifying available display UI views.
enum ScreenMode {
  SCREEN_RADAR,         // Main radar PPI display mode
  SCREEN_DETAIL,        // Nearest aircraft detailed telemetry mode
  SCREEN_HISTORY         // Top sighted aircraft historical list mode
};

// Active UI screen mode selector state variable.
ScreenMode currentScreen = SCREEN_RADAR;

// Primary dynamic array holding current active aircraft telemetry objects.
Aircraft aircraftList[MAX_AIRCRAFT];

// Current count of valid aircraft loaded into aircraftList array buffer.
int aircraftCount = 0;

// Storage array holding historical flight sighting totals.
FlightRecord history[MAX_HISTORY];

// Active element count of historical flight entries currently stored in history array.
int historyCount = 0;

// Temporary cache storing flight IDs present during previous HTTP fetch cycle.
String previousFlights[MAX_AIRCRAFT];

// Element count of flight IDs stored in previous flights comparison buffer.
int previousFlightsCount = 0;

// Flag indicating unpersisted modifications to history array pending NVS write.
bool historyDirty = false;

// Circular buffer cache tracking flight IDs that triggered recent Telegram alerts.
String lastAlertedFlights[10];

// Current element index count for Telegram alert tracking ring buffer.
int lastAlertedCount = 0;

// Flag active when an emergency transponder squawk (7500, 7600, 7700) is detected.
bool emergencyActive = false;

// Flight callsign associated with current active emergency alert state.
String emergencyFlight = "";

// Transponder code string associated with active emergency alert state.
String emergencyCode = "";

// Flag signaling an unsent Telegram notification is queued in memory.
bool pendingTelegramAlert = false;

// Pre-formatted text payload string containing pending Telegram alert message.
String pendingTelegramMsg = "";

// Target flight callsign triggering queued Telegram alert message.
String pendingTelegramFlight = "";

// Total count of successful API HTTP payload fetches executed since boot.
int fetchCount = 0;

// Array index of target aircraft displayed during automatic rotation mode.
int autoIndex = 0;

// Array index of manually selected aircraft target (-1 indicates automatic mode).
int selectedIndex = -1;

// Timestamp in milliseconds of last executed API HTTP fetch request.
unsigned long lastFetch = 0;

// Timestamp in milliseconds of last registered hardware button input press.
unsigned long lastButtonPress = 0;

// Logic state of button pin during last sampling interval (HIGH = idle, LOW = pressed).
bool lastButtonState = HIGH;

// Current rotational angle of radar sweep line in degrees (0.0 to 359.9).
float sweepAngle = 0;

// Center X/Y screen coordinate offsets and maximum radial pixel distance.
int centerX, centerY, maxRadius;

// Formatted text buffer displayed in scrolling footer marquee banner.
String marqueeText = "";

// Current horizontal rendering pixel offset of scrolling marquee text.
float marqueeX = 0;

// Callsign string of aircraft currently bound to marquee text buffer.
String lastMarqueeFlight = "";

// Packed 16-bit RGB565 color key value used for primary display accents.
uint16_t COLOR_VIOLET = 0;

// Primary TFT display hardware interface object instance.
TFT_eSPI tft = TFT_eSPI();

// Double-buffered graphics sprite object stored in RAM for flicker-free rendering.
TFT_eSprite radarSprite = TFT_eSprite(&tft);

// ESP32 Preferences library instance for NVS persistent flash storage.
Preferences prefs;

/*
 * Function Declarations and Detailed Specifications.
 */

/*
 * Calculates the great-circle distance between two geographic points using the Haversine formula.
 * @param lat1 Latitude of origin point in decimal degrees.
 * @param lon1 Longitude of origin point in decimal degrees.
 * @param lat2 Latitude of destination point in decimal degrees.
 * @param lon2 Longitude of destination point in decimal degrees.
 * @return Great-circle distance between coordinates in kilometers.
 */
float calculateDistance(float lat1, float lon1, float lat2, float lon2);

/*
 * Calculates initial true bearing angle from origin coordinates to target coordinates.
 * @param lat1 Latitude of origin point in decimal degrees.
 * @param lon1 Longitude of origin point in decimal degrees.
 * @param lat2 Latitude of target point in decimal degrees.
 * @param lon2 Longitude of target point in decimal degrees.
 * @return Initial azimuth bearing angle relative to true North in degrees (0.0 to 359.9).
 */
float calculateBearing(float lat1, float lon1, float lat2, float lon2);

/*
 * Converts a numerical bearing angle in degrees into an 8-point cardinal compass string.
 * @param bearing Azimuth angle in degrees (0.0 to 360.0).
 * @return Cardinal compass direction string ("N", "NE", "E", "SE", "S", "SW", "W", "NW").
 */
String bearingToCompass(float bearing);

/*
 * Evaluates whether aircraft description matches predefined high-priority aircraft models.
 * @param plane Reference to Aircraft structure to evaluate.
 * @return True if aircraft matches priority military or widebody criteria; False otherwise.
 */
bool isSpecialAircraft(Aircraft& plane);

/*
 * Searches aircraft list array for first entry located within configured maximum radar radius.
 * @return Array index of first valid aircraft in range; -1 if no aircraft are within range.
 */
int findFirstValidIndex();

/*
 * Searches aircraft list array for first high-priority aircraft located within radar radius.
 * @return Array index of priority aircraft in range; -1 if no priority aircraft are found.
 */
int findSpecialIndex();

/*
 * Performs circular array search for next valid aircraft in range starting after given index.
 * @param fromIndex Starting array index for search iteration.
 * @return Next valid array index within radar range; -1 if no valid aircraft exist.
 */
int getNextValidIndex(int fromIndex);

/*
 * Searches aircraft list array to locate target with minimum physical distance to origin.
 * @return Array index of nearest aircraft; -1 if no aircraft are currently loaded.
 */
int findNearestIndex();

/*
 * Connects to airplanes.live HTTP API, parses JSON response, and populates aircraft state array.
 */
void fetchAircraftData();

/*
 * Transmits a text alert message to configured Telegram bot chat ID via HTTPS GET request.
 * @param message Pre-formatted URL-encoded message string to transmit.
 * @return True if HTTP response code is 200 (OK); False on network error or HTTP failure.
 */
bool sendTelegramMessage(String message);

/*
 * Checks whether a specific flight callsign has already received a Telegram alert in this session.
 * @param flight Flight callsign string to check against recent alert buffer.
 * @return True if callsign exists in alert buffer; False otherwise.
 */
bool wasAlreadyAlerted(String flight);

/*
 * Adds flight callsign to recent alert ring buffer to prevent duplicate notifications.
 * @param flight Flight callsign string to append to alert buffer.
 */
void markAsAlerted(String flight);

/*
 * Restores historical flight sighting records from ESP32 NVS storage flash partition.
 */
void loadHistory();

/*
 * Serializes current historical sighting array into JSON and writes to ESP32 NVS storage.
 */
void saveHistory();

/*
 * Increments sighting counter for existing flight or appends new flight entry to history list.
 * @param flight Callsign string of aircraft target sighted in radar coverage area.
 */
void recordSighting(String flight);

/*
 * Verifies if flight callsign was present in aircraft tracking buffer during prior API fetch.
 * @param flight Flight callsign string to search within previous fetch buffer.
 * @return True if flight was observed in prior fetch; False if flight is newly detected.
 */
bool wasInPreviousFetch(String flight);

/*
 * Sorts historical sighting records by frequency count and returns top N entries.
 * @param n Number of highest-frequency flight records to extract.
 * @param result Pointer to output array of FlightRecord structs to hold output records.
 */
void getTopFlights(int n, FlightRecord* result);

/*
 * Retrieves synchronized NTP time and formats output string into 24-hour time representation.
 * @return Formatted time string ("HH:MM"); or placeholder ("--:--") if clock is unsynchronized.
 */
String getTimeString();

/*
 * Counts total number of active aircraft in current array within maximum radar radius.
 * @return Quantity of aircraft currently positioned within configured coverage radius.
 */
int countInRange();

/*
 * Truncates text string to fit maximum pixel width and appends ellipsis ("...") if necessary.
 * @param text Original input text string to evaluate.
 * @param maxWidth Maximum allowable rendering width in pixels.
 * @return Unmodified string if width fits constraint; or truncated string with ellipsis.
 */
String truncateToFit(String text, int maxWidth);

/*
 * Maps 3-character ICAO callsign prefix to corresponding full airline company name.
 * @param flightCode Full flight callsign identifier string (e.g., "IBE3120").
 * @return Full commercial airline name if prefix match is found; original prefix otherwise.
 */
String getAirlineName(String flightCode);

/*
 * Constructs detailed single-line string of aircraft parameters for footer marquee display.
 * @param big Reference to Aircraft struct containing telemetry to format.
 */
void updateMarqueeText(Aircraft& big);

/*
 * Manages scrolling step offsets and target index advancement logic for footer marquee.
 * @param displayIndex Array index of aircraft currently selected for marquee display.
 */
void updateMarquee(int displayIndex);

/*
 * Renders radar background grid, range rings, cardinal headers, and time onto sprite buffer.
 */
void drawRadarBase();

/*
 * Draws single vector line segment from display center outward to represent radar beam.
 * @param angleDeg Vector beam direction angle in degrees relative to North.
 * @param color Packed 16-bit RGB565 color code used to render line.
 */
void drawSweepLine(float angleDeg, uint16_t color);

/*
 * Renders rotating radar beam along with fading trailing gradient lines on sprite buffer.
 * @param angle Current leading sweep beam rotation angle in degrees.
 */
void drawSweepWithTrail(float angle);

/*
 * Renders spatial blip markers, heading tail vectors, and flight labels for all target aircraft.
 */
void drawAircraft();

/*
 * Renders scrolled marquee text string at bottom footer region of offscreen graphics sprite.
 */
void drawMarquee();

/*
 * Renders multi-line technical telemetry data page for nearest aircraft target on sprite buffer.
 */
void drawDetailScreen();

/*
 * Renders top 5 most frequently observed aircraft sighting records onto sprite graphics buffer.
 */
void drawHistoryScreen();

/*
 * Monitors hardware input pin state, applies debouncing timing, and triggers screen advancement.
 */
void checkButton();

/*
 * Advances state machine currentScreen selector variable sequentially to next view mode.
 */
void advanceScreen();

/*
 * Section: Geographic Calculations and Unit Utilities.
 * Contains mathematical functions used for computing spherical distances,
 * bearings between geographic coordinates, and cardinal compass conversions.
 */
float calculateDistance(float lat1, float lon1, float lat2, float lon2) {
  const float R = 6371.0;
  float dLat = radians(lat2 - lat1);
  float dLon = radians(lon2 - lon1);
  float a = sin(dLat / 2) * sin(dLat / 2) + cos(radians(lat1)) * cos(radians(lat2)) * sin(dLon / 2) * sin(dLon / 2);
  float c = 2 * atan2(sqrt(a), sqrt(1 - a));
  return R * c;
}

float calculateBearing(float lat1, float lon1, float lat2, float lon2) {
  float dLon = radians(lon2 - lon1);
  float lat1Rad = radians(lat1);
  float lat2Rad = radians(lat2);
  float y = sin(dLon) * cos(lat2Rad);
  float x = cos(lat1Rad) * sin(lat2Rad) - sin(lat1Rad) * cos(lat2Rad) * cos(dLon);
  float bearing = degrees(atan2(y, x));
  return fmod(bearing + 360, 360);
}

String bearingToCompass(float bearing) {
  const char* dirs[] = {"N", "NE", "E", "SE", "S", "SW", "W", "NW"};
  int idx = (int)((bearing + 22.5) / 45.0) % 8;
  return String(dirs[idx]);
}

/*
 * Section: Aircraft Filter and Index Selection Routines.
 * Functions responsible for identifying special interest aircraft,
 * filtering active track positions, and searching array indices.
 */
bool isSpecialAircraft(Aircraft& plane) {
  String d = plane.desc;
  d.toUpperCase();

  // Evaluate widebody commercial aircraft
  if (d.indexOf("787") >= 0) return true;
  if (d.indexOf("777") >= 0) return true;
  if (d.indexOf("747") >= 0) return true;
  if (d.indexOf("A350") >= 0) return true;
  if (d.indexOf("A380") >= 0) return true;

  // Evaluate military and transport aircraft
  if (d.indexOf("A400") >= 0) return true;
  if (d.indexOf("C-130") >= 0 || d.indexOf("HERCULES") >= 0) return true;
  if (d.indexOf("C-17") >= 0 || d.indexOf("GLOBEMASTER") >= 0) return true;
  if (d.indexOf("C-295") >= 0) return true;
  if (d.indexOf("KC-135") >= 0 || d.indexOf("STRATOTANKER") >= 0) return true;
  if (d.indexOf("MRTT") >= 0 || d.indexOf("VOYAGER") >= 0) return true;
  if (d.indexOf("P-8") >= 0 || d.indexOf("POSEIDON") >= 0) return true;
  if (d.indexOf("SENTRY") >= 0 || d.indexOf("AWACS") >= 0) return true;
  if (d.indexOf("CHINOOK") >= 0 || d.indexOf("CH-47") >= 0) return true;

  return false;
}

int findFirstValidIndex() {
  // Iterate to find the first target within configured radius
  for (int i = 0; i < aircraftCount; i++) {
    if (aircraftList[i].distance_km <= RADIUS_KM) return i;
  }
  return -1;
}

int findSpecialIndex() {
  // Search for any target matching special aircraft criteria within range
  for (int i = 0; i < aircraftCount; i++) {
    if (aircraftList[i].distance_km <= RADIUS_KM && isSpecialAircraft(aircraftList[i])) {
      return i;
    }
  }
  return -1;
}

int getNextValidIndex(int fromIndex) {
  int inRangeCount = countInRange();

  // Exit early if no aircraft are currently within the coverage area
  if (inRangeCount == 0) return -1;

  int next = fromIndex + 1;
  int checked = 0;

  // Perform circular search for the next valid target in range
  while (checked < aircraftCount) {
    if (next >= aircraftCount) next = 0;
    if (aircraftList[next].distance_km <= RADIUS_KM) return next;
    next++;
    checked++;
  }
  return -1;
}

int findNearestIndex() {
  int nearest = -1;
  float minDist = 999999;

  // Find aircraft with minimum absolute distance from origin
  for (int i = 0; i < aircraftCount; i++) {
    if (aircraftList[i].distance_km <= RADIUS_KM && aircraftList[i].distance_km < minDist) {
      minDist = aircraftList[i].distance_km;
      nearest = i;
    }
  }
  return nearest;
}

/*
 * Section: Network and External API Communications.
 * Code handling HTTP requests to Airplanes.live API, payload parsing,
 * emergency squawk detection, and sending alerts via Telegram.
 */
void fetchAircraftData() {
  // Abort request execution if local network connection is unavailable
  if (WiFi.status() != WL_CONNECTED) return;

  HTTPClient http;
  String url = "https://api.airplanes.live/v2/point/"
               + String(MY_LAT, 4) + "/"
               + String(MY_LON, 4) + "/"
               + String(RADIUS_NM, 0);

  http.begin(url);
  int httpCode = http.GET();

  if (httpCode == 200) {
    String payload = http.getString();
    JsonDocument doc;
    DeserializationError error = deserializeJson(doc, payload);

    if (!error) {
      JsonArray arr = doc["ac"];
      aircraftCount = 0;
      emergencyActive = false;

      // Parse JSON array elements into aircraft data structures
      for (JsonObject ac : arr) {
        if (aircraftCount >= MAX_AIRCRAFT) break;

        float lat = ac["lat"] | 0.0;
        float lon = ac["lon"] | 0.0;

        // Skip records containing invalid coordinate points
        if (lat == 0.0 && lon == 0.0) continue;

        Aircraft plane;
        plane.flight = String((const char*)(ac["flight"] | "N/A"));
        plane.flight.trim();
        plane.desc = String((const char*)(ac["desc"] | "Desconocido"));
        plane.category = String((const char*)(ac["category"] | ""));
        plane.lat = lat;
        plane.lon = lon;
        plane.alt_m = (ac["alt_baro"] | 0) * 0.3048;
        plane.gs_kmh = (ac["gs"] | 0.0) * 1.852;
        plane.track = ac["track"] | 0.0;
        plane.registration = String((const char*)(ac["r"] | "----"));
        plane.squawk = String((const char*)(ac["squawk"] | "----"));
        plane.baro_rate = ac["baro_rate"] | ac["geom_rate"] | 0.0;

        plane.distance_km = calculateDistance(MY_LAT, MY_LON, lat, lon);
        plane.bearing_deg = calculateBearing(MY_LAT, MY_LON, lat, lon);

        // Check if aircraft meets notification criteria
        if (plane.distance_km <= 15 && isSpecialAircraft(plane) && !wasAlreadyAlerted(plane.flight)) {
          pendingTelegramMsg = plane.desc + " flying nearby!!%0A" +
                              "Flight: " + plane.flight + " (" + getAirlineName(plane.flight) + ")%0A" +
                              "Direction: " + bearingToCompass(plane.bearing_deg) + " of you (" + String((int)plane.bearing_deg) + " deg)%0A" +
                              "Distance: " + String(plane.distance_km, 1) + "km";
          pendingTelegramFlight = plane.flight;
          pendingTelegramAlert = true;
        }

        // Flag emergency state on matching squawk codes
        if (plane.squawk == "7500" || plane.squawk == "7600" || plane.squawk == "7700") {
          emergencyActive = true;
          emergencyFlight = plane.flight;
          emergencyCode = plane.squawk;
        }

        plane.hex = String((const char*)(ac["hex"] | "------"));
        plane.typeCode = String((const char*)(ac["t"] | "----"));
        plane.alt_geom = (ac["alt_geom"] | 0) * 0.3048;
        plane.seenPos = ac["seen_pos"] | 0.0;

        // Parse navigation modes array to verify autopilot status
        plane.autopilotOn = false;
        if (ac["nav_modes"].is<JsonArray>()) {
          JsonArray modes = ac["nav_modes"].as<JsonArray>();
          for (JsonVariant mode : modes) {
            if (String((const char*)mode) == "autopilot") {
              plane.autopilotOn = true;
              break;
            }
          }
        }

        // Register sighting entry if aircraft is newly encountered
        if (plane.distance_km <= RADIUS_KM && !wasInPreviousFetch(plane.flight)) {
          recordSighting(plane.flight);
        }

        aircraftList[aircraftCount] = plane;
        aircraftCount++;
      }

      // Refresh list of current active flights for comparison in next cycle
      previousFlightsCount = 0;
      for (int i = 0; i < aircraftCount && previousFlightsCount < MAX_AIRCRAFT; i++) {
        if (aircraftList[i].distance_km <= RADIUS_KM) {
          previousFlights[previousFlightsCount] = aircraftList[i].flight;
          previousFlightsCount++;
        }
      }

      // Persist history changes to storage if flag is set
      if (historyDirty) {
        saveHistory();
        historyDirty = false;
      }

      int specialIdx = findSpecialIndex();

      // Lock selection to high-priority special aircraft when in auto mode
      if (specialIdx != -1 && selectedIndex == -1) {
        autoIndex = specialIdx;
      } else {
        // Fall back to first available valid target if index becomes stale
        if (autoIndex >= aircraftCount || (autoIndex >= 0 && aircraftList[autoIndex].distance_km > RADIUS_KM)) {
          autoIndex = findFirstValidIndex();
        }
      }

      // Reset manual index if target leaves range or list
      if (selectedIndex >= aircraftCount || (selectedIndex >= 0 && aircraftList[selectedIndex].distance_km > RADIUS_KM)) {
        selectedIndex = -1;
      }

      Serial.println("Aviones cargados: " + String(aircraftCount));
      fetchCount++;
    } else {
      Serial.println("Error JSON: " + String(error.c_str()));
    }
  } else {
    Serial.println("Error HTTP: " + String(httpCode));
  }

  http.end();

  // Dispatch pending Telegram notification after main HTTP session completes
  if (pendingTelegramAlert) {
    if (sendTelegramMessage(pendingTelegramMsg)) {
      markAsAlerted(pendingTelegramFlight);
    }
    pendingTelegramAlert = false;
  }
}

bool sendTelegramMessage(String message) {
  // Cancel transmission if network connection is lost
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("Telegram: WiFi no conectado, no se envia");
    return false;
  }

  WiFiClientSecure client;
  client.setInsecure();

  HTTPClient http;
  message.replace(" ", "%20");
  message.replace("°", "");

  String url = "https://api.telegram.org/bot" + String(TELEGRAM_TOKEN) +
               "/sendMessage?chat_id=" + String(TELEGRAM_CHAT_ID) +
               "&text=" + message;

  http.setConnectTimeout(10000);
  http.setTimeout(10000);
  http.begin(client, url);
  int httpCode = http.GET();

  bool success = (httpCode == 200);

  if (success) {
    Serial.println("Telegram enviado OK");
  } else {
    Serial.println("Error Telegram: " + String(httpCode) + " (" + http.errorToString(httpCode) + ")");
  }

  http.end();
  return success;
}

bool wasAlreadyAlerted(String flight) {
  // Check if flight code exists in alert cache
  for (int i = 0; i < lastAlertedCount; i++) {
    if (lastAlertedFlights[i] == flight) return true;
  }
  return false;
}

void markAsAlerted(String flight) {
  // Append new flight code to list or perform array shift when full
  if (lastAlertedCount < 10) {
    lastAlertedFlights[lastAlertedCount] = flight;
    lastAlertedCount++;
  } else {
    for (int i = 0; i < 9; i++) lastAlertedFlights[i] = lastAlertedFlights[i + 1];
    lastAlertedFlights[9] = flight;
  }
}

/*
 * Section: Persistent Storage and Sighting History Management.
 * Logic for loading, saving, and processing historical flight encounter statistics
 * using Non-Volatile Storage (NVS via Preferences).
 */
void loadHistory() {
  prefs.begin("radar", true);
  String json = prefs.getString("history", "[]");
  prefs.end();

  JsonDocument doc;
  deserializeJson(doc, json);
  JsonArray arr = doc.as<JsonArray>();

  historyCount = 0;

  // Restore history array entries from deserialized JSON
  for (JsonObject item : arr) {
    if (historyCount >= MAX_HISTORY) break;
    history[historyCount].flight = String((const char*)(item["f"] | ""));
    history[historyCount].count = item["c"] | 0;
    historyCount++;
  }

  Serial.println("Historial cargado: " + String(historyCount) + " vuelos");
}

void saveHistory() {
  JsonDocument doc;
  JsonArray arr = doc.to<JsonArray>();

  // Serialize flight history array into JSON format
  for (int i = 0; i < historyCount; i++) {
    JsonObject item = arr.add<JsonObject>();
    item["f"] = history[i].flight;
    item["c"] = history[i].count;
  }

  String json;
  serializeJson(doc, json);

  prefs.begin("radar", false);
  prefs.putString("history", json);
  prefs.end();
}

void recordSighting(String flight) {
  // Reject invalid or empty flight identifiers
  if (flight == "N/A" || flight.length() == 0) return;

  // Increment frequency counter if entry already exists
  for (int i = 0; i < historyCount; i++) {
    if (history[i].flight == flight) {
      history[i].count++;
      historyDirty = true;
      return;
    }
  }

  // Insert new record or overwrite lowest-count record if full
  if (historyCount < MAX_HISTORY) {
    history[historyCount].flight = flight;
    history[historyCount].count = 1;
    historyCount++;
    historyDirty = true;
  } else {
    int minIdx = 0;
    for (int i = 1; i < historyCount; i++) {
      if (history[i].count < history[minIdx].count) minIdx = i;
    }
    history[minIdx].flight = flight;
    history[minIdx].count = 1;
    historyDirty = true;
  }
}

bool wasInPreviousFetch(String flight) {
  // Search previous cycle buffer for matching flight string
  for (int i = 0; i < previousFlightsCount; i++) {
    if (previousFlights[i] == flight) return true;
  }
  return false;
}

void getTopFlights(int n, FlightRecord* result) {
  FlightRecord sorted[MAX_HISTORY];
  memcpy(sorted, history, sizeof(FlightRecord) * historyCount);

  // Perform bubble sort to arrange sightings in descending order
  for (int i = 0; i < historyCount - 1; i++) {
    for (int j = 0; j < historyCount - i - 1; j++) {
      if (sorted[j].count < sorted[j + 1].count) {
        FlightRecord temp = sorted[j];
        sorted[j] = sorted[j + 1];
        sorted[j + 1] = temp;
      }
    }
  }

  // Copy top N results into output buffer
  for (int i = 0; i < n; i++) {
    if (i < historyCount) {
      result[i] = sorted[i];
    } else {
      result[i].flight = "";
      result[i].count = 0;
    }
  }
}

/*
 * Section: Text Parsing, Formatting, and String Utilities.
 * Helper functions to format system time, match ICAO callsign prefixes to airline names,
 * truncate text strings, and generate marquee text buffers.
 */
String getTimeString() {
  struct tm timeinfo;

  // Return placeholder if system time is not synchronized
  if (!getLocalTime(&timeinfo)) {
    return "--:--";
  }

  char buf[6];
  sprintf(buf, "%02d:%02d", timeinfo.tm_hour, timeinfo.tm_min);
  return String(buf);
}

int countInRange() {
  int count = 0;

  // Count active aircraft located within configured geographical radius
  for (int i = 0; i < aircraftCount; i++) {
    if (aircraftList[i].distance_km <= RADIUS_KM) count++;
  }
  return count;
}

String truncateToFit(String text, int maxWidth) {
  // Return original text if display width is sufficient
  if (radarSprite.textWidth(text) <= maxWidth) {
    return text;
  }

  String truncated = text;

  // Trim character by character until text fits bounds with ellipsis
  while (radarSprite.textWidth(truncated + "...") > maxWidth && truncated.length() > 0) {
    truncated.remove(truncated.length() - 1);
  }
  return truncated + "...";
}

String getAirlineName(String flightCode) {
  // Reject short callsigns unable to hold a 3-character ICAO prefix
  if (flightCode.length() < 3) return "Desconocida";

  String prefix = flightCode.substring(0, 3);
  prefix.toUpperCase();

  // Match ICAO 3-letter prefix against known airline table
  if (prefix == "IBE") return "Iberia";
  if (prefix == "RYR") return "Ryanair";
  if (prefix == "ANE") return "Air Nostrum";
  if (prefix == "VLG") return "Vueling";
  if (prefix == "AEA") return "Air Europa";
  if (prefix == "EZY") return "easyJet";
  if (prefix == "BAW") return "British Airways";
  if (prefix == "AFR") return "Air France";
  if (prefix == "DLH") return "Lufthansa";
  if (prefix == "KLM") return "KLM";
  if (prefix == "TAP") return "TAP Portugal";
  if (prefix == "WZZ") return "Wizz Air";
  if (prefix == "UAE") return "Emirates";
  if (prefix == "QTR") return "Qatar Airways";
  if (prefix == "AAL") return "American Airlines";
  if (prefix == "DAL") return "Delta";
  if (prefix == "UAL") return "United Airlines";

  return prefix;
}

void updateMarqueeText(Aircraft& big) {
  String arrow = "-";

  // Determine vertical rate direction arrow indicator
  if (big.baro_rate > 100) arrow = "^";
  else if (big.baro_rate < -100) arrow = "v";

  marqueeText = big.desc + " - " + big.flight + " - " + getAirlineName(big.flight) + "  |  " + String((int)big.gs_kmh) + "km/h" + "  |  " + String((int)big.alt_m) + "m " + arrow + "  |  R: " + String((int)big.track) + "°" + "  ";
}

void updateMarquee(int displayIndex) {
  // Display idle marquee state if selected aircraft index is out of bounds
  if (displayIndex < 0 || displayIndex >= aircraftCount) {
    marqueeText = "Zzz...";
    lastMarqueeFlight = "";

    if (selectedIndex == -1) {
      autoIndex = findFirstValidIndex();
    }
    return;
  }

  String currentFlight = aircraftList[displayIndex].flight;

  // Rebuild marquee text string if target flight changes
  if (currentFlight != lastMarqueeFlight) {
    updateMarqueeText(aircraftList[displayIndex]);
    marqueeX = tft.width();
    lastMarqueeFlight = currentFlight;
  }

  marqueeX -= MARQUEE_SPEED;

  radarSprite.setFreeFont(&FreeSans12pt7b);
  int textWidth = radarSprite.textWidth(marqueeText);
  radarSprite.setFreeFont(NULL);

  // Reset scroll position and step selection index once text leaves screen
  if (marqueeX < -textWidth) {
    if (selectedIndex == -1) {
      int specialIdx = findSpecialIndex();
      if (specialIdx != -1) {
        autoIndex = specialIdx;
      } else {
        autoIndex = getNextValidIndex(autoIndex);
      }
    }
    marqueeX = tft.width();
  }
}

/*
 * Section: Display Rendering and Graphics Routines.
 * Functions drawing radar elements, sweep lines, blips, telemetry screens,
 * top history lists, and marquee text onto TFT sprites.
 */
void drawRadarBase() {
  radarSprite.fillSprite(TFT_BLACK);

  // Render header telemetry information
  radarSprite.setTextColor(TFT_GREEN);
  radarSprite.setFreeFont(&FreeSans9pt7b);
  radarSprite.setCursor(5, 15);
  radarSprite.print("Flights: " + String(countInRange()));
  radarSprite.setCursor(5, 35);
  radarSprite.print("Act: " + String(fetchCount));

  // Render current system time anchored to right header boundary
  radarSprite.setFreeFont(&FreeSansBold12pt7b);
  String timeStr = getTimeString();
  int timeW = radarSprite.textWidth(timeStr);
  radarSprite.setCursor(tft.width() - timeW - 5, 24);
  radarSprite.print(timeStr);

  radarSprite.setFreeFont(NULL);
  radarSprite.setTextFont(1);
  radarSprite.setTextSize(1);

  // Render radar range rings and crosshair axes
  for (int i = 1; i <= 4; i++) {
    int r = (maxRadius * i) / 4;
    radarSprite.drawCircle(centerX, centerY, r, TFT_DARKGREEN);
  }
  radarSprite.drawLine(centerX - maxRadius, centerY, centerX + maxRadius, centerY, TFT_DARKGREEN);
  radarSprite.drawLine(centerX, centerY - maxRadius, centerX, centerY + maxRadius, TFT_DARKGREEN);

  radarSprite.setCursor(centerX - 3, centerY - maxRadius - 12);
  radarSprite.print("N");
  radarSprite.setCursor(centerX - 3, centerY + maxRadius + 4);
  radarSprite.print("S");
  radarSprite.setCursor(centerX + maxRadius + 4, centerY - 3);
  radarSprite.print("E");
  radarSprite.setCursor(centerX - maxRadius - 10, centerY - 3);
  radarSprite.print("W");

  radarSprite.fillCircle(centerX, centerY, 3, TFT_YELLOW);

  int displayIndex = (selectedIndex != -1) ? selectedIndex : autoIndex;
  updateMarquee(displayIndex);
  drawMarquee();
}

void drawSweepLine(float angleDeg, uint16_t color) {
  float angleRad = radians(angleDeg - 90);
  int x = centerX + maxRadius * cos(angleRad);
  int y = centerY + maxRadius * sin(angleRad);
  radarSprite.drawLine(centerX, centerY, x, y, color);
}

void drawSweepWithTrail(float angle) {
  // Render fading trail lines behind current sweep beam angle
  for (int i = 0; i < TRAIL_LENGTH; i++) {
    float trailAngle = angle - (i * SWEEP_SPEED);
    int brightness = map(i, 0, TRAIL_LENGTH, 255, 0);
    uint16_t color = radarSprite.color565(0, brightness, 0);
    drawSweepLine(trailAngle, color);
  }
}

void drawAircraft() {
  bool specialBlinkOn = (millis() / 500) % 2 == 0;
  int displayIndex = (selectedIndex != -1) ? selectedIndex : autoIndex;

  for (int i = 0; i < aircraftCount; i++) {
    Aircraft& plane = aircraftList[i];

    // Omit aircraft located beyond configured radar radius
    if (plane.distance_km > RADIUS_KM) continue;

    float pixelDist = (plane.distance_km / RADIUS_KM) * maxRadius;
    float angleRad = radians(plane.bearing_deg - 90);

    int x = centerX + pixelDist * cos(angleRad);
    int y = centerY + pixelDist * sin(angleRad);

    bool isSpecial = isSpecialAircraft(plane);
    bool isHighlighted = isSpecial || (i == displayIndex);
    bool isSelected = !isSpecial && (i == displayIndex);

    uint16_t blipColor = TFT_WHITE;
    if (isSpecial) {
      blipColor = TFT_RED;
    } else if (i == displayIndex) {
      blipColor = COLOR_VIOLET;
    }

    float tailLength = isSpecial ? 38 : (isSelected ? 28 : 18);
    float trackRad = radians(plane.track - 90 + 180);
    int tailX = x + tailLength * cos(trackRad);
    int tailY = y + tailLength * sin(trackRad);

    radarSprite.drawLine(x, y, tailX, tailY, blipColor);

    // Draw blip marker with optional flash state for special aircraft
    if (!isSpecial || specialBlinkOn) {
      int blipRadius = isSpecial ? 9 : (isSelected ? 7 : 5);
      radarSprite.fillCircle(x, y, blipRadius, blipColor);
    }

    radarSprite.setTextColor(TFT_CYAN);

    if (isHighlighted) {
      radarSprite.setFreeFont(&FreeSans9pt7b);
    } else {
      radarSprite.setTextFont(1);
    }

    int textW = isHighlighted ? radarSprite.textWidth(plane.flight) : plane.flight.length() * 6;

    // Adjust label horizontal offset relative to center axis
    if (x > centerX) {
      radarSprite.setCursor(x - textW - 6, y - 8);
    } else {
      radarSprite.setCursor(x + 6, y - 8);
    }
    radarSprite.print(plane.flight);

    if (isHighlighted) {
      radarSprite.setFreeFont(NULL);
    }
  }
}

void drawMarquee() {
  radarSprite.setFreeFont(&FreeSans12pt7b);

  bool showingSpecial = false;

  // Determine if active marquee text belongs to a special aircraft
  for (int i = 0; i < aircraftCount; i++) {
    if (aircraftList[i].flight == lastMarqueeFlight) {
      showingSpecial = isSpecialAircraft(aircraftList[i]);
      break;
    }
  }

  uint16_t textColor = showingSpecial ? TFT_RED : COLOR_VIOLET;

  radarSprite.setTextColor(textColor);
  radarSprite.setCursor((int)marqueeX, tft.height() - 14);
  radarSprite.print(marqueeText);
  radarSprite.setFreeFont(NULL);
}

void drawDetailScreen() {
  radarSprite.fillSprite(TFT_BLACK);
  int nearestIdx = findNearestIndex();

  radarSprite.setTextColor(TFT_GREEN);
  radarSprite.setFreeFont(&FreeSansBold9pt7b);
  radarSprite.setCursor(5, 18);
  radarSprite.print("NEAREST AIRCRAFT");

  // Display status message if no aircraft are currently within range
  if (nearestIdx == -1) {
    radarSprite.setFreeFont(&FreeSans9pt7b);
    radarSprite.setCursor(5, 45);
    radarSprite.print("Everything seems quiet...");
    radarSprite.setFreeFont(NULL);
    return;
  }

  Aircraft& p = aircraftList[nearestIdx];

  radarSprite.setFreeFont(&FreeSans9pt7b);
  radarSprite.setTextColor(TFT_GREEN);

  int y = 45;
  int lineHeight = 22;

  // Render comprehensive aircraft status fields
  radarSprite.setCursor(5, y);
  y += lineHeight;
  radarSprite.print(p.flight + " (" + p.registration + ")");

  radarSprite.setCursor(5, y);
  y += lineHeight;
  String model = truncateToFit(p.desc + " (" + p.typeCode + ")", tft.width() - 10);
  radarSprite.print(model);

  radarSprite.setCursor(5, y);
  y += lineHeight;
  radarSprite.print("Airline: " + getAirlineName(p.flight));

  radarSprite.setCursor(5, y);
  y += lineHeight;
  radarSprite.print("ICAO: " + p.hex);

  radarSprite.setCursor(5, y);
  y += lineHeight;
  radarSprite.print("Alt baro: " + String((int)p.alt_m) + "m");

  radarSprite.setCursor(5, y);
  y += lineHeight;
  radarSprite.print("Alt geom: " + String((int)p.alt_geom) + "m");

  radarSprite.setCursor(5, y);
  y += lineHeight;
  radarSprite.print("Vel: " + String((int)p.gs_kmh) + "km/h");

  radarSprite.setCursor(5, y);
  y += lineHeight;
  radarSprite.print("Rumbo: " + String((int)p.track) + "°");

  radarSprite.setCursor(5, y);
  y += lineHeight;
  radarSprite.print("Dist: " + String(p.distance_km, 1) + "km");

  radarSprite.setCursor(5, y);
  y += lineHeight;
  radarSprite.print("Squawk: " + p.squawk);

  radarSprite.setCursor(5, y);
  y += lineHeight;
  radarSprite.print("Autopilot: " + String(p.autopilotOn ? "SI" : "NO"));

  radarSprite.setFreeFont(NULL);
  radarSprite.setTextFont(1);
  radarSprite.setTextColor(TFT_DARKGREEN);
  radarSprite.setCursor(5, tft.height() - 12);
}

void drawHistoryScreen() {
  radarSprite.fillSprite(TFT_BLACK);

  radarSprite.setTextColor(TFT_GREEN);
  radarSprite.setFreeFont(&FreeSansBold9pt7b);
  radarSprite.setCursor(5, 18);
  radarSprite.print("TOP AVIONES");

  radarSprite.setFreeFont(&FreeSans9pt7b);
  radarSprite.setTextColor(TFT_WHITE);

  // Render placeholder text if history log contains no entries
  if (historyCount == 0) {
    radarSprite.setCursor(5, 50);
    radarSprite.print("Sin historial aun");
    radarSprite.setFreeFont(NULL);
    return;
  }

  FlightRecord top[5];
  getTopFlights(5, top);

  int y = 45;
  int lineHeight = 24;

  // Render top 5 recorded flight sightings
  for (int i = 0; i < 5; i++) {
    if (top[i].flight == "") continue;
    radarSprite.setCursor(5, y);
    radarSprite.print(String(i + 1) + ". " + top[i].flight + " (" + String(top[i].count) + ")");
    y += lineHeight;
  }

  radarSprite.setFreeFont(NULL);
  radarSprite.setTextFont(1);
}

/*
 * Section: Input Hardware Handling and User Interface Flow.
 * Debounces physical pushbuttons and manages active UI screen states.
 */
void checkButton() {
  bool currentState = digitalRead(BUTTON_PIN);

  // Detect transition state from HIGH to LOW with debouncing
  if (lastButtonState == HIGH && currentState == LOW) {
    delay(30);
    if (digitalRead(BUTTON_PIN) == LOW) {
      advanceScreen();
    }
  }

  lastButtonState = currentState;
}

void advanceScreen() {
  // Cycle forward through available display screens
  if (currentScreen == SCREEN_RADAR) {
    currentScreen = SCREEN_DETAIL;
  } else if (currentScreen == SCREEN_DETAIL) {
    currentScreen = SCREEN_HISTORY;
  } else {
    currentScreen = SCREEN_RADAR;
  }
}

/*
 * Section: Initialization and Main Program Loop.
 * Arduino entry points for system setup, peripheral initialization,
 * timing control, and state machine updates.
 */
void setup() {
  Serial.begin(115200);
  delay(500);

  loadHistory();

  WiFi.begin(ssid, password);
  Serial.print("Conectando a WiFi");

  // Block execution until Wi-Fi association is complete
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nConectado!");

  configTzTime("CET-1CEST,M3.5.0,M10.5.0/3", ntpServer);

  pinMode(BUTTON_VCC_PIN, OUTPUT);
  digitalWrite(BUTTON_VCC_PIN, HIGH);
  pinMode(BUTTON_PIN, INPUT);

  tft.init();
  tft.setRotation(2);
  tft.fillScreen(TFT_BLACK);

  COLOR_VIOLET = tft.color565(180, 60, 255);

  centerX = tft.width() / 2;
  centerY = tft.height() / 2 - 10;
  maxRadius = min((int)tft.width(), (int)tft.height() - 55) / 2 - 10;
  radarSprite.setColorDepth(8);
  radarSprite.createSprite(tft.width(), tft.height());

  fetchAircraftData();
}

void loop() {
  // Attempt network reconnection if Wi-Fi link drops
  if (WiFi.status() != WL_CONNECTED) {
    WiFi.reconnect();
    delay(500);
  }

  checkButton();

  // Revert manual selection to automatic mode after timeout expires
  if (selectedIndex != -1 && millis() - lastButtonPress > AUTO_REVERT_TIME) {
    selectedIndex = -1;
  }

  // Trigger period polling of aircraft telemetry data
  if (millis() - lastFetch > FETCH_INTERVAL) {
    fetchAircraftData();
    lastFetch = millis();
  }

  // Render content matching the currently active UI screen mode
  if (currentScreen == SCREEN_RADAR) {
    drawRadarBase();
    drawSweepWithTrail(sweepAngle);
    drawAircraft();
  } else if (currentScreen == SCREEN_DETAIL) {
    drawDetailScreen();
  } else if (currentScreen == SCREEN_HISTORY) {
    drawHistoryScreen();
  }

  radarSprite.pushSprite(0, 0);

  sweepAngle += SWEEP_SPEED;

  // Wrap sweep angle around unit circle boundary
  if (sweepAngle >= 360) sweepAngle -= 360;

  delay(30);
}