/**
 * Marlin 3D Printer Firmware
 * Copyright (C) 2016 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
 *
 * Based on Sprinter and grbl.
 * Copyright (C) 2011 Camiel Gubbels / Erik van der Zalm
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

#include "../../inc/MarlinConfig.h"

#if ENABLED(PANELDUE)

#include "../gcode.h"
#include "../../lcd/ultralcd.h"
#include "../../sd/cardreader.h"
#include "../../module/planner.h"
#include "../../module/printcounter.h"
#include "../../module/temperature.h"

#define MAX_MESSAGE_LENGTH 80

char pd_status_message[MAX_MESSAGE_LENGTH + 1];

void MarlinUI::setstatus(const char* message, const bool persist/*=false*/) {
  UNUSED(persist);
  strncpy(pd_status_message, message, MAX_MESSAGE_LENGTH);
}

void MarlinUI::setstatusPGM(const char* message, const int8_t level/*=0*/) {
  UNUSED(level);
  strncpy_P(pd_status_message, message, MAX_MESSAGE_LENGTH);
}

/**
 * M408: Report machine state in JSON format for PanelDue
 *
 *  S<style> Include static values with S1
 *
 * Since Marlin doesn't use sequence numbers, the R parameter is omitted.
 *
 */

inline void json_key(const int16_t port, const char * const name) {
  #if NUM_SERIAL < 2
    UNUSED(port);
  #endif
  SERIAL_PROTOCOLPGM_P(port, ",\"");
  serialprintPGM_P(port, name);
  SERIAL_PROTOCOLPGM_P(port, "\":");
}

inline void json_array_print(const int16_t port, const char * const name, const float val[], const int8_t size) {
  #if NUM_SERIAL < 2
    UNUSED(port);
  #endif
  json_key(port, name);
  SERIAL_PROTOCOLCHAR_P(port, '[');
  for (uint8_t i = 0; i < size; i++) {
    SERIAL_PROTOCOL_P(port, val[i]);
    if (i < size - 1) SERIAL_PROTOCOLCHAR_P(port, ',');
  }
  SERIAL_PROTOCOLCHAR_P(port, ']');
  safe_delay(1);
}

void GcodeSuite::M408() {
  #if NUM_SERIAL > 1
    const int16_t port = command_queue_port[cmd_queue_index_r];
  #else
    constexpr int16_t port = 0;
  #endif

  float tmp[10];
  SERIAL_PROTOCOLCHAR_P(port, '{');

  // status: SD printing or idle
  json_key(port, PSTR("status"));
  SERIAL_PROTOCOL_P(port, IS_SD_PRINTING() ? "\"P\"" : "\"I\"");

  // heaters: current bed, hotend temperatures
  tmp[0] = (
    #if HAS_HEATED_BED
      thermalManager.degBed()
    #else
      0
    #endif
  );
  HOTEND_LOOP() tmp[e + 1] = thermalManager.degHotend(e);
  json_array_print(port, PSTR("heaters"), tmp, HOTENDS + 1);

  // active: target bed, hotend temperatures
  tmp[0] = (
    #if HAS_HEATED_BED
      thermalManager.degTargetBed()
    #else
      0
    #endif
  );
  HOTEND_LOOP() tmp[e + 1] = thermalManager.degTargetHotend(e);
  json_array_print(port, PSTR("active"), tmp, HOTENDS + 1);

  // standby: in Marlin, same as active
  json_array_print(port, PSTR("standby"), tmp, HOTENDS + 1);

  // hstat: in Marlin, heating or off
  json_key(port, PSTR("hstat"));
  SERIAL_PROTOCOLCHAR_P(port, '[');
  SERIAL_PROTOCOLCHAR_P(port,
    #if HAS_HEATED_BED
      thermalManager.degTargetBed() ? '2' : '0'
    #else
      '0'
    #endif
  );
  HOTEND_LOOP() {
    SERIAL_PROTOCOLCHAR_P(port, ',');
    SERIAL_PROTOCOLCHAR_P(port, thermalManager.degTargetHotend(e) ? '2' : '0');
  }
  SERIAL_PROTOCOLCHAR_P(port, ']');

  // pos: tool position
  tmp[0] = current_position[X_AXIS]; tmp[1] = current_position[Y_AXIS]; tmp[2] = current_position[Z_AXIS];
  json_array_print(port, PSTR("pos"), tmp, 3);

  // extr: extruder position
  for (uint8_t e = 0; e < EXTRUDERS; e++) tmp[e] = current_position[E_AXIS];
  json_array_print(port, PSTR("extr"), tmp, EXTRUDERS);

  // sfactor: feedrate %
  json_key(port, PSTR("sfactor"));
  SERIAL_PROTOCOL_P(port, feedrate_percentage);

  // efactor: flow %
  for (uint8_t e = 0; e < EXTRUDERS; e++) tmp[e] = planner.flow_percentage[e];
  json_array_print(port, PSTR("efactor"), tmp, EXTRUDERS);

  // tool: selected extruder
  json_key(port, PSTR("tool"));
  SERIAL_PROTOCOL_P(port, active_extruder);

  // probe: the last Z probe reading (just 0 for now)
  json_key(port, PSTR("probe"));
  SERIAL_PROTOCOLPGM_P(port, "\"0\"");

  #if FAN_COUNT > 0
    // fanPercent: the last Z probe reading
    for (uint8_t i = 0; i < FAN_COUNT; i++) tmp[i] = map(fan_speed[i], 0, 255, 0, 100);
    json_array_print(port, PSTR("fanPercent"), tmp, FAN_COUNT);

    // fanRPM: print cooling fan faux RPM
    json_key(port, PSTR("fanRPM"));
    SERIAL_PROTOCOL_P(port, int(fan_speed[0] * 10));
  #endif

  // homed: axis homed status
  json_key(port, PSTR("homed"));
  SERIAL_PROTOCOLCHAR_P(port, '[');
  SERIAL_PROTOCOLCHAR_P(port, '0' + TEST(axis_homed, X_AXIS));
  SERIAL_PROTOCOLCHAR_P(port, ',');
  SERIAL_PROTOCOLCHAR_P(port, '0' + TEST(axis_homed, Y_AXIS));
  SERIAL_PROTOCOLCHAR_P(port, ',');
  SERIAL_PROTOCOLCHAR_P(port, '0' + TEST(axis_homed, Z_AXIS));
  SERIAL_PROTOCOLCHAR_P(port, ']');

  #if HAS_PRINT_PROGRESS
    // fraction_printed: print progress
    json_key(port, PSTR("fraction_printed"));
    SERIAL_PROTOCOL_P(port, 0.01 * ui.get_progress());
  #endif

  // message
  if (pd_status_message[0]) {
    json_key(port, PSTR("message"));
    SERIAL_PROTOCOLPAIR_P(port, "\"", pd_status_message);
    SERIAL_PROTOCOLCHAR_P(port, '"');
  }

  // Extra fields
  if (parser.intval('S') == 1) {
    // myName
    json_key(port, PSTR("myName"));
    SERIAL_PROTOCOLPGM_P(port, "\"" MACHINE_NAME "\"");
    // firmwareName
    json_key(port, PSTR("firmwareName"));
    SERIAL_PROTOCOLPGM_P(port, "\"Marlin\"");
    // geometry
    json_key(port, PSTR("geometry"));
    SERIAL_PROTOCOLPGM_P(port, "\""
      #if IS_SCARA
        "scara"
      #elif ENABLED(HANGPRINTER)
        "hangprinter"
      #elif ENABLED(DELTA)
        "delta"
      #elif ENABLED(COREXY)
        "corexy"
      #elif ENABLED(COREXY)
        "corexy"
      #elif ENABLED(COREXZ)
        "corexz"
      #elif ENABLED(COREYZ)
        "coreyz"
      #elif ENABLED(COREYX)
        "coreyx"
      #elif ENABLED(COREZX)
        "corezx"
      #elif ENABLED(COREZY)
        "corezy"
      #else
        "cartesian"
      #endif
      "\""
    );
    // axes
    json_key(port, PSTR("axes"));
    SERIAL_PROTOCOLCHAR_P(port, '3');
    // volumes: the number of SD card slots available
    json_key(port, PSTR("volumes"));
    SERIAL_PROTOCOLCHAR_P(port,
      #if ENABLED(SDSUPPORT)
        '1'
      #else
        '0'
      #endif
    );
    // numTools: extruder count
    json_key(port, PSTR("numTools"));
    SERIAL_PROTOCOLCHAR_P(port, '0' + EXTRUDERS);
  }

  SERIAL_EOL_P(port);
}

#endif // PANELDUE
