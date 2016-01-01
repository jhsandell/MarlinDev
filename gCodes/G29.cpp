/**
 * G29: Detailed Z probe, probes the bed at 3 or more points.
 */

#include "MarlinFirmware.h"

#include "planner.h"
//#include "configuration_store.h"
#include "parser.h"
#include "probe_management/probe_management.h"
#include "kinematics/coefficients.h"
#include "motion/motion.h"
#include "host_interface/host_io.h"
#include "messages/language.h"
#include "display/display.h"
#include "movement.h"
#include "debug_only_routines.h"

#include "vector_3.h"
#include "qr_solve.h"

// Transform required to compensate for bed level
extern matrix_3x3 plan_bed_level_matrix;

/**
 * Get the position applying the bed level matrix
 */
vector_3 plan_get_position();

static void set_bed_level_equation_lsq(double* plane_equation_coefficients);
static void set_bed_level_equation_3pts(float z_at_pt_1, float z_at_pt_2, float z_at_pt_3);

#if ENABLED(MESH_BED_LEVELING)

  enum MeshLevelingState { MeshReport, MeshStart, MeshNext, MeshSet };

  /**
   * G29: Mesh-based Z probe, probes a grid and produces a
   *      mesh to compensate for variable bed height
   *
   * Parameters With MESH_BED_LEVELING:
   *
   *  S0              Produce a mesh report
   *  S1              Start probing mesh points
   *  S2              Probe the next mesh point
   *  S3 Xn Yn Zn.nn  Manually modify a single point
   *
   * The S0 report the points as below
   *
   *  +----> X-axis
   *  |
   *  |
   *  v Y-axis
   *
   */
  void gcode_G29() {

    static int probe_point = -1;
    MeshLevelingState state = code_seen('S') ? (MeshLevelingState)code_value_short() : MeshReport;
    if (state < 0 || state > 3) {
      SERIAL_PROTOCOLLNPGM("S out of range (0-3).");
      return;
    }

    int ix, iy;
    float z;

    switch (state) {
      case MeshReport:
        if (mbl.active) {
          SERIAL_PROTOCOLPGM("Num X,Y: ");
          SERIAL_PROTOCOL(MESH_NUM_X_POINTS);
          SERIAL_PROTOCOLCHAR(',');
          SERIAL_PROTOCOL(MESH_NUM_Y_POINTS);
          SERIAL_PROTOCOLPGM("\nZ search height: ");
          SERIAL_PROTOCOL(MESH_HOME_SEARCH_Z);
          SERIAL_PROTOCOLLNPGM("\nMeasured points:");
          for (int y = 0; y < MESH_NUM_Y_POINTS; y++) {
            for (int x = 0; x < MESH_NUM_X_POINTS; x++) {
              SERIAL_PROTOCOLPGM("  ");
              SERIAL_PROTOCOL_F(mbl.z_values[y][x], 5);
            }
            SERIAL_EOL;
          }
        }
        else
          SERIAL_PROTOCOLLNPGM("Mesh bed leveling not active.");
        break;

      case MeshStart:
        mbl.reset();
        probe_point = 0;
        enqueuecommands_P(PSTR("G28\nG29 S2"));
        break;

      case MeshNext:
        if (probe_point < 0) {
          SERIAL_PROTOCOLLNPGM("Start mesh probing with \"G29 S1\" first.");
          return;
        }
        if (probe_point == 0) {
          // Set Z to a positive value before recording the first Z.
          current_position[Z_AXIS] = MESH_HOME_SEARCH_Z;
          sync_plan_position();
        }
        else {
          // For others, save the Z of the previous point, then raise Z again.
          ix = (probe_point - 1) % MESH_NUM_X_POINTS;
          iy = (probe_point - 1) / MESH_NUM_X_POINTS;
          if (iy & 1) ix = (MESH_NUM_X_POINTS - 1) - ix; // zig-zag
          mbl.set_z(ix, iy, current_position[Z_AXIS]);
          current_position[Z_AXIS] = MESH_HOME_SEARCH_Z;
          plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS], homing_feedrate[X_AXIS] / 60, active_extruder);
          st_synchronize();
        }
        // Is there another point to sample? Move there.
        if (probe_point < MESH_NUM_X_POINTS * MESH_NUM_Y_POINTS) {
          ix = probe_point % MESH_NUM_X_POINTS;
          iy = probe_point / MESH_NUM_X_POINTS;
          if (iy & 1) ix = (MESH_NUM_X_POINTS - 1) - ix; // zig-zag
          current_position[X_AXIS] = mbl.get_x(ix);
          current_position[Y_AXIS] = mbl.get_y(iy);
          plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS], homing_feedrate[X_AXIS] / 60, active_extruder);
          st_synchronize();
          probe_point++;
        }
        else {
          // After recording the last point, activate the mbl and home
          SERIAL_PROTOCOLLNPGM("Mesh probing done.");
          probe_point = -1;
          mbl.active = 1;
          enqueuecommands_P(PSTR("G28"));
        }
        break;

      case MeshSet:
        if (code_seen('X')) {
          ix = code_value_long() - 1;
          if (ix < 0 || ix >= MESH_NUM_X_POINTS) {
            SERIAL_PROTOCOLPGM("X out of range (1-" AS_QUOTED_STRING(MESH_NUM_X_POINTS) ").\n");
            return;
          }
        }
        else {
          SERIAL_PROTOCOLPGM("X not entered.\n");
          return;
        }
        if (code_seen('Y')) {
          iy = code_value_long() - 1;
          if (iy < 0 || iy >= MESH_NUM_Y_POINTS) {
            SERIAL_PROTOCOLPGM("Y out of range (1-" AS_QUOTED_STRING(MESH_NUM_Y_POINTS) ").\n");
            return;
          }
        }
        else {
          SERIAL_PROTOCOLPGM("Y not entered.\n");
          return;
        }
        if (code_seen('Z')) {
          z = code_value();
        }
        else {
          SERIAL_PROTOCOLPGM("Z not entered.\n");
          return;
        }
        mbl.z_values[iy][ix] = z;

    } // switch(state)
  }

#elif ENABLED(AUTO_BED_LEVELING_FEATURE)
  #if ENABLED(DELTA)
    static void extrapolate_unprobed_bed_level();
    static void print_bed_level();
  #endif // DELTA

  void out_of_range_error(const char* p_edge) {
    SERIAL_PROTOCOLPGM("?Probe ");
    serialprintPGM(p_edge);
    SERIAL_PROTOCOLLNPGM(" position out of range.");
  }

  /**
   * G29: Detailed Z probe, probes the bed at 3 or more points.
   *      Will fail if the printer has not been homed with G28.
   *
   * Enhanced G29 Auto Bed Leveling Probe Routine
   *
   * Parameters With AUTO_BED_LEVELING_GRID:
   *
   *  P  Set the size of the grid that will be probed (P x P points).
   *     Not supported by non-linear delta printer bed leveling.
   *     Example: "G29 P4"
   *
   *  S  Set the XY travel speed between probe points (in mm/min)
   *
   *  D  Dry-Run mode. Just evaluate the bed Topology - Don't apply
   *     or clean the rotation Matrix. Useful to check the topology
   *     after a first run of G29.
   *
   *  V  Set the verbose level (0-4). Example: "G29 V3"
   *
   *  T  Generate a Bed Topology Report. Example: "G29 P5 T" for a detailed report.
   *     This is useful for manual bed leveling and finding flaws in the bed (to
   *     assist with part placement).
   *     Not supported by non-linear delta printer bed leveling.
   *
   *  F  Set the Front limit of the probing grid
   *  B  Set the Back limit of the probing grid
   *  L  Set the Left limit of the probing grid
   *  R  Set the Right limit of the probing grid
   *
   * Global Parameters:
   *
   * E/e By default G29 will engage the Z probe, test the bed, then disengage.
   *     Include "E" to engage/disengage the Z probe for each sample.
   *     There's no extra effect if you have a fixed Z probe.
   *     Usage: "G29 E" or "G29 e"
   *
   */
  void gcode_G29() {

    #if ENABLED(DEBUG_LEVELING_FEATURE)
      if (marlin_debug_flags & DEBUG_LEVELING) {
        SERIAL_ECHOLNPGM("gcode_G29 >>>");
      }
    #endif

    // Don't allow auto-leveling without homing first
    if (!axis_known_position[X_AXIS] || !axis_known_position[Y_AXIS]) {
      LCD_MESSAGEPGM(MSG_POSITION_UNKNOWN);
      SERIAL_ECHO_START;
      SERIAL_ECHOLNPGM(MSG_POSITION_UNKNOWN);
      return;
    }

    int verbose_level = code_seen('V') ? code_value_short() : 1;
    if (verbose_level < 0 || verbose_level > 4) {
      SERIAL_ECHOLNPGM("?(V)erbose Level is implausible (0-4).");
      return;
    }

    bool dryrun = code_seen('D'),
         deploy_probe_for_each_reading = code_seen('E');

    #if ENABLED(AUTO_BED_LEVELING_GRID)

      #if DISABLED(DELTA)
        bool do_topography_map = verbose_level > 2 || code_seen('T');
      #endif

      if (verbose_level > 0) {
        SERIAL_PROTOCOLPGM("G29 Auto Bed Leveling\n");
        if (dryrun) SERIAL_ECHOLNPGM("Running in DRY-RUN mode");
      }

      int auto_bed_leveling_grid_points = AUTO_BED_LEVELING_GRID_POINTS;

      #if DISABLED(DELTA)
        if (code_seen('P')) auto_bed_leveling_grid_points = code_value_short();
        if (auto_bed_leveling_grid_points < 2) {
          SERIAL_PROTOCOLPGM("?Number of probed (P)oints is implausible (2 minimum).\n");
          return;
        }
      #endif

      xy_travel_speed = code_seen('S') ? code_value_short() : XY_TRAVEL_SPEED;

      int left_probe_bed_position = code_seen('L') ? code_value_short() : LEFT_PROBE_BED_POSITION,
          right_probe_bed_position = code_seen('R') ? code_value_short() : RIGHT_PROBE_BED_POSITION,
          front_probe_bed_position = code_seen('F') ? code_value_short() : FRONT_PROBE_BED_POSITION,
          back_probe_bed_position = code_seen('B') ? code_value_short() : BACK_PROBE_BED_POSITION;

      bool left_out_l = left_probe_bed_position < MIN_PROBE_X,
           left_out = left_out_l || left_probe_bed_position > right_probe_bed_position - MIN_PROBE_EDGE,
           right_out_r = right_probe_bed_position > MAX_PROBE_X,
           right_out = right_out_r || right_probe_bed_position < left_probe_bed_position + MIN_PROBE_EDGE,
           front_out_f = front_probe_bed_position < MIN_PROBE_Y,
           front_out = front_out_f || front_probe_bed_position > back_probe_bed_position - MIN_PROBE_EDGE,
           back_out_b = back_probe_bed_position > MAX_PROBE_Y,
           back_out = back_out_b || back_probe_bed_position < front_probe_bed_position + MIN_PROBE_EDGE;

      if (left_out || right_out || front_out || back_out) {
        if (left_out) {
          out_of_range_error(PSTR("(L)eft"));
          left_probe_bed_position = left_out_l ? MIN_PROBE_X : right_probe_bed_position - MIN_PROBE_EDGE;
        }
        if (right_out) {
          out_of_range_error(PSTR("(R)ight"));
          right_probe_bed_position = right_out_r ? MAX_PROBE_X : left_probe_bed_position + MIN_PROBE_EDGE;
        }
        if (front_out) {
          out_of_range_error(PSTR("(F)ront"));
          front_probe_bed_position = front_out_f ? MIN_PROBE_Y : back_probe_bed_position - MIN_PROBE_EDGE;
        }
        if (back_out) {
          out_of_range_error(PSTR("(B)ack"));
          back_probe_bed_position = back_out_b ? MAX_PROBE_Y : front_probe_bed_position + MIN_PROBE_EDGE;
        }
        return;
      }

    #endif // AUTO_BED_LEVELING_GRID

    #if ENABLED(Z_PROBE_SLED)
      dock_sled(false); // engage (un-dock) the Z probe
    #elif ENABLED(Z_PROBE_ALLEN_KEY) //|| SERVO_LEVELING
      deploy_z_probe();
    #endif

    st_synchronize();

    if (!dryrun) {
      // make sure the bed_level_rotation_matrix is identity or the planner will get it wrong
      plan_bed_level_matrix.set_to_identity();

      #if ENABLED(DELTA)
        reset_bed_level();
      #else //!DELTA
        //vector_3 corrected_position = plan_get_position_mm();
        //corrected_position.debug("position before G29");
        vector_3 uncorrected_position = plan_get_position();
        //uncorrected_position.debug("position during G29");
        current_position[X_AXIS] = uncorrected_position.x;
        current_position[Y_AXIS] = uncorrected_position.y;
        current_position[Z_AXIS] = uncorrected_position.z;
        sync_plan_position();
      #endif // !DELTA
    }

    setup_for_endstop_move();

    feedrate = homing_feedrate[Z_AXIS];

    #if ENABLED(AUTO_BED_LEVELING_GRID)

      // probe at the points of a lattice grid
      const int xGridSpacing = (right_probe_bed_position - left_probe_bed_position) / (auto_bed_leveling_grid_points - 1),
                yGridSpacing = (back_probe_bed_position - front_probe_bed_position) / (auto_bed_leveling_grid_points - 1);

      #if ENABLED(DELTA)
        delta_grid_spacing[0] = xGridSpacing;
        delta_grid_spacing[1] = yGridSpacing;
        float z_offset = zprobe_zoffset;
        if (code_seen(axis_codes[Z_AXIS])) z_offset += code_value();
      #else // !DELTA
        // solve the plane equation ax + by + d = z
        // A is the matrix with rows [x y 1] for all the probed points
        // B is the vector of the Z positions
        // the normal vector to the plane is formed by the coefficients of the plane equation in the standard form, which is Vx*x+Vy*y+Vz*z+d = 0
        // so Vx = -a Vy = -b Vz = 1 (we want the vector facing towards positive Z

        int abl2 = auto_bed_leveling_grid_points * auto_bed_leveling_grid_points;

        double eqnAMatrix[abl2 * 3], // "A" matrix of the linear system of equations
               eqnBVector[abl2],     // "B" vector of Z points
               mean = 0.0;
        int8_t indexIntoAB[auto_bed_leveling_grid_points][auto_bed_leveling_grid_points];
      #endif // !DELTA

      int probePointCounter = 0;
      bool zig = (auto_bed_leveling_grid_points & 1) ? true : false; //always end at [RIGHT_PROBE_BED_POSITION, BACK_PROBE_BED_POSITION]

      for (int yCount = 0; yCount < auto_bed_leveling_grid_points; yCount++) {
        double yProbe = front_probe_bed_position + yGridSpacing * yCount;
        int xStart, xStop, xInc;

        if (zig) {
          xStart = 0;
          xStop = auto_bed_leveling_grid_points;
          xInc = 1;
        }
        else {
          xStart = auto_bed_leveling_grid_points - 1;
          xStop = -1;
          xInc = -1;
        }

        zig = !zig;

        for (int xCount = xStart; xCount != xStop; xCount += xInc) {
          double xProbe = left_probe_bed_position + xGridSpacing * xCount;

          // raise extruder
          float measured_z,
                z_before = probePointCounter ? Z_RAISE_BETWEEN_PROBINGS + current_position[Z_AXIS] : Z_RAISE_BEFORE_PROBING;

          if (probePointCounter) {
            #if ENABLED(DEBUG_LEVELING_FEATURE)
              if (marlin_debug_flags & DEBUG_LEVELING) {
                SERIAL_ECHOPAIR("z_before = (between) ", (float)(Z_RAISE_BETWEEN_PROBINGS + current_position[Z_AXIS]));
                SERIAL_EOL;
              }
            #endif
          }
          else {
            #if ENABLED(DEBUG_LEVELING_FEATURE)
              if (marlin_debug_flags & DEBUG_LEVELING) {
                SERIAL_ECHOPAIR("z_before = (before) ", (float)Z_RAISE_BEFORE_PROBING);
                SERIAL_EOL;
              }
            #endif
          }

          #if ENABLED(DELTA)
            // Avoid probing the corners (outside the round or hexagon print surface) on a delta printer.
            float distance_from_center = sqrt(xProbe * xProbe + yProbe * yProbe);
            if (distance_from_center > DELTA_PROBEABLE_RADIUS) continue;
          #endif //DELTA

          ProbeAction act;
          if (deploy_probe_for_each_reading) // G29 E - Stow between probes
            act = ProbeDeployAndStow;
          else if (yCount == 0 && xCount == xStart)
            act = ProbeDeploy;
          else if (yCount == auto_bed_leveling_grid_points - 1 && xCount == xStop - xInc)
            act = ProbeStow;
          else
            act = ProbeStay;

          measured_z = probe_pt(xProbe, yProbe, z_before, act, verbose_level);

          #if DISABLED(DELTA)
            mean += measured_z;

            eqnBVector[probePointCounter] = measured_z;
            eqnAMatrix[probePointCounter + 0 * abl2] = xProbe;
            eqnAMatrix[probePointCounter + 1 * abl2] = yProbe;
            eqnAMatrix[probePointCounter + 2 * abl2] = 1;
            indexIntoAB[xCount][yCount] = probePointCounter;
          #else
            bed_level[xCount][yCount] = measured_z + z_offset;
          #endif

          probePointCounter++;

          idle();

        } //xProbe
      } //yProbe

      #if ENABLED(DEBUG_LEVELING_FEATURE)
        if (marlin_debug_flags & DEBUG_LEVELING) {
          print_xyz("> probing complete > current_position", current_position);
        }
      #endif

      clean_up_after_endstop_move();

      #if ENABLED(DELTA)

        if (!dryrun) extrapolate_unprobed_bed_level();
        print_bed_level();

      #else // !DELTA

        // solve lsq problem
        double plane_equation_coefficients[3];
        qr_solve(plane_equation_coefficients, abl2, 3, eqnAMatrix, eqnBVector);

        mean /= abl2;

        if (verbose_level) {
          SERIAL_PROTOCOLPGM("Eqn coefficients: a: ");
          SERIAL_PROTOCOL_F(plane_equation_coefficients[0], 8);
          SERIAL_PROTOCOLPGM(" b: ");
          SERIAL_PROTOCOL_F(plane_equation_coefficients[1], 8);
          SERIAL_PROTOCOLPGM(" d: ");
          SERIAL_PROTOCOL_F(plane_equation_coefficients[2], 8);
          SERIAL_EOL;
          if (verbose_level > 2) {
            SERIAL_PROTOCOLPGM("Mean of sampled points: ");
            SERIAL_PROTOCOL_F(mean, 8);
            SERIAL_EOL;
          }
        }

        if (!dryrun) set_bed_level_equation_lsq(plane_equation_coefficients);

        // Show the Topography map if enabled
        if (do_topography_map) {

          SERIAL_PROTOCOLPGM(" \nBed Height Topography: \n");
          SERIAL_PROTOCOLPGM("+-----------+\n");
          SERIAL_PROTOCOLPGM("|...Back....|\n");
          SERIAL_PROTOCOLPGM("|Left..Right|\n");
          SERIAL_PROTOCOLPGM("|...Front...|\n");
          SERIAL_PROTOCOLPGM("+-----------+\n");

          float min_diff = 999;

          for (int yy = auto_bed_leveling_grid_points - 1; yy >= 0; yy--) {
            for (int xx = 0; xx < auto_bed_leveling_grid_points; xx++) {
              int ind = indexIntoAB[xx][yy];
              float diff = eqnBVector[ind] - mean;

              float x_tmp = eqnAMatrix[ind + 0 * abl2],
                    y_tmp = eqnAMatrix[ind + 1 * abl2],
                    z_tmp = 0;

              apply_rotation_xyz(plan_bed_level_matrix, x_tmp, y_tmp, z_tmp);

              if (eqnBVector[ind] - z_tmp < min_diff)
                min_diff = eqnBVector[ind] - z_tmp;

              if (diff >= 0.0)
                SERIAL_PROTOCOLPGM(" +");   // Include + for column alignment
              else
                SERIAL_PROTOCOLCHAR(' ');
              SERIAL_PROTOCOL_F(diff, 5);
            } // xx
            SERIAL_EOL;
          } // yy
          SERIAL_EOL;
          if (verbose_level > 3) {
            SERIAL_PROTOCOLPGM(" \nCorrected Bed Height vs. Bed Topology: \n");

            for (int yy = auto_bed_leveling_grid_points - 1; yy >= 0; yy--) {
              for (int xx = 0; xx < auto_bed_leveling_grid_points; xx++) {
                int ind = indexIntoAB[xx][yy];
                float x_tmp = eqnAMatrix[ind + 0 * abl2],
                      y_tmp = eqnAMatrix[ind + 1 * abl2],
                      z_tmp = 0;

                apply_rotation_xyz(plan_bed_level_matrix, x_tmp, y_tmp, z_tmp);

                float diff = eqnBVector[ind] - z_tmp - min_diff;
                if (diff >= 0.0)
                  SERIAL_PROTOCOLPGM(" +");
                // Include + for column alignment
                else
                  SERIAL_PROTOCOLCHAR(' ');
                SERIAL_PROTOCOL_F(diff, 5);
              } // xx
              SERIAL_EOL;
            } // yy
            SERIAL_EOL;
          }
        } //do_topography_map
      #endif //!DELTA

    #else // !AUTO_BED_LEVELING_GRID

      #if ENABLED(DEBUG_LEVELING_FEATURE)
        if (marlin_debug_flags & DEBUG_LEVELING) {
          SERIAL_ECHOLNPGM("> 3-point Leveling");
        }
      #endif

      // Actions for each probe
      ProbeAction p1, p2, p3;
      if (deploy_probe_for_each_reading)
        p1 = p2 = p3 = ProbeDeployAndStow;
      else
        p1 = ProbeDeploy, p2 = ProbeStay, p3 = ProbeStow;

      // Probe at 3 arbitrary points
      float z_at_pt_1 = probe_pt(ABL_PROBE_PT_1_X, ABL_PROBE_PT_1_Y, Z_RAISE_BEFORE_PROBING, p1, verbose_level),
            z_at_pt_2 = probe_pt(ABL_PROBE_PT_2_X, ABL_PROBE_PT_2_Y, current_position[Z_AXIS] + Z_RAISE_BETWEEN_PROBINGS, p2, verbose_level),
            z_at_pt_3 = probe_pt(ABL_PROBE_PT_3_X, ABL_PROBE_PT_3_Y, current_position[Z_AXIS] + Z_RAISE_BETWEEN_PROBINGS, p3, verbose_level);
      clean_up_after_endstop_move();
      if (!dryrun) set_bed_level_equation_3pts(z_at_pt_1, z_at_pt_2, z_at_pt_3);

    #endif // !AUTO_BED_LEVELING_GRID

    #if ENABLED(DELTA)
      // Allen Key Probe for Delta
      #if ENABLED(Z_PROBE_ALLEN_KEY)
        stow_z_probe();
      #elif Z_RAISE_AFTER_PROBING > 0
        raise_z_after_probing();
      #endif
    #else // !DELTA
      if (verbose_level > 0)
        plan_bed_level_matrix.debug(" \n\nBed Level Correction Matrix:");

      if (!dryrun) {
        // Correct the Z height difference from Z probe position and nozzle tip position.
        // The Z height on homing is measured by Z probe, but the Z probe is quite far from the nozzle.
        // When the bed is uneven, this height must be corrected.
        float x_tmp = current_position[X_AXIS] + X_PROBE_OFFSET_FROM_EXTRUDER,
              y_tmp = current_position[Y_AXIS] + Y_PROBE_OFFSET_FROM_EXTRUDER,
              z_tmp = current_position[Z_AXIS],
              real_z = st_get_position_mm(Z_AXIS);  //get the real Z (since plan_get_position is now correcting the plane)

        #if ENABLED(DEBUG_LEVELING_FEATURE)
          if (marlin_debug_flags & DEBUG_LEVELING) {
            SERIAL_ECHOPAIR("> BEFORE apply_rotation_xyz > z_tmp  = ", z_tmp);
            SERIAL_EOL;
            SERIAL_ECHOPAIR("> BEFORE apply_rotation_xyz > real_z = ", real_z);
            SERIAL_EOL;
          }
        #endif

        apply_rotation_xyz(plan_bed_level_matrix, x_tmp, y_tmp, z_tmp); // Apply the correction sending the Z probe offset

        // Get the current Z position and send it to the planner.
        //
        // >> (z_tmp - real_z) : The rotated current Z minus the uncorrected Z (most recent plan_set_position/sync_plan_position)
        //
        // >> zprobe_zoffset : Z distance from nozzle to Z probe (set by default, M851, EEPROM, or Menu)
        //
        // >> Z_RAISE_AFTER_PROBING : The distance the Z probe will have lifted after the last probe
        //
        // >> Should home_offset[Z_AXIS] be included?
        //
        //      Discussion: home_offset[Z_AXIS] was applied in G28 to set the starting Z.
        //      If Z is not tweaked in G29 -and- the Z probe in G29 is not actually "homing" Z...
        //      then perhaps it should not be included here. The purpose of home_offset[] is to
        //      adjust for inaccurate endstops, not for reasonably accurate probes. If it were
        //      added here, it could be seen as a compensating factor for the Z probe.
        //
        #if ENABLED(DEBUG_LEVELING_FEATURE)
          if (marlin_debug_flags & DEBUG_LEVELING) {
            SERIAL_ECHOPAIR("> AFTER apply_rotation_xyz > z_tmp  = ", z_tmp);
            SERIAL_EOL;
          }
        #endif

        current_position[Z_AXIS] = -zprobe_zoffset + (z_tmp - real_z)
          #if HAS_SERVO_ENDSTOPS || ENABLED(Z_PROBE_ALLEN_KEY) || ENABLED(Z_PROBE_SLED)
             + Z_RAISE_AFTER_PROBING
          #endif
          ;
        // current_position[Z_AXIS] += home_offset[Z_AXIS]; // The Z probe determines Z=0, not "Z home"
        sync_plan_position();

        #if ENABLED(DEBUG_LEVELING_FEATURE)
          if (marlin_debug_flags & DEBUG_LEVELING) {
            print_xyz("> corrected Z in G29", current_position);
          }
        #endif
      }

      // Sled assembly for Cartesian bots
      #if ENABLED(Z_PROBE_SLED)
        dock_sled(true); // dock the sled
      #endif

    #endif // !DELTA

    #ifdef Z_PROBE_END_SCRIPT
      #if ENABLED(DEBUG_LEVELING_FEATURE)
        if (marlin_debug_flags & DEBUG_LEVELING) {
          SERIAL_ECHO("Z Probe End Script: ");
          SERIAL_ECHOLNPGM(Z_PROBE_END_SCRIPT);
        }
      #endif
      enqueuecommands_P(PSTR(Z_PROBE_END_SCRIPT));
      st_synchronize();
    #endif

    #if ENABLED(DEBUG_LEVELING_FEATURE)
      if (marlin_debug_flags & DEBUG_LEVELING) {
        SERIAL_ECHOLNPGM("<<< gcode_G29");
      }
    #endif

  }

#endif //AUTO_BED_LEVELING_FEATURE

#if ENABLED(AUTO_BED_LEVELING_GRID)

  #if DISABLED(DELTA)

    static void set_bed_level_equation_lsq(double* plane_equation_coefficients) {
      vector_3 planeNormal = vector_3(-plane_equation_coefficients[0], -plane_equation_coefficients[1], 1);
      planeNormal.debug("planeNormal");
      plan_bed_level_matrix = matrix_3x3::create_look_at(planeNormal);
      //bedLevel.debug("bedLevel");

      //plan_bed_level_matrix.debug("bed level before");
      //vector_3 uncorrected_position = plan_get_position_mm();
      //uncorrected_position.debug("position before");

      vector_3 corrected_position = plan_get_position();
      //corrected_position.debug("position after");
      current_position[X_AXIS] = corrected_position.x;
      current_position[Y_AXIS] = corrected_position.y;
      current_position[Z_AXIS] = corrected_position.z;

      #if ENABLED(DEBUG_LEVELING_FEATURE)
        if (marlin_debug_flags & DEBUG_LEVELING) {
          print_xyz("set_bed_level_equation_lsq > current_position", current_position);
        }
      #endif

      sync_plan_position();
    }

  #endif // !DELTA

#else // !AUTO_BED_LEVELING_GRID

  #ifndef ABL_PROBE_PT_1_X
    #define ABL_PROBE_PT_1_X 0.0
    #define ABL_PROBE_PT_1_Y 0.0
    #define ABL_PROBE_PT_2_X 100.0
    #define ABL_PROBE_PT_2_Y 0.0
    #define ABL_PROBE_PT_3_X 0.0
    #define ABL_PROBE_PT_3_Y 100.0
  #endif
  static void set_bed_level_equation_3pts(float z_at_pt_1, float z_at_pt_2, float z_at_pt_3) {

    plan_bed_level_matrix.set_to_identity();

    vector_3 pt1 = vector_3(ABL_PROBE_PT_1_X, ABL_PROBE_PT_1_Y, z_at_pt_1);
    vector_3 pt2 = vector_3(ABL_PROBE_PT_2_X, ABL_PROBE_PT_2_Y, z_at_pt_2);
    vector_3 pt3 = vector_3(ABL_PROBE_PT_3_X, ABL_PROBE_PT_3_Y, z_at_pt_3);
    vector_3 planeNormal = vector_3::cross(pt1 - pt2, pt3 - pt2).get_normal();

    if (planeNormal.z < 0) {
      planeNormal.x = -planeNormal.x;
      planeNormal.y = -planeNormal.y;
      planeNormal.z = -planeNormal.z;
    }

    plan_bed_level_matrix = matrix_3x3::create_look_at(planeNormal);

    vector_3 corrected_position = plan_get_position();
    current_position[X_AXIS] = corrected_position.x;
    current_position[Y_AXIS] = corrected_position.y;
    current_position[Z_AXIS] = corrected_position.z;

    #if ENABLED(DEBUG_LEVELING_FEATURE)
      if (marlin_debug_flags & DEBUG_LEVELING) {
        print_xyz("set_bed_level_equation_3pts > current_position", current_position);
      }
    #endif

    sync_plan_position();
  }

#endif // !AUTO_BED_LEVELING_GRID

#if ENABLED(AUTO_BED_LEVELING_FEATURE)

  #if ENABLED(DELTA)

    /**
     * All DELTA leveling in the Marlin uses NONLINEAR_BED_LEVELING
     */

    static void extrapolate_one_point(int x, int y, int xdir, int ydir) {
      if (bed_level[x][y] != 0.0) {
        return;  // Don't overwrite good values.
      }
      float a = 2 * bed_level[x + xdir][y] - bed_level[x + xdir * 2][y]; // Left to right.
      float b = 2 * bed_level[x][y + ydir] - bed_level[x][y + ydir * 2]; // Front to back.
      float c = 2 * bed_level[x + xdir][y + ydir] - bed_level[x + xdir * 2][y + ydir * 2]; // Diagonal.
      float median = c;  // Median is robust (ignores outliers).
      if (a < b) {
        if (b < c) median = b;
        if (c < a) median = a;
      }
      else {  // b <= a
        if (c < b) median = b;
        if (a < c) median = a;
      }
      bed_level[x][y] = median;
    }

    // Fill in the unprobed points (corners of circular print surface)
    // using linear extrapolation, away from the center.
    static void extrapolate_unprobed_bed_level() {
      int half = (AUTO_BED_LEVELING_GRID_POINTS - 1) / 2;
      for (int y = 0; y <= half; y++) {
        for (int x = 0; x <= half; x++) {
          if (x + y < 3) continue;
          extrapolate_one_point(half - x, half - y, x > 1 ? +1 : 0, y > 1 ? +1 : 0);
          extrapolate_one_point(half + x, half - y, x > 1 ? -1 : 0, y > 1 ? +1 : 0);
          extrapolate_one_point(half - x, half + y, x > 1 ? +1 : 0, y > 1 ? -1 : 0);
          extrapolate_one_point(half + x, half + y, x > 1 ? -1 : 0, y > 1 ? -1 : 0);
        }
      }
    }

    // Print calibration results for plotting or manual frame adjustment.
    static void print_bed_level() {
//    for (int y = 0; y < AUTO_BED_LEVELING_GRID_POINTS; y++) {
      for (int y=AUTO_BED_LEVELING_GRID_POINTS-1; y>=0; y--) {
        for (int x = 0; x < AUTO_BED_LEVELING_GRID_POINTS; x++) {

          if (bed_level[x][y] >= 0.0)		// We need an extra space to make the columns line
          	SERIAL_PROTOCOLCHAR(' ');	// up if the number is positive.

          SERIAL_PROTOCOL_F(bed_level[x][y], 3);
          SERIAL_PROTOCOLCHAR(' ');
        }
        SERIAL_EOL;
      }
    }

    // Reset calibration results to zero.
    void reset_bed_level() {
      #if ENABLED(DEBUG_LEVELING_FEATURE)
        if (marlin_debug_flags & DEBUG_LEVELING) {
          SERIAL_ECHOLNPGM("reset_bed_level");
        }
      #endif
      for (int y = 0; y < AUTO_BED_LEVELING_GRID_POINTS; y++) {
        for (int x = 0; x < AUTO_BED_LEVELING_GRID_POINTS; x++) {
          bed_level[x][y] = 0.0;
        }
      }
    }

  #endif // DELTA

#endif // AUTO_BED_LEVELING_FEATURE
