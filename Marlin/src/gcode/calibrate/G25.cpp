
#include "../../inc/MarlinConfig.h"

#include "../gcode.h"

#include "../../module/stepper.h"
#include "../../module/endstops.h"
#include "../parser.h"

#include "../../module/motion.h"
#define DEBUG_OUT ENABLED(DEBUG_LEVELING_FEATURE)
#include "../../core/debug_out.h"

void GcodeSuite::G25() {
  DEBUG_SECTION(log_G25, "G25", DEBUGGING(LEVELING));
  if (DEBUGGING(LEVELING)) log_machine_info();
  float feedrate_dhm = 15.0;
  float dist_dhm_xy = 10.0;
  float dist_dhm_z = 7.0;

  if( parser.seenval('Y') ) 
    dist_dhm_xy = parser.value_axis_units(Y_AXIS);

  if( parser.seenval('Z') ) 
    dist_dhm_z = parser.value_axis_units(Z_AXIS);
    
  if (parser.floatval('F') > 0)
    feedrate_dhm = parser.value_feedrate();
  

  laser_calibrate_disp(active_extruder,dist_dhm_xy,dist_dhm_z,feedrate_dhm );

  
}
void  GcodeSuite::laser_calibrate_disp(int8_t disp,float distXY, float distZ,float feedrate)
{
  laser_calibrate_start_move(disp);
  laser_cycle_one_Axis(X_AXIS,disp,distXY, distZ, feedrate);
  laser_calibrate_start_move(disp);
  laser_cycle_one_Axis(Y_AXIS,disp,distXY, distZ, feedrate);
  update_workspace_offset_Z(disp);
  laser_calibrate_start_move(disp);
}

void GcodeSuite::laser_cycle_one_Axis(AxisEnum AxisXY, int8_t disp,float distXY, float distZ,float feedrate)
{
  endstops.enable(true);
  abce_pos_t st_pos_beg = planner.get_axis_positions_mm();
  float xy_dhm_cur = planner.triggered_position_mm(AxisXY);
  Serial.print("st_pos_beg.x: ");
  Serial.print(st_pos_beg.x);
  Serial.print("xy_dhm_cur: ");
  Serial.println(xy_dhm_cur);
  AxisEnum AxisZ;
  switch (disp)
  {
  case 0:
    AxisZ = Z_AXIS;
    break;
  case 1:
    AxisZ = I_AXIS;
    break;
  case 2:
    AxisZ = J_AXIS;
    break;
  default:
    AxisZ = Z_AXIS;
    break;
  }

  laser_cycle(AxisXY,AxisZ,distXY,distZ,feedrate, &xy_dhm_cur);
  laser_cycle(AxisXY,AxisZ,distXY/5,distZ/10,feedrate/5, &xy_dhm_cur);
  laser_cycle(AxisXY,AxisZ,distXY/25,distZ/100,feedrate/10, &xy_dhm_cur);

  abce_pos_t st_pos_end = planner.get_axis_positions_mm();

  switch (disp)
  {
  case 0:
    position_shift_Z[AxisXY] = st_pos_end[AxisXY] - st_pos_beg[AxisXY];
    position_shift_Z[AxisZ] = st_pos_end[AxisZ] - st_pos_beg[AxisZ];
    break;
  case 1:
    position_shift_A[AxisXY] = st_pos_end[AxisXY] - st_pos_beg[AxisXY];
    position_shift_A[AxisZ] = st_pos_end[AxisZ] - st_pos_beg[AxisZ];
    break;
  case 2:
    position_shift_B[AxisXY] = st_pos_end[AxisXY] - st_pos_beg[AxisXY];
    position_shift_B[AxisZ] = st_pos_end[AxisZ] - st_pos_beg[AxisZ];
    break;
  default:
    break;
  } 

  endstops.not_homing();
  endstops.enable(false);
}



void GcodeSuite::laser_calibrate_start_move(int8_t disp)
{
  LOOP_NUM_AXES(i) 
  {
    switch (disp)
    {
      case 0: 
        destination[i] =  home_offset_Z[i];
        break;  
      case 1:
        destination[i] =  home_offset_A[i];
        break; 
      case 2:
        destination[i] =  home_offset_B[i];
        break; 
      default:
        destination[i] =  current_position[i];
        break;
    }
  } 

  prepare_line_to_destination();
  planner.set_machine_position_mm(destination);
}

void GcodeSuite::laser_cycle(AxisEnum AxisXY, AxisEnum AxisZ,float dist, float z, float feedrate,float* xy_dhm)
{
  
  float dhm = *xy_dhm;
  int8_t count_cycle = 0;
  do_homing_move_laser(AxisXY,-dist/2,feedrate,false);
  
  while(abs(*xy_dhm-dhm)<0.0001  && count_cycle<20)
  {
    do_homing_move_laser(AxisZ,-z,feedrate,false);
    do_homing_move_laser(AxisXY,dist,feedrate,false);
    do_homing_move_laser(AxisXY,-dist,feedrate,false);
    *xy_dhm = planner.triggered_position_mm(AxisXY);
    count_cycle++;
  }
  if (count_cycle==0)
  {
    do_homing_move_laser(AxisXY,dist/2,feedrate,false);
  }
  abce_pos_t cur_pos = planner.get_axis_positions_mm();
  cur_pos[AxisXY] = *xy_dhm;
  planner.set_machine_position_mm(cur_pos);
  do_homing_move_laser(AxisZ,1.2f*z,feedrate,false);
}
