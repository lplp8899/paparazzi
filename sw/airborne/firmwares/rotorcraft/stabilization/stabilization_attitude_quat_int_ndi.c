/*
 * Copyright (C) 2008-2009 Antoine Drouin <poinix@gmail.com>
 *
 * This file is part of paparazzi.
 *
 * paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with paparazzi; see the file COPYING.  If not, write to
 * the Free Software Foundation, 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 */

/** @file stabilization_attitude_quat_int_ndi.c
 * Rotorcraft quaternion attitude stabilization
 */

#include "generated/airframe.h"

#include "firmwares/rotorcraft/stabilization/stabilization_attitude.h"
#include "firmwares/rotorcraft/stabilization/stabilization_attitude_rc_setpoint.h"
#include "firmwares/rotorcraft/stabilization/stabilization_attitude_quat_transformations.h"

#include "std.h"
#include "paparazzi.h"
#include "math/pprz_algebra_float.h"
#include "math/pprz_algebra_int.h"
#include "state.h"

struct Int32AttitudeGains stabilization_gains = {
  {STABILIZATION_ATTITUDE_PHI_PGAIN, STABILIZATION_ATTITUDE_THETA_PGAIN, STABILIZATION_ATTITUDE_PSI_PGAIN },
  {STABILIZATION_ATTITUDE_PHI_DGAIN, STABILIZATION_ATTITUDE_THETA_DGAIN, STABILIZATION_ATTITUDE_PSI_DGAIN },
  {STABILIZATION_ATTITUDE_PHI_DDGAIN, STABILIZATION_ATTITUDE_THETA_DDGAIN, STABILIZATION_ATTITUDE_PSI_DDGAIN },
  {STABILIZATION_ATTITUDE_PHI_IGAIN, STABILIZATION_ATTITUDE_THETA_IGAIN, STABILIZATION_ATTITUDE_PSI_IGAIN }
};

/* warn if some gains are still negative */
#if (STABILIZATION_ATTITUDE_PHI_PGAIN < 0) ||   \
  (STABILIZATION_ATTITUDE_THETA_PGAIN < 0) ||   \
  (STABILIZATION_ATTITUDE_PSI_PGAIN < 0)   ||   \
  (STABILIZATION_ATTITUDE_PHI_DGAIN < 0)   ||   \
  (STABILIZATION_ATTITUDE_THETA_DGAIN < 0) ||   \
  (STABILIZATION_ATTITUDE_PSI_DGAIN < 0)   ||   \
  (STABILIZATION_ATTITUDE_PHI_IGAIN < 0)   ||   \
  (STABILIZATION_ATTITUDE_THETA_IGAIN < 0) ||   \
  (STABILIZATION_ATTITUDE_PSI_IGAIN  < 0)
#error "ALL control gains have to be positive!!!"
#endif

struct Int32Quat stabilization_att_sum_err_quat;

int32_t stabilization_att_fb_cmd[COMMANDS_NB];
int32_t stabilization_att_ff_cmd[COMMANDS_NB];

/*
#define IERROR_SCALE 1024
#define GAIN_PRESCALER_FF 48
#define GAIN_PRESCALER_P 48
#define GAIN_PRESCALER_D 48
#define GAIN_PRESCALER_I 48
*/
#define IERROR_SCALE 1024
#define GAIN_PRESCALER_FF 48
#define GAIN_PRESCALER_P 12
#define GAIN_PRESCALER_D 3
#define GAIN_PRESCALER_I 24


#if PERIODIC_TELEMETRY
#include "subsystems/datalink/telemetry.h"


//----------------- added for NDI -----------------
// global variables
//struct FloatRates V_w = {0., 0., 0.};
struct FloatRates w_filter = {0., 0., 0.};
struct FloatRates w_dot_0 = {0., 0., 0.};
struct FloatRates w_ddot_filter = {0., 0., 0.};
struct FloatRates de_in = {0., 0., 0.};
struct FloatRates in_0 = {0., 0., 0.};
struct FloatRates in_c = {0., 0., 0.};
struct FloatRates in = {0., 0., 0.};
struct FloatRates in_0_f = {0., 0., 0.}; 
struct FloatRates in_dot_filter = {0., 0., 0.};
struct FloatRates in_ddot_filter = {0., 0., 0.};


struct FloatRates g1 = { 0.01, 0.01, 0.02 };
float tau_act = 0.06; //0.02 actuator time constant
float zeta_f = 0.8;
float omega_f = 30.;





//----------------- added for NDI -----------------


static void send_att(struct transport_tx *trans, struct link_device *dev)   //FIXME really use this message here ?
{
  struct Int32Rates *body_rate = stateGetBodyRates_i();
  struct Int32Eulers *att = stateGetNedToBodyEulers_i();
  pprz_msg_send_STAB_ATTITUDE_INT(trans, dev, AC_ID,
                                  &(body_rate->p), &(body_rate->q), &(body_rate->r),
                                  &(att->phi), &(att->theta), &(att->psi),
                                  &stab_att_sp_euler.phi,
                                  &stab_att_sp_euler.theta,
                                  &stab_att_sp_euler.psi,
                                  &stabilization_att_sum_err_quat.qx,
                                  &stabilization_att_sum_err_quat.qy,
                                  &stabilization_att_sum_err_quat.qz,
                                  &stabilization_att_fb_cmd[COMMAND_ROLL],
                                  &stabilization_att_fb_cmd[COMMAND_PITCH],
                                  &stabilization_att_fb_cmd[COMMAND_YAW],
                                  &stabilization_att_ff_cmd[COMMAND_ROLL],
                                  &stabilization_att_ff_cmd[COMMAND_PITCH],
                                  &stabilization_att_ff_cmd[COMMAND_YAW],
                                  &stabilization_cmd[COMMAND_ROLL],
                                  &stabilization_cmd[COMMAND_PITCH],
                                  &stabilization_cmd[COMMAND_YAW]);
}

static void send_att_ref(struct transport_tx *trans, struct link_device *dev)
{
  pprz_msg_send_STAB_ATTITUDE_REF_INT(trans, dev, AC_ID,
                                      &stab_att_sp_euler.phi,
                                      &stab_att_sp_euler.theta,
                                      &stab_att_sp_euler.psi,
                                      &stab_att_ref_euler.phi,
                                      &stab_att_ref_euler.theta,
                                      &stab_att_ref_euler.psi,
                                      &stab_att_ref_rate.p,
                                      &stab_att_ref_rate.q,
                                      &stab_att_ref_rate.r,
                                      &stab_att_ref_accel.p,
                                      &stab_att_ref_accel.q,
                                      &stab_att_ref_accel.r);
}

static void send_ahrs_ref_quat(struct transport_tx *trans, struct link_device *dev)
{
  struct Int32Quat *quat = stateGetNedToBodyQuat_i();
  pprz_msg_send_AHRS_REF_QUAT(trans, dev, AC_ID,
                              &stab_att_ref_quat.qi,
                              &stab_att_ref_quat.qx,
                              &stab_att_ref_quat.qy,
                              &stab_att_ref_quat.qz,
                              &(quat->qi),
                              &(quat->qx),
                              &(quat->qy),
                              &(quat->qz));
}
#endif

void stabilization_attitude_init(void)
{

  stabilization_attitude_ref_init();

  int32_quat_identity(&stabilization_att_sum_err_quat);

#if PERIODIC_TELEMETRY
  register_periodic_telemetry(DefaultPeriodic, "STAB_ATTITUDE", send_att);
  register_periodic_telemetry(DefaultPeriodic, "STAB_ATTITUDE_REF", send_att_ref);
  register_periodic_telemetry(DefaultPeriodic, "AHRS_REF_QUAT", send_ahrs_ref_quat);
#endif
}

void stabilization_attitude_enter(void)
{

  /* reset psi setpoint to current psi angle */
  stab_att_sp_euler.psi = stabilization_attitude_get_heading_i();

  stabilization_attitude_ref_enter();

  int32_quat_identity(&stabilization_att_sum_err_quat);
  
  //
  FLOAT_RATES_ZERO(in_0);
  FLOAT_RATES_ZERO(in_c);
  FLOAT_RATES_ZERO(in);
  FLOAT_RATES_ZERO(de_in);
  FLOAT_RATES_ZERO(in_0_f);
  FLOAT_RATES_ZERO(in_dot_filter);
  FLOAT_RATES_ZERO(in_ddot_filter);
  FLOAT_RATES_ZERO(w_filter);
  FLOAT_RATES_ZERO(w_dot_0);
  FLOAT_RATES_ZERO(w_ddot_filter);

  
}

void stabilization_attitude_set_failsafe_setpoint(void)
{
  /* set failsafe to zero roll/pitch and current heading */
  int32_t heading2 = stabilization_attitude_get_heading_i() / 2;
  PPRZ_ITRIG_COS(stab_att_sp_quat.qi, heading2);
  stab_att_sp_quat.qx = 0;
  stab_att_sp_quat.qy = 0;
  PPRZ_ITRIG_SIN(stab_att_sp_quat.qz, heading2);
  
}

void stabilization_attitude_set_rpy_setpoint_i(struct Int32Eulers *rpy)
{
  // stab_att_sp_euler.psi still used in ref..
  memcpy(&stab_att_sp_euler, rpy, sizeof(struct Int32Eulers));

  int32_quat_of_eulers(&stab_att_sp_quat, &stab_att_sp_euler);
}

void stabilization_attitude_set_earth_cmd_i(struct Int32Vect2 *cmd, int32_t heading)
{
  // stab_att_sp_euler.psi still used in ref..
  stab_att_sp_euler.psi = heading;

  //
  float heading_f = heading * (1>>INT32_ANGLE_FRAC);
  //printf("heading: %i\n",&heading);
  printf("heading_f: %f\n",&heading_f);
  
  // compute sp_euler phi/theta for debugging/telemetry
  /* Rotate horizontal commands to body frame by psi */
  int32_t psi = stateGetNedToBodyEulers_i()->psi;
  
  //
  float psi_f = psi *(1 >> INT32_ANGLE_FRAC );
  printf("psi: %f\n",&psi_f);
  
  int32_t s_psi, c_psi;
  PPRZ_ITRIG_SIN(s_psi, psi);
  PPRZ_ITRIG_COS(c_psi, psi);
  stab_att_sp_euler.phi = (-s_psi * cmd->x + c_psi * cmd->y) >> INT32_TRIG_FRAC;
  stab_att_sp_euler.theta = -(c_psi * cmd->x + s_psi * cmd->y) >> INT32_TRIG_FRAC;

  quat_from_earth_cmd_i(&stab_att_sp_quat, cmd, heading);
}

#define OFFSET_AND_ROUND(_a, _b) (((_a)+(1<<((_b)-1)))>>(_b))
#define OFFSET_AND_ROUND2(_a, _b) (((_a)+(1<<((_b)-1))-((_a)<0?1:0))>>(_b))


static void attitude_run_fb(int32_t Com_actu[], struct Int32AttitudeGains *gains, struct Int32Quat *att_err,    struct Int32Rates *rate_err, struct Int32Quat *sum_err, struct Int32Rates *body_rate)
{
  /*  PID feedback */
  //int32_t fb_commands[];
  //fb_commands[COMMAND_ROLL] =
  struct FloatRates V_w;
  
  V_w.p = 
    GAIN_PRESCALER_P * gains->p.x  * QUAT1_FLOAT_OF_BFP(att_err->qx) +
    GAIN_PRESCALER_D * gains->d.x  * RATE_FLOAT_OF_BFP(rate_err->p)  +
    GAIN_PRESCALER_I * gains->i.x  * QUAT1_FLOAT_OF_BFP(sum_err->qx) ;

  //fb_commands[COMMAND_PITCH] =
  V_w.q = 
    GAIN_PRESCALER_P * gains->p.y  * QUAT1_FLOAT_OF_BFP(att_err->qy)  +
    GAIN_PRESCALER_D * gains->d.y  * RATE_FLOAT_OF_BFP(rate_err->q)   +
    GAIN_PRESCALER_I * gains->i.y  * QUAT1_FLOAT_OF_BFP(sum_err->qy) ;

  //fb_commands[COMMAND_YAW] =
  V_w.r = 
    GAIN_PRESCALER_P * gains->p.z  * QUAT1_FLOAT_OF_BFP(att_err->qz) +
    GAIN_PRESCALER_D * gains->d.z  * RATE_FLOAT_OF_BFP(rate_err->r)  +
    GAIN_PRESCALER_I * gains->i.z  * QUAT1_FLOAT_OF_BFP(sum_err->qz) ;
    
  

     
   // please refer to the paper about quadrotor
   // however, in the reference, not err_p, but p
   // virtual control for rate loop
   // V_w = fb_commands;
   de_in.p = 1.0 / g1.p * ( V_w.p - w_dot_0.p );
   de_in.q = 1.0 / g1.q * ( V_w.q - w_dot_0.q );
   de_in.r = 1.0 / g1.r * ( V_w.r - w_dot_0.r );
   
   
   // final command for the actuator  
   // uc = u0 + du    
   in_c.p = in_0_f.p + de_in.p;
   in_c.q = in_0_f.q + de_in.q;
   in_c.r = in_0_f.r + de_in.r;
    
      
   //bound the total control input
   
  Bound(in_c.p, -4500, 4500);
  Bound(in_c.q, -4500, 4500);
  Bound(in_c.r, -4500, 4500);
     
   // 
   // ? should convert to int!
   Com_actu[COMMAND_ROLL] = in_c.p; //?
   Com_actu[COMMAND_PITCH] = in_c.q;//?
   Com_actu[COMMAND_YAW] = in_c.r;//?
   
   
   // predict the present actuator deflection 
   // in = act(in_c);
   in.p = in.p + (in_c.p - in.p) / tau_act / PERIODIC_FREQUENCY;
   in.q = in.q + (in_c.q - in.q) / tau_act / PERIODIC_FREQUENCY;
   in.r = in.r + (in_c.r - in.r) / tau_act / PERIODIC_FREQUENCY;
   
   //printf("%d\n", PERIODIC_FREQUENCY);
   //printf("%f\n", de_in.p);
         
   // do not forget to add the same delay to the actuator commands
   // low_pass_filter_2nd_order 
   // u_0_f = filter_2nd(u)
   filter_low_pass_2nd_scalar(&in_0_f.p, &in_dot_filter.p, &in_ddot_filter.p, &zeta_f, &omega_f, &in.p);
   filter_low_pass_2nd_scalar(&in_0_f.q, &in_dot_filter.q, &in_ddot_filter.q, &zeta_f, &omega_f, &in.q);
   filter_low_pass_2nd_scalar(&in_0_f.r, &in_dot_filter.r, &in_ddot_filter.r, &zeta_f, &omega_f, &in.r); 
   
    ///*
   //Don't increment if thrust is off
   // this is according to Ewoud's suggestion
  if (stabilization_cmd[COMMAND_THRUST] < 300) {
    FLOAT_RATES_ZERO(in);
    FLOAT_RATES_ZERO(in_c);
    FLOAT_RATES_ZERO(de_in);
    //FLOAT_RATES_ZERO(in_0_f);

  }
   //*/
   
}





void stabilization_attitude_run(bool_t enable_integrator)
{

  /*
   * Update reference
   */
  stabilization_attitude_ref_update();

  /*
   * Compute errors for feedback
   */

  /* attitude error                          */
  struct Int32Quat att_err;
  struct Int32Quat *att_quat = stateGetNedToBodyQuat_i();
  INT32_QUAT_INV_COMP(att_err, *att_quat, stab_att_ref_quat);
  /* wrap it in the shortest direction       */
  int32_quat_wrap_shortest(&att_err);
  int32_quat_normalize(&att_err);
  
  // euqal to 
  INT32_QUAT_INV_COMP_NORM_SHORTEST(att_err, *att_quat, stab_att_ref_quat);
  
  
  /*  rate error                */
  const struct Int32Rates rate_ref_scaled = {
    OFFSET_AND_ROUND(stab_att_ref_rate.p, (REF_RATE_FRAC - INT32_RATE_FRAC)),
    OFFSET_AND_ROUND(stab_att_ref_rate.q, (REF_RATE_FRAC - INT32_RATE_FRAC)),
    OFFSET_AND_ROUND(stab_att_ref_rate.r, (REF_RATE_FRAC - INT32_RATE_FRAC))
  };
  struct Int32Rates rate_err;
  struct Int32Rates *body_rate = stateGetBodyRates_i();
  RATES_DIFF(rate_err, rate_ref_scaled, (*body_rate));

#define INTEGRATOR_BOUND 100000
  /* integrated error */
  if (enable_integrator) {
    stabilization_att_sum_err_quat.qx += att_err.qx / IERROR_SCALE;
    stabilization_att_sum_err_quat.qy += att_err.qy / IERROR_SCALE;
    stabilization_att_sum_err_quat.qz += att_err.qz / IERROR_SCALE;
    Bound(stabilization_att_sum_err_quat.qx, -INTEGRATOR_BOUND, INTEGRATOR_BOUND);
    Bound(stabilization_att_sum_err_quat.qy, -INTEGRATOR_BOUND, INTEGRATOR_BOUND);
    Bound(stabilization_att_sum_err_quat.qz, -INTEGRATOR_BOUND, INTEGRATOR_BOUND);
  } else {
    /* reset accumulator */
    int32_quat_identity(&stabilization_att_sum_err_quat);
  }



  // calculate the rate dot
  struct FloatRates *body_rate_f = stateGetBodyRates_f();
  //
  printf("rate_psi: %f\n",body_rate_f->r);
  
  filter_low_pass_2nd_scalar(&w_filter.p, &w_dot_0.p, &w_ddot_filter.p, &zeta_f, &omega_f, &(*body_rate_f).p );
  filter_low_pass_2nd_scalar(&w_filter.q, &w_dot_0.q, &w_ddot_filter.q, &zeta_f, &omega_f, &(*body_rate_f).q );
  filter_low_pass_2nd_scalar(&w_filter.r, &w_dot_0.r, &w_ddot_filter.r, &zeta_f, &omega_f, &(*body_rate_f).r );
  //w_dot_0 = w_dot_filter;
 
  
  /* compute the feed forward command */
  // attitude_run_ff(stabilization_att_ff_cmd, &stabilization_gains, &stab_att_ref_accel);

  /* compute the feed back command */
  attitude_run_fb(stabilization_att_fb_cmd, &stabilization_gains, &att_err, &rate_err, &stabilization_att_sum_err_quat, body_rate);

  /* sum feedforward and feedback */
  stabilization_cmd[COMMAND_ROLL] = stabilization_att_fb_cmd[COMMAND_ROLL] ;
  stabilization_cmd[COMMAND_PITCH] = stabilization_att_fb_cmd[COMMAND_PITCH];
  stabilization_cmd[COMMAND_YAW] = stabilization_att_fb_cmd[COMMAND_YAW];

  /* bound the result */
  BoundAbs(stabilization_cmd[COMMAND_ROLL], MAX_PPRZ);
  BoundAbs(stabilization_cmd[COMMAND_PITCH], MAX_PPRZ);
  BoundAbs(stabilization_cmd[COMMAND_YAW], MAX_PPRZ);
}

void stabilization_attitude_read_rc(bool_t in_flight, bool_t in_carefree, bool_t coordinated_turn)
{
  struct FloatQuat q_sp;
#if USE_EARTH_BOUND_RC_SETPOINT
  stabilization_attitude_read_rc_setpoint_quat_earth_bound_f(&q_sp, in_flight, in_carefree, coordinated_turn);
#else
  stabilization_attitude_read_rc_setpoint_quat_f(&q_sp, in_flight, in_carefree, coordinated_turn);
#endif
  QUAT_BFP_OF_REAL(stab_att_sp_quat, q_sp);
}


//---------------------------- added for NDI
void filter_low_pass_2nd_scalar(float *q, float *qd, float *qdd, float *zeta , float *omega , float *qi)
// input: qi, 
//output: q, qd, qdd
{
   float zeta_omega = 2. * (*zeta) * (*omega);
   float omega_2 = (*omega) * (*omega);
   //printf("Value of *pPointer1: %f\n", *omega); 
   float q_v1, q_v2, q_v3;
   q_v1 = *q;
   q_v2 = *qd;
   float q_dot1 = q_v2;
   float q_dot2 = omega_2 * ( (*qi) - q_v1 ) - zeta_omega * q_v2;
   //printf("Value of *pPointer1: %f\n", q_dot1); 
   //printf("Value of *pPointer1: %f\n", q_dot2); 
     
   float dt = 1./PERIODIC_FREQUENCY;
   q_v1 = q_v1 + q_dot1 * dt;
   q_v2 = q_v2 + q_dot2 * dt;
   q_v3 = omega_2 * ( (*qi) - q_v1 ) - zeta_omega * q_v2 ;
   //printf("Value of *pPointer1: %f\n", q_v1);  
   //printf("Value of *pPointer1: %f\n", q_v2); 
   *q = q_v1; // do not use q = &(q_v1), change the number pointed by the pointer
   *qd = q_v2;
   *qdd = q_v3;
   //printf("Value of q: %f\n", *q); 
   //printf("Value of qd: %f\n", *qd); 
   //printf("Value of qdd: %f\n", *qdd);     
   
    //cannot just use qdd = (omega_2 * ( (*qi) - (*q) ) - zeta_omega *(*qd)); 
    //also cannot use qdd = & (...) since the ... is a constant                                           

}

/*
void filter_low_pass_2nd_Vect3(float *q, float *qd, float *qdd, float zeta, float omega, float *qi, float *q1, float *q2)
{

}
*/



