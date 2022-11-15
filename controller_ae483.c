#include "controller_ae483.h"
#include "stabilizer_types.h"
#include "power_distribution.h"
#include "log.h"
#include "param.h"
#include "num.h"
#include "math3d.h"

// Sensor measurements
// - tof (from the z ranger on the flow deck)
static uint16_t tof_count = 0;
static float tof_distance = 0.0f;
// - flow (from the optical flow sensor on the flow deck)
static uint16_t flow_count = 0;
static float flow_dpixelx = 0.0f;
static float flow_dpixely = 0.0f;

// Parameters
static bool use_observer = false;
static bool reset_observer = false;


// State
static float o_x = 0.0f;
static float o_y = 0.0f;
static float o_z = 0.0f;
static float psi = 0.0f;
static float theta = 0.0f;
static float phi = 0.0f;
static float v_x = 0.0f;
static float v_y = 0.0f;
static float v_z = 0.0f;
static float w_x = 0.0f;
static float w_y = 0.0f;
static float w_z = 0.0f;

// Setpoint
static float o_x_des = 0.0f;
static float o_y_des = 0.0f;
static float o_z_des = 0.0f;

// Input
static float tau_x = 0.0f;
static float tau_y = 0.0f;
static float tau_z = 0.0f;
static float f_z = 0.0f;

// Motor power command
static uint16_t m_1 = 0;
static uint16_t m_2 = 0;
static uint16_t m_3 = 0;
static uint16_t m_4 = 0;

// Measurements
static float n_x = 0.0f;
static float n_y = 0.0f;
static float r = 0.0f;
static float a_z = 0.0f;

// Loco Position Measurments
static float x0 = 0.0f;
static float y_0 = 0.0f;
static float z0 = 0.0f;
static float d0 = 0.0f;
static float x1 = 0.0f;
static float y_1 = 0.0f;
static float z1 = 0.0f;
static float d1 = 0.0f;
static float x2 = 0.0f;
static float y2 = 0.0f;
static float z2 = 0.0f;
static float d2 = 0.0f;
static float x3 = 0.0f;
static float y3 = 0.0f;
static float z3 = 0.0f;
static float d3 = 0.0f;
static float x4 = 0.0f;
static float y4 = 0.0f;
static float z4 = 0.0f;
static float d4 = 0.0f;
static float x5 = 0.0f;
static float y5 = 0.0f;
static float z5 = 0.0f;
static float d5 = 0.0f;
static float x6 = 0.0f;
static float y6 = 0.0f;
static float z6 = 0.0f;
static float d6 = 0.0f;
static float x7 = 0.0f;
static float y7 = 0.0f;
static float z7 = 0.0f;
static float d7 = 0.0f;

// Constants
// static float k_flow = 4.09255568f;
static float g = 9.81f;
static float dt = 0.002f;
// static float o_z_eq = 0.5f; // FIXME: replace with your choice of equilibrium height

// Measurement errors
// static float n_x_err = 0.0f;
// static float n_y_err = 0.0f;
// static float r_err = 0.0f;


void ae483UpdateWithTOF(tofMeasurement_t *tof)
{
  tof_distance = tof->distance;
  tof_count++;
}

void ae483UpdateWithFlow(flowMeasurement_t *flow)
{
  flow_dpixelx = flow->dpixelx;
  flow_dpixely = flow->dpixely;
  flow_count++;
}

void ae483UpdateWithDistance(distanceMeasurement_t *meas)
{
  // If you have a loco positioning deck, this function will be called
  // each time a distance measurement is available. You will have to write
  // code to handle these measurements. These data are available:
  //
  //  meas->anchorId  uint8_t   id of anchor with respect to which distance was measured
  //  meas->x         float     x position of this anchor
  //  meas->y         float     y position of this anchor
  //  meas->z         float     z position of this anchor
  //  meas->distance  float     the measured distance

  if (meas->anchorId == 0) {
    // Get position of node
    x0 = meas->x;
    y_0 = meas->y;
    z0 = meas->z;

    // Get distance from tag to node
    d0 = meas->distance;
  }
  if (meas->anchorId == 1) {
    // Get position of node
    x1 = meas->x;
    y_1 = meas->y;
    z1 = meas->z;

    // Get distance from tag to node
    d1 = meas->distance;
  }
  if (meas->anchorId == 2) {
    // Get position of node
    x2 = meas->x;
    y2 = meas->y;
    z2 = meas->z;

    // Get distance from tag to node
    d2 = meas->distance;
  }
  if (meas->anchorId == 3) {
    // Get position of node
    x3 = meas->x;
    y3 = meas->y;
    z3 = meas->z;

    // Get distance from tag to node
    d3 = meas->distance;
  }
  if (meas->anchorId == 4) {
    // Get position of node
    x4 = meas->x;
    y4 = meas->y;
    z4 = meas->z;

    // Get distance from tag to node
    d4 = meas->distance;
  }
  if (meas->anchorId == 5) {
    // Get position of node
    x5 = meas->x;
    y5 = meas->y;
    z5 = meas->z;

    // Get distance from tag to node
    d5 = meas->distance;
  }
  if (meas->anchorId == 7) {
    // Get position of node
    x6 = meas->x;
    y6 = meas->y;
    z6 = meas->z;

    // Get distance from tag to node
    d6 = meas->distance;
  }
  if (meas->anchorId == 7) {
    // Get position of node
    x7 = meas->x;
    y7 = meas->y;
    z7 = meas->z;

    // Get distance from tag to node
    d7 = meas->distance;
  }
}

void ae483UpdateWithPosition(positionMeasurement_t *meas)
{
  // This function will be called each time you send an external position
  // measurement (x, y, z) from the client, e.g., from a motion capture system.
  // You will have to write code to handle these measurements. These data are
  // available:
  //
  //  meas->x         float     x component of external position measurement
  //  meas->y         float     y component of external position measurement
  //  meas->z         float     z component of external position measurement
}

void ae483UpdateWithPose(poseMeasurement_t *meas)
{
  // This function will be called each time you send an external "pose" measurement
  // (position as x, y, z and orientation as quaternion) from the client, e.g., from
  // a motion capture system. You will have to write code to handle these measurements.
  // These data are available:
  //
  //  meas->x         float     x component of external position measurement
  //  meas->y         float     y component of external position measurement
  //  meas->z         float     z component of external position measurement
  //  meas->quat.x    float     x component of quaternion from external orientation measurement
  //  meas->quat.y    float     y component of quaternion from external orientation measurement
  //  meas->quat.z    float     z component of quaternion from external orientation measurement
  //  meas->quat.w    float     w component of quaternion from external orientation measurement
}

void ae483UpdateWithData(const struct AE483Data* data)
{
  // This function will be called each time AE483-specific data are sent
  // from the client to the drone. You will have to write code to handle
  // these data. For the example AE483Data struct, these data are:
  //
  //  data->x         float
  //  data->y         float
  //  data->z         float
  //
  // Exactly what "x", "y", and "z" mean in this context is up to you.
}


void controllerAE483Init(void)
{
  // Do nothing
}

bool controllerAE483Test(void)
{
  // Do nothing (test is always passed)
  return true;
}

void controllerAE483(control_t *control,
                     setpoint_t *setpoint,
                     const sensorData_t *sensors,
                     const state_t *state,
                     const uint32_t tick)
{
    if (RATE_DO_EXECUTE(ATTITUDE_RATE, tick)) {
    // Everything in here runs at 500 Hz

    // Desired position
    o_x_des = setpoint->position.x;
    o_y_des = setpoint->position.y;
    o_z_des = setpoint->position.z;

    // Measurements
    w_x = radians(sensors->gyro.x);
    w_y = radians(sensors->gyro.y);
    w_z = radians(sensors->gyro.z);
    a_z = g * sensors->acc.z;
    n_x = flow_dpixelx;
    n_y = flow_dpixely;
    r = tof_distance;

    if (reset_observer) {
      o_x = 0.0f;
      o_y = 0.0f;
      o_z = 0.0f;
      psi = 0.0f;
      theta = 0.0f;
      phi = 0.0f;
      v_x = 0.0f;
      v_y = 0.0f;
      v_z = 0.0f;
      reset_observer = false;
    }

    // State estimates
    if (use_observer) {
    
      // Compute each element of:
      // 
      //   C x + D u - y
      // 
      // FIXME: your code goes here

      
      // Update estimates
      // FIXME: your code goes here
      o_x += dt * v_x;
      o_y += dt * v_y;
      o_z += (-2.30809f*(o_z - r) + v_z) * dt;
      psi += dt * w_z;
      theta += (-0.016509f*(-n_x + 8.18511f*v_x - 4.09256f*w_y) + w_y) * dt;
      phi += (w_x + 0.020713f*(-n_y + 8.18511f*v_y + 4.09256f*w_x)) * dt;
      v_x += (9.81f*theta - 0.270954f*(-n_x + 8.18511f*v_x - 4.09256f*w_y)) * dt;
      v_y += (-9.81f*phi - 0.296135f*(-n_y + 8.18511f*v_y + 4.09256f*w_x)) * dt;
      v_z += (a_z - g - 2.5946f*(o_z - r)) * dt;

    } else {
      o_x = state->position.x;
      o_y = state->position.y;
      o_z = state->position.z;
      psi = radians(state->attitude.yaw);
      theta = - radians(state->attitude.pitch);
      phi = radians(state->attitude.roll);
      v_x = state->velocity.x*cosf(psi)*cosf(theta) + state->velocity.y*sinf(psi)*cosf(theta) - state->velocity.z*sinf(theta);
      v_y = state->velocity.x*(sinf(phi)*sinf(theta)*cosf(psi) - sinf(psi)*cosf(phi)) + state->velocity.y*(sinf(phi)*sinf(psi)*sinf(theta) + cosf(phi)*cosf(psi)) + state->velocity.z*sinf(phi)*cosf(theta);
      v_z = state->velocity.x*(sinf(phi)*sinf(psi) + sinf(theta)*cosf(phi)*cosf(psi)) + state->velocity.y*(-sinf(phi)*cosf(psi) + sinf(psi)*sinf(theta)*cosf(phi)) + state->velocity.z*cosf(phi)*cosf(theta);
    }

    if (setpoint->mode.z == modeDisable) {
      // If there is no desired position, then all
      // motor power commands should be zero

      powerSet(0, 0, 0, 0);
    } else {
      // Otherwise, motor power commands should be
      // chosen by the controller

      tau_x = 0.00141421f * (o_y - o_y_des) -0.00555862f * phi + 0.00145006f * v_y -0.00108379f * w_x;
      tau_y = -0.00141421f * (o_x - o_x_des) -0.00461078f * theta -0.00135255f * v_x -0.00079774f * w_y;
      tau_z = -0.00100000f * psi -0.00074738f * w_z;
      f_z = -0.31622777f * (o_z - o_z_des) -0.17261477f * v_z + 0.30705300f;
      
      // FIXMEs
      m_1 = limitUint16( -4144218.8f * tau_x -4144218.8f * tau_y -56818181.8f * tau_z + 131578.9f * f_z );
      m_2 = limitUint16( -4144218.8f * tau_x + 4144218.8f * tau_y + 56818181.8f * tau_z + 131578.9f * f_z );
      m_3 = limitUint16( 4144218.8f * tau_x + 4144218.8f * tau_y -56818181.8f * tau_z + 131578.9f * f_z );
      m_4 = limitUint16( 4144218.8f * tau_x -4144218.8f * tau_y + 56818181.8f * tau_z + 131578.9f * f_z );
      
      // Apply motor power commands
      powerSet(m_1, m_2, m_3, m_4);
    }
  }
}

LOG_GROUP_START(ae483log)
LOG_ADD(LOG_UINT16,         num_tof,                &tof_count)
LOG_ADD(LOG_UINT16,         num_flow,               &flow_count)
LOG_ADD(LOG_FLOAT,          o_x,                    &o_x)
LOG_ADD(LOG_FLOAT,          o_y,                    &o_y)
LOG_ADD(LOG_FLOAT,          o_z,                    &o_z)
LOG_ADD(LOG_FLOAT,          psi,                    &psi)
LOG_ADD(LOG_FLOAT,          theta,                  &theta)
LOG_ADD(LOG_FLOAT,          phi,                    &phi)
LOG_ADD(LOG_FLOAT,          v_x,                    &v_x)
LOG_ADD(LOG_FLOAT,          v_y,                    &v_y)
LOG_ADD(LOG_FLOAT,          v_z,                    &v_z)
LOG_ADD(LOG_FLOAT,          w_x,                    &w_x)
LOG_ADD(LOG_FLOAT,          w_y,                    &w_y)
LOG_ADD(LOG_FLOAT,          w_z,                    &w_z)
LOG_ADD(LOG_FLOAT,          o_x_des,                &o_x_des)
LOG_ADD(LOG_FLOAT,          o_y_des,                &o_y_des)
LOG_ADD(LOG_FLOAT,          o_z_des,                &o_z_des)
LOG_ADD(LOG_FLOAT,          tau_x,                  &tau_x)
LOG_ADD(LOG_FLOAT,          tau_y,                  &tau_y)
LOG_ADD(LOG_FLOAT,          tau_z,                  &tau_z)
LOG_ADD(LOG_FLOAT,          f_z,                    &f_z)
LOG_ADD(LOG_UINT16,         m_1,                    &m_1)
LOG_ADD(LOG_UINT16,         m_2,                    &m_2)
LOG_ADD(LOG_UINT16,         m_3,                    &m_3)
LOG_ADD(LOG_UINT16,         m_4,                    &m_4)
LOG_ADD(LOG_FLOAT,          n_x,                    &n_x)
LOG_ADD(LOG_FLOAT,          n_y,                    &n_y)
LOG_ADD(LOG_FLOAT,          r,                      &r)
LOG_ADD(LOG_FLOAT,          a_z,                    &a_z)
LOG_ADD(LOG_FLOAT,          d0,                     &d0)
LOG_ADD(LOG_FLOAT,          x0,                     &x0)
LOG_ADD(LOG_FLOAT,          y_0,                     &y_0)
LOG_ADD(LOG_FLOAT,          z0,                     &z0)
LOG_ADD(LOG_FLOAT,          d1,                     &d1)
LOG_ADD(LOG_FLOAT,          x1,                     &x1)
LOG_ADD(LOG_FLOAT,          y_1,                     &y_1)
LOG_ADD(LOG_FLOAT,          z1,                     &z1)
LOG_ADD(LOG_FLOAT,          d2,                     &d2)
LOG_ADD(LOG_FLOAT,          x2,                     &x2)
LOG_ADD(LOG_FLOAT,          y2,                     &y2)
LOG_ADD(LOG_FLOAT,          z2,                     &z2)
LOG_ADD(LOG_FLOAT,          d3,                     &d3)
LOG_ADD(LOG_FLOAT,          x3,                     &x3)
LOG_ADD(LOG_FLOAT,          y3,                     &y3)
LOG_ADD(LOG_FLOAT,          z3,                     &z3)
LOG_ADD(LOG_FLOAT,          d4,                     &d4)
LOG_ADD(LOG_FLOAT,          x4,                     &x4)
LOG_ADD(LOG_FLOAT,          y4,                     &y4)
LOG_ADD(LOG_FLOAT,          z4,                     &z4)
LOG_ADD(LOG_FLOAT,          d5,                     &d5)
LOG_ADD(LOG_FLOAT,          x5,                     &x5)
LOG_ADD(LOG_FLOAT,          y5,                     &y5)
LOG_ADD(LOG_FLOAT,          z5,                     &z5)
LOG_ADD(LOG_FLOAT,          d6,                     &d6)
LOG_ADD(LOG_FLOAT,          x6,                     &x6)
LOG_ADD(LOG_FLOAT,          y6,                     &y6)
LOG_ADD(LOG_FLOAT,          z6,                     &z6)
LOG_ADD(LOG_FLOAT,          d7,                     &d7)
LOG_ADD(LOG_FLOAT,          x7,                     &x7)
LOG_ADD(LOG_FLOAT,          y7,                     &y7)
LOG_ADD(LOG_FLOAT,          z7,                     &z7)
LOG_GROUP_STOP(ae483log)

//                1234567890123456789012345678 <-- max total length
//                group   .name
PARAM_GROUP_START(ae483par)
PARAM_ADD(PARAM_UINT8,     use_observer,            &use_observer)
PARAM_ADD(PARAM_UINT8,     reset_observer,          &reset_observer)
PARAM_GROUP_STOP(ae483par)

