/**
 * @file  main_test.cpp
 * @brief Test-build selector: runs the chosen bench test (not a CubeMX file).
 *
 * Active when BOTATO_TEST_BUILD is defined in Config.h. main.cpp calls Test_Main() after
 * peripheral init. Enable exactly ONE test; watch its report global in Live Expressions.
 */
#include "Config.h"

#ifdef BOTATO_TEST_BUILD

#include "Tests.hpp"

// ---- Bus diagnostics (nothing else initialised) -------------------- report / setup
//#define TEST_I2C_GYRO      // i2cGyro    battery on
//#define TEST_I2C_TOF       // i2cToF     all five sensors fitted
//#define TEST_TOF_SINGLE    // tof1       one sensor wired to I2C1
// ---- Sensors (LED blinks = calibrating, keep still) -------------------------
//#define TEST_TOF_LIVE      // tofLive    hold between two walls for sideSum / sideDiff
//#define TEST_TOF_CAL       // tofCal     barrier; PA5 next sensor, PA6 capture
//#define TEST_GYRO          // gyroTest   still -> drift; PA6 + hand 360 -> scale
//#define TEST_ENCODER       // encTest    PA6 zero; push along a ruler
#define TEST_BUTTONS  // btnTest    press everything
// ---- Motion (PA6 short starts, any press stops) -----------------------------
//#define TEST_MOTOR         // motorTest  WHEELS OFF THE GROUND
//#define TEST_HEADING_HOLD  // holdTest   holds once calibrated; twist by hand
//#define TEST_TURN          // turnTest   floor, room to pivot
//#define TEST_DRIVE_CELLS   // cellTest   straight clear floor; PA5 drives back
//#define TEST_CURVE         // curveTest  open floor, taped start; PA6 right, PA5 left
//#define TEST_NAV           // navTest    in the maze, same as competition

#if (defined(TEST_I2C_GYRO) + defined(TEST_I2C_TOF) + defined(TEST_TOF_SINGLE) +                   \
     defined(TEST_TOF_LIVE) + defined(TEST_TOF_CAL) + defined(TEST_GYRO) + defined(TEST_ENCODER) + \
     defined(TEST_BUTTONS) + defined(TEST_MOTOR) + defined(TEST_HEADING_HOLD) +                    \
     defined(TEST_TURN) + defined(TEST_DRIVE_CELLS) + defined(TEST_CURVE) + defined(TEST_NAV)) !=  \
    1
#error "main_test.cpp: enable exactly one TEST_* define"
#endif

#if defined(TEST_I2C_GYRO)
#define TEST_INIT   Test_I2CGyro_Init
#define TEST_UPDATE Test_I2CGyro_Update
#elif defined(TEST_I2C_TOF)
#define TEST_INIT   Test_I2CToF_Init
#define TEST_UPDATE Test_I2CToF_Update
#elif defined(TEST_TOF_SINGLE)
#define TEST_INIT   Test_ToFSingle_Init
#define TEST_UPDATE Test_ToFSingle_Update
#elif defined(TEST_TOF_LIVE)
#define TEST_INIT   Test_ToFLive_Init
#define TEST_UPDATE Test_ToFLive_Update
#elif defined(TEST_TOF_CAL)
#define TEST_INIT   Test_ToFCal_Init
#define TEST_UPDATE Test_ToFCal_Update
#elif defined(TEST_GYRO)
#define TEST_INIT   Test_Gyro_Init
#define TEST_UPDATE Test_Gyro_Update
#elif defined(TEST_ENCODER)
#define TEST_INIT   Test_Encoder_Init
#define TEST_UPDATE Test_Encoder_Update
#elif defined(TEST_BUTTONS)
#define TEST_INIT   Test_Buttons_Init
#define TEST_UPDATE Test_Buttons_Update
#elif defined(TEST_MOTOR)
#define TEST_INIT   Test_Motor_Init
#define TEST_UPDATE Test_Motor_Update
#elif defined(TEST_HEADING_HOLD)
#define TEST_INIT   Test_HeadingHold_Init
#define TEST_UPDATE Test_HeadingHold_Update
#elif defined(TEST_TURN)
#define TEST_INIT   Test_Turn_Init
#define TEST_UPDATE Test_Turn_Update
#elif defined(TEST_DRIVE_CELLS)
#define TEST_INIT   Test_DriveCells_Init
#define TEST_UPDATE Test_DriveCells_Update
#elif defined(TEST_CURVE)
#define TEST_INIT   Test_Curve_Init
#define TEST_UPDATE Test_Curve_Update
#elif defined(TEST_NAV)
#define TEST_INIT   Test_Nav_Init
#define TEST_UPDATE Test_Nav_Update
#endif

void Test_Main(void) {
    TEST_INIT();
    for (;;) { TEST_UPDATE(); }
}

#endif  // BOTATO_TEST_BUILD
