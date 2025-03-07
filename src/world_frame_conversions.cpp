/* Copyright (c) 2017-2018, Waterloo Autonomous Vehicles Laboratory (WAVELab),
 * Waterloo Intelligent Systems Engineering Lab (WISELab),
 * University of Waterloo.
 *
 * Refer to the accompanying LICENSE file for license information.
 *
 * ############################################################################
 ******************************************************************************
 |                                                                            |
 |                         /\/\__/\_/\      /\_/\__/\/\                       |
 |                         \          \____/          /                       |
 |                          '----________________----'                        |
 |                              /                \                            |
 |                            O/_____/_______/____\O                          |
 |                            /____________________\                          |
 |                           /    (#UNIVERSITY#)    \                         |
 |                           |[**](#OFWATERLOO#)[**]|                         |
 |                           \______________________/                         |
 |                            |_""__|_,----,_|__""_|                          |
 |                            ! !                ! !                          |
 |                            '-'                '-'                          |
 |       __    _   _  _____  ___  __  _  ___  _    _  ___  ___   ____  ____   |
 |      /  \  | | | ||_   _|/ _ \|  \| |/ _ \| \  / |/ _ \/ _ \ /     |       |
 |     / /\ \ | |_| |  | |  ||_||| |\  |||_|||  \/  |||_||||_|| \===\ |====   |
 |    /_/  \_\|_____|  |_|  \___/|_| \_|\___/|_|\/|_|\___/\___/ ____/ |____   |
 |                                                                            |
 ******************************************************************************
 * ############################################################################
 *
 * File: world_frame_conversions.cpp
 * Desc: Implementation file for world frame conversion functions
 * Auth: Michael Smart <michael.smart@uwaterloo.ca>
 *
 * ############################################################################
*/

#include "world_frame_conversions.hpp"

namespace wave_spatial_utils {

void ecefPointFromLLH(const double llh[3], double ecef[3]) {
    double latitude = llh[0], longitude = llh[1], height = llh[2];

    GeographicLib::Geocentric earth = GeographicLib::Geocentric::WGS84();

    double X, Y, Z;
    earth.Forward(latitude, longitude, height, X, Y, Z);

    ecef[0] = X;
    ecef[1] = Y;
    ecef[2] = Z;
}

void llhPointFromECEF(const double ecef[3], double llh[3]) {
    double X = ecef[0], Y = ecef[1], Z = ecef[2];

    GeographicLib::Geocentric earth = GeographicLib::Geocentric::WGS84();

    double latitude, longitude, height;
    earth.Reverse(X, Y, Z, latitude, longitude, height);

    llh[0] = latitude;
    llh[1] = longitude;
    llh[2] = height;
}

void ecefFromENUTransformMatrix(const double datum[3],
                                double T_ecef_enu[4][4],
                                bool datum_is_llh) {
    // Both Forward() and Reverse() return the same rotation matrix from ENU
    // to ECEF
    std::vector<double> R_ecef_enu(9, 0.0);
    GeographicLib::Geocentric earth = GeographicLib::Geocentric::WGS84();
    double datum_X, datum_Y, datum_Z;

    if (datum_is_llh) {
        double latitude = datum[0], longitude = datum[1], height = datum[2];

        earth.Forward(
          latitude, longitude, height, datum_X, datum_Y, datum_Z, R_ecef_enu);
    } else {
        // Datum is already given in ECEF
        datum_X = datum[0];
        datum_Y = datum[1];
        datum_Z = datum[2];

        double latitude, longitude, height;
        earth.Reverse(
          datum_X, datum_Y, datum_Z, latitude, longitude, height, R_ecef_enu);
    }

    T_ecef_enu[0][0] = R_ecef_enu[0];
    T_ecef_enu[0][1] = R_ecef_enu[1];
    T_ecef_enu[0][2] = R_ecef_enu[2];
    T_ecef_enu[0][3] = datum_X;
    T_ecef_enu[1][0] = R_ecef_enu[3];
    T_ecef_enu[1][1] = R_ecef_enu[4];
    T_ecef_enu[1][2] = R_ecef_enu[5];
    T_ecef_enu[1][3] = datum_Y;
    T_ecef_enu[2][0] = R_ecef_enu[6];
    T_ecef_enu[2][1] = R_ecef_enu[7];
    T_ecef_enu[2][2] = R_ecef_enu[8];
    T_ecef_enu[2][3] = datum_Z;
    T_ecef_enu[3][0] = 0.0;
    T_ecef_enu[3][1] = 0.0;
    T_ecef_enu[3][2] = 0.0;
    T_ecef_enu[3][3] = 1.0;
}

void enuFromECEFTransformMatrix(const double datum[3],
                                double T_enu_ecef[4][4],
                                bool datum_is_llh) {
    // Get T_ecef_enu and then invert it
    double T_ecef_enu[4][4];
    ecefFromENUTransformMatrix(datum, T_ecef_enu, datum_is_llh);

    // Affine inverse: [R | t]^(-1) = [ R^T | - R^T * t]
    // TODO(msmart/benskikos) - Move these functions to somewhere where we have
    // matrix classes and ownership since it is ours. Manual transposition is
    // undesirable. The below should be 2 lines:
    // R_new = R.transpose(); t_new = - R.transpose()*t;
    T_enu_ecef[0][0] = T_ecef_enu[0][0];
    T_enu_ecef[0][1] = T_ecef_enu[1][0];
    T_enu_ecef[0][2] = T_ecef_enu[2][0];
    T_enu_ecef[1][0] = T_ecef_enu[0][1];
    T_enu_ecef[1][1] = T_ecef_enu[1][1];
    T_enu_ecef[1][2] = T_ecef_enu[2][1];
    T_enu_ecef[2][0] = T_ecef_enu[0][2];
    T_enu_ecef[2][1] = T_ecef_enu[1][2];
    T_enu_ecef[2][2] = T_ecef_enu[2][2];

    // Affine inverse translation component: -R_inverse * b
    //    with b as the 4th column of T_ecef_enu
    T_enu_ecef[0][3] = -T_enu_ecef[0][0] * T_ecef_enu[0][3]    //
                       - T_enu_ecef[0][1] * T_ecef_enu[1][3]   //
                       - T_enu_ecef[0][2] * T_ecef_enu[2][3];  //

    T_enu_ecef[1][3] = -T_enu_ecef[1][0] * T_ecef_enu[0][3]    //
                       - T_enu_ecef[1][1] * T_ecef_enu[1][3]   //
                       - T_enu_ecef[1][2] * T_ecef_enu[2][3];  //

    T_enu_ecef[2][3] = -T_enu_ecef[2][0] * T_ecef_enu[0][3]    //
                       - T_enu_ecef[2][1] * T_ecef_enu[1][3]   //
                       - T_enu_ecef[2][2] * T_ecef_enu[2][3];  //

    // Last row is the same
    T_enu_ecef[3][0] = 0.0;
    T_enu_ecef[3][1] = 0.0;
    T_enu_ecef[3][2] = 0.0;
    T_enu_ecef[3][3] = 1.0;
}

void enuPointFromLLH(const double point_llh[3],
                     const double enu_datum[3],
                     double point_enu[3],
                     bool datum_is_llh) {
    double enu_datum_llh[3];
    if (datum_is_llh) {
        enu_datum_llh[0] = enu_datum[0];
        enu_datum_llh[1] = enu_datum[1];
        enu_datum_llh[2] = enu_datum[2];
    } else {
        // Datum is ECEF
        llhPointFromECEF(enu_datum, enu_datum_llh);
    }

    GeographicLib::Geocentric earth = GeographicLib::Geocentric::WGS84();
    GeographicLib::LocalCartesian localENU(
      enu_datum_llh[0], enu_datum_llh[1], enu_datum_llh[2], earth);

    localENU.Forward(point_llh[0],
                     point_llh[1],
                     point_llh[2],
                     point_enu[0],
                     point_enu[1],
                     point_enu[2]);
}

void llhPointFromENU(const double point_enu[3],
                     const double enu_datum[3],
                     double point_llh[3],
                     bool datum_is_llh) {
    double enu_datum_llh[3];
    if (datum_is_llh) {
        enu_datum_llh[0] = enu_datum[0];
        enu_datum_llh[1] = enu_datum[1];
        enu_datum_llh[2] = enu_datum[2];
    } else {
        // Datum is ECEF
        llhPointFromECEF(enu_datum, enu_datum_llh);
    }

    GeographicLib::Geocentric earth = GeographicLib::Geocentric::WGS84();
    GeographicLib::LocalCartesian localENU(
      enu_datum_llh[0], enu_datum_llh[1], enu_datum_llh[2], earth);

    localENU.Reverse(point_enu[0],
                     point_enu[1],
                     point_enu[2],
                     point_llh[0],
                     point_llh[1],
                     point_llh[2]);
}

void fix_to_point(const sensor_msgs::NavSatFix &fix,
                  const sensor_msgs::NavSatFix &datum,
                  geometry_msgs::Point *point_ptr) {
    double point_llh[3] = {fix.latitude, fix.longitude, fix.altitude};
    double datum_llh[3] = {datum.latitude, datum.longitude, datum.altitude};

    double point_enu[3];
    enuPointFromLLH(point_llh, datum_llh, point_enu, true);

    point_ptr->x = point_enu[0];
    point_ptr->y = point_enu[1];
    point_ptr->z = point_enu[2];
}

void point_to_fix(const geometry_msgs::Point &point,
                  const sensor_msgs::NavSatFix &datum,
                  sensor_msgs::NavSatFix *fix_ptr) {
    double point_enu[3] = {point.x, point.y, point.z};
    double datum_llh[3] = {datum.latitude, datum.longitude, datum.altitude};

    double point_llh[3];
    llhPointFromENU(point_enu, datum_llh, point_llh, true);

    fix_ptr->latitude = point_llh[0];
    fix_ptr->longitude = point_llh[1];
    fix_ptr->altitude = point_llh[2];
}

tf::Transform makeTfENUFromECEF(const sensor_msgs::NavSatFix &datum) {
    double datum_llh[3] = {datum.latitude, datum.longitude, datum.altitude};
    return makeTfENUFromECEF(datum_llh, true);
}
tf2::Transform makeTf2ENUFromECEF(const sensor_msgs::NavSatFix &datum) {
    double datum_llh[3] = {datum.latitude, datum.longitude, datum.altitude};
    return makeTf2ENUFromECEF(datum_llh, true);
}

tf::Transform makeTfENUFromECEF(const double datum[3], bool datum_is_llh) {
    double T_enu_ecef[4][4];
    enuFromECEFTransformMatrix(datum, T_enu_ecef, datum_is_llh);

    tf::Matrix3x3 tf_Rotation(T_enu_ecef[0][0],
                              T_enu_ecef[0][1],
                              T_enu_ecef[0][2],
                              T_enu_ecef[1][0],
                              T_enu_ecef[1][1],
                              T_enu_ecef[1][2],
                              T_enu_ecef[2][0],
                              T_enu_ecef[2][1],
                              T_enu_ecef[2][2]);
    tf::Vector3 tf_Origin(T_enu_ecef[0][3], T_enu_ecef[1][3], T_enu_ecef[2][3]);

    tf::Transform tf_enu_ecef(tf_Rotation, tf_Origin);
    return tf_enu_ecef;
}
tf2::Transform makeTf2ENUFromECEF(const double datum[3], bool datum_is_llh) {
    double T_enu_ecef[4][4];
    enuFromECEFTransformMatrix(datum, T_enu_ecef, datum_is_llh);

    tf2::Matrix3x3 tf_Rotation(T_enu_ecef[0][0],
                               T_enu_ecef[0][1],
                               T_enu_ecef[0][2],
                               T_enu_ecef[1][0],
                               T_enu_ecef[1][1],
                               T_enu_ecef[1][2],
                               T_enu_ecef[2][0],
                               T_enu_ecef[2][1],
                               T_enu_ecef[2][2]);
    tf2::Vector3 tf_Origin(
      T_enu_ecef[0][3], T_enu_ecef[1][3], T_enu_ecef[2][3]);

    tf2::Transform tf_enu_ecef(tf_Rotation, tf_Origin);
    return tf_enu_ecef;
}

tf::Transform makeTfECEFFromENU(const sensor_msgs::NavSatFix &datum) {
    double datum_llh[3] = {datum.latitude, datum.longitude, datum.altitude};
    return makeTfECEFFromENU(datum_llh, true);
}
tf2::Transform makeTf2ECEFFromENU(const sensor_msgs::NavSatFix &datum) {
    double datum_llh[3] = {datum.latitude, datum.longitude, datum.altitude};
    return makeTf2ECEFFromENU(datum_llh, true);
}

tf::Transform makeTfECEFFromENU(const double datum[3], bool datum_is_llh) {
    double T_ecef_enu[4][4];
    ecefFromENUTransformMatrix(datum, T_ecef_enu, datum_is_llh);

    tf::Matrix3x3 tf_Rotation(T_ecef_enu[0][0],
                              T_ecef_enu[0][1],
                              T_ecef_enu[0][2],
                              T_ecef_enu[1][0],
                              T_ecef_enu[1][1],
                              T_ecef_enu[1][2],
                              T_ecef_enu[2][0],
                              T_ecef_enu[2][1],
                              T_ecef_enu[2][2]);
    tf::Vector3 tf_Origin(T_ecef_enu[0][3], T_ecef_enu[1][3], T_ecef_enu[2][3]);

    tf::Transform tf_ecef_enu(tf_Rotation, tf_Origin);
    return tf_ecef_enu;
}
tf2::Transform makeTf2ECEFFromENU(const double datum[3], bool datum_is_llh) {
    double T_ecef_enu[4][4];
    ecefFromENUTransformMatrix(datum, T_ecef_enu, datum_is_llh);

    tf2::Matrix3x3 tf_Rotation(T_ecef_enu[0][0],
                               T_ecef_enu[0][1],
                               T_ecef_enu[0][2],
                               T_ecef_enu[1][0],
                               T_ecef_enu[1][1],
                               T_ecef_enu[1][2],
                               T_ecef_enu[2][0],
                               T_ecef_enu[2][1],
                               T_ecef_enu[2][2]);
    tf2::Vector3 tf_Origin(
      T_ecef_enu[0][3], T_ecef_enu[1][3], T_ecef_enu[2][3]);

    tf2::Transform tf_ecef_enu(tf_Rotation, tf_Origin);
    return tf_ecef_enu;
}
}  // namespace wave_spatial_utils
