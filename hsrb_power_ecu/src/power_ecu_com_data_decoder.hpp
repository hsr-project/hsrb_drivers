/*
Copyright (c) 2026 TOYOTA MOTOR CORPORATION
All rights reserved.
Redistribution and use in source and binary forms, with or without
modification, are permitted (subject to the limitations in the disclaimer
below) provided that the following conditions are met:
* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.
* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.
* Neither the name of the copyright holder nor the names of its contributors may be used
  to endorse or promote products derived from this software without specific
  prior written permission.
NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
DAMAGE.
*/
#ifndef POWER_ECU_COM_DATA_DECODER_HPP_
#define POWER_ECU_COM_DATA_DECODER_HPP_
#include <string>
#include <vector>

#include <boost/array.hpp>
#include <boost/function.hpp>
#include <boost/system/error_code.hpp>

#include "power_ecu_com_common.hpp"
#include "power_ecu_com_element_decoder.hpp"
#include "power_ecu_com_frame_decoder.hpp"

namespace hsrb_power_ecu {
/* *
* @brief Decoder for ecu1 command
*/
class PowerEcuComEcu1DataDecoder : public hsrb_power_ecu::IPowerEcuComDataDecoder {
 private:
  PowerEcuComEcu1DataDecoder(PowerEcuComEcu1DataDecoder const&);             // = delete;
  PowerEcuComEcu1DataDecoder& operator=(PowerEcuComEcu1DataDecoder const&);  // = delete;

 public:
  /**
   * @brief Member variable pointer container for hw class of ecu1 command
   */
  struct PacketData {
    uint32_t time_stamp;                             //!< Timestamp [ms]
    std::string ecu1_date;                           //!< Date and time YYYYMMDDhhmmss
    std::string power_ecu_status_flag;               //!< Power ECU status S part
    std::string diag_status;                         //!< Diagnostic information 32-digit hexadecimal
    double battery_total_capacity;                   //!< Total battery capacity [mAh]
    double battery_remaining_capacity;               //!< Remaining battery capacity [mAh]
    double electric_current;                         //!< Electric current value [mA]
    double battery_voltage;                          //!< Battery voltage [mV]
    double battery_temperature;                      //!< Battery temperature [C]
    bool is_battery_crgov;                           //!< Overcharge 1: Overcharged
    bool is_battery_23par;                           //!< Parallel count 0: 2 parallel 1: 3 parallel
    bool is_battery_std;                             //!< Learning permission 1: Learning permitted
    bool is_battery_full;                            //!< Full charge 1: Fully charged state
    bool is_battery_discov;                          //!< Over-discharge 1: Over-discharged
    bool is_battery_chg;                             //!< Charging permission 1: Charging permitted
    bool is_battery_disc;                            //!< Discharge permission 1: Discharge permitted
    bool is_battery_0per;                            //!< 0% detection 1: 0% detected state
    bool is_battery_45par;                           //!< Parallel count 0: 4 parallel 1: 5 parallel
    bool is_battery_sel;                             //!< Minimum cell voltage 0% detected state 1: 0% detected state
    bool is_battery_bal;                             //!< Cell balance disruption 1: Disrupted
    uint16_t battery_initial_learning_capacity;      //!< Initial battery learning capacity [mAh]
    uint16_t battery_error_status;                   //!< Battery error status Undefined specification
    double battery_relative_capacity;                //!< Relative capacity [%]
    uint32_t power_ecu_internal_state;               //!< (New) Number of S** in 4.5.3 Power ECU internal state
    bool is_powerecu_bat_stat;                       //!< (New) Battery charging state
    bool is_powerecu_sw_kinoko;                      //!< (New) Wired emergency stop switch
    bool is_powerecu_sw_pwr;                         //!< (New) Power (Prius switch)
    bool is_powerecu_sw_drv;                         //!< (New) Drive switch
    bool is_powerecu_sw_latch;                       //!< (New) Latch release switch
    bool is_powerecu_sw_w_sel;                       //!< (New) Wireless switch
    bool is_powerecu_sw_w_stop;                      //!< (New) Wireless emergency stop switch
    bool is_bumper_bumper2;                          //!< Bumper sensor state 2 1: Contact detected
    bool is_bumper_bumper1;                          //!< Bumper sensor state 1 1: Contact detected
    bool is_bumper_prox5;                            //!< Proximity sensor latch state 5 1: Proximity detected
    bool is_bumper_prox4;                            //!< Proximity sensor latch state 4 1: Proximity detected
    bool is_bumper_prox3;                            //!< Proximity sensor latch state 3 1: Proximity detected
    bool is_bumper_prox2;                            //!< Proximity sensor latch state 2 1: Proximity detected
    bool is_bumper_prox1;                            //!< Proximity sensor latch state 1 1: Proximity detected
    std::string gyro_status;                         //!< (New) Gyro attitude angle calculation status
    boost::array<double, 4> imu_quaternions;         //!< quaternion x,y,z,t -1.0~1.0
    boost::array<double, 3> imu_angular_velocities;  //!< Angular velocity x, y, z [rad/s]
    boost::array<double, 3> imu_accelerations;       //!< Acceleration x, y, z [m/s^2]
    uint8_t charger_state;                           //!< Automatic charging status
  };

 private:
  /**
   * @brief Internal container for receiving packet data
   */
  struct PacketRawData {
    uint32_t time_stamp;                         //!< Timestamp Decimal 10 digits
    std::string date;                            //!< Date and time String
    std::string power_ecu_status_flag;           //!< Power ECU status Hexadecimal 2 digits
    std::string power_ecu_status;                //!< (New) Power ECU state Hexadecimal 16 digits
    std::string diag_status;                     //!< uint816 Diagnostic information Hexadecimal 32 digits
    uint16_t battery_total_capacity;             //!< Total battery capacity Signed Decimal 5 digits
    uint16_t battery_remaining_capacity;         //!< Remaining battery capacity Signed Decimal 5 digits
    int16_t electric_current;                    //!< Electric current Signed Decimal 5 digits
    uint16_t battery_voltage;                    //!< Battery voltage Signed Decimal 5 digits
    int8_t battery_temperature;                  //!< Battery temperature Signed Decimal 3 digits
    uint16_t battery_state_flag;                 //!< Battery state flag Hexadecimal 4 digits
    uint16_t battery_initial_learning_capacity;  //!< Initial battery learning capacity Decimal 5 digits
    uint16_t battery_error_status;               //!< Battery error status Hexadecimal 4 digits
    uint8_t battery_relative_capacity;           //!< Relative capacity Decimal 3 digits
    uint8_t bumper_status;                       //!< Proximity, bumper sensor state Hexadecimal 2 digits
    std::string gyro_status;                     //!< (New) Gyro attitude angle calculation status Hexadecimal 16 digits
    int32_t quaternion_t;                        //!< quaternion_t Signed Decimal 10 digits
    int32_t quaternion_x;                        //!< quaternion_x Signed Decimal 10 digits
    int32_t quaternion_y;                        //!< quaternion_y Signed Decimal 10 digits
    int32_t quaternion_z;                        //!< quaternion_z Signed Decimal 10 digits
    int32_t angular_velocity_x;                  //!< Angular velocity x Signed Decimal 10 digits
    int32_t angular_velocity_y;                  //!< Angular velocity y Signed Decimal 10 digits
    int32_t angular_velocity_z;                  //!< Angular velocity z Signed Decimal 10 digits
    int32_t acceleration_x;                      //!< Acceleration x Signed Decimal 10 digits
    int32_t acceleration_y;                      //!< Acceleration y Signed Decimal 10 digits
    int32_t acceleration_z;                      //!< Acceleration z Signed Decimal 10 digits
    uint8_t charger_state;                       //!< Automatic charging status
  };

 public:
  /**
   * @brief Constructor
   * @param packet_data Packet data
   */
  PowerEcuComEcu1DataDecoder();
  /**
   * @brief Destructor
   */
  virtual ~PowerEcuComEcu1DataDecoder() {}

 private:
  /**
   * @brief Post-decoding process
   * @return True on success
   */
  virtual bool Update();

  PacketRawData packet_raw_data_;  //!< Hw class variable pointer container
  PacketData packet_out_;          //!< Buffer for receiving data
  std::string temp_str;            //!< Temporary
  uint32_t temp_uint;
};

/**
* @brief Decoder for ecu2 command
*/
class PowerEcuComEcu2DataDecoder : public hsrb_power_ecu::IPowerEcuComDataDecoder {
 private:
  PowerEcuComEcu2DataDecoder(PowerEcuComEcu2DataDecoder const&);             // = delete;
  PowerEcuComEcu2DataDecoder& operator=(PowerEcuComEcu2DataDecoder const&);  // = delete;

 public:
  /**
  * @brief Packet data for ecu2 command
  */
  struct PacketData {
    std::string ecu2_date;               //!< Date and time (YYYYMMDDhhmmss) String
    uint16_t d12V_D0_V;                  //!< 12Vd0 voltage [mV] Signed 1 digit + Decimal 5 digits
    int16_t d12V_D0_A;                   //!< 12Vd0 current [mA] Signed 1 digit + Decimal 5 digits
    uint16_t d12V_D1_V;                  //!< 12Vd1 voltage [mV] Signed 1 digit Decimal 5 digits
    int16_t d12V_D1_A;                   //!< 12Vd1 current [mA] Signed 1 digit Decimal 5 digits
    uint16_t d12V_D2_V;                  //!< 12Vd2 voltage [mV] Signed 1 digit Decimal 5 digits
    int16_t d12V_D2_A;                   //!< 12Vd2 current [mA] Signed 1 digit Decimal 5 digits
    uint16_t d12V_D3_V;                  //!< 12Vd3 voltage [mV] Signed 1 digit Decimal 5 digits
    int16_t d12V_D3_A;                   //!< 12Vd3 current [mA] Signed 1 digit Decimal 5 digits
    uint16_t d12V_O1_V;                  //!< 12Vo1 voltage [mV] Signed 1 digit Decimal 5 digits
    int16_t d12V_O1_A;                   //!< 12Vo1 current [mA] Signed 1 digit Decimal 5 digits
    uint16_t d12V_O2_V;                  //!< 12Vo2 voltage [mV] Signed 1 digit Decimal 5 digits
    int16_t d12V_O2_A;                   //!< 12Vo2 current [mA] Signed 1 digit Decimal 5 digits
    uint16_t d5VA_V;                     //!< 5Va voltage [mV] Signed 1 digit Decimal 5 digits
    uint16_t d5VD1_V;                    //!< 5Vd1 voltage [mV] Signed 1 digit Decimal 5 digits
    int16_t d5VD1_A;                     //!< 5Vd1 current [mA] Signed 1 digit Decimal 5 digits
    uint16_t d5VD2_V;                    //!< 5Vd2 voltage [mV] Signed 1 digit Decimal 5 digits
    int16_t d5VD2_A;                     //!< 5Vd2 current [mA] Signed 1 digit Decimal 5 digits
    uint16_t d5VD3_V;                    //!< 5Vd3 voltage [mV] Signed 1 digit Decimal 5 digits
    int16_t d5VD3_A;                     //!< 5Vd3 current [mA] Signed 1 digit Decimal 5 digits
    uint16_t d5VD4_V;                    //!< 5Vd4 voltage [mV] Signed 1 digit Decimal 5 digits
    int16_t d5VD4_A;                     //!< 5Vd4 current [mA] Signed 1 digit Decimal 5 digits
    uint16_t d5VD5_V;                    //!< 5Vd5 voltage [mV] Signed 1 digit Decimal 5 digits
    int16_t d5VD5_A;                     //!< 5Vd5 current [mA] Signed 1 digit Decimal 5 digits
    uint16_t Chgsense;                   //!< Automatic sequential charging terminal voltage [mV] Signed 1 digit Decimal 5 digits
    uint16_t d2V5VDA1_V;                 //!< 2.5Va1 voltage (A/D1) [mV] Signed 1 digit Decimal 5 digits
    uint16_t d2V5VDA2_V;                 //!< 2.5Va2 voltage (A/D2) [mV] Signed 1 digit Decimal 5 digits
    uint16_t ACDC_V;                     //!< ACDC voltage [mV] Signed 1 digit Decimal 5 digits
    int16_t ADCD_A;                      //!< ACDC current [mA] Signed 1 digit Decimal 5 digits
    uint16_t BATT_V;                     //!< BATT voltage [mV] Signed 1 digit Decimal 5 digits
    int16_t BATT_A;                      //!< BATT current [mA] Signed 1 digit Decimal 5 digits
    int16_t BATT_A2;                     //!< BATT current 2 [10mA] Signed 1 digit Decimal 5 digits
    uint16_t PBM_V;                      //!< PBM voltage [mV] Signed 1 digit Decimal 5 digits
    int16_t PBM_A;                       //!< PBM current [mA] Signed 1 digit Decimal 5 digits
    int16_t PBM_A2;                      //!< PBM current 2 [10mA] Signed 1 digit Decimal 5 digits
    uint16_t PUMP_V;                     //!< Pump sensor voltage [mV] Signed 1 digit Decimal 5 digits
    int16_t ECU_TEMP;                    //!< Power ECU temperature [℃] Signed 1 digit Decimal 3 digits
    int16_t ECU_TEMP1;                   //!< Power ECU temperature 1 [℃] Signed 1 digit Decimal 3 digits
    int16_t ECU_TEMP2;                   //!< Power ECU temperature 2 [℃] Signed 1 digit Decimal 3 digits
    int16_t ECU_TEMP3;                   //!< Power ECU temperature 3 [℃] Signed 1 digit Decimal 3 digits
  };

  /**
   * @brief Constructor
   * @param packet_data Packet data
   */
  PowerEcuComEcu2DataDecoder();
  /**
   * @brief Destructor
   */
  virtual ~PowerEcuComEcu2DataDecoder() {}

 private:
  /**
   * @brief Post-decoding process
   * @return True on success
   */
  virtual bool Update();
  PacketData packet_data_;  //!< Packet data
};

/**
 * @brief Decoder for RXACK command
 */
class PowerEcuComRxackDataDecoder : public hsrb_power_ecu::IPowerEcuComDataDecoder {
 private:
  PowerEcuComRxackDataDecoder(PowerEcuComRxackDataDecoder const&);             // = delete;
  PowerEcuComRxackDataDecoder& operator=(PowerEcuComRxackDataDecoder const&);  // = delete;

 public:
  /**
   * @brief Packet data for Rxack
   */
  struct PacketData {
    /**
     * @brief Constructor
     */
    PacketData() : is_receive_ack(false), ack_value(0) {}
    bool is_receive_ack;  //!< Whether Ack was received
    uint8_t ack_value;    //!< Return value of reply command
  };

  /**
   * @brief Constructor
   * @param packet_data Packet data
   */
  PowerEcuComRxackDataDecoder();
  /**
   * @brief Destructor
   */
  virtual ~PowerEcuComRxackDataDecoder() {}

 private:
  /**
   * @brief Post-decoding process
   * @return True on success
   */
  virtual bool Update();

  PacketData packet_data_;  //!< Packet data
};

/**
 * @brief Decoder for Ver command
 */
class PowerEcuComVerDataDecoder : public hsrb_power_ecu::IPowerEcuComDataDecoder {
 private:
  PowerEcuComVerDataDecoder(PowerEcuComVerDataDecoder const&);             // = delete;
  PowerEcuComVerDataDecoder& operator=(PowerEcuComVerDataDecoder const&);  // = delete;

 public:
  /**
   * @brief Packet data for Ver
   */
  struct PacketData {
    PacketData() : is_receive_version(false) {}
    std::string ver_power_ecu_version;      //!< Power ECU firmware Ver [git hash 20 bytes] Hexadecimal 40 digits
    std::string ver_power_ecu_com_version;  //!< Power ECU communication structure HASH [hash 20 bytes] Hexadecimal 40 digits
    bool is_receive_version;                //!< Whether Ver command was received
  };

  /**
   * @brief Constructor
   * @param[in] packet_data Packet data
   */
  PowerEcuComVerDataDecoder();
  /**
   * @brief Destructor
   */
  virtual ~PowerEcuComVerDataDecoder() {}

 private:
  /**
   * @brief Post-decoding process
   * @return True on success
   */
  virtual bool Update();

  PacketData packet_data_;                 //!< Packet data
  std::string power_ecu_version_raw_;      //!< Power ECU firmware Ver [git hash 20 bytes] Hexadecimal 40 digits
  std::string power_ecu_com_version_raw_;  //!< Power ECU communication structure HASH [hash 20 bytes] Hexadecimal 40 digits
};

}  // namespace hsrb_power_ecu
#endif  // POWER_ECU_COM_DATA_DECODER_HPP_
