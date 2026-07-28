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
#ifndef POWER_ECU_COM_FRAME_ENCODER_HPP_
#define POWER_ECU_COM_FRAME_ENCODER_HPP_

#include <stdint.h>
#include <string>
#include <vector>

#include <boost/foreach.hpp>
#include <boost/smart_ptr/shared_ptr.hpp>
#include <boost/system/error_code.hpp>
#include <boost/unordered/unordered_map.hpp>

#include "any_type_pointer_map.hpp"
#include "power_ecu_com_common.hpp"

namespace hsrb_power_ecu {
/**
 * @brief Interface for packet element encoders
 */
class IElementEncoder {
 public:
  /**
   * @brief Smart pointer for IElementEncoder
   */
  typedef boost::shared_ptr<IElementEncoder> Ptr;
  /**
   * @brief Destructor
   */
  virtual ~IElementEncoder() {}
  /**
   * @brief Encode
   * @param[out] buffer Output buffer
   * @return true on successful encoding
   */
  virtual bool Encode(PacketBuffer &buffer) = 0;
};

/**
 * @brief Interface for data encoders
 */
class IPowerEcuComDataEncoder {
 private:
  IPowerEcuComDataEncoder(IPowerEcuComDataEncoder const &);             // = delete;
  IPowerEcuComDataEncoder &operator=(IPowerEcuComDataEncoder const &);  // = delete;

 protected:
  /**
   * @brief Constructor
   * @param packet_size Size of the packet header
   * @param packet_name Packet type in the packet header
   */
  IPowerEcuComDataEncoder(const std::string &packet_size, const std::string &packet_name)
      : packet_size_(packet_size), packet_name_(packet_name) {}

 public:
  /**
   * @brief Destructor
   */
  virtual ~IPowerEcuComDataEncoder() {}
  /**
   * @brief Encode
   * @param[out] buffer Output buffer
   * @return
   * Normal termination boost::system::errc::success
   * Encoding failure boost::system::errc::protocol_error
   */
  virtual inline boost::system::error_code Encode(PacketBuffer &buffer) {
    BOOST_FOREACH (IElementEncoder::Ptr const p, element_encoder_list_) {
      if (!p->Encode(buffer)) {
        return boost::system::errc::make_error_code(
            boost::system::errc::protocol_error);  // TODO(kitsunai): テスト未実施
      }
      buffer.push_back(',');
    }
    return boost::system::errc::make_error_code(boost::system::errc::success);
  }
  /**
   * @brief Get the size of the packet header
   * @return Packet size
   */
  virtual inline std::string GetPacketSizeStr() const { return packet_size_; }
  /**
   * @brief Get the packet type in the packet header
   * @return Packet type
   */
  inline std::string GetPacketName() const { return packet_name_; }

  /**
   * @brief Get the pointer to the data
   *
   * @tparam T Data type
   * @param[in] name Name of the data
   *
   * @return On success: Pointer to the data<br>
   *         On failure: NULL<br>
   *         Failure occurs if the data name is unregistered or the data type does not match
   */
  template <typename T>
  T* GetParamPtr(const std::string& name) const {
    return parameter_map_.GetPtr<T>(name);
  }

 protected:
  std::vector<IElementEncoder::Ptr> element_encoder_list_;  //!< List of encoding instructions for each element
  const std::string packet_size_;                           //!< Packet size in the header
  const std::string packet_name_;                           //!< Packet type in the header
  any_type_pointer_map::Map parameter_map_;                 //!< Parameter map for control commands
};

/**
 * @brief Frame encoder for transmission packets
 * Communication format is classified into frame, data, and element components.
 *
 * Example: For the command "H,ledc_,23,000,100,255,h12345678,\0",
 * - frame :
 *   Refers to the entire communication packet, e.g., "H,ledc_,23,000,100,255,h12345678,\0"
 *   The frame encoder is responsible for calculating the header and footer of the command.
 *   The frame encoder knows the specifications of the header and footer of the communication packet.
 *   Manages data encoders using the command name as a key.
 *
 * - data :
 *   Refers to the part of the communication packet excluding the header and footer, e.g., "000,100,255,"
 *   The data encoder divides the data into elements and calls the element encoder.
 *   Additionally, after encoding all elements, it performs post-processing such as conversion to physical quantities.
 *   The data encoder is defined for each command, including the command name, command size,
 *   the structure of elements (order, type, and digit count of each element), and
 *   references to PacketData containing information necessary for encoding.
 *   Additionally, it holds information for encoding as PacketData for each command.
 *
 * - element :
 *   Refers to the part of the communication packet excluding the header and footer, e.g., "000"
 *   The element encoder converts between packet strings and the types of each element.
 *   The element encoder is defined for each notation (e.g., signed decimal, hexadecimal).
 *   Knows the format of each element (e.g., signed decimal is [+-][0-9]+).
 *
 * Each encoder has a parent-child relationship: frame->data->element.
 * Additionally, the design of the encoder and decoder is symmetric.
 *
 * RobotHW updates the values in PacketData
 * Calls the Encode method of the frame encoder to perform encoding.
 *
 */
class PowerEcuComFrameEncoder {
 public:
  typedef boost::shared_ptr<hsrb_power_ecu::IPowerEcuComDataEncoder> DataEncoderType;

 private:
  PowerEcuComFrameEncoder(PowerEcuComFrameEncoder const &);             // = delete;
  PowerEcuComFrameEncoder &operator=(PowerEcuComFrameEncoder const &);  // = delete;

  typedef boost::unordered_map<std::string, DataEncoderType> DataEncoderMap;  //!< Dictionary of data encoders

 public:
  /**
   * @brief Constructor
   */
  PowerEcuComFrameEncoder();
  /**
   * @brief Destructor
   */
  ~PowerEcuComFrameEncoder();
  /**
   * @brief Encode
   * @param[out] buffer     Output buffer
   * @param[in] packet_name Name of the packet to encode for data lookup
   * @return On success: boost::system::error::success
   */
  boost::system::error_code Encode(PacketBuffer &buffer, const std::string &packet_name);
  /**
   * @brief Register data encoder
   * @param[in] encoder Encoder to register
   * @return On success: boost::system::errc::success
   */
  boost::system::error_code RegisterDataEncoder(DataEncoderType encoder);

  /**
   * @brief Get the pointer to the data
   *
   * @tparam T Data type
   * @param[in] name Name of the data
   *
   * @return On success: Pointer to the data<br>
   *         On failure: NULL<br>
   *         Failure occurs if the data name is unregistered or the data type does not match
   */
  template <typename T>
  T* GetParamPtr(const std::string& name) const {
    BOOST_FOREACH (DataEncoderMap::value_type const encoder, data_encoder_map_) {
      T* ret = encoder.second->GetParamPtr<T>(name);
      if (ret != NULL) {
        return ret;
      }
    }
    return NULL;
  }

 private:
  DataEncoderMap data_encoder_map_;         //!< Dictionary of data encoders
  uint32_t check_sum_;                      //!< Temporary storage for checksum calculation
  IElementEncoder::Ptr check_sum_encoder_;  //!< Encoder for converting checksum value to string
};
}  // namespace hsrb_power_ecu
#endif  // POWER_ECU_COM_FRAME_ENCODER_HPP_
