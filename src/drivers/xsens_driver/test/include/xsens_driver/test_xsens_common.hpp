// Copyright 2019 Apex.AI, Inc.
// Co-developed by Tier IV, Inc. and Apex.AI, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/// \copyright Copyright 2017-2019 Apex.AI, Inc.

#ifndef XSENS_DRIVER__TEST_XSENS_COMMON_HPP_
#define XSENS_DRIVER__TEST_XSENS_COMMON_HPP_

#include <vector>
#include "gtest/gtest.h"

using autoware::drivers::xsens_driver::MID;

template<typename TranslatorT, typename MessageT>
class xsens_driver_common : public ::testing::Test
{
public:
  xsens_driver_common()
  {}

protected:
  typename TranslatorT::Packet pkt;
  MessageT out;

  void xsens_driver_common_test(const std::vector<uint8_t> & data)
  {
    const typename TranslatorT::Config cfg{};
    TranslatorT driver(cfg);
    pkt.data = 0xFA;
    ASSERT_FALSE(driver.convert(pkt, out));
    pkt.data = 0xFF;
    ASSERT_FALSE(driver.convert(pkt, out));
    pkt.data = static_cast<std::underlying_type_t<MID>>(MID::MT_DATA2);
    ASSERT_FALSE(driver.convert(pkt, out));
    uint8_t length = data.size() - 1;
    pkt.data = length;
    ASSERT_FALSE(driver.convert(pkt, out));

    for(uint8_t i = 0; i < length; ++i) {
      pkt.data = data[i];
      ASSERT_FALSE(driver.convert(pkt, out));
    }
    pkt.data = data[length];
    ASSERT_TRUE(driver.convert(pkt, out));
  }
};  // class xsens_driver_common

#endif  // XSENS_DRIVER__TEST_XSENS_COMMON_HPP_
