// --------------------------------------------------------------------------
/// @file BitStream.h
/// @brief Cylinder lens model
///
/// The lens model provides a warping and an un-warping function.
///
/// --------------------------------------------------------------------------
/// @copyright MAGNA Electronics - C O N F I D E N T I A L <br>
/// This document in its entirety is CONFIDENTIAL and may not be disclosed,
/// disseminated or distributed to parties outside MAGNA Electronics
/// without written permission from MAGNA Electronics.
///
/// @author Michael Schulte (michael.schulte@magna.com)
/// @date 28.11.2014
///
// --------------------------------------------------------------------------
/// \addtogroup mecl
/// @{
/// \addtogroup core
/// @{

#ifndef BITSTREAM_H_
#define BITSTREAM_H_

#include "MeclTypes.h"
#include "MeclAssert.h"

namespace mecl
{
namespace core
{

// forward declare helper template
template <typename DataType,  bool_t MsbFirst, bool_t BigEndian>
class BitStreamHelper;

//--------------------------------------------------------------------------
/// @class BitStream
/// @brief Bitstream data type template
///
/// Implements access and iteration of data stored as bitstream. Class handles
/// both MSB/LSB storage as well as big and little endian formats.
/// The bits in the stream can also be changed.
// --------------------------------------------------------------------------
template <typename DataType, bool_t MsbFirst = false, bool_t BigEndian = true>
class BitStream
{
public:
  // Helper template (with same parameters) has full access to this class
  friend class BitStreamHelper<DataType, MsbFirst, BigEndian>;

  static const uint32_t numberOfBits_u32 = 8U*sizeof(DataType);                             ///< Number of bits
  static const DataType highestBit_t = static_cast<DataType>(1U << (numberOfBits_u32-1U));///< Index of highest bit

  // --------------------------------------------------------------------------
  /// @brief Default class constructor
  ///
  /// @par Example usage:
  /// @snippet CoreTester.cpp BitStream_Constructor1
  ///
  /// @param[in] i_Data_po Pointer to \p DataType memory buffer to be used as data storage
  /// @param[in] i_Size_u16 Number of \p DataType elements available
  /// @param[in] i_MsbFirst_b Boolean, bit order, True = MSB, False = LSB
  /// @param[in] i_BigEndian_b Boolean, byte order, True = big endian (Motorola), False = little endian (Intel)
  // --------------------------------------------------------------------------
  BitStream(DataType* i_Data_po, uint16_t i_Size_u16) :
    data_po(i_Data_po),
    bitMask_t(1U),
    dataIndex_u32(0U),
    currentIndex_u32(0U),
    endIndex_u32(numberOfBits_u32*i_Size_u16)
  {
    setInitialBitmask_v();
    if (BigEndian)
    {
      dataIndex_u32 = i_Size_u16-1U;
    }
  }

  // --------------------------------------------------------------------------
  /// @brief Test for end of bitstream
  ///
  /// @par Example usage:
  /// @snippet CoreTester.cpp BitStream_next_b_hasNext_b
  ///
  /// @return Boolean, True = Bits remaining, False = End of bitstream
  // --------------------------------------------------------------------------
  bool_t hasNext_b() const
  {
    return currentIndex_u32 < endIndex_u32;
  }

  // --------------------------------------------------------------------------
  /// @brief Return next bit value
  ///
  /// @par Example usage:
  /// @snippet CoreTester.cpp BitStream_next_b_hasNext_b
  ///
  /// @return Boolean, True = Bit value 1, False = Bit value 0
  // --------------------------------------------------------------------------
  bool_t next_b();


  void set_v(bool_t i_OnOff_b);

private:
  // --------------------------------------------------------------------------
  /// @brief Set initial bit mask depending on MSB/LSB configuration
  // --------------------------------------------------------------------------
  void setInitialBitmask_v();

  DataType *data_po;///< Data type of storage
  DataType bitMask_t;///<  Current bit mask
  uint32_t dataIndex_u32;///<  Current DataType data index
  uint32_t currentIndex_u32;///<  Current bit index
  uint32_t endIndex_u32;///<  Last valid bit index
};


//--------------------------------------------------------------------------
/// @class BitStreamHelper
/// @brief Bitstream helper template
///
/// Implements access and iteration of data stored as bitstream. Class specializes on
/// either MSB/LSB storage and big and little endian formats.
// --------------------------------------------------------------------------
template <typename DataType, bool_t MsbFirst, bool_t BigEndian>
class BitStreamHelper
{
public:
  static bool_t next_b(BitStream<DataType, MsbFirst, BigEndian>& b_BitStream_ro);
  static void setInitialBitmask_v(BitStream<DataType, MsbFirst, BigEndian>& b_BitStream_ro);
};


//--------------------------------------------------------------------------
// Implementations (or rather specializations) of BitStremHelper implement all possible combinations of
// MSB/LSB first and endianess.
//--------------------------------------------------------------------------

//--------------------------------------------------------------------------
// MSB first + big Endian specialization
//--------------------------------------------------------------------------
template <typename DataType>
class BitStreamHelper<DataType, true, true>
{
public:
  static bool_t next_b(BitStream<DataType, true, true>& b_BitStream_ro)
  {
    bool_t v_Ret_b = (0U != (b_BitStream_ro.data_po[b_BitStream_ro.dataIndex_u32] & b_BitStream_ro.bitMask_t));
    b_BitStream_ro.bitMask_t >>= 1U; // MSB first
    if (0U == b_BitStream_ro.bitMask_t)
    {
      // go to next data index
      --b_BitStream_ro.dataIndex_u32; // big Endian
      setInitialBitmask_v(b_BitStream_ro);
    }
    ++b_BitStream_ro.currentIndex_u32;
    return v_Ret_b;
  }

  static void setInitialBitmask_v(BitStream<DataType, true, true>& b_BitStream_ro)
  {
    b_BitStream_ro.bitMask_t = BitStream<DataType, true, true>::highestBit_t; // MSB First
  }
};

//--------------------------------------------------------------------------
// MSB first + little Endian specialization
//--------------------------------------------------------------------------
template <typename DataType>
class BitStreamHelper<DataType, true, false>
{
public:
  static bool_t next_b(BitStream<DataType, true, false>& b_BitStream_ro)
  {
    bool_t v_Ret_b = (0U != (b_BitStream_ro.data_po[b_BitStream_ro.dataIndex_u32] & b_BitStream_ro.bitMask_t));
    b_BitStream_ro.bitMask_t >>= 1U; // MSB First
    if (0U == b_BitStream_ro.bitMask_t)
    {
      // go to next data index
      ++b_BitStream_ro.dataIndex_u32; // little Endian
      setInitialBitmask_v(b_BitStream_ro);
    }
    ++b_BitStream_ro.currentIndex_u32;
    return v_Ret_b;
  }
  static void setInitialBitmask_v(BitStream<DataType, true, false>& b_BitStream_ro)
  {
    b_BitStream_ro.bitMask_t = BitStream<DataType, true, true>::highestBit_t; // MSB First
  }
};

//--------------------------------------------------------------------------
// LSB first + big Endian specialization
//--------------------------------------------------------------------------
template <typename DataType>
class BitStreamHelper<DataType, false, true>
{
public:
  static bool_t next_b(BitStream<DataType, false, true>& b_BitStream_ro)
  {
    bool_t v_Ret_b = (0U != (b_BitStream_ro.data_po[b_BitStream_ro.dataIndex_u32] & b_BitStream_ro.bitMask_t));
    b_BitStream_ro.bitMask_t <<= 1U; // LSB First
    if (0U == b_BitStream_ro.bitMask_t)
    {
      // go to next data index
      --b_BitStream_ro.dataIndex_u32; // big Endian
      setInitialBitmask_v(b_BitStream_ro);
    }
    ++b_BitStream_ro.currentIndex_u32;
    return v_Ret_b;
  }
  static void setInitialBitmask_v(BitStream<DataType, false, true>& b_BitStream_ro)
  {
    b_BitStream_ro.bitMask_t = 1U; // LSB first
  }
};

//--------------------------------------------------------------------------
// LSB first + little Endian specialization
//--------------------------------------------------------------------------
template <typename DataType>
class BitStreamHelper<DataType, false, false>
{
public:
  static bool_t next_b(BitStream<DataType, false, false>& b_BitStream_ro)
  {
    bool_t v_Ret_b = (0U != (b_BitStream_ro.data_po[b_BitStream_ro.dataIndex_u32] & b_BitStream_ro.bitMask_t));
    b_BitStream_ro.bitMask_t <<= 1U; // LSB First
    if (0U == b_BitStream_ro.bitMask_t)
    {
      // go to next data index
      ++b_BitStream_ro.dataIndex_u32; // little Endian
      setInitialBitmask_v(b_BitStream_ro);
    }
    ++b_BitStream_ro.currentIndex_u32;
    return v_Ret_b;
  }
  static void setInitialBitmask_v(BitStream<DataType, false, false>& b_BitStream_ro)
  {
    b_BitStream_ro.bitMask_t = 1U; // LSB first
  }
};

//--------------------------------------------------------------------------
// next_b implementation that calls on one of the 4 specializations in the helper template
//--------------------------------------------------------------------------
template <typename DataType, bool_t MsbFirst, bool_t BigEndian>
bool_t BitStream<DataType, MsbFirst, BigEndian>::next_b()
{
  return BitStreamHelper<DataType, MsbFirst, BigEndian>::next_b(*this);
}

//--------------------------------------------------------------------------
// setIntialBitmask_v implementation that calls on one of the 4 specializations in the helper template
//--------------------------------------------------------------------------
template <typename DataType, bool_t MsbFirst, bool_t BigEndian>
void BitStream<DataType, MsbFirst, BigEndian>::setInitialBitmask_v()
{
  BitStreamHelper<DataType, MsbFirst, BigEndian>::setInitialBitmask_v(*this);
}

template <typename DataType, bool_t MsbFirst, bool_t BigEndian>
void BitStream<DataType, MsbFirst, BigEndian>::set_v(bool_t i_OnOff_b)
{
  if (i_OnOff_b)
  {
    data_po[dataIndex_u32] |= bitMask_t;
  }
  else
  {
    data_po[dataIndex_u32] &= (~bitMask_t);
  }
}

} // namespace core
} // namespace mecl

#endif /* BITSTREAM_H_ */
/// @}
/// @}
