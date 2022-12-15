/*
 * UAVCAN data structure definition for libuavcan.
 *
 * Autogenerated, do not edit.
 *
 * Source file: C:\Users\tstro\Documents\PlatformIO\Projects\Battery_Leveling_Board\lib\libuavcan\libuavcan\dsdl\uavcan\equipment\safety\1100.ArmingStatus.uavcan
 */

#ifndef UAVCAN_EQUIPMENT_SAFETY_ARMINGSTATUS_HPP_INCLUDED
#define UAVCAN_EQUIPMENT_SAFETY_ARMINGSTATUS_HPP_INCLUDED

#include <uavcan/build_config.hpp>
#include <uavcan/node/global_data_type_registry.hpp>
#include <uavcan/marshal/types.hpp>

/******************************* Source text **********************************
#
# This message represents the system arming status.
# Some nodes may refuse to operate unless the system is fully armed.
#

uint8 STATUS_DISARMED           = 0
uint8 STATUS_FULLY_ARMED        = 255

uint8 status
******************************************************************************/

/********************* DSDL signature source definition ***********************
uavcan.equipment.safety.ArmingStatus
saturated uint8 status
******************************************************************************/

#undef status
#undef STATUS_DISARMED
#undef STATUS_FULLY_ARMED

namespace uavcan
{
namespace equipment
{
namespace safety
{

template <int _tmpl>
struct UAVCAN_EXPORT ArmingStatus_
{
    typedef const ArmingStatus_<_tmpl>& ParameterType;
    typedef ArmingStatus_<_tmpl>& ReferenceType;

    struct ConstantTypes
    {
        typedef ::uavcan::IntegerSpec< 8, ::uavcan::SignednessUnsigned, ::uavcan::CastModeSaturate > STATUS_DISARMED;
        typedef ::uavcan::IntegerSpec< 8, ::uavcan::SignednessUnsigned, ::uavcan::CastModeSaturate > STATUS_FULLY_ARMED;
    };

    struct FieldTypes
    {
        typedef ::uavcan::IntegerSpec< 8, ::uavcan::SignednessUnsigned, ::uavcan::CastModeSaturate > status;
    };

    enum
    {
        MinBitLen
            = FieldTypes::status::MinBitLen
    };

    enum
    {
        MaxBitLen
            = FieldTypes::status::MaxBitLen
    };

    // Constants
    static const typename ::uavcan::StorageType< typename ConstantTypes::STATUS_DISARMED >::Type STATUS_DISARMED; // 0
    static const typename ::uavcan::StorageType< typename ConstantTypes::STATUS_FULLY_ARMED >::Type STATUS_FULLY_ARMED; // 255

    // Fields
    typename ::uavcan::StorageType< typename FieldTypes::status >::Type status;

    ArmingStatus_()
        : status()
    {
        ::uavcan::StaticAssert<_tmpl == 0>::check();  // Usage check

#if UAVCAN_DEBUG
        /*
         * Cross-checking MaxBitLen provided by the DSDL compiler.
         * This check shall never be performed in user code because MaxBitLen value
         * actually depends on the nested types, thus it is not invariant.
         */
        ::uavcan::StaticAssert<8 == MaxBitLen>::check();
#endif
    }

    bool operator==(ParameterType rhs) const;
    bool operator!=(ParameterType rhs) const { return !operator==(rhs); }

    /**
     * This comparison is based on @ref uavcan::areClose(), which ensures proper comparison of
     * floating point fields at any depth.
     */
    bool isClose(ParameterType rhs) const;

    static int encode(ParameterType self, ::uavcan::ScalarCodec& codec,
                      ::uavcan::TailArrayOptimizationMode tao_mode = ::uavcan::TailArrayOptEnabled);

    static int decode(ReferenceType self, ::uavcan::ScalarCodec& codec,
                      ::uavcan::TailArrayOptimizationMode tao_mode = ::uavcan::TailArrayOptEnabled);

    /*
     * Static type info
     */
    enum { DataTypeKind = ::uavcan::DataTypeKindMessage };
    enum { DefaultDataTypeID = 1100 };

    static const char* getDataTypeFullName()
    {
        return "uavcan.equipment.safety.ArmingStatus";
    }

    static void extendDataTypeSignature(::uavcan::DataTypeSignature& signature)
    {
        signature.extend(getDataTypeSignature());
    }

    static ::uavcan::DataTypeSignature getDataTypeSignature();

};

/*
 * Out of line struct method definitions
 */

template <int _tmpl>
bool ArmingStatus_<_tmpl>::operator==(ParameterType rhs) const
{
    return
        status == rhs.status;
}

template <int _tmpl>
bool ArmingStatus_<_tmpl>::isClose(ParameterType rhs) const
{
    return
        ::uavcan::areClose(status, rhs.status);
}

template <int _tmpl>
int ArmingStatus_<_tmpl>::encode(ParameterType self, ::uavcan::ScalarCodec& codec,
    ::uavcan::TailArrayOptimizationMode tao_mode)
{
    (void)self;
    (void)codec;
    (void)tao_mode;
    int res = 1;
    res = FieldTypes::status::encode(self.status, codec,  tao_mode);
    return res;
}

template <int _tmpl>
int ArmingStatus_<_tmpl>::decode(ReferenceType self, ::uavcan::ScalarCodec& codec,
    ::uavcan::TailArrayOptimizationMode tao_mode)
{
    (void)self;
    (void)codec;
    (void)tao_mode;
    int res = 1;
    res = FieldTypes::status::decode(self.status, codec,  tao_mode);
    return res;
}

/*
 * Out of line type method definitions
 */
template <int _tmpl>
::uavcan::DataTypeSignature ArmingStatus_<_tmpl>::getDataTypeSignature()
{
    ::uavcan::DataTypeSignature signature(0x8700F375556A8003ULL);

    FieldTypes::status::extendDataTypeSignature(signature);

    return signature;
}

/*
 * Out of line constant definitions
 */

template <int _tmpl>
const typename ::uavcan::StorageType< typename ArmingStatus_<_tmpl>::ConstantTypes::STATUS_DISARMED >::Type
    ArmingStatus_<_tmpl>::STATUS_DISARMED = 0U; // 0

template <int _tmpl>
const typename ::uavcan::StorageType< typename ArmingStatus_<_tmpl>::ConstantTypes::STATUS_FULLY_ARMED >::Type
    ArmingStatus_<_tmpl>::STATUS_FULLY_ARMED = 255U; // 255

/*
 * Final typedef
 */
typedef ArmingStatus_<0> ArmingStatus;

namespace
{

const ::uavcan::DefaultDataTypeRegistrator< ::uavcan::equipment::safety::ArmingStatus > _uavcan_gdtr_registrator_ArmingStatus;

}

} // Namespace safety
} // Namespace equipment
} // Namespace uavcan

/*
 * YAML streamer specialization
 */
namespace uavcan
{

template <>
class UAVCAN_EXPORT YamlStreamer< ::uavcan::equipment::safety::ArmingStatus >
{
public:
    template <typename Stream>
    static void stream(Stream& s, ::uavcan::equipment::safety::ArmingStatus::ParameterType obj, const int level);
};

template <typename Stream>
void YamlStreamer< ::uavcan::equipment::safety::ArmingStatus >::stream(Stream& s, ::uavcan::equipment::safety::ArmingStatus::ParameterType obj, const int level)
{
    (void)s;
    (void)obj;
    (void)level;
    if (level > 0)
    {
        s << '\n';
        for (int pos = 0; pos < level; pos++)
        {
            s << "  ";
        }
    }
    s << "status: ";
    YamlStreamer< ::uavcan::equipment::safety::ArmingStatus::FieldTypes::status >::stream(s, obj.status, level + 1);
}

}

namespace uavcan
{
namespace equipment
{
namespace safety
{

template <typename Stream>
inline Stream& operator<<(Stream& s, ::uavcan::equipment::safety::ArmingStatus::ParameterType obj)
{
    ::uavcan::YamlStreamer< ::uavcan::equipment::safety::ArmingStatus >::stream(s, obj, 0);
    return s;
}

} // Namespace safety
} // Namespace equipment
} // Namespace uavcan

#endif // UAVCAN_EQUIPMENT_SAFETY_ARMINGSTATUS_HPP_INCLUDED