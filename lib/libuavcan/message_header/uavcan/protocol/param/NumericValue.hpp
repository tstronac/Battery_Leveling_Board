/*
 * UAVCAN data structure definition for libuavcan.
 *
 * Autogenerated, do not edit.
 *
 * Source file: C:\Users\tstro\Documents\PlatformIO\Projects\Battery_Leveling_Board\lib\libuavcan\libuavcan\dsdl\uavcan\protocol\param\NumericValue.uavcan
 */

#ifndef UAVCAN_PROTOCOL_PARAM_NUMERICVALUE_HPP_INCLUDED
#define UAVCAN_PROTOCOL_PARAM_NUMERICVALUE_HPP_INCLUDED

#include <uavcan/build_config.hpp>
#include <uavcan/node/global_data_type_registry.hpp>
#include <uavcan/marshal/types.hpp>

#include <uavcan\protocol\param\Empty.hpp>

/******************************* Source text **********************************
#
# Numeric-only value.
#
# This is a union, which means that this structure can contain either one of the fields below.
# The structure is prefixed with tag - a selector value that indicates which particular field is encoded.
#

@union                          # Tag is 2 bits long.

Empty empty                     # Empty field, used to represent an undefined value.

int64   integer_value
float32 real_value
******************************************************************************/

/********************* DSDL signature source definition ***********************
uavcan.protocol.param.NumericValue
@union
uavcan.protocol.param.Empty empty
saturated int64 integer_value
saturated float32 real_value
******************************************************************************/

#undef empty
#undef integer_value
#undef real_value

namespace uavcan
{
namespace protocol
{
namespace param
{

template <int _tmpl>
struct UAVCAN_EXPORT NumericValue_
{
    typedef const NumericValue_<_tmpl>& ParameterType;
    typedef NumericValue_<_tmpl>& ReferenceType;

    struct ConstantTypes
    {
    };

    struct FieldTypes
    {
        typedef ::uavcan::protocol::param::Empty empty;
        typedef ::uavcan::IntegerSpec< 64, ::uavcan::SignednessSigned, ::uavcan::CastModeSaturate > integer_value;
        typedef ::uavcan::FloatSpec< 32, ::uavcan::CastModeSaturate > real_value;
    };

    struct Tag
    {
        enum Type
        {
            empty,
            integer_value,
            real_value
        };
    };

    typedef ::uavcan::IntegerSpec< ::uavcan::IntegerBitLen< 3U - 1U >::Result,
                                   ::uavcan::SignednessUnsigned, ::uavcan::CastModeTruncate > TagType;

    enum
    {
        MinBitLen = TagType::BitLen +
            ::uavcan::EnumMin<FieldTypes::empty::MinBitLen,
            ::uavcan::EnumMin<FieldTypes::integer_value::MinBitLen,
                FieldTypes::real_value::MinBitLen >::Result>::Result
    };

    enum
    {
        MaxBitLen = TagType::BitLen +
            ::uavcan::EnumMax<FieldTypes::empty::MaxBitLen,
            ::uavcan::EnumMax<FieldTypes::integer_value::MaxBitLen,
                FieldTypes::real_value::MaxBitLen >::Result>::Result
    };

    // Constants

    // Fields
    typename ::uavcan::StorageType< typename FieldTypes::empty >::Type empty;
    typename ::uavcan::StorageType< typename FieldTypes::integer_value >::Type integer_value;
    typename ::uavcan::StorageType< typename FieldTypes::real_value >::Type real_value;

private:
    typename ::uavcan::StorageType< TagType >::Type _tag_;  // The name is mangled to avoid clashing with fields

    template <typename Tag::Type T>
    struct TagToType;

public:

    NumericValue_()
        : empty()
        , integer_value()
        , real_value()
        , _tag_()
    {
        ::uavcan::StaticAssert<_tmpl == 0>::check();  // Usage check

#if UAVCAN_DEBUG
        /*
         * Cross-checking MaxBitLen provided by the DSDL compiler.
         * This check shall never be performed in user code because MaxBitLen value
         * actually depends on the nested types, thus it is not invariant.
         */
        ::uavcan::StaticAssert<66 == MaxBitLen>::check();
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

    /**
     * Explicit access to the tag.
     * It is safer to use is()/as()/to() instead.
     */
    typename Tag::Type getTag() const { return typename Tag::Type(_tag_); }
    void setTag(typename Tag::Type x) { _tag_ = typename ::uavcan::StorageType< TagType >::Type(x); }

    /**
     * Whether the union is set to the given type.
     * Access by tag; this will work even if there are non-unique types within the union.
     */
    bool is(typename Tag::Type x) const { return typename Tag::Type(_tag_) == x; }

    /**
     * If the union is currently set to the type T, returns pointer to the appropriate field.
     * If the union is set to another type, returns null pointer.
     */
    template <typename Tag::Type T>
    inline const typename TagToType<T>::StorageType* as() const;

    /**
     * Switches the union to the given type and returns a mutable reference to the appropriate field.
     * If the previous type was different, a default constructor will be called first.
     */
    template <typename Tag::Type T>
    inline typename TagToType<T>::StorageType& to();

    /*
     * Static type info
     */
    enum { DataTypeKind = ::uavcan::DataTypeKindMessage };
    // This type has no default data type ID

    static const char* getDataTypeFullName()
    {
        return "uavcan.protocol.param.NumericValue";
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
bool NumericValue_<_tmpl>::operator==(ParameterType rhs) const
{
    if (_tag_ != rhs._tag_)
    {
        return false;
    }
    if (_tag_ == 0)
    {
        return empty == rhs.empty;
    }
    if (_tag_ == 1)
    {
        return integer_value == rhs.integer_value;
    }
    if (_tag_ == 2)
    {
        return real_value == rhs.real_value;
    }
    UAVCAN_ASSERT(0);   // Invalid tag
    return false;
}

template <int _tmpl>
bool NumericValue_<_tmpl>::isClose(ParameterType rhs) const
{
    if (_tag_ != rhs._tag_)
    {
        return false;
    }
    if (_tag_ == 0)
    {
        return ::uavcan::areClose(empty, rhs.empty);
    }
    if (_tag_ == 1)
    {
        return ::uavcan::areClose(integer_value, rhs.integer_value);
    }
    if (_tag_ == 2)
    {
        return ::uavcan::areClose(real_value, rhs.real_value);
    }
    UAVCAN_ASSERT(0);   // Invalid tag
    return false;
}

template <int _tmpl>
int NumericValue_<_tmpl>::encode(ParameterType self, ::uavcan::ScalarCodec& codec,
    ::uavcan::TailArrayOptimizationMode tao_mode)
{
    (void)self;
    (void)codec;
    (void)tao_mode;
    const int res = TagType::encode(self._tag_, codec, ::uavcan::TailArrayOptDisabled);
    if (res <= 0)
    {
        return res;
    }
    if (self._tag_ == 0)
    {
        return FieldTypes::empty::encode(self.empty, codec, tao_mode);
    }
    if (self._tag_ == 1)
    {
        return FieldTypes::integer_value::encode(self.integer_value, codec, tao_mode);
    }
    if (self._tag_ == 2)
    {
        return FieldTypes::real_value::encode(self.real_value, codec, tao_mode);
    }
    return -1;          // Invalid tag value
}

template <int _tmpl>
int NumericValue_<_tmpl>::decode(ReferenceType self, ::uavcan::ScalarCodec& codec,
    ::uavcan::TailArrayOptimizationMode tao_mode)
{
    (void)self;
    (void)codec;
    (void)tao_mode;
    const int res = TagType::decode(self._tag_, codec, ::uavcan::TailArrayOptDisabled);
    if (res <= 0)
    {
        return res;
    }
    if (self._tag_ == 0)
    {
        return FieldTypes::empty::decode(self.empty, codec, tao_mode);
    }
    if (self._tag_ == 1)
    {
        return FieldTypes::integer_value::decode(self.integer_value, codec, tao_mode);
    }
    if (self._tag_ == 2)
    {
        return FieldTypes::real_value::decode(self.real_value, codec, tao_mode);
    }
    return -1;          // Invalid tag value
}

template <>
template <>
struct NumericValue_<0>::TagToType<NumericValue_<0>::Tag::empty>
{
    typedef typename NumericValue_<0>::FieldTypes::empty Type;
    typedef typename ::uavcan::StorageType<Type>::Type StorageType;
};

template <>
template <>
inline const typename NumericValue_<0>::TagToType< NumericValue_<0>::Tag::empty >::StorageType*
NumericValue_<0>::as< NumericValue_<0>::Tag::empty >() const
{
    return is(NumericValue_<0>::Tag::empty) ? &empty : UAVCAN_NULLPTR;
}

template <>
template <>
inline typename NumericValue_<0>::TagToType< NumericValue_<0>::Tag::empty >::StorageType&
NumericValue_<0>::to< NumericValue_<0>::Tag::empty >()
{
    if (_tag_ != 0)
    {
        _tag_ = 0;
        empty = typename TagToType< NumericValue_<0>::Tag::empty >::StorageType();
    }
    return empty;
}

template <>
template <>
struct NumericValue_<0>::TagToType<NumericValue_<0>::Tag::integer_value>
{
    typedef typename NumericValue_<0>::FieldTypes::integer_value Type;
    typedef typename ::uavcan::StorageType<Type>::Type StorageType;
};

template <>
template <>
inline const typename NumericValue_<0>::TagToType< NumericValue_<0>::Tag::integer_value >::StorageType*
NumericValue_<0>::as< NumericValue_<0>::Tag::integer_value >() const
{
    return is(NumericValue_<0>::Tag::integer_value) ? &integer_value : UAVCAN_NULLPTR;
}

template <>
template <>
inline typename NumericValue_<0>::TagToType< NumericValue_<0>::Tag::integer_value >::StorageType&
NumericValue_<0>::to< NumericValue_<0>::Tag::integer_value >()
{
    if (_tag_ != 1)
    {
        _tag_ = 1;
        integer_value = typename TagToType< NumericValue_<0>::Tag::integer_value >::StorageType();
    }
    return integer_value;
}

template <>
template <>
struct NumericValue_<0>::TagToType<NumericValue_<0>::Tag::real_value>
{
    typedef typename NumericValue_<0>::FieldTypes::real_value Type;
    typedef typename ::uavcan::StorageType<Type>::Type StorageType;
};

template <>
template <>
inline const typename NumericValue_<0>::TagToType< NumericValue_<0>::Tag::real_value >::StorageType*
NumericValue_<0>::as< NumericValue_<0>::Tag::real_value >() const
{
    return is(NumericValue_<0>::Tag::real_value) ? &real_value : UAVCAN_NULLPTR;
}

template <>
template <>
inline typename NumericValue_<0>::TagToType< NumericValue_<0>::Tag::real_value >::StorageType&
NumericValue_<0>::to< NumericValue_<0>::Tag::real_value >()
{
    if (_tag_ != 2)
    {
        _tag_ = 2;
        real_value = typename TagToType< NumericValue_<0>::Tag::real_value >::StorageType();
    }
    return real_value;
}

/*
 * Out of line type method definitions
 */
template <int _tmpl>
::uavcan::DataTypeSignature NumericValue_<_tmpl>::getDataTypeSignature()
{
    ::uavcan::DataTypeSignature signature(0x1222EEA596AD701CULL);

    FieldTypes::empty::extendDataTypeSignature(signature);
    FieldTypes::integer_value::extendDataTypeSignature(signature);
    FieldTypes::real_value::extendDataTypeSignature(signature);

    return signature;
}

/*
 * Out of line constant definitions
 */

/*
 * Final typedef
 */
typedef NumericValue_<0> NumericValue;

// No default registration

} // Namespace param
} // Namespace protocol
} // Namespace uavcan

/*
 * YAML streamer specialization
 */
namespace uavcan
{

template <>
class UAVCAN_EXPORT YamlStreamer< ::uavcan::protocol::param::NumericValue >
{
public:
    template <typename Stream>
    static void stream(Stream& s, ::uavcan::protocol::param::NumericValue::ParameterType obj, const int level);
};

template <typename Stream>
void YamlStreamer< ::uavcan::protocol::param::NumericValue >::stream(Stream& s, ::uavcan::protocol::param::NumericValue::ParameterType obj, const int level)
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
    if (static_cast<int>(obj.getTag()) == 0)
    {
        s << "empty: ";
        YamlStreamer< ::uavcan::protocol::param::NumericValue::FieldTypes::empty >::stream(s, obj.empty, level + 1);
    }
    if (static_cast<int>(obj.getTag()) == 1)
    {
        s << "integer_value: ";
        YamlStreamer< ::uavcan::protocol::param::NumericValue::FieldTypes::integer_value >::stream(s, obj.integer_value, level + 1);
    }
    if (static_cast<int>(obj.getTag()) == 2)
    {
        s << "real_value: ";
        YamlStreamer< ::uavcan::protocol::param::NumericValue::FieldTypes::real_value >::stream(s, obj.real_value, level + 1);
    }
}

}

namespace uavcan
{
namespace protocol
{
namespace param
{

template <typename Stream>
inline Stream& operator<<(Stream& s, ::uavcan::protocol::param::NumericValue::ParameterType obj)
{
    ::uavcan::YamlStreamer< ::uavcan::protocol::param::NumericValue >::stream(s, obj, 0);
    return s;
}

} // Namespace param
} // Namespace protocol
} // Namespace uavcan

#endif // UAVCAN_PROTOCOL_PARAM_NUMERICVALUE_HPP_INCLUDED