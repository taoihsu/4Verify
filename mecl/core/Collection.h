//--------------------------------------------------------------------------
/// @file Collection.h
/// @brief Contains the definition for all collection type classes.
///
/// Collection is a common base class for array and list types that support iterators.
///
/// --------------------------------------------------------------------------
/// @copyright MAGNA Electronics - C O N F I D E N T I A L <br>
/// This document in its entirety is CONFIDENTIAL and may not be disclosed,
/// disseminated or distributed to parties outside MAGNA Electronics
/// without written permission from MAGNA Electronics.
///
/// @author Michael Schulte (michael.schulte@magna.com)
///
//  --------------------------------------------------------------------------
/// @section Collections
///
/// @subsection Classes
/// The namespace core contains collection templates.
/// All have in common that the used memory is static - there is no new operator involved!
/// So it should be considered in advance how many elements each collection should use at most.
/// All collections can be iterated upon, some also implement the []-access operator which is
/// checked to be in bounds.
/// @subsubsection Arrays
/// - @ref mecl::core::Array
///     - serves as replacement for C-arrays
///     - constant size, all elements initialized and available
/// - @ref mecl::core::ArrayN
///     - contains multidimensional arrays with [][]... operators
/// - @ref mecl::core::MemArray
///     - instead of allocating a static amount of memory, pre-defined memory regions are used
/// @subsubsection Lists
/// All list types have a dynamic size that is limited by a constant maximum size.
/// - @ref mecl::core::ArrayList
///     - serves as replacement for std::vector
///     - check with assert when size is exceeded
///     - elements have to be added and can be removed at the end
///     - can be cleared
/// - @ref mecl::core::MemArrayList
///     - mixture of ArrayList and MemArray to operate directly on memory
/// - @ref mecl::core::CircularBuffer
///     - similar to ArrayList, but overwrites oldest elements when size is exceeded
/// - @ref mecl::core::LinkedList
///     - elements are stored with linked previous and next elements
///     - elements can be inserted and removed arbitrarily
///     - can be sorted
/// @subsubsection Pooling
/// ObjectPool and PooledSmartPointer may be used to have smartpointer-like behavior
/// without need of new and delete. A smartpointer is removed, when all copies of it have
/// been destroyed.
///
//  --------------------------------------------------------------------------
#ifndef COLLECTION_H_
#define COLLECTION_H_

// PRQA S 2621 EOF // warning about small type used as reference does not make sense
// a) in templates and b) on 32-bit architecture

#include "Helpers.h"
#include "MeclAssert.h"


namespace mecl
{
/// Contains the collection templates
namespace core
{

/// Common base class for collection types such as List or ArrayList.
/// Already defines the iterator access and checks that the Collection is not of size 0.
template<typename T, uint32_t MaxSize, typename ConstIteratorType, bool_t Mutable = true>
class Collection
{
public:
    typedef typename ConstRefType<T>::type ConstRefType_t;

    /// Implementation of the Iterator that allows writable access to the contents by the '*' operator.
    class MutableIteratorType : public ConstIteratorType
    {
    public:
        explicit MutableIteratorType (const ConstIteratorType& i_Other_ro) : ConstIteratorType(i_Other_ro) {}
        /// Here the access is explicitly granted.
        T& operator*()
        {
            // PRQA S 3083 1 // const_cast is OK here.
            return const_cast<T&>(ConstIteratorType::operator*());
        }
        const T& operator*() const { return ConstIteratorType::operator*(); }
    private:
        MutableIteratorType();
    };
    typedef typename ConditionalType<Mutable, MutableIteratorType, ConstIteratorType>::type IteratorType_t;
    typedef typename ConditionalType<Mutable, T*, const T*>::type PtrType;

    /// Helper typedef to access the type of the collection elements
    typedef T innerType;

    /// Default constructor.
    Collection()
    {
        StaticAssert(MaxSize > 0, "MaxSize must not be zero");
    }

    /// Destructor - should actually not be called.
    virtual ~Collection() {}

    /// Constant access to the collection by iterator - needs to be implemented
    virtual ConstIteratorType begin_o() const = 0;

    /// Constant end iterator - needs to be implemented
    virtual ConstIteratorType end_o() const = 0;

    /// Implementation for begin_o requesting a const iterator explicitly.
    const ConstIteratorType roBegin_o() const { return begin_o(); }
    /// Implementation for end_o requesting a const iterator explicitly.
    const ConstIteratorType roEnd_o() const { return end_o(); }

    /// Implementation for begin_o requesting a non-const iterator explicitly.
    IteratorType_t rwBegin_o()
    {
        IteratorType_t ret(roBegin_o());
        return ret;
    }

    /// The maximum number of elements that this collection can hold
    uint32_t maxSize_u32() const
    {
        return MaxSize;
    }

    /// the number of items in the collection
    virtual uint32_t size_u32() const = 0;

    /// checks whether a list contains elements
    virtual bool_t isEmpty_b() const = 0;

    /// isFull_b only makes sense for lists, where size_u32() is incremented from 0 to maxSize.
    /// For Array it doesn2't make sense to use this method.
    bool_t isFull_b() const { return size_u32() == maxSize_u32(); }

    /// Returns an element from the list given a reference element.
    /// To find the element the "operator==" has to be available.
    /// @param i_Item_ro the item to find.
    /// @return writable access to the contained element.
    PtrType find_po(ConstRefType_t i_Item_ro);

    /// Checks if a given element has already been added to the collection.
    /// @param i_Item_ro the item to check against.
    /// @return true if the item is already contained, else false.
    bool_t contains_b(ConstRefType_t i_Item_ro) const;

protected:
    void copyItem_v(T& obj_ro, ConstRefType_t toCopy_ro);
    static bool_t compareItem_b(ConstRefType_t obj_ro, ConstRefType_t toCopy_ro);

private:
    IteratorType_t rwEnd_o() { return end_o(); }

    Collection(const Collection& i_Other_ro);
    Collection& operator= (const Collection& i_Other_ro);
};



// Helper templates for copying and comparing of items
// PRQA S 1062 ++ // wrong detection of forward declaration of class in QACPP
// PRQA S 1063 ++ // wrong detection of unused class in QACPP
// PRQA S 2106 ++ // for better readability the implementation is just defined in the short template definitions
// PRQA S 2122 ++ // It is OK here to define public static functions
/// Common helper templates for copying and comparing items
namespace impl
{
    /// Helper template used in implementation classes to copy the content with the default "=".
    /// This should only be called for non-primitive types.
    template<typename T, bool_t IsPrimitiveType>
    struct CopyOperator // IsPrimitive<T> == true
    {
        static void copy_v(T& obj_ro, T toCopy_ro) { obj_ro = toCopy_ro; }
    };

    /// Helper template used in implementation classes to copy the content with an explicit "operator="-call
    /// Needed to comply to MISRA-C++ Rule 14-6-2. This should only be called for non-primitive types.
    template<typename T>
    struct CopyOperator<T, false> // IsPrimitive<T> == false
    {
        static void copy_v(T& obj_ro, typename ConstRefType<T>::type toCopy_ro) { obj_ro.operator=(toCopy_ro); }
    };

    /// Helper template used in implementation classes to compare the content with the default "==".
    /// This is the implementation for primitive type.
    template<typename T, bool_t b> // IsPrimitive<T> == true
    struct CompareOperator
    {
        static bool_t compare_b(T obj_ro, T compare_ro) { return obj_ro == compare_ro; }
    };

    /// Helper template used in implementation classes to compare the contents with an explicit "operator=="-call
    /// Needed to comply to MISRA-C++ Rule 14-6-2. This should only be called for non-primitive types.
    template<typename T>
    struct CompareOperator<T, false> // IsPrimitive<T> == false
    {
        static bool_t compare_b(typename ConstRefType<T>::type obj_ro, typename ConstRefType<T>::type compare_ro) { return obj_ro.operator==(compare_ro); }
    };
}
// PRQA S 1062 -- // PRQA S 1063 -- // PRQA S 2106 -- // PRQA S 2122 --

// PRQA S 2620 ++ // cannot help using references here also for built-in types

template<typename T, uint32_t MaxSize, typename ConstIteratorType, bool_t Mutable>
typename Collection<T, MaxSize, ConstIteratorType, Mutable>::PtrType
  Collection<T, MaxSize, ConstIteratorType, Mutable>::find_po(ConstRefType_t i_Item_ro)
{
    // because of "argument dependent lookup" warning (QACPP 2088) some operators are used explicitly here
    PtrType v_Ret_po = NULL;
    IteratorType_t v_Iter_ro = rwBegin_o();
    while ((NULL == v_Ret_po) && (end_o().operator!=(v_Iter_ro)))
    {
        T& v_ActualItem_ro = v_Iter_ro.operator*();
        if (compareItem_b(v_ActualItem_ro, i_Item_ro))
        {
            v_Ret_po = &v_ActualItem_ro;
        }
        v_Iter_ro.operator++();
    }
    return v_Ret_po;
}


template<typename T, uint32_t MaxSize, typename ConstIteratorType, bool_t Mutable>
bool_t Collection<T, MaxSize, ConstIteratorType, Mutable>::contains_b(ConstRefType_t i_Item_ro) const
{
    // PRQA S 3083 1 // implementation refers to non-const implementation, but this is totally safe here
    return (NULL != (const_cast<Collection<T, MaxSize, ConstIteratorType, Mutable>&>(*this)).find_po(i_Item_ro));
}

template<typename T, uint32_t MaxSize, typename ConstIteratorType, bool_t Mutable>
void Collection<T, MaxSize, ConstIteratorType, Mutable>::copyItem_v(T& obj_ro, ConstRefType_t toCopy_ro)
{
    impl::CopyOperator<T,IsPrimitive<T>::value>::copy_v(obj_ro, toCopy_ro);
}

template<typename T, uint32_t MaxSize, typename ConstIteratorType, bool_t Mutable>
bool_t Collection<T, MaxSize, ConstIteratorType, Mutable>::compareItem_b(ConstRefType_t obj_ro, ConstRefType_t toCopy_ro)
{
    return impl::CompareOperator<T,IsPrimitive<T>::value>::compare_b(obj_ro, toCopy_ro);
}

// PRQA S 2620 --

}
// namespace core
}// namespace mecl


#endif // COLLECTION_H_
