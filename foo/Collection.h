/*
 * Collection.h
 *
 *  Created on: 11.06.2014
 *      Author: MEE_MiSch
 */

#ifndef TSC_COLLECTION_H_
#define TSC_COLLECTION_H_

#if 0

#include <cstddef>
#include <stdint.h>
#include <cassert>
#include "Helpers.h"

namespace core
{

/// Common base class for collection types such as List or ArrayList.
/// Already defines the iterator access and checks that the Collection is not of size 0.
template <typename T, typename SizeType, SizeType MaxSize, typename IteratorType>
class Collection
{
public:
    /// Creates a collection and checks that the MaxSize is not 0.
	Collection() { StaticAssertAlt(MaxSize > 0); }
	/// Destructor - should actually not be called.
	virtual ~Collection() {}

	/// Constant access to the collection by iterator - needs to be implemented
    virtual const IteratorType begin_o() const = 0;
    /// Constant end iterator - needs to be implemented
    virtual const IteratorType end_o() const = 0;
    /// Non-const access to the collection by iterator. Simply wraps the const version by default.
	virtual IteratorType begin_o() { return const_cast<const Collection*>(this)->begin_o(); }
    /// Non-const access to end of the collection. Simply wraps the const version by default.
	virtual IteratorType end_o() { return const_cast<const Collection*>(this)->end_o(); }

	/// The maximum number of elements that this collection can hold
	inline SizeType maxSize_u() const { return MaxSize; }
	/// the number of items in the collection
	virtual SizeType size_u() const { return MaxSize; }
	/// checks whether a list contains elements
	virtual bool isEmpty_b() const { return size_u() == 0U; }
	/// isFull_b only makes sense for lists, where size_u() is incremented from 0 to maxSize.
	/// For Array it doesn't make sense to use this method.
	virtual bool isFull_b() const { return size_u() == maxSize_u(); }

	/// Helper typedef to access the type of the collection elements
	typedef T type;
};

/// use this macro to get the type of the iterator more easily
/// @code{
/// for (iterator_type(myCollection_o) iter_o = myCollection_o.begin_o(); ....)
/// }
#define iterator_type(i_container_o) \
    ConditionalType<IsConst<__typeof(i_container_o)>::value, const makeType((i_container_o).begin_o()), makeType((i_container_o).begin_o())>::type

#define element_type(i_container_o) \
    ConditionalType<IsConst<__typeof(i_container_o)>::value, const makeType(*((i_container_o).begin_o())), makeType(*((i_container_o).begin_o()))>::type

} // namespace core
#endif
#endif /* COLLECTION_H_ */
