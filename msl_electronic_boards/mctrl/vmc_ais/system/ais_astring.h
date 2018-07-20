//////////////////////////////////////////////////////////////////////////////
/// \ingroup astrings Strings
//@{
/// \file ais_astring.h
///
/// \brief 	Functions for dealing with ais Strings.
///
/// ais Strings are not 0 terminated, that way 0 can also be part of String.
/// First Byte in the ais String contains length of String.\n
/// Maximal String length is limited to 254 + 1
///
/// \author Adam Cwientzek
///
/// \version 0.8
///
/// \date 31.06.2005
///
//////////////////////////////////////////////////////////////////////////////

#ifndef _AIS_ASTRING_H_
#define _AIS_ASTRING_H_

#include "ais_typedef.h"

VMC_UCHAR_8 astring_create(VMC_UCHAR_8 *dest, VMC_UCHAR_8 length, VMC_UCHAR_8 *data);
void astring_copy(VMC_UCHAR_8 *dest, VMC_UCHAR_8 *src);
VMC_UCHAR_8 astring_len(VMC_UCHAR_8 *astring);
void astring_fromChar(VMC_UCHAR_8 *dest, VMC_UCHAR_8 src);
VMC_UCHAR_8 astring_to_string(VMC_UCHAR_8 *string, VMC_UCHAR_8 *astring);
VMC_UCHAR_8 string_to_astring(VMC_UCHAR_8 *astring, VMC_UCHAR_8 *string);

#endif /* _AIS_ASTRING_H_ */

//@}
