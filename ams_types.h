/*
 *****************************************************************************
 * Copyright by ams AG                                                       *
 * All rights are reserved.                                                  *
 *                                                                           *
 * IMPORTANT - PLEASE READ CAREFULLY BEFORE COPYING, INSTALLING OR USING     *
 * THE SOFTWARE.                                                             *
 *                                                                           *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS       *
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT         *
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS         *
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT  *
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,     *
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT          *
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,     *
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY     *
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT       *
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE     *
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.      *
 *****************************************************************************
 */
/*
 *      PROJECT:   ASxxxx firmware
 *      $Revision: $
 *      LANGUAGE:  ANSI C
 */

/*! \file ams_types.h
 *
 *  \author 
 *
 *  \brief Basic datatypes
 *  
 */

#ifndef AMS_TYPES_H
#define AMS_TYPES_H

#include "GenericTypeDefs.h"
#include <asm/types.h>
/*! \defgroup datatypes Basic datatypes
 * Basic datatypes are mapped to ams datatypes that
 * shall be used in all ams projects.
 */

#define U8_C(x)     (x) /*!< \ingroup datatypes
                         Define a constant of type u8 */
#define S8_C(x)     (x) /*!< \ingroup datatypes
                         Define a constant of type s8 */
#define U16_C(x)    (x) /*!< \ingroup datatypes
                         Define a constant of type u16 */
#define S16_C(x)    (x) /*!< \ingroup datatypes
                         Define a constant of type s16 */
#define U32_C(x)    (x##UL) /*!< \ingroup datatypes
                             Define a constant of type u32 */
#define S32_C(x)    (x##L) /*!< \ingroup datatypes
                            Define a constant of type s32 */
#define U64_C(x)    (x##ULL) /*!< \ingroup datatypes
                              Define a constant of type u64 */
#define S64_C(x)    (x##LL) /*!< \ingroup datatypes
                             Define a constant of type s64 */
#define UMWORD_C(x) (x) /*!< \ingroup datatypes
                         Define a constant of type umword */
#define MWORD_C(x)  (x) /*!< \ingroup datatypes
                         Define a constant of type mword */

#if 0
typedef umword bool_t; /*!< \ingroup datatypes
                            represents a boolean type */

#ifndef TRUE
#define TRUE 1 /*!< \ingroup datatypes
		 used for the #bool_t type */
#endif
#ifndef FALSE
#define FALSE 0 /*!< \ingroup datatypes
		 used for the #bool_t type */
#endif

#else 
//typedef BOOL bool_t;
#endif

#define bool_t char 

#ifndef TRUE
#define TRUE 1
#endif

/*
#ifndef false
#define  false 0
#endif
*/
#ifndef FALSE
#define  FALSE 0
#endif

/*
#ifndef NULL
#define NULL (void*)0 
#endif
#define bool_t char 
#define bool_t char */

#ifndef NULL
#define NULL (void*)0 
#endif

#endif /* AMS_TYPES_H */

