/* LzmaDec.h -- LZMA Decoder
2023-04-02 : Igor Pavlov : Public domain */

/** With changes by Nordic Semiconductor ASA */

#ifndef ZIP7_INC_LZMA_DEC_H
#define ZIP7_INC_LZMA_DEC_H

#include "7zTypes.h"

EXTERN_C_BEGIN

/* #define Z7_LZMA_PROB32 */
/* Z7_LZMA_PROB32 can increase the speed on some CPUs,
   but memory usage for CLzmaDec::probs will be doubled in that case */

typedef
#ifdef Z7_LZMA_PROB32
	UInt32
#else
	UInt16
#endif
		CLzmaProb;

/* ---------- LZMA Properties ---------- */

#define LZMA_PROPS_SIZE 5

typedef struct {
	Byte lc;
	Byte lp;
	Byte pb;
	Byte _pad_;
	UInt32 dicSize;
} CLzmaProps;

/* LzmaProps_Decode - decodes properties
Returns:
  SZ_OK
  SZ_ERROR_UNSUPPORTED - Unsupported properties
*/

SRes LzmaProps_Decode(CLzmaProps *p, const Byte *data, unsigned size);

/* ---------- LZMA Decoder state ---------- */

/* LZMA_REQUIRED_INPUT_MAX = number of required input bytes for worst case.
   Num bits = log2((2^11 / 31) ^ 22) + 26 < 134 + 26 = 160; */

#define LZMA_REQUIRED_INPUT_MAX 20

#ifdef CONFIG_NRF_COMPRESS_EXTERNAL_DICTIONARY
/* Handler type for external dictionary operations.
 * See LzmaDictionaryOpen() and other dictionary interfaces to be implemented
 * by the user of this library below.
 */
typedef struct DictHandle_t {
	BoolInt isOpened;
	SizeT dicBufSize;

} DictHandle;
#endif

typedef struct {
	/* Don't change this structure. ASM code can use it. */
	CLzmaProps prop;
	CLzmaProb *probs;
	CLzmaProb *probs_1664;
#ifdef CONFIG_NRF_COMPRESS_EXTERNAL_DICTIONARY
	DictHandle *dicHandle;
#else
	Byte *dic;
	SizeT dicBufSize;
#endif
	SizeT dicPos;
	const Byte *buf;
	UInt32 range;
	UInt32 code;
	UInt32 processedPos;
	UInt32 checkDicSize;
	UInt32 reps[4];
	UInt32 state;
	UInt32 remainLen;

	UInt32 numProbs;
	unsigned tempBufSize;
	Byte tempBuf[LZMA_REQUIRED_INPUT_MAX];
} CLzmaDec;

#ifdef CONFIG_NRF_COMPRESS_EXTERNAL_DICTIONARY
#define LzmaDec_CONSTRUCT(p)                                                                       \
	{                                                                                          \
		(p)->dicHandle = NULL;                                                                   \
		(p)->probs = NULL;                                                                 \
	}
#else
#define LzmaDec_CONSTRUCT(p)                                                                       \
	{                                                                                          \
		(p)->dic = NULL;                                                                   \
		(p)->probs = NULL;                                                                 \
	}
#endif
#define LzmaDec_Construct(p) LzmaDec_CONSTRUCT(p)

void LzmaDec_Init(CLzmaDec *p);

/* There are two types of LZMA streams:
     - Stream with end mark. That end mark adds about 6 bytes to compressed size.
     - Stream without end mark. You must know exact uncompressed size to decompress such stream. */

typedef enum {
	LZMA_FINISH_ANY, /* finish at any point */
	LZMA_FINISH_END	 /* block must be finished at the end */
} ELzmaFinishMode;

/* ELzmaFinishMode has meaning only if the decoding reaches output limit !!!

   You must use LZMA_FINISH_END, when you know that current output buffer
   covers last bytes of block. In other cases you must use LZMA_FINISH_ANY.

   If LZMA decoder sees end marker before reaching output limit, it returns SZ_OK,
   and output value of destLen will be less than output buffer size limit.
   You can check status result also.

   You can use multiple checks to test data integrity after full decompression:
     1) Check Result and "status" variable.
     2) Check that output(destLen) = uncompressedSize, if you know real uncompressedSize.
     3) Check that output(srcLen) = compressedSize, if you know real compressedSize.
	You must use correct finish mode in that case. */

typedef enum {
	LZMA_STATUS_NOT_SPECIFIED,		/* use main error code instead */
	LZMA_STATUS_FINISHED_WITH_MARK,		/* stream was finished with end mark. */
	LZMA_STATUS_NOT_FINISHED,		/* stream was not finished */
	LZMA_STATUS_NEEDS_MORE_INPUT,		/* you must provide more input bytes */
	LZMA_STATUS_MAYBE_FINISHED_WITHOUT_MARK /* there is probability that stream was finished
						   without end mark */
} ELzmaStatus;

/* ELzmaStatus is used only as output value for function call */

/* ---------- Interfaces ---------- */

/* There are 3 levels of interfaces:
     1) Dictionary Interface
     2) Buffer Interface
     3) One Call Interface
   You can select any of these interfaces, but don't mix functions from different
   groups for same object. */

/* There are two variants to allocate state for Dictionary Interface:
     1) LzmaDec_Allocate / LzmaDec_Free
     2) LzmaDec_AllocateProbs / LzmaDec_FreeProbs
   You can use variant 2, if you set dictionary buffer manually.
   For Buffer Interface you must always use variant 1.

LzmaDec_Allocate* can return:
  SZ_OK
  SZ_ERROR_MEM         - Memory allocation error
  SZ_ERROR_UNSUPPORTED - Unsupported properties
*/

SRes LzmaDec_AllocateProbs(CLzmaDec *p, const Byte *props, unsigned propsSize, ISzAllocPtr alloc);
void LzmaDec_FreeProbs(CLzmaDec *p, ISzAllocPtr alloc);

SRes LzmaDec_Allocate(CLzmaDec *p, const Byte *props, unsigned propsSize, ISzAllocPtr alloc);
void LzmaDec_Free(CLzmaDec *p, ISzAllocPtr alloc);

/* ---------- Dictionary Interface ---------- */

/* You can use it, if you want to eliminate the overhead for data copying from
   dictionary to some other external buffer.
   You must work with CLzmaDec variables directly in this interface.

   STEPS:
     LzmaDec_Construct()
     LzmaDec_Allocate()
     for (each new stream)
     {
       LzmaDec_Init()
       while (it needs more decompression)
       {
	 LzmaDec_DecodeToDic()
	 use data from CLzmaDec::dic and update CLzmaDec::dicPos
       }
     }
     LzmaDec_Free()
*/

/* LzmaDec_DecodeToDic

   The decoding to internal dictionary buffer (CLzmaDec::dic).
   You must manually update CLzmaDec::dicPos, if it reaches CLzmaDec::dicBufSize !!!

finishMode:
  It has meaning only if the decoding reaches output limit (dicLimit).
  LZMA_FINISH_ANY - Decode just dicLimit bytes.
  LZMA_FINISH_END - Stream must be finished after dicLimit.

Returns:
  SZ_OK
    status:
      LZMA_STATUS_FINISHED_WITH_MARK
      LZMA_STATUS_NOT_FINISHED
      LZMA_STATUS_NEEDS_MORE_INPUT
      LZMA_STATUS_MAYBE_FINISHED_WITHOUT_MARK
  SZ_ERROR_DATA - Data error
  SZ_ERROR_FAIL - Some unexpected error: internal error of code, memory corruption or hardware
failure
*/

SRes LzmaDec_DecodeToDic(CLzmaDec *p, SizeT dicLimit, const Byte *src, SizeT *srcLen,
			 ELzmaFinishMode finishMode, ELzmaStatus *status);

/* ---------- Buffer Interface ---------- */

/* It's zlib-like interface.
   See LzmaDec_DecodeToDic description for information about STEPS and return results,
   but you must use LzmaDec_DecodeToBuf instead of LzmaDec_DecodeToDic and you don't need
   to work with CLzmaDec variables manually.

finishMode:
  It has meaning only if the decoding reaches output limit (*destLen).
  LZMA_FINISH_ANY - Decode just destLen bytes.
  LZMA_FINISH_END - Stream must be finished after (*destLen).
*/

SRes LzmaDec_DecodeToBuf(CLzmaDec *p, Byte *dest, SizeT *destLen, const Byte *src, SizeT *srcLen,
			 ELzmaFinishMode finishMode, ELzmaStatus *status);

/* ---------- One Call Interface ---------- */

/* LzmaDecode

finishMode:
  It has meaning only if the decoding reaches output limit (*destLen).
  LZMA_FINISH_ANY - Decode just destLen bytes.
  LZMA_FINISH_END - Stream must be finished after (*destLen).

Returns:
  SZ_OK
    status:
      LZMA_STATUS_FINISHED_WITH_MARK
      LZMA_STATUS_NOT_FINISHED
      LZMA_STATUS_MAYBE_FINISHED_WITHOUT_MARK
  SZ_ERROR_DATA - Data error
  SZ_ERROR_MEM  - Memory allocation error
  SZ_ERROR_UNSUPPORTED - Unsupported properties
  SZ_ERROR_INPUT_EOF - It needs more bytes in input buffer (src).
  SZ_ERROR_FAIL - Some unexpected error: internal error of code, memory corruption or hardware
failure
*/

SRes LzmaDecode(Byte *dest, SizeT *destLen, const Byte *src, SizeT *srcLen, const Byte *propData,
		unsigned propSize, ELzmaFinishMode finishMode, ELzmaStatus *status,
		ISzAllocPtr alloc);


#ifdef CONFIG_NRF_COMPRESS_EXTERNAL_DICTIONARY
/* Interfaces to be implemented by the user of this library.
   They are added to allow for different memory storage of dictionary,
   not only as directly referenced memory through CLzmaDec::dic.
*/

/* Open dictionary with requested size.

minSize:
  Minimal size of the dictionary.
Returns:
  Pointer to a dictionary handle struct for subsequent API calls
  or NULL if failed to open.
*/
DictHandle *LzmaDictionaryOpen(SizeT minSize);

/* Write to dictionary.

handle:
  Pointer to dictionary handle struct, returned from LzmaDictionaryOpen().
pos:
  Position (byte-wise) in dictionary to start writing to.
data
  Data to be written.
len
  Length of @a data.
Returns:
  Number of written bytes.
*/
SizeT LzmaDictionaryWrite(DictHandle *handle, SizeT pos, const Byte *data, SizeT len);

/* Read from dictionary.

handle:
  Pointer to dictionary handle struct, returned from LzmaDictionaryOpen().
pos:
  Position (byte-wise) in dictionary to start reading from.
data
  Data buffer to fill in with read data.
len
  Length of @a data; number of bytes to be read.
Returns:
  Number of read bytes.
*/
SizeT LzmaDictionaryRead(DictHandle *handle, SizeT pos, Byte *data, SizeT len);

/* Close dictionary.

handle:
  Pointer to dictionary handle struct, returned from LzmaDictionaryOpen().

Returns:
  SZ_OK
  SZ_ERROR_PARAM - Invalid input parameters
  SZ_ERROR_FAIL  - Some unexpected error: internal error of code, memory corruption or hardware
 */
SRes LzmaDictionaryClose(DictHandle *handle);
#endif

EXTERN_C_END

#endif
