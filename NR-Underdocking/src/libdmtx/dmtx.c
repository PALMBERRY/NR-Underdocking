/**
 * libdmtx - Data Matrix Encoding/Decoding Library
 * Copyright 2008, 2009 Mike Laughton. All rights reserved.
 *
 * See LICENSE file in the main project directory for full
 * terms of use and distribution.
 *
 * Contact: Mike Laughton <mike@dragonflylogic.com>
 *
 * \file dmtx.c
 * \brief Main libdmtx source file
 */

#include "libdmtx/dmtx.h"
#include "libdmtx/dmtxstatic.h"

// #ifndef CALLBACK_POINT_PLOT
// #define CALLBACK_POINT_PLOT(a,b,c,d)
// #endif
// 
// #ifndef CALLBACK_POINT_XFRM
// #define CALLBACK_POINT_XFRM(a,b,c,d)
// #endif
// 
// #ifndef CALLBACK_MODULE
// #define CALLBACK_MODULE(a,b,c,d,e)
// #endif
// 
// #ifndef CALLBACK_MATRIX
// #define CALLBACK_MATRIX(a)
// #endif
// 
// #ifndef CALLBACK_FINAL
// #define CALLBACK_FINAL(a,b)
// #endif

/**
 * Use #include to merge the individual .c source files into a single combined
 * file during preprocessing. This allows the project to be organized in files
 * of like-functionality while still keeping a clean namespace. Specifically,
 * internal functions can be static without losing the ability to access them
 * "externally" from the other source files in this list.
 */

// #include "dmtxencode.c"
// #include "dmtxencodestream.c"
// #include "dmtxencodescheme.c"
// #include "dmtxencodeoptimize.c"
// #include "dmtxencodeascii.c"
// #include "dmtxencodec40textx12.c"
// #include "dmtxencodeedifact.c"
// #include "dmtxencodebase256.c"
// 
// #include "dmtxdecode.c"
// #include "dmtxdecodescheme.c"
// 
// #include "dmtxmessage.c"
// #include "dmtxregion.c"
// #include "dmtxsymbol.c"
// #include "dmtxplacemod.c"
// #include "dmtxreedsol.c"
// #include "dmtxscangrid.c"
// 
// #include "dmtximage.c"
// #include "dmtxbytelist.c"
// #include "dmtxtime.c"
// #include "dmtxvector2.c"
// #include "dmtxmatrix3.c"

extern char *
dmtxVersion(void)
{
   return DmtxVersion;
}


/*static*/ const int dmtxNeighborNone = 8;
/*static*/ const int dmtxPatternX[] = { -1, 0, 1, 1, 1, 0, -1, -1 };
/*static*/ const int dmtxPatternY[] = { -1, -1, -1, 0, 1, 1, 1, 0 };
/*static*/ const DmtxPointFlow dmtxBlankEdge = { 0, 0, 0, DmtxUndefined, { -1, -1 } };

/*@ +charint @*/

/*static*/ int rHvX[] =
{ 256, 256, 256, 256, 255, 255, 255, 254, 254, 253, 252, 251, 250, 249, 248,
247, 246, 245, 243, 242, 241, 239, 237, 236, 234, 232, 230, 228, 226, 224,
222, 219, 217, 215, 212, 210, 207, 204, 202, 199, 196, 193, 190, 187, 184,
181, 178, 175, 171, 168, 165, 161, 158, 154, 150, 147, 143, 139, 136, 132,
128, 124, 120, 116, 112, 108, 104, 100, 96, 92, 88, 83, 79, 75, 71,
66, 62, 58, 53, 49, 44, 40, 36, 31, 27, 22, 18, 13, 9, 4,
0, -4, -9, -13, -18, -22, -27, -31, -36, -40, -44, -49, -53, -58, -62,
-66, -71, -75, -79, -83, -88, -92, -96, -100, -104, -108, -112, -116, -120, -124,
-128, -132, -136, -139, -143, -147, -150, -154, -158, -161, -165, -168, -171, -175, -178,
-181, -184, -187, -190, -193, -196, -199, -202, -204, -207, -210, -212, -215, -217, -219,
-222, -224, -226, -228, -230, -232, -234, -236, -237, -239, -241, -242, -243, -245, -246,
-247, -248, -249, -250, -251, -252, -253, -254, -254, -255, -255, -255, -256, -256, -256 };

/*static*/ int rHvY[] =
{ 0, 4, 9, 13, 18, 22, 27, 31, 36, 40, 44, 49, 53, 58, 62,
66, 71, 75, 79, 83, 88, 92, 96, 100, 104, 108, 112, 116, 120, 124,
128, 132, 136, 139, 143, 147, 150, 154, 158, 161, 165, 168, 171, 175, 178,
181, 184, 187, 190, 193, 196, 199, 202, 204, 207, 210, 212, 215, 217, 219,
222, 224, 226, 228, 230, 232, 234, 236, 237, 239, 241, 242, 243, 245, 246,
247, 248, 249, 250, 251, 252, 253, 254, 254, 255, 255, 255, 256, 256, 256,
256, 256, 256, 256, 255, 255, 255, 254, 254, 253, 252, 251, 250, 249, 248,
247, 246, 245, 243, 242, 241, 239, 237, 236, 234, 232, 230, 228, 226, 224,
222, 219, 217, 215, 212, 210, 207, 204, 202, 199, 196, 193, 190, 187, 184,
181, 178, 175, 171, 168, 165, 161, 158, 154, 150, 147, 143, 139, 136, 132,
128, 124, 120, 116, 112, 108, 104, 100, 96, 92, 88, 83, 79, 75, 71,
66, 62, 58, 53, 49, 44, 40, 36, 31, 27, 22, 18, 13, 9, 4 };

/*static*/ char *dmtxErrorMessage[] = {
	"Unknown error",
	"Unsupported character",
	"Not on byte boundary",
	"Illegal parameter value",
	"Encountered empty list",
	"Out of bounds",
	"Message too large",
	"Can't compact non-digits",
	"Encountered unexpected scheme",
	"Encountered incomplete value list"
};
