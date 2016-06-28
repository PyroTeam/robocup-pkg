#ifndef _COMMON_UTILS__TYPES__H_
#define _COMMON_UTILS__TYPES__H_

namespace common_utils {

/**
 * \brief      Team color
 */
enum team_t
{
	CYAN = 0,
	MAGENTA = 1
};

/**
 * \brief      MPS ARTag identifiers
 */
enum mpsARTags_T
{
	C_CS1_IN = 1,
	C_CS1_OUT = 2,
	C_CS2_IN = 17,
	C_CS2_OUT = 18,
	C_RS1_IN = 33,
	C_RS1_OUT = 34,
	C_RS2_IN = 177,
	C_RS2_OUT = 178,
	C_BS_IN = 65,
	C_BS_OUT = 66,
	C_DS_IN = 81,
	C_DS_OUT = 82,

	M_CS1_IN = 97,
	M_CS1_OUT = 98,
	M_CS2_IN = 113,
	M_CS2_OUT = 114,
	M_RS1_IN = 129,
	M_RS1_OUT = 130,
	M_RS2_IN = 145,
	M_RS2_OUT = 146,
	M_BS_IN = 161,
	M_BS_OUT = 162,
	M_DS_IN = 49,
	M_DS_OUT = 50
};

} // namespace common_utils

#endif // _COMMON_UTILS__TYPES__H_
