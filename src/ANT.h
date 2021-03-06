//
// Copyright (c) 2014 Jon Escombe <jone@dresco.co.uk>
//
// This program is free software; you can redistribute it and/or modify it
// under the terms of the GNU General Public License as published by the Free
// Software Foundation; either version 2 of the License, or (at your option)
// any later version.
//
// This program is distributed in the hope that it will be useful, but WITHOUT
// ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
// FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for
// more details.
//
// You should have received a copy of the GNU General Public License along
// with this program; if not, write to the Free Software Foundation, Inc., 51
// Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA
//

// ANT Protocol Definitions
#include "ANTNetworkKey.h"

#define ANT_SYNC_BYTE           0xA4
#define ANT_MAX_MESSAGE_SIZE    12
#define ANT_SYSTEM_RESET        0x4A
#define ANT_SET_NETWORK         0x46
#define ANT_ASSIGN_CHANNEL      0x42
#define ANT_CHANNEL_ID          0x51
#define ANT_CHANNEL_PERIOD      0x43
#define ANT_CHANNEL_FREQUENCY   0x45

#define ANT_OPEN_CHANNEL        0x4B
#define ANT_CLOSE_CHANNEL       0x4C
#define ANT_CHANNEL_1           1
#define ANT_NETWORK_0           0
#define ANT_CHANNEL_TYPE_TX     0x10
#define ANT_SPORT_HR_TYPE       0x78
#define ANT_TRANSMISSION_TYPE   1
#define ANT_SPORT_FREQUENCY     57
#define ANT_SPORT_HR_PERIOD     8070
#define ANT_BROADCAST_DATA      0x4E
#define ANT_STANDARD_POWER      0x10
#define ANT_SPORT_POWER_PERIOD  8182
#define ANT_SPORT_POWER_TYPE    11

