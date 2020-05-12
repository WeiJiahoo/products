#include <string.h>
#include <stdio.h>

#include "imu_data.h"
#include "imu_data_decode.h"


//! @brief Computes the number of elements in an array.
#ifndef ARRAY_SIZE
#define ARRAY_SIZE(x)	(sizeof(x) / sizeof((x)[0]))
#endif


const imu_item_t itmes[] = 
{
    {kItemTest8F,           "kItemTest8F"      },
    {kItemExtend,           "kItemExtend"      },
    {kItemIMU,              "IMUData"          },
    {kItemRFQuat,           "RFQuat"           },
    {kItemRFEul,            "RFEul"            },
    {kItemRFAccCalibrated,  "RFAccCalibrated"  },
    {kItemRFGyrCalibrated,  "RFGyrCalibrated"  },
    {kItemRFMagCalibrated,  "RFMagCalibrated"  },
    {kItemCPUTimeStamp,     "CPUTimeStamp"     },
    {kItemID,               "ID"               },
    {kItemAccRaw,           "AccRaw"           },
    {kItemAccCalibrated,    "AccCalibrated"    },
    {kItemAccFiltered,      "AccFiltered"      },
    {kItemAccLinear,        "AccLinear"        },
    {kItemGyrRaw,           "GyrRaw"           },
    {kItemGyrFiltered,      "GyrFiltered"      },
    {kItemGyrCalibrated,    "GyrCalibrated"    },
    {kItemMagRaw,           "MagRaw"           },
    {kItemMagCalibrated,    "MagCalibrated"    },
    {kItemMagFiltered,      "MagFiltered"      },
    {kItemRFTemperature,    "kItemRFTemperature"},
    {kItemRotationEular,    "EularAngle"       },
    {kItemRotationEular2,   "EularAngleFloat"  },
    {kItemRotationQuat,     "Quaternion"       },
    {kItemTemperature,      "kItemTemperature"  },
    {kItemPressure,         "kItemPressure"     },
    {kItemEnd,              "kItemEnd"          },
};


const char *imu_data_get_name(ItemID_t item)
{
    int i;
    for(i=0; i<ARRAY_SIZE(itmes); i++)
    {
        if(item == itmes[i].item)
        {
            return itmes[i].name;
        }
    }
    return NULL;
}



