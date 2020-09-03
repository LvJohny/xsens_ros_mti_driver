
//  ==> COPYRIGHT (C) 2019 XSENS TECHNOLOGIES OR SUBSIDIARIES WORLDWIDE <==
//  WARNING: COPYRIGHT (C) 2019 XSENS TECHNOLOGIES OR SUBSIDIARIES WORLDWIDE. ALL RIGHTS RESERVED.
//  THIS FILE AND THE SOURCE CODE IT CONTAINS (AND/OR THE BINARY CODE FILES FOUND IN THE SAME
//  FOLDER THAT CONTAINS THIS FILE) AND ALL RELATED SOFTWARE (COLLECTIVELY, "CODE") ARE SUBJECT
//  TO AN END USER LICENSE AGREEMENT ("AGREEMENT") BETWEEN XSENS AS LICENSOR AND THE AUTHORIZED
//  LICENSEE UNDER THE AGREEMENT. THE CODE MUST BE USED SOLELY WITH XSENS PRODUCTS INCORPORATED
//  INTO LICENSEE PRODUCTS IN ACCORDANCE WITH THE AGREEMENT. ANY USE, MODIFICATION, COPYING OR
//  DISTRIBUTION OF THE CODE IS STRICTLY PROHIBITED UNLESS EXPRESSLY AUTHORIZED BY THE AGREEMENT.
//  IF YOU ARE NOT AN AUTHORIZED USER OF THE CODE IN ACCORDANCE WITH THE AGREEMENT, YOU MUST STOP
//  USING OR VIEWING THE CODE NOW, REMOVE ANY COPIES OF THE CODE FROM YOUR COMPUTER AND NOTIFY
//  XSENS IMMEDIATELY BY EMAIL TO INFO@XSENS.COM. ANY COPIES OR DERIVATIVES OF THE CODE (IN WHOLE
//  OR IN PART) IN SOURCE CODE FORM THAT ARE PERMITTED BY THE AGREEMENT MUST RETAIN THE ABOVE
//  COPYRIGHT NOTICE AND THIS PARAGRAPH IN ITS ENTIRETY, AS REQUIRED BY THE AGREEMENT.
//  
//  THIS SOFTWARE CAN CONTAIN OPEN SOURCE COMPONENTS WHICH CAN BE SUBJECT TO 
//  THE FOLLOWING GENERAL PUBLIC LICENSES:
//  ==> Qt GNU LGPL version 3: http://doc.qt.io/qt-5/lgpl.html <==
//  ==> LAPACK BSD License:  http://www.netlib.org/lapack/LICENSE.txt <==
//  ==> StackWalker 3-Clause BSD License: https://github.com/JochenKalmbach/StackWalker/blob/master/LICENSE <==
//  ==> Icon Creative Commons 3.0: https://creativecommons.org/licenses/by/3.0/legalcode <==
//  

#ifndef GNSSSATPUBLISHER_H
#define GNSSSATPUBLISHER_H

#include <iostream>
#include <string>
#include "packetcallback.h"
#include <sensor_msgs/NavSatFix.h>
#include "xsens_mti_driver/NavSatInfo.h"
#include "xsens_mti_driver/SatInfo.h"



struct GnssSatPublisher : public PacketCallback
{
    ros::Publisher pub;

    GnssSatPublisher(ros::NodeHandle &node)
    {
        int pub_queue_size = 5;
        ros::param::get("~publisher_queue_size", pub_queue_size);
        pub = node.advertise<xsens_mti_driver::NavSatInfo>("/gnss_sat", pub_queue_size);
    }

    void operator()(const XsDataPacket &packet, ros::Time timestamp)
    {
        if (packet.containsRawGnssSatInfo())
        {
            xsens_mti_driver::NavSatInfo msg;

            std::string frame_id = DEFAULT_FRAME_ID;
            ros::param::getCached("~frame_id", frame_id);

            msg.header.stamp = timestamp;
            msg.header.frame_id = frame_id;

            XsRawGnssSatInfo gnss = packet.rawGnssSatInfo();
            for(int index = 0; index < gnss.m_numSvs; index++){
                xsens_mti_driver::SatInfo sat_info;
                sat_info.gnssID = (unsigned int)(gnss.m_satInfos[index].m_gnssId);
                sat_info.svID = (unsigned int)(gnss.m_satInfos[index].m_svId);
                sat_info.cn0 = (unsigned int)(gnss.m_satInfos[index].m_cno);
                sat_info.flags = (unsigned int)(gnss.m_satInfos[index].m_flags);
                msg.satinfo.push_back(sat_info);
            }
            pub.publish(msg);

        }
    }
};

#endif
