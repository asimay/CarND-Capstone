/*
 *  Copyright (c) 2018, Udacity
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *
 *  * Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 *  * Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 *  * Neither the name of Autoware nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 *  FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 *  DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 *  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 *  OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#ifndef DBW_NODE_CORE_H
#define DBW_NODE_CORE_H

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/TwistStamped.h>
#include <dbw_mkz_msgs/ThrottleCmd.h>
#include <dbw_mkz_msgs/SteeringCmd.h>
#include <dbw_mkz_msgs/BrakeCmd.h>
#include "predicted_control_values.h"
#include "twist_controller.h"

#define LOOP_RATE (50)

namespace DBWNODE_NS {

class DBWNode {
    public:
        DBWNode();
        ~DBWNode();
        void run();

    private:
        ros::NodeHandle nh_;
        ros::NodeHandle private_nh_;

        ros::Publisher throttle_pub_, brake_pub_, steer_pub_;
        ros::Subscriber sub_vel_, sub_cur_vel_, sub_enable_; /*  */

        void initForROS();
        void cbFromTwistCmd(const geometry_msgs::TwistStamped::ConstPtr &msg);
        void cbFromRecvEnable(const std_msgs::Bool::ConstPtr &msg);
        void cbFromCurrentVelocity(const geometry_msgs::TwistStamped::ConstPtr &msg);
        PredictedControlValues getPredictedControlValues();
        void publishControlCmd(const float throttle, const float brake, const float steer);

        double vehicle_mass_, fuel_capacity_, brake_deadband_, decel_limit_, accel_limit_, 
                wheel_radius_, wheel_base_, steer_ratio_, max_lat_accel_, max_steer_angle_;
        bool sys_enable_;
        float throttle_, brake_, steer_;

        geometry_msgs::TwistStamped twist_cmd_;
        geometry_msgs::TwistStamped cur_velocity_;

        Controller controller_;
};
}

#endif
