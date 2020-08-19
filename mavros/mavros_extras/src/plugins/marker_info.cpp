#include <mavros/mavros_plugin.h>
 #include <pluginlib/class_list_macros.h>
 #include <iostream>
 #include <std_msgs/Char.h>
 #include <std_msgs/Int32.h>
 #include <geometry_msgs/Point.h>
 #include <mavros_msgs/MarkerTarget.h>

 namespace mavros {
 namespace extra_plugins{

 class markerInfoPlugin : public plugin::PluginBase {
 public:
     markerInfoPlugin() : PluginBase(),
         nh("~marker_Info")

    { };

     void initialize(UAS &uas_)
     {
         PluginBase::initialize(uas_);
         marker_info_sub = nh.subscribe("marker_info_sub", 10, &markerInfoPlugin::markerInfo_cb, this);
     };

     Subscriptions get_subscriptions()
     {
         return {/* RX disabled */ };
     }

 private:
     ros::NodeHandle nh;
     ros::Subscriber marker_info_sub;

    void markerInfo_cb(const mavros_msgs::MarkerTarget::ConstPtr &req)
     {
	mavlink::common::msg::MARKER_INFO msg = {};
	msg.x_rel = req->marker_position.x;
	msg.y_rel = req->marker_position.y;
	msg.z_rel = req->marker_position.z;
	msg.spotted = req->marker_spotted;

         std::cout << "Got x Pos : " << req->marker_position.x <<  std::endl;
         std::cout << "Got y Pos : " << req->marker_position.y <<  std::endl;
         std::cout << "Got z Pos : " << req->marker_position.z <<  std::endl;
	 std::cout << "Got marker_spotted : " << req->marker_spotted <<  std::endl;
	
         UAS_FCU(m_uas)->send_message_ignore_drop(msg);
     }
 };
 }   // namespace extra_plugins
 }   // namespace mavros

PLUGINLIB_EXPORT_CLASS(mavros::extra_plugins::markerInfoPlugin, mavros::plugin::PluginBase)
