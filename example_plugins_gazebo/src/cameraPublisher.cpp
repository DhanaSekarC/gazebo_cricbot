    #include "gazebo/gazebo.hh"
    #include "gazebo/plugins/CameraPlugin.hh"
    #include <gazebo/common/common.hh>
    #include <stdio.h>
    #include "ros/ros.h"
    #include "sensor_msgs/Image.h"
    #include "image_transport/image_transport.h"

    // #include <gazebo/transport/transport.hh>
    // #include "gazebo_plugins/gazebo_ros_camera.h"
    // #include <string>
    // #include <gazebo/sensors/Sensor.hh>
    // #include <gazebo/sensors/CameraSensor.hh>
    // #include <gazebo/sensors/SensorTypes.hh>

    namespace gazebo
    {   
      class CameraPublisher : public CameraPlugin
      { 
        public: CameraPublisher() : CameraPlugin(),saveCount(0) 

        {
            std::string name = "CameraPublisher";
            int argc = 0;
            ros::init(argc, NULL, name);

        }

        public: ~CameraPublisher()
        {
            delete this->node;
        }   

        public: void Load(sensors::SensorPtr _parent, sdf::ElementPtr _sdf)
        {

          // Don't forget to load the camera plugin
          CameraPlugin::Load(_parent,_sdf);
            //this->model = _parent;            

            //ROS Nodehandle


            //current_time = this->model->GetWorld()->GetSimTime();

            this->node = new ros::NodeHandle("~");

            //this->it = this->node;
            ros::NodeHandle node_temp = *this->node;
            image_transport::ImageTransport trans(node_temp);
            char topicname[12];
            char fullname[50];
            std::cout<<this->parentSensor->Camera()->Name().c_str()<<std::endl;

            snprintf(fullname, 30, "%s", this->parentSensor->Camera()->Name().c_str()); 

            strncpy(&topicname[0], &fullname[9], 11);
            topicname[11] ='\0';
            std::cout<<topicname<<std::endl;
            //ROS Publisher
            //this->pub = this->node->advertise<sensor_msgs::Image>(topicname, 1000);
            this->pub = trans.advertise(topicname, 1);



        } 

        // Update the controller
        public: void OnNewFrame(const unsigned char *_image, 
            unsigned int _width, unsigned int _height, unsigned int _depth, 
            const std::string &_format)
        {




                sensor_msgs::Image msg;
                //msg.header.stamp = this->model->GetWorld()->GetSimTime();

                msg.is_bigendian = 0;




                if(this->pub.getNumSubscribers() >0)
                {               
                    //for (int i =0; i<(msg.height*msg.width*3);i++)
                    //{
                        //char pixel = this->frame[i];
                        //std::cout<<"pixel "<<i<<" filled: "<<pixel<<std::endl;
                        //msg.data[i] = this->frame[i];
                        //std::cout<<"data is not gay"<<std::endl;
                    //}
                    frame = this->parentSensor->Camera()->ImageData(0);
                msg.height = this->parentSensor->Camera()->ImageHeight();
                msg.width = this->parentSensor->Camera()->ImageWidth();
                msg.step = msg.width*3;
                size_t st0 = msg.step*msg.height;
                msg.data.resize(st0);               
                msg.encoding = "rgb8";
                memcpy(&msg.data[0], frame, st0);

                    this->pub.publish(msg);
                    //std::cout<<"published"<<std::endl;
                }                               


        }         

         // Pointer to the model
    //private: sensor::SensorPtr model;

        private: int saveCount;
        private: const unsigned char* frame;

        // ROS Nodehandle
        private: ros::NodeHandle* node;

        //Image Transport
        //private:image_transport::ImageTransport it;

        //ROS Publisher
        //private: ros::Publisher pub;
        image_transport::Publisher pub;
        common::Time current_time;
      };

      // Register this plugin with the simulator
      GZ_REGISTER_SENSOR_PLUGIN(CameraPublisher)
    }
