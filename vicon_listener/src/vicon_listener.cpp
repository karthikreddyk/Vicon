/*Olalekan O'molu. 
SeRViCe Lab, 
Nov. 11, 2015*/

#include "ros/ros.h"
#include "std_msgs/String.h"

#include <string>
#include <fstream>

#include <vicon_bridge/Markers.h>
#include <vicon_listener/structs.h>

unsigned short port = 801;
ros::NodeHandle n_rpy;
ros::NodeHandle n_xyz;

class Receiver
{
public:
    enum Mode
    {
        SAVE
    };

private:
    Mode mode;

public:
    Receiver()
        :  save(false)
    {  
    }

    void callback(const vicon_bridge::Markers::ConstPtr& markers_msg)
    {   
        std::cout << "\nConstructor Called"  << std::endl;
        std::cout << "Header stamp: "<< markers_msg -> header.stamp <<  "   | Frame Number: " <<
                       markers_msg -> frame_number << std::endl;
    
        //Retrieve geometry_msgs translation for four markers on superchicko    
        forehead            = markers_msg -> markers[0].translation;
        leftcheek           = markers_msg -> markers[1].translation;
        rightcheek          = markers_msg -> markers[2].translation;
        chin                = markers_msg -> markers[3].translation;

        foreheadname        = markers_msg -> markers[0].marker_name;
        leftcheekname       = markers_msg -> markers[1].marker_name;
        rightcheekname      = markers_msg -> markers[2].marker_name;
        chinname            = markers_msg -> markers[3].marker_name;

        //Print a bunch'o'stuff to assert correctness of the above
        std::cout << "\n" << std::endl;
        std::cout << foreheadname   <<  ": "    << forehead     << std::endl;
        std::cout << leftcheekname  <<  ": "    << leftcheek    << std::endl;
        std::cout << rightcheekname <<  ": "    << rightcheek   << std::endl;
        std::cout << chinname       <<  ": "    << chin         << std::endl;

        headmarkers markers = {forehead, leftcheek, rightcheek, chin};
        facemidpts midFacePoints_       = midpoint(markers);
        }

    //Here, I basically find the midpoint of the four pts intersecting at the middle of the face
    facemidpts midpoint(headmarkers markers)                     
    {
        xm = 0.25 * (markers.foreo.x + markers.lefto.x + markers.righto.x + markers.chino.x);
        ym = 0.25 * (markers.foreo.y + markers.lefto.y + markers.righto.y + markers.chino.y);
        zm = 0.25 * (markers.foreo.z + markers.lefto.z + markers.righto.z + markers.chino.z); 

        if (save)
          {
            savepoints(xm, ym, zm);
          }        

        facepoints = {xm, ym, zm};

        std::cout << "\nMidface(x, y, z): (" << facepoints.x << ", " << facepoints.y << ", " << facepoints.z << ")" << std::endl;

        ros::Publisher trans_pub = n_xyz.advertise<std_msgs::String>("trans", 1000);
        ros::Rate  loop_rate(10);
        while(ros::ok())
        {
            std_msgs::String msg;

            std::stringstream ss;
            // ss << std::fixed << std::setfill('0') << std::setprecision(5) << xm << ", " << ym << ", " << zm;
            msg.data = ss.str();

            ROS_INFO("%s", msg.data.c_str());

            trans_pub.publish(msg);

            ros::spinOnce();                //to enable callbacks getting called
            loop_rate.sleep();
        }

        modgramschmidt(markers);                    //compute stable orthogonalization and normalization

        return facepoints;
    }
    
//   Now I compute the Gram-Schmidt orthonormalization and orthogonalization for the four vectors
//     https://en.m.wikipedia.org/wiki/Gram%E2%80%93Schmidt_process#CITEREFGolubVan_Loan1996
//     http://www.cs.cmu.edu/~kiranb/animation/p245-shoemake.pdf
    void modgramschmidt(headmarkers markers)
    {
        //define req'd sys of finitely independent set
        Vector3d v1(markers.foreo.x, markers.foreo.y, markers.foreo.z);
        Vector3d v2(markers.lefto.x, markers.lefto.y, markers.lefto.z);
        Vector3d v3(markers.righto.x, markers.righto.y, markers.righto.z);
        Vector3d v4(markers.chino.x, markers.chino.y, markers.chino.z);    

        Vector3d u1, u2, u3, u4, u0;                            //define orthogonal set S' = {u1, u2, u3, u4}
        Vector3d u31, u32, u41, u42, u43;                       //define intermediate orthogonal that avoids round-off errors

        u1 = v1 ;                                               //we set u1 to v1
        u2 = v2     - proj(u1, v2);

        u31 = v3    - proj(u1, v3);
        u32 = u31   - proj(u2, u31);
        u3 = u32;

        u41 = v4    - proj(u1, v4);
        u42 = u41   - proj(u2, u41);
        u43 = u42   - proj(u3, u42);
        u4 = u43;

        //orthogonality check: dot products among vectors should be null
        float a1, a2, a3, a4;
        a1 = u1.dot(u2); a2 = u2.dot(u3); a3 = u3.dot(u4); a4 = u2.dot(u4);
        if( !((a1 < 0.01) || (a2 < 0.01) || (a3 < 0.01) || (a4 < 0.01)) )
        {
            std::cout << "u1.u2: " << a1 << " | u2.u3: " << a2 << "   | u3.u4: " << a3 << "    | u2.u4: " << a4 <<std::endl;            
        }

        Vector3d e1, e2, e3, e4;                                //define orthonormal set S' = {e1, e2, e3, e4}
        e1 = u1 / u1.norm();
        e2 = u2 / u2.norm();
        e3 = u3 / u3.norm();
        e4 = u4 / u4.norm();

        //orthonormality check:: ||e1||, ||e2||, ||e3||, ||e4|| should be 1
        if( !(e1.norm() == 1 || e2.norm() || e3.norm() || e4.norm()) )
        {            
            std::cout <<"||e1|| : " << e1.norm() << " ||e2||: " << e2.norm() << "  ||e3||: "  << e3.norm() <<\
                        "  ||e4||: " << e4.norm() << std::endl;
        }
        orth gonal  = {u1, u2, u3, u4};
        orth normal = {e1, e2, e3, e4};

        rot(normal);                                //compute rotation matrix
    }

    Vector3d proj(Vector3d u, Vector3d v)                       //computes the projection of vector v onto u.
    {
        float ratio = v.dot(u) / u.dot(u);
        Vector3d projuv = ratio * u;

        return projuv;
    }

    void savepoints(float xm, float ym, float zm)
    {
        //Now we write the points to a text file for visualization processing
        std::ofstream midface;
        midface.open("midface.csv", std::ofstream::out | std::ofstream::app);
        midface << xm <<"\t" <<ym << "\t" << zm << "\n";
        midface.close();
    }

    MatrixXd rot(orth normal)
    {
        Vector3d col1, col2, col3, col4;
        col1 = normal.e1;                       //each of these are 3 X 1 in dim
        col2 = normal.e2;
        col3 = normal.e3;
        col4 = normal.e4;

        // std::cout << "col1: " << col1 << "\n col1.size()" << col1.rows() << ", " << col1.cols() << std::endl;

        MatrixXd E(3, 4);
        E.col(0) = col1;
        E.col(1) = col2;
        E.col(2) = col3;
        E.col(3) = col4;

        std::cout << "E Matrix: \n" << E << std::endl;

        MatrixXd R(3,3);
        R.col(0)   = -col1;
        R.col(1)   = col2;
        R.col(2)   = col3;

        MatrixXd Rt = R.transpose().eval();

        MatrixXd I(3,3);
        I = R * Rt;

        std::cout << "I: \n" << I << std::endl;

        float det = R.determinant();

        if(!det == 1)                           //check if R is unitary
        {
            std::cout << "R is not unitary." << "\t" << "|R|: " << det << std::endl;
        }

        rollpy(R);                  //computes roll-pitch-yaw motion
        return R;
    }

    //From Rotation Matrix, find rpy
    Vector3f rollpy(MatrixXd R)
    {
        float sp, cp;
        MatrixXf Rf = R.cast<float>();

        ros::Publisher rot_pub = n_rpy.advertise<std_msgs::String>("rot_" , 1000);
        ros::Rate loop_rate2(10);                       //publish at 10Hz
        std_msgs::String rotmsg;
        std::stringstream os;
        
        if (abs(R(0,0)) < .001 & abs(R(1,0)) < .001)
        {
            // singularity
            rpy(0) = 0;
            rpy(1) = atan2(-Rf(2,0), Rf(0,0));
            rpy(2) = atan2(-Rf(1,2), Rf(1,1));
            std::cout << "rpy: " << rpy.transpose() << std::endl;

           while(ros::ok())
            {
                os << std::fixed << std::setfill('0') << std::setprecision(5) << rpy(0) << ", " << rpy(1) << ", " << rpy(2);
                rotmsg.data = os.str();

                ROS_INFO("%s", rotmsg.data.c_str());

                rot_pub.publish(rotmsg);

                ros::spinOnce();

                loop_rate2.sleep();                     //wait while messages arrive
            }
        }
        else
        {            
            rpy(0) = atan2(Rf(1,0), Rf(0,0));
            sp = sin(rpy(0));
            cp = cos(rpy(0));
            rpy(1) = atan2(-Rf(2,0), cp * Rf(0,0) + sp * Rf(1,0));
            rpy(2) = atan2(sp * Rf(0,2) - cp * Rf(1,2), cp*Rf(1,1) - sp*Rf(0,1));
            std::cout << "rpy: " << rpy.transpose() << std::endl;

            while(ros::ok())
             {
                 os << std::fixed << std::setfill('0') << std::setprecision(5) << rpy(0) << ", " << rpy(1) << ", " << rpy(2);
                 rotmsg.data = os.str();

                 ROS_INFO("%s", rotmsg.data.c_str());

                 rot_pub.publish(rotmsg);

                 ros::spinOnce();

                 loop_rate2.sleep();                     //wait while messages arrive
             }
        }

        return rpy;
    }



    ~Receiver()
    {
        ROS_INFO("\nDestructor Called.");
    }


    
protected:
    float xm, ym, zm;
    geometry_msgs::Point_<std::allocator<void> > chin, forehead, leftcheek, rightcheek;
    std::string foreheadname, leftcheekname, rightcheekname, chinname;
    unsigned short port; bool save; 
    facemidpts facepoints;
    Vector3f rpy;

// friend server;               //somehow I could not get g++ to compile by exposing everything within  Receiver to server
};

void help()
{
  
    std::cout<< "Error ******************************************************************\n";
    std::cout <<  std::setw(20)  << "\nUsage: rosrun vicon_listener vicon_listener" << std::endl;
    std::cout <<  std::setw(20) << "To Save facepoints: rosrun vicon_listener vicon_listener -s (or --save)\n" << std::endl;
}


int main(int argc, char **argv)
{

    ros::init(argc, argv, "listener");

    ros::NodeHandle nm;
    Receiver::Mode mode = Receiver::SAVE;
    Receiver obj;
    ros::Subscriber sub = nm.subscribe("vicon/markers", 1000, &Receiver::callback, &obj ); 
    
       try
        {

            if(argc != (1 || 2))
                {
                  help();
                  ros::shutdown();
                  return 1;
                }

            else if (argc = 1)
            {   
              ros::spin();    
              return 1;
            }

            else if (argc = 2)
            {
                std::string param(argv[2]);
                if(param == "-s" || "--save")
                {
                    mode = Receiver::SAVE;
                }

                ros::spin();    
                return 1;
            }
        }

        catch (std::exception& e)
          {
            std::cerr << "Exception: " << e.what() << "\n";
          }
}
    
    
    
    