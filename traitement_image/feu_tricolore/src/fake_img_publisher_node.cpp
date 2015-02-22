/*==========  Includes  ==========*/
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <pthread.h>
#include <string>

/*==========  Enum  ==========*/
enum signal {s000, s001, s010, s011, s100, s101, s110, s111};

/*==========  Prototypes  ==========*/
void *thread_entree_utilisateur(void* unused);

/*==========  Global variables  ==========*/
int choice = 0;

/*==========  Main  ==========*/
int main(int argc, char** argv)
{
// Environment
    // First diplay
    std::cout << "Welcome on the \" fake publisher \"" << std::endl;
    std::cout << "Choose a signal to simulate on \\image_draw. Use a number (green-orange-red : 0 OFF, 1 ON, 2 BLINK )" << std::endl;
    std::cout << "For example : 210 for green blinking, orange on and red off ...\n" << std::endl;
    
    // Thread
    pthread_t thread_entree_utilisateur_id;
    pthread_create (&thread_entree_utilisateur_id, NULL, &thread_entree_utilisateur, NULL);

    // ROS
    ros::init(argc, argv, "image_publisher");
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    image_transport::Publisher pub = it.advertise("image_raw", 1);



    cv::VideoCapture cap("/home/leak/Projets/catkin_ws/src/robocup-pkg/traitement_image/feu_tricolore/img/video/Carologistics_Navigation_Challenge_Robot_Perspective.mp4"); // open the default camera
    if(!cap.isOpened())  // check if we succeeded
        return -1;
    int frameRate = cap.get(CV_CAP_PROP_FPS);



    // Images
    #define NB_IMAGE_MAX       40
    std::string path_str = "/home/leak/Projets/catkin_ws/src/robocup-pkg/traitement_image/feu_tricolore/";
    cv_bridge::CvImage cv_im_1[NB_IMAGE_MAX];
    cv_bridge::CvImage cv_im_2[NB_IMAGE_MAX];
    for(int i = 0; i < NB_IMAGE_MAX; i++)
    {
        switch(i)
        {
            case 0:
                cv_im_1[i].image = cv::imread(path_str + "img/screen_robotino/000.jpg", 1) ;
                cv_im_2[i].image = cv::imread(path_str + "img/screen_robotino/000.jpg", 1) ;
            break;

            case 1:
                cv_im_1[i].image = cv::imread(path_str + "img/screen_robotino/001.jpg", 1) ;
                cv_im_2[i].image = cv::imread(path_str + "img/screen_robotino/001.jpg", 1) ;
            break;
            
            case 2:
                cv_im_1[i].image = cv::imread(path_str + "img/screen_robotino/000.jpg", 1) ;
                cv_im_2[i].image = cv::imread(path_str + "img/screen_robotino/001.jpg", 1) ;
            break;
            
            case 3:
                cv_im_1[i].image = cv::imread(path_str + "img/screen_robotino/010.jpg", 1) ;
                cv_im_2[i].image = cv::imread(path_str + "img/screen_robotino/010.jpg", 1) ;
            break;
            
            case 4:
                cv_im_1[i].image = cv::imread(path_str + "img/screen_robotino/011.jpg", 1) ;
                cv_im_2[i].image = cv::imread(path_str + "img/screen_robotino/011.jpg", 1) ;
            break;
            
            case 5:
                cv_im_1[i].image = cv::imread(path_str + "img/screen_robotino/010.jpg", 1) ;
                cv_im_2[i].image = cv::imread(path_str + "img/screen_robotino/011.jpg", 1) ;
            break;
            
            case 6:
                cv_im_1[i].image = cv::imread(path_str + "img/screen_robotino/000.jpg", 1) ;
                cv_im_2[i].image = cv::imread(path_str + "img/screen_robotino/010.jpg", 1) ;
            break;
            
            case 7:
                cv_im_1[i].image = cv::imread(path_str + "img/screen_robotino/001.jpg", 1) ;
                cv_im_2[i].image = cv::imread(path_str + "img/screen_robotino/011.jpg", 1) ;
            break;
            
            case 8:
                cv_im_1[i].image = cv::imread(path_str + "img/screen_robotino/000.jpg", 1) ;
                cv_im_2[i].image = cv::imread(path_str + "img/screen_robotino/011.jpg", 1) ;
            break;
            
            case 9:
                cv_im_1[i].image = cv::imread(path_str + "img/screen_robotino/100.jpg", 1) ;
                cv_im_2[i].image = cv::imread(path_str + "img/screen_robotino/100.jpg", 1) ;
            break;
            
            case 10:
                cv_im_1[i].image = cv::imread(path_str + "img/screen_robotino/101.jpg", 1) ;
                cv_im_2[i].image = cv::imread(path_str + "img/screen_robotino/101.jpg", 1) ;
            break;
            
            case 11:
                cv_im_1[i].image = cv::imread(path_str + "img/screen_robotino/100.jpg", 1) ;
                cv_im_2[i].image = cv::imread(path_str + "img/screen_robotino/101.jpg", 1) ;
            break;
            
            case 12:
                cv_im_1[i].image = cv::imread(path_str + "img/screen_robotino/110.jpg", 1) ;
                cv_im_2[i].image = cv::imread(path_str + "img/screen_robotino/110.jpg", 1) ;
            break;
            
            case 13:
                cv_im_1[i].image = cv::imread(path_str + "img/screen_robotino/111.jpg", 1) ;
                cv_im_2[i].image = cv::imread(path_str + "img/screen_robotino/111.jpg", 1) ;
            break;
            
            case 14:
                cv_im_1[i].image = cv::imread(path_str + "img/screen_robotino/110.jpg", 1) ;
                cv_im_2[i].image = cv::imread(path_str + "img/screen_robotino/111.jpg", 1) ;
            break;
            
            case 15:
                cv_im_1[i].image = cv::imread(path_str + "img/screen_robotino/100.jpg", 1) ;
                cv_im_2[i].image = cv::imread(path_str + "img/screen_robotino/110.jpg", 1) ;
            break;
            
            case 16:
                cv_im_1[i].image = cv::imread(path_str + "img/screen_robotino/101.jpg", 1) ;
                cv_im_2[i].image = cv::imread(path_str + "img/screen_robotino/111.jpg", 1) ;
            break;
            
            case 17:
                cv_im_1[i].image = cv::imread(path_str + "img/screen_robotino/100.jpg", 1) ;
                cv_im_2[i].image = cv::imread(path_str + "img/screen_robotino/111.jpg", 1) ;
            break;
            
            case 18:
                cv_im_1[i].image = cv::imread(path_str + "img/screen_robotino/000.jpg", 1) ;
                cv_im_2[i].image = cv::imread(path_str + "img/screen_robotino/100.jpg", 1) ;
            break;
            
            case 19:
                cv_im_1[i].image = cv::imread(path_str + "img/screen_robotino/001.jpg", 1) ;
                cv_im_2[i].image = cv::imread(path_str + "img/screen_robotino/101.jpg", 1) ;
            break;
            
            case 20:
                cv_im_1[i].image = cv::imread(path_str + "img/screen_robotino/000.jpg", 1) ;
                cv_im_2[i].image = cv::imread(path_str + "img/screen_robotino/101.jpg", 1) ;
            break;
            
            case 21:
                cv_im_1[i].image = cv::imread(path_str + "img/screen_robotino/010.jpg", 1) ;
                cv_im_2[i].image = cv::imread(path_str + "img/screen_robotino/110.jpg", 1) ;
            break;
            
            case 22:
                cv_im_1[i].image = cv::imread(path_str + "img/screen_robotino/011.jpg", 1) ;
                cv_im_2[i].image = cv::imread(path_str + "img/screen_robotino/111.jpg", 1) ;
            break;
            
            case 23:
                cv_im_1[i].image = cv::imread(path_str + "img/screen_robotino/010.jpg", 1) ;
                cv_im_2[i].image = cv::imread(path_str + "img/screen_robotino/111.jpg", 1) ;
            break;
            
            case 24:
                cv_im_1[i].image = cv::imread(path_str + "img/screen_robotino/000.jpg", 1) ;
                cv_im_2[i].image = cv::imread(path_str + "img/screen_robotino/110.jpg", 1) ;
            break;
            
            case 25:
                cv_im_1[i].image = cv::imread(path_str + "img/screen_robotino/001.jpg", 1) ;
                cv_im_2[i].image = cv::imread(path_str + "img/screen_robotino/111.jpg", 1) ;
            break;
            
            case 26:
                cv_im_1[i].image = cv::imread(path_str + "img/screen_robotino/000.jpg", 1) ;
                cv_im_2[i].image = cv::imread(path_str + "img/screen_robotino/111.jpg", 1) ;
            break;

            case 30:
                cv_im_1[i].image = cv::imread(path_str + "img/joao_pessoa/frame0000.jpg", 1) ;
                cv_im_2[i].image = cv::imread(path_str + "img/joao_pessoa/frame0000.jpg", 1) ;
            break;
            
            case 31:
                cv_im_1[i].image = cv::imread(path_str + "img/joao_pessoa/frame0001.jpg", 1) ;
                cv_im_2[i].image = cv::imread(path_str + "img/joao_pessoa/frame0001.jpg", 1) ;
            break;
            
            case 32:
                cv_im_1[i].image = cv::imread(path_str + "img/joao_pessoa/frame0002.jpg", 1) ;
                cv_im_2[i].image = cv::imread(path_str + "img/joao_pessoa/frame0002.jpg", 1) ;
            break;
            
            case 33:
                cv_im_1[i].image = cv::imread(path_str + "img/joao_pessoa/frame0003.jpg", 1) ;
                cv_im_2[i].image = cv::imread(path_str + "img/joao_pessoa/frame0003.jpg", 1) ;
            break;
            
            case 34:
                cv_im_1[i].image = cv::imread(path_str + "img/joao_pessoa/frame0004.jpg", 1) ;
                cv_im_2[i].image = cv::imread(path_str + "img/joao_pessoa/frame0004.jpg", 1) ;
            break;
            
            case 35:
                cv_im_1[i].image = cv::imread(path_str + "img/joao_pessoa/frame0005.jpg", 1) ;
                cv_im_2[i].image = cv::imread(path_str + "img/joao_pessoa/frame0005.jpg", 1) ;
            break;
            
            case 36:
                cv_im_1[i].image = cv::imread(path_str + "img/joao_pessoa/frame0006.jpg", 1) ;
                cv_im_2[i].image = cv::imread(path_str + "img/joao_pessoa/frame0006.jpg", 1) ;
            break;
            
            case 37:
                cv_im_1[i].image = cv::imread(path_str + "img/joao_pessoa/frame0007.jpg", 1) ;
                cv_im_2[i].image = cv::imread(path_str + "img/joao_pessoa/frame0007.jpg", 1) ;
            break;
            
            case 38:
                cv_im_1[i].image = cv::imread(path_str + "img/joao_pessoa/frame0008.jpg", 1) ;
                cv_im_2[i].image = cv::imread(path_str + "img/joao_pessoa/frame0008.jpg", 1) ;
            break;
            
            case 39:
                cv_im_1[i].image = cv::imread(path_str + "img/ff0000.jpg", 1) ;
                cv_im_1[i].encoding = sensor_msgs::image_encodings::BGR8;    
                cv_im_2[i].image = cv::imread(path_str + "img/ff0000.jpg", 1) ;
                cv_im_2[i].encoding = sensor_msgs::image_encodings::BGR8;        
            break;
        }
        if(i != 39)
        {
            cv_im_1[i].encoding = sensor_msgs::image_encodings::RGB8;        
            cv_im_2[i].encoding = sensor_msgs::image_encodings::RGB8;
        }  
    }

    // Algo
    sensor_msgs::ImagePtr msg;
    // ros::Rate std_rate(4);  // Blink at 2hz
    ros::Rate std_rate(24);  // Blink at 2hz
    ros::Rate vid_rate(frameRate); // Vid at 25fps
    ros::Rate *loop_rate = &std_rate;
    char toSend = 1;
    int cpt = 0;

    while (nh.ok()) 
    {
        int prec_rate = 0;

        if (choice < NB_IMAGE_MAX)
        {
            loop_rate = &std_rate;

            cpt++;
            if(cpt<6)
                toSend=1;
            else if(cpt<12)
                toSend=2;
            else if(cpt>=12 || cpt<0)
            {
                toSend=1;
                cpt=0;
            }

            if(toSend == 1)
            {
                msg = cv_im_1[choice].toImageMsg();
            }
            else if(toSend == 2)
            {
                msg = cv_im_2[choice].toImageMsg();
            }     
        }
        else
        {
            loop_rate = &vid_rate;

            cv_bridge::CvImage frame;
            cap >> frame.image;
            frame.encoding = sensor_msgs::image_encodings::BGR8;
            msg = frame.toImageMsg();
            if(cap.get(CV_CAP_PROP_POS_FRAMES) != cap.get(CV_CAP_PROP_FRAME_COUNT))
            {
            }
            else
            {
                cap.set(CV_CAP_PROP_POS_FRAMES,0);
            }
        }

        pub.publish(msg);
        ros::spinOnce();
        loop_rate->sleep();
    }
}


// void *thread_entree_utilisateur(void* unused)
// {
//     int code_rouge=0, code_orange=1, code_vert=2;
//     choice = code_vert*9+code_orange*3+code_rouge;
//     while(1)
//     {
//         std::cout << "Choix actuel : " << (code_vert*100+code_orange*10+code_rouge) << " | nouveau choix -> " << std::endl;    
//         std::cin >> choice;

//         code_rouge = choice %10;
//         code_orange = (choice/10)%10;
//         code_vert = (choice/100)%10;

//         code_rouge = ((code_rouge<0)?0:((code_rouge>2)?2:code_rouge));
//         code_orange = ((code_orange<0)?0:((code_orange>2)?2:code_orange));
//         code_vert = ((code_vert<0)?0:((code_vert>2)?2:code_vert));

//         choice = code_vert*9+code_orange*3+code_rouge;

//     }

//     return 0;
// }

void *thread_entree_utilisateur(void* unused)
{
    int code_rouge=0, code_orange=1, code_vert=2;
    choice = code_vert*9+code_orange*3+code_rouge;
    while(1)
    {
        std::cout << "Choix actuel : " << (code_vert*100+code_orange*10+code_rouge) << " - " << choice << " | nouveau choix -> " << std::endl;    
        std::cin >> choice;

        code_rouge = choice %10;
        code_orange = (choice/10)%10;
        code_vert = (choice/100)%10;

        // code_rouge = ((code_rouge<0)?0:((code_rouge>2)?2:code_rouge));
        // code_orange = ((code_orange<0)?0:((code_orange>2)?2:code_orange));
        // code_vert = ((code_vert<0)?0:((code_vert>2)?2:code_vert));

        if(code_orange != 3 && code_orange != 4)
            choice = code_vert*9+code_orange*3+code_rouge;        
        if(choice > 26 && choice < 30 || choice > NB_IMAGE_MAX)
            choice = 0;      
    }

    return 0;
}