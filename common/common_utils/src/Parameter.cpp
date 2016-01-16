/**
 * \file        common_utils/Parameter.cpp
 *
 * \brief
 *
 * \author      Coelen Vincent (vincent.coelen@polytech-lille.net)
 * \date        2015-12-23
 * \copyright   PyroTeam, Polytech-Lille
 * \license
 * \version
 */

#include "common_utils/Parameter.h"
#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose2D.h>

namespace common_utils {


/**
 * fonction qui execute un processus et renvoie le resultat de la sortie standard
 *
 * \param cmd une chaine de caractère contenant la ligne de commande à executer
 * \return une chaine de charactère contenant le résultat de la ligne de commande reçu sur la sortie standard
 */
std::string execProcess(std::string cmd)
{
    std::shared_ptr<FILE> pipe(popen(cmd.c_str(), "r"), pclose);
    if (!pipe)
    {
        return "ERROR";
    }
    char buffer[128];
    std::string result = "";
    while (!feof(pipe.get()))
    {
        if (fgets(buffer, 128, pipe.get()) != NULL)
        {
            result += buffer;
        }
    }
    return result;
}

/**
 * fonction qui récupère un paramètre sur le serveur de paramètre de ros
 *
 * \param nh un nodehandle pour l'accès au serveur de paramètre
 * \param paramName une chaine de charactère contenant le nom du paramètre à obtenir sous forma d'un chemin (ne pas mettre de "/" à la fin du nom)
 * \param p la valeur obtenue de type geometry_msg::Point
 *
 */
int getParameter(ros::NodeHandle &nh, const std::string &paramName, geometry_msgs::Point &p)
{
    std::vector<double> param_vec;
    std::vector<double> defaultParam_vec;
    nh.param<std::vector<double>>(paramName, param_vec, defaultParam_vec);
    if (param_vec.size()<3)
    {
        ROS_ERROR("Loading Map parameters : Wrong Size");
        return -1;
    }
    p.x = param_vec[0];
    p.y = param_vec[1];
    p.z = param_vec[2];
    return 0;
}

/**
 * fonction qui récupère un paramètre sur le serveur de paramètre de ros
 *
 * \param nh un nodehandle pour l'accès au serveur de paramètre
 * \param paramName une chaine de charactère contenant le nom du paramètre à obtenir sous forma d'un chemin (ne pas mettre de "/" à la fin du nom)
 * \param p la valeur obtenue de type geometry_msg::Pose2D
 *
 */
int getParameter(ros::NodeHandle &nh, const std::string &paramName, geometry_msgs::Pose2D &p)
{
    std::vector<double> param_vec;
    std::vector<double> defaultParam_vec;
    nh.param<std::vector<double>>(paramName, param_vec, defaultParam_vec);
    if (param_vec.size()<3)
    {
        ROS_ERROR("Loading Map parameters : Wrong Size");
        return -1;
    }
    p.x = param_vec[0];
    p.y = param_vec[1];
    p.theta = param_vec[2];
    return 0;
}

/**
 * fonction qui récupère un paramètre sur le serveur de paramètre de ros
 *
 * \param nh un nodehandle pour l'accès au serveur de paramètre
 * \param paramName une chaine de charactère contenant le nom du paramètre à obtenir sous forma d'un chemin (ne pas mettre de "/" à la fin du nom)
 * \param segment la valeur obtenue de type occupancy_grid_utils::LineSegment
 *
 */
int getParameter(ros::NodeHandle &nh, const std::string &paramName, occupancy_grid_utils::LineSegment &segment)
{
    //charge la config d'un segment
    geometry_msgs::Point start;
    if (getParameter(nh, paramName+"/start", start) != 0)
    {
        ROS_ERROR_STREAM("Loading Segment parameters : " << paramName << " start : Wrong Size");
        return -1;
    }

    geometry_msgs::Point end;
    if (getParameter(nh, paramName+"/end", end) !=0)
    {
        ROS_ERROR_STREAM("Loading Segment parameters : " << paramName << " end : Wrong Size");
        return -1;
    }


    occupancy_grid_utils::LineSegment s(start, end);
    s.setName(paramName);
    segment = s;
    return 0;
}

/**
 * fonction qui récupère un paramètre sur le serveur de paramètre de ros
 *
 * \param nh un nodehandle pour l'accès au serveur de paramètre
 * \param paramName une chaine de charactère contenant le nom du paramètre à obtenir sous forma d'un chemin (ne pas mettre de "/" à la fin du nom)
 * \param rectangle la valeur obtenue de type occupancy_grid_utils::Rectangle
 *
 */
int getParameter(ros::NodeHandle &nh, const std::string &paramName, occupancy_grid_utils::Rectangle &rectangle)
{
    geometry_msgs::Pose2D pose;
    if (getParameter(nh, paramName+"/pose", pose) != 0)
    {
        ROS_ERROR_STREAM("Loading rectangle parameters : " << paramName << " pose : Wrong Size");
        return -1;
    }

    geometry_msgs::Point size;
    if (getParameter(nh, paramName+"/size", size) !=0)
    {
        ROS_ERROR_STREAM("Loading rectangle parameters : " << paramName << " size : Wrong Size");
        return -1;
    }
    occupancy_grid_utils::Rectangle r(pose, size);
    r.setName(paramName);
    rectangle = r;
    return 0;
}

/**
 * fonction qui récupère une liste de nom de sous-paramètres contenus dans un paramètre
 *
 * \param nh un nodehandle pour l'accès au serveur de paramètre
 * \param paramName une chaine de charactère contenant le nom du paramètre à obtenir sous forma d'un chemin (ne pas mettre de "/" à la fin du nom)
 * \param paramSet un set contenant les noms des sous-paramètres
 *
 */
void getSetOfParam(ros::NodeHandle &nh, const std::string &paramName, std::set<std::string> &paramSet)
{
    std::string command("rosparam list ");
    std::string rosNameSpace("");
    std::string returnStr = execProcess(command + rosNameSpace + paramName);

    std::stringstream sReturnStr(returnStr);

    //+1 pour tenir compte du '/' de separation après 'field/wall' par exemple
    size_t paramNameSize = rosNameSpace.size() + paramName.size() + 1;

    while (!sReturnStr.eof())
    {
        std::string param;
        std::getline(sReturnStr, param, '\n');
        if (!sReturnStr.eof() && param.size() > paramNameSize)
        {
            std::size_t prefixFound = param.find(rosNameSpace + paramName);
            prefixFound += paramNameSize;
            param = param.substr(prefixFound);
            std::size_t found = param.find("/");
            if (found != std::string::npos)
            {
                param = param.substr(0, found);
                paramSet.insert(param);
            }
        }
    }

}

/**
 * fonction qui récupère un paramètre sur le serveur de paramètre de ros
 *
 * \param nh un nodehandle pour l'accès au serveur de paramètre
 * \param paramName une chaine de charactère contenant le nom du paramètre à obtenir sous forma d'un chemin (ne pas mettre de "/" à la fin du nom)
 * \param listOfSegments la valeur obtenue de type std::list<occupancy_grid_utils::LineSegment>
 *
 */
int getParameter(ros::NodeHandle &nh, const std::string &paramName, std::list<occupancy_grid_utils::LineSegment> &listOfSegments)
{
    std::set<std::string> paramSet;
    getSetOfParam(nh, paramName, paramSet);

    for (auto param : paramSet)
    {
        occupancy_grid_utils::LineSegment segment;
        if (getParameter(nh, paramName+"/"+param, segment) !=0)
        {
            ROS_ERROR("Error Loading segment parameters");
            return -1;
        }
        listOfSegments.push_back(segment);
    }
    return 0;
}

/**
 * fonction qui récupère un paramètre sur le serveur de paramètre de ros
 *
 * \param nh un nodehandle pour l'accès au serveur de paramètre
 * \param paramName une chaine de charactère contenant le nom du paramètre à obtenir sous forma d'un chemin (ne pas mettre de "/" à la fin du nom)
 * \param listOfRectangles la valeur obtenue de type std::list<occupancy_grid_utils::Rectangle>
 *
 */
int getParameter(ros::NodeHandle &nh, const std::string &paramName, std::list<occupancy_grid_utils::Rectangle> &listOfRectangles)
{
    std::set<std::string> paramSet;
    getSetOfParam(nh, paramName, paramSet);

    for (auto param : paramSet)
    {
        occupancy_grid_utils::Rectangle rectangle;
        if (getParameter(nh, paramName+"/"+param, rectangle) != 0)
        {
            ROS_ERROR("Error Loading rectangle parameters");
            return -1;
        }
        listOfRectangles.push_back(rectangle);
    }
    return 0;
}


} // namespace common_utils
