#ifndef _PYRO_TRAIT_IM__CV_UTILS__HEADER_
#define _PYRO_TRAIT_IM__CV_UTILS__HEADER_

/* Includes */
//StdLib
    #include <iostream>
    #include <stdio.h> 
    #include <sstream>
    #include <string>
    #include <math.h>
    #include <vector>
    #include <algorithm>
// OpenCV
    /* Les deux bibliothèques nécessaires d'opencv :
        - cv.h contient les structures et fonctions de manipulation d'images
        - highgui.h contient les fonctions d'affichage des images
    */
    #include <opencv2/core/core.hpp>
    #include <opencv2/imgproc/imgproc.hpp>
    #include <opencv2/opencv.hpp>
    #include <opencv2/highgui/highgui.hpp>

// Perso

    /* VARIABLES GLOBALES */
    // Valeurs du seuillage HSV, définie initialement aux valeurs extrêmes
    // Seront changées par l'utilisation de trackbars ou du clique souris
    #define HSV_MIN 0
    #define HSV_MAX 255
    extern int H_MIN;
    extern int H_MAX;
    extern int S_MIN;
    extern int S_MAX;
    extern int V_MIN;
    extern int V_MAX;

    // init varialbes of Color tracked and our tolerance towards it
    extern int h , s, v, TOLERANCE, TOLERANCE_V; //TOLERANCE_V la tolérance niveau luminosité


    //default capture width and height
    extern const int FRAME_WIDTH;
    extern const int FRAME_HEIGHT;
    //max number of objects to be detected in frame
    extern const int MAX_NUM_OBJECTS;
    //minimum and maximum object area
    extern const int MIN_OBJECT_AREA;
    extern const int MAX_OBJECT_AREA;


    extern const std::string trackbarWindowName;


/* Prototypes */
// ------------------------------- UTILS ---------------------------- 

    typedef struct {
        double r;       // percent
        double g;       // percent
        double b;       // percent
    } rgb;

    typedef struct {
        double h;       // angle in degrees
        double s;       // percent
        double v;       // percent
    } hsv;

    static hsv rgb2hsv(rgb in);
    static rgb hsv2rgb(hsv in);

    /**
     * @brief Convert int to string
     * 
     * @param number Number to be converted
     * @return Number converted into string format
     */
    std::string intToString(int number);


// ------------------------------- FILTRAGE ---------------------------- 
   
    //sort the window using insertion sort
    //insertion sort is best for this sorting
    /**
     * @brief [brief description]
     * @details [long description]
     * 
     * @param window [description]
     */
    void insertionSort(int window[]);

    /**
     * @brief [brief description]
     * @details [long description]
     * 
     * @param imgToFilter [description]
     */
    void medianfilter(cv::Mat imgToFilter);


// ---------------------------- opération de traitement d'image -------------------------------

    /* effectue le traitement de l'image binaire pour obtenir un objet plus fidèle à l'objet réel */
    /**
     * @brief [brief description]
     * @details [long description]
     * 
     * @param mask [description]
     */
    void morphOps(cv::Mat &mask);

    /**
     * @brief [brief description]
     * @details [long description]
     * 
     * @param x [description]
     * @param y [description]
     * @param frame [description]
     */
    void drawObject(int x, int y,cv::Mat &frame);
    void drawColorRect(cv::Mat &frame);

    // ---------------------------- COLOR TREATEMENT -------------------------------
    /*
     * Transform the image into a two colored image, one color for the color we want to track, another color for the others colors
     * From this image, we get two datas : the number of pixel detected, and the center of gravity of these pixel
     */
    /**
     * @brief [brief description]
     * @details [long description]
     * 
     * @param image [description]
     * @param nbPixels [description]
     * 
     * @return [description]
     */
    CvPoint binarisation(cv::Mat & image, int *nbPixels);

    /*
     * Recupération de la couleur d'un pixel par clique de la souris 
     */
    /**
     * @brief [brief description]
     * @details [long description]
     * 
     * @param  [description]
     * @return [description]
     */
    void getObjectColor(int event, int x, int y, int flags, void *param = NULL);
    
    /*
     * Traque l'objet voulu en fonction de sa teinte HSV et de la taille choisie (OBJECT_AREA) 
     * et marquage de celui-ci
     */
    /**
     * @brief [brief description]
     * @details [long description]
     * 
     * @param x [description]
     * @param y [description]
     * @param threshold [description]
     * @param cameraFeed [description]
     */
    void trackFilteredObject(int &x, int &y, cv::Mat threshold, cv::Mat &cameraFeed);

#endif // _PYRO_TRAIT_IM__CV_UTILS__HEADER_