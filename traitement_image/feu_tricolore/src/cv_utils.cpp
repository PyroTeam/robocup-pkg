#include "feu_tricolore/cv_utils.h"

/* VARIABLES GLOBALES */
    // Valeurs du seuillage HSV, définie initialement aux valeurs extrêmes
    // Seront changées par l'utilisation de trackbars ou du clique souris
    #define HSV_MIN 0
    #define HSV_MAX 255
    int H_MIN = HSV_MIN;
    int H_MAX = HSV_MAX;
    int S_MIN = HSV_MIN;
    int S_MAX = HSV_MAX;
    int V_MIN = HSV_MIN;
    int V_MAX = HSV_MAX;

    // init varialbes of Color tracked and our tolerance towards it
    int h = 0, s = 0, v = 0, TOLERANCE = 10, TOLERANCE_V = 100; //TOLERANCE_V la tolérance niveau luminosité


    //default capture width and height
    const int FRAME_WIDTH = 640;
    const int FRAME_HEIGHT = 480;
    //max number of objects to be detected in frame
    const int MAX_NUM_OBJECTS=50;
    //minimum and maximum object area
    const int MIN_OBJECT_AREA = 20*20;
    const int MAX_OBJECT_AREA = FRAME_HEIGHT*FRAME_WIDTH/1.5;


    const std::string trackbarWindowName ("Trackbars");


// ------------------------------- UTILS ---------------------------- 
hsv rgb2hsv(rgb in)
{
    hsv         out;
    double      min, max, delta;

    min = std::min(in.r,in.g);
    min = std::min(min,in.b);

    max = std::max(in.r,in.g);
    max = std::max(max,in.b);

    out.v = max;                                // v
    delta = max - min;
    if( max > 0.0 ) {
        out.s = (delta / max);                  // s
    } else {
        // r = g = b = 0                        // s = 0, v is undefined
        out.s = 0.0;
        out.h = NAN;                            // its now undefined
        return out;
    }
    if( in.r >= max )                           // > is bogus, just keeps compilor happy
        out.h = ( in.g - in.b ) / delta;        // between yellow & magenta
    else
    if( in.g >= max )
        out.h = 2.0 + ( in.b - in.r ) / delta;  // between cyan & yellow
    else
        out.h = 4.0 + ( in.r - in.g ) / delta;  // between magenta & cyan

    out.h *= 60.0;                              // degrees

    if( out.h < 0.0 )
        out.h += 360.0;

    return out;
}


rgb hsv2rgb(hsv in)
{
    double      hh, p, q, t, ff;
    long        i;
    rgb         out;

    if(in.s <= 0.0) {       // < is bogus, just shuts up warnings
        out.r = in.v;
        out.g = in.v;
        out.b = in.v;
        return out;
    }
    hh = in.h;
    if(hh >= 360.0) hh = 0.0;
    hh /= 60.0;
    i = (long)hh;
    ff = hh - i;
    p = in.v * (1.0 - in.s);
    q = in.v * (1.0 - (in.s * ff));
    t = in.v * (1.0 - (in.s * (1.0 - ff)));

    switch(i) {
    case 0:
        out.r = in.v;
        out.g = t;
        out.b = p;
        break;
    case 1:
        out.r = q;
        out.g = in.v;
        out.b = p;
        break;
    case 2:
        out.r = p;
        out.g = in.v;
        out.b = t;
        break;

    case 3:
        out.r = p;
        out.g = q;
        out.b = in.v;
        break;
    case 4:
        out.r = t;
        out.g = p;
        out.b = in.v;
        break;
    case 5:
    default:
        out.r = in.v;
        out.g = p;
        out.b = q;
        break;
    }
    return out;     
}


std::string intToString(int number){
    std::stringstream ss;
    ss << number;
    return ss.str();
}


// ------------------------------- FILTRAGE ---------------------------- 
   
    //sort the window using insertion sort
    //insertion sort is best for this sorting
    void insertionSort(int window[]){
        int temp, i , j;
        for(i = 0; i < 9; i++){
            temp = window[i];
            for(j = i-1; j >= 0 && temp < window[j]; j--){
                window[j+1] = window[j];
            }
            window[j+1] = temp;
        }   
    }   

    void medianfilter(cv::Mat imgToFilter){
        //create a sliding window of size 9
        int window[9];
 
        cv::Mat afterMedian = imgToFilter.clone();
        for(int y = 0; y < imgToFilter.rows; y++)
            for(int x = 0; x < imgToFilter.cols; x++)
                afterMedian.at<uchar>(y,x) = 0.0;
 
        for(int y = 1; y < imgToFilter.rows - 1; y++){
            for(int x = 1; x < imgToFilter.cols - 1; x++){
 
                // Pick up window element
 
                window[0] = imgToFilter.at<uchar>(y - 1 ,x - 1);
                window[1] = imgToFilter.at<uchar>(y, x - 1);
                window[2] = imgToFilter.at<uchar>(y + 1, x - 1);

                window[3] = imgToFilter.at<uchar>(y - 1, x);
                window[4] = imgToFilter.at<uchar>(y, x);
                window[5] = imgToFilter.at<uchar>(y + 1, x);

                window[6] = imgToFilter.at<uchar>(y - 1, x + 1);
                window[7] = imgToFilter.at<uchar>(y, x + 1);
                window[8] = imgToFilter.at<uchar>(y + 1, x + 1);

 
                // sort the window to find median
                insertionSort(window);
 
                // assign the median to centered element of the matrix
                afterMedian.at<uchar>(y,x) = window[4];
            }
        }
    }


// ---------------------------- opération de traitement d'image -------------------------------

    /* effectue le traitement de l'image binaire pour obtenir un objet plus fidèle à l'objet réel */
    void morphOps(cv::Mat &mask){ //en paramètre le flux à traiter

        //create structuring element that will be used to "dilate" and "erode" image.
        //the element chosen here is a 3px by 3px ellipse
        cv::Mat erodeElement = getStructuringElement(cv::MORPH_RECT,cv::Size(3,3));
        //dilate with larger element so make sure object is nicely visible
        cv::Mat dilateElement = getStructuringElement(cv::MORPH_RECT,cv::Size(8,8)); //ancien 3.3

        for(int k=0;k<3;k++){
            dilate(mask,mask,dilateElement);
            erode(mask,mask,erodeElement);
        }
    }

    void drawObject(int x, int y,cv::Mat &frame){
        #define B 0
        #define G 0
        #define R 255
        #define COLOR_TARGET cv::Scalar(B,G,R) 
        //use some of the openCV drawing functions to draw crosshairs
        //on your tracked image!
        //added 'if' and 'else' statements to prevent
        //memory errors from writing off the screen (ie. (-25,-25) is not within the window!)

        circle(frame,cv::Point(x,y),20,COLOR_TARGET,2);
        if(y-25>0)
        line(frame,cv::Point(x,y),cv::Point(x,y-25),COLOR_TARGET,2);
        else line(frame,cv::Point(x,y),cv::Point(x,0),COLOR_TARGET,2);
        if(y+25<FRAME_HEIGHT)
        line(frame,cv::Point(x,y),cv::Point(x,y+25),COLOR_TARGET,2);
        else line(frame,cv::Point(x,y),cv::Point(x,FRAME_HEIGHT),COLOR_TARGET,2);
        if(x-25>0)
        line(frame,cv::Point(x,y),cv::Point(x-25,y),COLOR_TARGET,2);
        else line(frame,cv::Point(x,y),cv::Point(0,y),COLOR_TARGET,2);
        if(x+25<FRAME_WIDTH)
        line(frame,cv::Point(x,y),cv::Point(x+25,y),COLOR_TARGET,2);
        else line(frame,cv::Point(x,y),cv::Point(FRAME_WIDTH,y),COLOR_TARGET,2);

        putText(frame,intToString(x)+","+intToString(y),cv::Point(x,y+30),1,1,cv::Scalar(255,0,0),2);

    }

    void drawColorRect(cv::Mat &frame){
        float b,g,r;
        float H,S,V;
        H = (H_MAX + H_MIN)/2;
        S = (S_MAX + S_MIN)/2;
        V = (V_MAX + V_MIN)/2;

        #ifdef DEBUG
            std::cout << "HSV : (" << H << "," << S << "," << V << ")" << std::endl;
        #endif // DEBUG

        H = H * 360 / 255;
        S = S / 255;
        V = V / 255;

        #ifdef DEBUG
            std::cout << "HSV2 : (" << H << "," << S << "," << V << ")" << std::endl;
        #endif // DEBUG

        hsv HSV = {H,S,V};

        #ifdef DEBUG
            std::cout << "HSV3 : (" << HSV.h << "," << HSV.s << "," << HSV.v << ")" << std::endl;
        #endif // DEBUG

        rgb RGB = hsv2rgb(HSV);

        r = RGB.r * 255;
        g = RGB.g * 255;
        b = RGB.b * 255;

        #ifdef DEBUG
            std::cout << "RGB : (" << r << "," << g << "," << b << ")" << std::endl;
        #endif

        rectangle(frame,cv::Point(0,0),cv::Point(20,20),cv::Scalar(b,g,r),CV_FILLED);
    }

    // ---------------------------- COLOR TREATEMENT -------------------------------
    /*
     * Transform the image into a two colored image, one color for the color we want to track, another color for the others colors
     * From this image, we get two datas : the number of pixel detected, and the center of gravity of these pixel
     */
    CvPoint binarisation(cv::Mat & image, int *nbPixels){
        int x, y;
        //CvScalar pixel;
        cv::Mat hsv, binary;
        int sommeX = 0, sommeY = 0;
        *nbPixels = 0;

        #ifdef DEBUG
            std::cout << "DEBUG :Entree binarisation" << std::endl;
        #endif

        // Create the mask &initialize it to white (no color detected)
        binary = cvCreateImage(image.size(), IPL_DEPTH_8U, 1);
        // Create the hsv image
        hsv = image.clone();
        //Conversion from BGR to HSV
        cv::cvtColor(image,hsv,CV_BGR2HSV);
        // We create the mask
        #ifdef DEBUG
            std::cout << "DEBUG : Seuillage " << std::endl;
        #endif
        cv::inRange(hsv,cv::Scalar(H_MIN,S_MIN,V_MIN),cv::Scalar(H_MAX,S_MAX,V_MAX),binary);      
        #ifdef DEBUG
            std::cout << "DEBUG : Sortie seuillage " << std::endl;
        #endif

        #ifdef DEBUG
        std::cout << "binary width " << binary.rows << std::endl;
        std::cout << "binary height " << binary.cols << std::endl;
        #endif

 //SEGFAULT juste après??
        // Show the result of the mask image
        imshow("Flux binarisée", binary);   
        
        #ifdef DEBUG
        std::cout << "DEBUG5.3 " << std::endl;
        imshow("hsv", hsv);
        #endif
    
        //pour récupérer en sortie le résultat des opérations car image est un parametre resultat
        image = binary.clone();

        // If there is no pixel, we return a center outside the image, else we return the center of gravity
        if(*nbPixels > 0)
            return cvPoint((int)(sommeX / (*nbPixels)), (int)(sommeY / (*nbPixels)));
        else
            return cvPoint(-1, -1);
    }






    /*
     * Recupération de la couleur d'un pixel par clique de la souris 
     */
    void getObjectColor( int event, int x, int y, int flags, void *param) {

        // Vars
        CvScalar pixel;
        cv::Mat hsv;
        cv::Mat imgBGR; // XXX ajout recent
        // Pour évaluer le rectangle nous permettant de selectionner une couleur moyenne
        int XPixelMin=0, XPixelMax=0;
        int YPixelMin=0, YPixelMax=0;
        int nbTotPix=0;
        static int XSaved, YSaved;

        // Get the hsv image
            hsv = imgBGR.clone();
            cv::cvtColor(imgBGR, hsv, CV_BGR2HSV);
            nbTotPix = 0; //nombre total de pixel de la région choisie
            uint8_t* pixelPtr = (uint8_t*)hsv.data;

    #ifdef DEBUG
        std::cout << "CallBack Souris" << std::endl;
        if(event == CV_EVENT_LBUTTONDOWN)
        std::cout << "\t DOWN" << std::endl;
        if(event == CV_EVENT_LBUTTONUP)
        std::cout << "\t UP" << std::endl;
    #endif // DEBUG

    switch(event)
       {
        //Recup x et y du pixel: XPixelMin et YPixelMin au clique
        case  CV_EVENT_LBUTTONDOWN:
            pixel = hsv.at<cv::Vec3b>(y, x);
            XSaved = x;
            YSaved = y;
        #ifdef DEBUG
            std::cout << "DEBUG : getObjectColor First Values :" << std::endl;
            std::cout << "\t HSV(" << pixel.val[0] << "," << pixel.val[1] << "," << pixel.val[2] << ")" << std::endl;
        #endif // DEBUG

        break;

        // Au relachement XPixelMax et YPixelMax
        case  CV_EVENT_LBUTTONUP:
            XPixelMin = std::min(XSaved,x);
            XPixelMax = std::max(XSaved,x);
            YPixelMin = std::min(YSaved,y);
            YPixelMax = std::max(YSaved,y);
       
    //sommes des valeurs HSV des pixels pour faire une moyenne
    int hSum = 0, sSum = 0, vSum = 0;

    #ifdef DEBUG
        std::cout << "DEBUG : getObjectColor FOR" << std::endl;
    #endif // DEBUG
    for (int i = XPixelMin; i <= XPixelMax; ++i)
    {
        for (int j = YPixelMin; j <= YPixelMax; ++j)
        {
            // Get the selected pixel
            pixel = hsv.at<cv::Vec3b>(j, i);
            // Change the value of the tracked color with the color of the selected pixel
            hSum = hSum+(int)pixel.val[0];
            sSum = sSum+(int)pixel.val[1];
            vSum = vSum+(int)pixel.val[2]; 
        }
    }
    
    #ifdef DEBUG
        std::cout << "DEBUG : getObjectColor END FOR" << std::endl;
        std::cout << "Ymin : " << YPixelMin << " Ymax : " << YPixelMax << std::endl;
        std::cout << "Xmin : " << XPixelMin << " Xmax : " << XPixelMax << std::endl;
    #endif // DEBUG

    nbTotPix=((YPixelMax-YPixelMin+1)*(XPixelMax-XPixelMin+1));    
    #ifdef DEBUG
        std::cout << "DEBUG : Avant calculs floattans. nbTotPix : " << nbTotPix << std::endl;
    #endif // DEBUG
    // Change the value of the tracked color with the color of the selected pixel area
    h = hSum/nbTotPix;
    s = sSum/nbTotPix;
    v = vSum/nbTotPix;
    #ifdef DEBUG
        std::cout << "DEBUG : Après calculs floattans" << std::endl;
    #endif // DEBUG
    std::cout << "h   " << h<<"hSum   " << hSum << std::endl;
    std::cout << "s   " << s<<"sSum   " << sSum << std::endl;
    std::cout << "v   " << v<<"vSum   " << vSum << std::endl;

    // Reglage des niveaux
    H_MIN = h - TOLERANCE;
    S_MIN = s - TOLERANCE;
    V_MIN = v - TOLERANCE_V;
    H_MAX = h + TOLERANCE;
    S_MAX = s + TOLERANCE;
    V_MAX = v + TOLERANCE_V;

    // Protection pour ne pas sortir des valeurs limites
    H_MIN = ((H_MIN < HSV_MIN)?HSV_MIN:H_MIN);
    S_MIN = ((S_MIN < HSV_MIN)?HSV_MIN:S_MIN);
    V_MIN = ((V_MIN < HSV_MIN)?HSV_MIN:V_MIN);
    H_MAX = ((H_MAX > HSV_MAX)?HSV_MAX:H_MAX);
    S_MAX = ((S_MAX > HSV_MAX)?HSV_MAX:S_MAX);
    V_MAX = ((V_MAX > HSV_MAX)?HSV_MAX:V_MAX);

    break;
    }
    
}















// ANCIENNE VERSION FONCTIONNELLE



    /*
     * Recupération de la couleur d'un pixel par clique de la souris 
     */
    // void getObjectColor(int event, int x, int y, int flags, void *param) {
    //     // cv::Mat & image, 
    //     // Vars
    //     CvScalar pixel;
    //     cv::Mat hsv;


    //     if(event == CV_EVENT_LBUTTONUP) {

/*tests*/
/*
// Get the hsv image
 {  
            hsv = cvCloneImage(img);
            cvCvtColor(img, hsv, CV_BGR2HSV);
            nbTotPix = 0; //nombre total de pixel de la région choisie

    Recup x et y du pixel: XPixelMin et YPixelMin au clique

    au relachement XPixelMax et YPixelMax
    //sommes des valeurs HSV des pixels pour faire une moyenne
    int hSum = 0, sSum = 0, vSum = 0;

    for (int i = XPixelMin; i < XPixelMax; ++i)
    {
        for (int i = YPixelMin; i < YPixelMax; ++i)
        {
            // Get the selected pixel
            pixel = cvGet2D(hsv, y, x);
            // Change the value of the tracked color with the color of the selected pixel
            hSum = hSum+(int)pixel.val[0];
            sSum = sSum+(int)pixel.val[1];
            vSum = vSum+(int)pixel.val[2];
        }
    }
    nbTotPix=(YPixelMax-YPixelMin*XPixelMax-XPixelMin)
            // Change the value of the tracked color with the color of the selected pixel area
            h = hSum/nbTotPix;
            s = sSum/nbTotPix;
            v = vSum/nbTotPix;
}

// Reglage des niveaux
            H_MIN = h - TOLERANCE;
            S_MIN = s - TOLERANCE;
            V_MIN = v - TOLERANCE_V;
            H_MAX = h + TOLERANCE;
            S_MAX = s + TOLERANCE;
            V_MAX = v + TOLERANCE_V;

            // Protection pour ne pas sortir des valeurs limites
            H_MIN = ((H_MIN < 0)?0:H_MIN);
            S_MIN = ((S_MIN < 0)?0:S_MIN);
            V_MIN = ((V_MIN < 0)?0:V_MIN);
            H_MAX = ((H_MAX > 255)?0:H_MAX);
            S_MAX = ((S_MAX > 255)?0:S_MAX);
            V_MAX = ((V_MAX > 255)?0:V_MAX);
        
            // Release the memory of the hsv image
                cvReleaseImage(&hsv);
*/

//fonction fonctionnelle

            // // Get the hsv image
            // hsv = image.clone();
            // //cv::cvtColor(image, hsv, CV_BGR2HSV);
     
            // // Get the selected pixel
            // pixel = cvGet2D(hsv, y, x);
   
            // // Change the value of the tracked color with the color of the selected pixel
            // h = (int)pixel.val[0];
            // s = (int)pixel.val[1];
            // v = (int)pixel.val[2];

            // // Reglage des niveaux
            // H_MIN = h - TOLERANCE;
            // S_MIN = s - TOLERANCE;
            // V_MIN = v - TOLERANCE_V;
            // H_MAX = h + TOLERANCE;
            // S_MAX = s + TOLERANCE;
            // V_MAX = v + TOLERANCE_V;

            // // Protection pour ne pas sortir des valeurs limites
            // H_MIN = ((H_MIN < 0)?0:H_MIN);
            // S_MIN = ((S_MIN < 0)?0:S_MIN);
            // V_MIN = ((V_MIN < 0)?0:V_MIN);
            // H_MAX = ((H_MAX > 255)?0:H_MAX);
            // S_MAX = ((S_MAX > 255)?0:S_MAX);
            // V_MAX = ((V_MAX > 255)?0:V_MAX);
        
            // // Release the memory of the hsv image
            //     cvReleaseImage(&hsv);
    //     }
    // }
    



    /*
     * Traque l'objet voulu en fonction de sa teinte HSV et de la taille choisie (OBJECT_AREA) 
     * et marquage de celui-ci
     */
    void trackFilteredObject(int &x, int &y, cv::Mat threshold, cv::Mat &cameraFeed){

        cv::Mat temp;
        threshold.copyTo(temp);
        //these two vectors needed for output of findContours
        std::vector<std::vector <cv::Point> > contours;
        std::vector<cv::Vec4i> hierarchy;
        std::vector<int> myints;
        //find contours of filtered image using openCV findContours function
        findContours(temp,contours,hierarchy,CV_RETR_CCOMP,CV_CHAIN_APPROX_SIMPLE );
        //use moments method to find our filtered object
        double refArea = 0;
        bool objectFound = false;
        if (hierarchy.size() > 0) {
            int numObjects = hierarchy.size();
            //if number of objects greater than MAX_NUM_OBJECTS we have a noisy filter
            if(numObjects<MAX_NUM_OBJECTS){
                for (int index = 0; index >= 0; index = hierarchy[index][0]) {

                    cv::Moments moment = moments((cv::Mat)contours[index]);
                    double area = moment.m00;

                    //if the area is less than MIN_OBJECT_AREA then it is probably just noise
                    //if the area is the same as the 3/2 of the image size, probably just a bad filter
                    //we only want the object with the largest area so we safe a reference area each
                    //iteration and compare it to the area in the next iteration.
                    if(area>MIN_OBJECT_AREA && area<MAX_OBJECT_AREA && area>refArea){
                        x = moment.m10/area;
                        y = moment.m01/area;
                        objectFound = true;
                        refArea = area;
                    }else objectFound = false;

                }
                //let user know you found an object
                if(objectFound ==true){
                    putText(cameraFeed,"Tracking Object",cv::Point(0,50),2,1,cv::Scalar(255,0,0),2);
                    //draw object location on screen
                    drawObject(x,y,cameraFeed);}

            }else putText(cameraFeed,"TOO MUCH NOISE! ADJUST FILTER",cv::Point(0,50),1,2,cv::Scalar(0,0,255),2);
        }
    }