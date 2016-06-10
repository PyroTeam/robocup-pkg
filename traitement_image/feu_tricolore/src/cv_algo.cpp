/*==========  Inludes  ==========*/
#include <feu_tricolore/cv_algo.h>


/*==========  Utils  ==========*/

cv::Mat singleToMultChannels(cv::Mat binary, int numChannels)
{
	// Add copies of binary img in a tab
	std::vector<cv::Mat> binTab(numChannels);
	for(int i = 0;i<numChannels;++i) { binTab[i] = binary; }

	// Merge them in one img
	cv::Mat imgMultChannels;
	cv::merge(binTab, imgMultChannels);

	return imgMultChannels;
}


cv::Mat calcHist(cv::Mat imgToHist, int channel, bool normalize)
{
	if(channel > imgToHist.channels())
	{
		ROS_ERROR_STREAM("LightDetection::calcHist : Requested channel "<<channel<<" on a "
							<<imgToHist.channels()<<" channel(s) image");
		return cv::Mat();
	}

	// Separate channels
	std::vector<cv::Mat> channels;
	cv::split(imgToHist, channels);

	// Number of bins
	int bins = 256;
	// Initalize histogram arrays
	cv::Mat hist = Mat::zeros(1, bins, CV_32SC1);

	// Calculate the histogram of the image
	for (int i = 0; i < imgToHist.rows; i++)
	{
		for (int j = 0; j < imgToHist.cols; j++)
		{
			uchar val;

			// Ignore black pixels
			if(imgToHist.at<Vec3b>(i,j)[0] == 0
				&& imgToHist.at<Vec3b>(i,j)[1] == 0
				&& imgToHist.at<Vec3b>(i,j)[2] == 0)
				continue;
			else
				val = imgToHist.at<Vec3b>(i,j)[channel];
			hist.at<float>(val) += 1;
		}
	}

	// Normalize
	if(normalize)
	{
		// Search max bin value
		int hmax;
		for (int j = 0; j < bins-1; j++)
			hmax = hist.at<float>(j) > hmax ? hist.at<float>(j) : hmax;

		for (int j = 0; j < bins-1; j++)
			hist.at<float>(j) /= hmax;
	}

	return hist;
}


cv::Mat histToImg(cv::Mat hist)
{
	// Image for the histogram
	int histSize = 256;
	// int histSize = hist.cols;
	int hist_w = 512; int hist_h = 400;
	int bin_w = cvRound( (double) hist_w/histSize );

	cv::Mat histImage( hist_h, hist_w, CV_8UC3, Scalar( 0,0,0) );

	/// Draw
	for( int i = 1; i < histSize; i++ )
	{
		// // Graduation
		// if(i%5 == 0)
		// line( histImage, Point( bin_w*(i), hist_h ) ,
		//                Point( bin_w*(i), 0 ),
		//                Scalar( 0, 30, 0), 2, 8, 0  );
		// if(i%20 == 0)
		// line( histImage, Point( bin_w*(i), hist_h ) ,
		//                Point( bin_w*(i), 0 ),
		//                Scalar( 30, 0, 0), 2, 8, 0  );

		// Espace couleur FEU
		// TODO: Paramétrer ces plages de couleurs
		// Rouge
		if(i < 10 || (i >= 170 && i <= 180))
		line( histImage, Point( bin_w*(i), hist_h ) ,
					   Point( bin_w*(i), 0 ),
					   Scalar( 0, 0, 50), 2, 8, 0  );
		// Vert
		if(i < 90 && i >= 70)
		line( histImage, Point( bin_w*(i), hist_h ) ,
					   Point( bin_w*(i), 0 ),
					   Scalar( 0, 50, 0), 2, 8, 0  );
		// Jaune
		if(i < 30 && i >= 10)
		line( histImage, Point( bin_w*(i), hist_h ) ,
					   Point( bin_w*(i), 0 ),
					   Scalar( 0, 30, 50), 2, 8, 0  );

		// Histo
		line( histImage, Point( bin_w*(i-1), hist_h - cvRound(hist.at<float>(i-1)*hist_h) ) ,
					   Point( bin_w*(i), hist_h - cvRound(hist.at<float>(i)*hist_h) ),
					   Scalar( 200, 200, 200), 2, 8, 0  );
	}

	return histImage;
}



/*-----  End of Class Definition  ------*/