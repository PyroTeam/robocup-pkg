#ifndef _TRAITEMENT_IMAGE__CV_ALGO__H_
#define _TRAITEMENT_IMAGE__CV_ALGO__H_

#include "feu_tricolore/LightDetection.h"

/**
 * \brief      Convertit une image binaire en image multichannel
 *
 * \details    Devient particulièrement utile quand vous voulez faire ....
 *             euh ... je ne sais plus quand c'est nécessaire, mais si un
 *             jour ça arrive, la fonction est là pour ça !
 *
 * \param[in]  binary       The binary image
 * \param[in]  numChannels  The number of desired channels
 *
 * \return     The mutlichannel image
 */
cv::Mat singleToMultChannels(cv::Mat binary, int numChannels = 3);

/**
 * \brief      Calcule l'histograme d'un canal d'une image
 *
 * \param[in]  imgToHist  L'image dont on veut calculer l'histograme
 * \param[in]  channel    Le canal de l'image à utiliser
 * \param[in]  normalize  Normalise ou non l'histograme
 *
 * \return     L'histogramme calculé
 */
cv::Mat calcHist(cv::Mat imgToHist, int channel = 0, bool normalize = true);
/**
 * \brief      Convertit un histogramme en image
 *
 * \param[in]  hist  L'histogramme à convertir
 *
 * \return     L'image représentant l'histograme
 */
cv::Mat histToImg(cv::Mat hist);

#endif // _TRAITEMENT_IMAGE__CV_ALGO__H_
