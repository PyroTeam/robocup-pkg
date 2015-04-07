/**
 * \file 		encryptUtils.h
 *
 * \brief		
 *
 * \author		Coelen Vincent (vincent.coelen@polytech-lille.net)
 *              Tissot Elise
 * \date		2015-04-07
 * \copyright	PyroTeam, Polytech-Lille
 * \license
 * \version
 */

#ifndef ENCRYPTUTILS_H_
#define ENCRYPTUTILS_H_

class EncryptUtils
{
public:
    EncryptUtils(std::string key = "random", std::string cipher = "aes-128");
    ~EncryptUtils();
    
    void setConfig(std::string key, std::string cipher)
    {
        m_key = key;
        m_cipher = cipher;
    }
    
    void encrypt(const std::string &bufferIn, std::string &bufferOut, std::string &initialisationVector);
    void decrypt(const std::string &bufferIn, std::string &bufferOut, const std::string &initialisationVector);
    
private:
    std::string m_key;
    std::string m_cipher;
};

#endif /* ENCRYPTUTILS_H_ */
