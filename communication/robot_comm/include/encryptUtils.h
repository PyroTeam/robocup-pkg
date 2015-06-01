/**
 * \file        encryptUtils.h
 *
 * \brief
 *
 * \author      Coelen Vincent (vincent.coelen@polytech-lille.net)
 *              Tissot Elise
 * \date        2015-04-07
 * \copyright   PyroTeam, Polytech-Lille
 * \license
 * \version
 */

#ifndef ENCRYPTUTILS_H_
#define ENCRYPTUTILS_H_

#include <string>
#include <vector>
#include <initializer_list>

typedef std::vector<unsigned char> Buffer_type;

class EncryptUtils
{
public:
    enum CIPHER_TYPE
    {
        AES_CBC_128,
        AES_CBC_192,
        AES_CBC_256
    };

    EncryptUtils();
    EncryptUtils(Buffer_type &key, CIPHER_TYPE cipher = EncryptUtils::AES_CBC_128);
    ~EncryptUtils();

    void setConfig(Buffer_type &key, CIPHER_TYPE cipher)
    {
        m_key = key;
        m_key.resize(32, 0);
        m_cipher = cipher;
    }


    void encrypt(const Buffer_type &message, Buffer_type &encryptedMessaget, Buffer_type &initialisationVector);
    void decrypt(const Buffer_type &encryptedMessage, Buffer_type &decryptedMessage, const Buffer_type &initialisationVector);

private:
    Buffer_type m_key;
    CIPHER_TYPE m_cipher;

    void getRandomIV(Buffer_type &iv);

    int encrypt_(const unsigned char *plaintext, int plaintext_len, unsigned char *key, const unsigned char *iv, unsigned char *ciphertext);
    int decrypt_(const unsigned char *ciphertext, int ciphertext_len, unsigned char *key, const unsigned char *iv, unsigned char *plaintext);

};

#endif /* ENCRYPTUTILS_H_ */
