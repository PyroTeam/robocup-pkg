/**
 * \file         encryptUtils.cpp
 *
 * \brief
 *
 * \author        Coelen Vincent (vincent.coelen@polytech-lille.net)
 *              Tissot Elise
 * \date        2015-04-07
 * \copyright    PyroTeam, Polytech-Lille
 * \license
 * \version
 */


#include "encryptUtils.h"

#include <iostream>

#include <openssl/conf.h>
#include <openssl/evp.h>
#include <openssl/err.h>
#include <openssl/aes.h>
#include <openssl/rand.h>

EncryptUtils::EncryptUtils()
{
    std::string skey = "random_k";
    m_key.assign(skey.begin(), skey.end());
    m_key.push_back(0);
    m_cipher = AES_CBC_128;
}

EncryptUtils::EncryptUtils(Buffer_type &key, CIPHER_TYPE cipher)
{

    setConfig(key, cipher);
}

EncryptUtils::~EncryptUtils()
{

}


void handleErrors()
{
    ERR_print_errors_fp(stderr);
    abort();
}

void EncryptUtils::getRandomIV(Buffer_type &iv)
{
    iv.resize(AES_BLOCK_SIZE);
    RAND_bytes(iv.data(), AES_BLOCK_SIZE);
}

void EncryptUtils::encrypt(const Buffer_type &message, Buffer_type &encryptedMessage, Buffer_type &initialisationVector)
{
    getRandomIV(initialisationVector);
    unsigned char cipherMessage[message.size()+16];
    int len = encrypt_(message.data(), message.size(), m_key.data(), initialisationVector.data(), cipherMessage);

    encryptedMessage.assign(cipherMessage, cipherMessage+len);

}

void EncryptUtils::decrypt(const Buffer_type &encryptedMessage, Buffer_type &decryptedMessage, const Buffer_type &initialisationVector)
{
    unsigned char decryptedMsg[encryptedMessage.size()+16];
    int len = decrypt_(encryptedMessage.data(), encryptedMessage.size(), m_key.data(), initialisationVector.data(), decryptedMsg);

    decryptedMessage.assign(decryptedMsg, decryptedMsg + len);
}


int EncryptUtils::encrypt_(const unsigned char *plaintext, int plaintext_len, unsigned char *key,
                           const unsigned char *iv, unsigned char *ciphertext)
{
  EVP_CIPHER_CTX *ctx;

  int len;

  int ciphertext_len;

  /* Create and initialise the context */
  if(!(ctx = EVP_CIPHER_CTX_new())) handleErrors();

  /* Initialise the encryption operation. IMPORTANT - ensure you use a key
   * and IV size appropriate for your cipher
   * In this example we are using 256 bit AES (i.e. a 256 bit key). The
   * IV size for *most* modes is the same as the block size. For AES this
   * is 128 bits */

    std::cout << "Clé : " << key << std::endl;

  if(1 != EVP_EncryptInit_ex(ctx, EVP_aes_256_cbc(), NULL, key, iv))
    handleErrors();

  /* Provide the message to be encrypted, and obtain the encrypted output.
   * EVP_EncryptUpdate can be called multiple times if necessary
   */
  if(1 != EVP_EncryptUpdate(ctx, ciphertext, &len, plaintext, plaintext_len))
    handleErrors();
  ciphertext_len = len;

  /* Finalise the encryption. Further ciphertext bytes may be written at
   * this stage.
   */
  if(1 != EVP_EncryptFinal_ex(ctx, ciphertext + len, &len)) handleErrors();
  ciphertext_len += len;

  /* Clean up */
  EVP_CIPHER_CTX_free(ctx);

  return ciphertext_len;
}


int EncryptUtils::decrypt_(const unsigned char *ciphertext, int ciphertext_len, unsigned char *key,
                           const unsigned char *iv, unsigned char *plaintext)
{
  EVP_CIPHER_CTX *ctx;

  int len;

  int plaintext_len;

  /* Create and initialise the context */
  if(!(ctx = EVP_CIPHER_CTX_new())) handleErrors();

  /* Initialise the decryption operation. IMPORTANT - ensure you use a key
   * and IV size appropriate for your cipher
   * In this example we are using 256 bit AES (i.e. a 256 bit key). The
   * IV size for *most* modes is the same as the block size. For AES this
   * is 128 bits */
  if(1 != EVP_DecryptInit_ex(ctx, EVP_aes_256_cbc(), NULL, key, iv))
    handleErrors();

  /* Provide the message to be decrypted, and obtain the plaintext output.
   * EVP_DecryptUpdate can be called multiple times if necessary
   */
  if(1 != EVP_DecryptUpdate(ctx, plaintext, &len, ciphertext, ciphertext_len))
    handleErrors();
  plaintext_len = len;

  /* Finalise the decryption. Further plaintext bytes may be written at
   * this stage.
   */
  if(1 != EVP_DecryptFinal_ex(ctx, plaintext + len, &len)) handleErrors();
  plaintext_len += len;

  /* Clean up */
  EVP_CIPHER_CTX_free(ctx);

  return plaintext_len;
}
