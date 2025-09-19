#include "aes_handler.h"
#include <openssl/evp.h>
#include <openssl/aes.h>
#include <openssl/bio.h>
#include <openssl/buffer.h>
#include <stdexcept>
#include <memory>
#include <iostream>

AESHandler::AESHandler(const unsigned char* key, const unsigned char* iv)
    : m_key(key), m_iv(iv) {}

std::string AESHandler::base64_encode(const std::vector<unsigned char>& data) {
    BIO *bio, *b64;
    BUF_MEM *bufferPtr;

    b64 = BIO_new(BIO_f_base64());
    bio = BIO_new(BIO_s_mem());
    bio = BIO_push(b64, bio);

    BIO_set_flags(bio, BIO_FLAGS_BASE64_NO_NL);
    BIO_write(bio, data.data(), data.size());
    BIO_flush(bio);

    BIO_get_mem_ptr(bio, &bufferPtr);
    std::string result(bufferPtr->data, bufferPtr->length);
    BIO_free_all(bio);

    return result;
}

std::vector<unsigned char> AESHandler::base64_decode(const std::string& b64_string) {
    BIO *bio, *b64;
    std::vector<unsigned char> buffer(b64_string.length());
    
    b64 = BIO_new(BIO_f_base64());
    bio = BIO_new_mem_buf(b64_string.c_str(), -1);
    bio = BIO_push(b64, bio);

    BIO_set_flags(bio, BIO_FLAGS_BASE64_NO_NL);
    int decoded_size = BIO_read(bio, buffer.data(), b64_string.length());
    buffer.resize(decoded_size);
    BIO_free_all(bio);

    return buffer;
}


std::string AESHandler::encrypt(const std::string& plaintext) {
    EVP_CIPHER_CTX *ctx;
    int len;
    int ciphertext_len;
    std::vector<unsigned char> ciphertext(plaintext.length() + AES_BLOCK_SIZE);

    if(!(ctx = EVP_CIPHER_CTX_new())) throw std::runtime_error("Failed to create new EVP Cipher CTX");

    if(1 != EVP_EncryptInit_ex(ctx, EVP_aes_128_cbc(), NULL, m_key, m_iv))
        throw std::runtime_error("Failed to initialize encryption");

    if(1 != EVP_EncryptUpdate(ctx, ciphertext.data(), &len, (const unsigned char*)plaintext.c_str(), plaintext.length()))
        throw std::runtime_error("Failed to update encryption");
    ciphertext_len = len;

    if(1 != EVP_EncryptFinal_ex(ctx, ciphertext.data() + len, &len))
        throw std::runtime_error("Failed to finalize encryption");
    ciphertext_len += len;
    
    EVP_CIPHER_CTX_free(ctx);

    ciphertext.resize(ciphertext_len);
    return base64_encode(ciphertext);
}


std::string AESHandler::decrypt(const std::string& b64_ciphertext) {
    auto ciphertext = base64_decode(b64_ciphertext);
    EVP_CIPHER_CTX *ctx;
    int len;
    int plaintext_len;
    std::vector<unsigned char> plaintext(ciphertext.size());

    if(!(ctx = EVP_CIPHER_CTX_new())) throw std::runtime_error("Failed to create new EVP Cipher CTX");

    if(1 != EVP_DecryptInit_ex(ctx, EVP_aes_128_cbc(), NULL, m_key, m_iv))
        throw std::runtime_error("Failed to initialize decryption");
    
    // It's important to tell OpenSSL that you're not padding your data if it's not padded
    // EVP_CIPHER_CTX_set_padding(ctx, 0);

    if(1 != EVP_DecryptUpdate(ctx, plaintext.data(), &len, ciphertext.data(), ciphertext.size()))
        throw std::runtime_error("Failed to update decryption");
    plaintext_len = len;

    if(1 != EVP_DecryptFinal_ex(ctx, plaintext.data() + len, &len)) {
        // This can fail if padding is incorrect or the data is corrupt
        EVP_CIPHER_CTX_free(ctx);
        // std::cerr << "Decryption finalization failed. Likely incorrect key/IV or corrupt data." << std::endl;
        return ""; // Return empty string on failure
    }
    plaintext_len += len;
    
    EVP_CIPHER_CTX_free(ctx);

    return std::string(reinterpret_cast<const char*>(plaintext.data()), plaintext_len);
}
