#pragma once

#include <string>
#include <vector>

class AESHandler {
public:
    AESHandler(const unsigned char* key, const unsigned char* iv);
    
    // Returns base64 encoded encrypted string
    std::string encrypt(const std::string& plaintext);
    
    // Takes base64 encoded encrypted string
    std::string decrypt(const std::string& b64_ciphertext);

private:
    const unsigned char* m_key;
    const unsigned char* m_iv;
    
    static std::string base64_encode(const std::vector<unsigned char>& data);
    static std::vector<unsigned char> base64_decode(const std::string& b64_string);
};
