#ifndef FTP_UPLOAD_H
#define FTP_UPLOAD_H

#include <string>

namespace abb_file_suite
{

bool uploadFile(const std::string& ftp_addr, const std::string& filepath);
}

#endif // FTP_UPLOAD_H
