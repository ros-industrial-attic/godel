#include "ftp_upload.h"

#include <stdlib.h>
#include <stdio.h>

#include <curl/curl.h>

/*
 * The functions in this file are based on example software for libcurl
 * including the ftpuploadresume.c example.
 */

const static long DEFAULT_TIMEOUT = 0; // Don't timeout
const static long DEFAULT_RETRIES = 5;

/* parse headers for Content-Length */
static size_t getcontentlengthfunc(void* ptr, size_t size, size_t nmemb, void* stream)
{
  int r;
  long len = 0;

  /* _snscanf() is Win32 specific */
  r = sscanf(static_cast<const char*>(ptr), "Content-Length: %ld\n", &len);

  if (r) /* Microsoft: we don't read the specs */
    *((long*)stream) = len;

  return size * nmemb;
}

/* discard downloaded data */
static size_t discardfunc(void* ptr, size_t size, size_t nmemb, void* stream)
{
  return size * nmemb;
}

/* read data to upload */
static size_t readfunc(void* ptr, size_t size, size_t nmemb, void* stream)
{
  FILE* f = static_cast<FILE*>(stream);
  size_t n;

  if (ferror(f))
    return CURL_READFUNC_ABORT;

  n = fread(ptr, size, nmemb, f) * size;

  return n;
}

static int upload(CURL* curlhandle, const char* remotepath, const char* localpath, long timeout,
                  long tries, const char* user_and_pwd)
{
  FILE* f;
  long uploaded_len = 0;
  CURLcode r = CURLE_GOT_NOTHING;
  int c;

  f = fopen(localpath, "rb");
  if (f == NULL)
  {
    perror(NULL);
    return 0;
  }

  curl_easy_setopt(curlhandle, CURLOPT_UPLOAD, 1L);

  curl_easy_setopt(curlhandle, CURLOPT_URL, remotepath);

  if (user_and_pwd)
    curl_easy_setopt(curlhandle, CURLOPT_USERPWD, user_and_pwd); //"Default User:robotics" by default

  curl_easy_setopt(curlhandle, CURLOPT_CONNECTTIMEOUT, 2L);

  if (timeout)
    curl_easy_setopt(curlhandle, CURLOPT_FTP_RESPONSE_TIMEOUT, timeout);

  curl_easy_setopt(curlhandle, CURLOPT_HEADERFUNCTION, getcontentlengthfunc);
  curl_easy_setopt(curlhandle, CURLOPT_HEADERDATA, &uploaded_len);

  curl_easy_setopt(curlhandle, CURLOPT_WRITEFUNCTION, discardfunc);

  curl_easy_setopt(curlhandle, CURLOPT_READFUNCTION, readfunc);
  curl_easy_setopt(curlhandle, CURLOPT_READDATA, f);

  curl_easy_setopt(curlhandle, CURLOPT_FTPPORT, "-"); /* disable passive mode */
  curl_easy_setopt(curlhandle, CURLOPT_FTP_CREATE_MISSING_DIRS, 1L);

  curl_easy_setopt(curlhandle, CURLOPT_VERBOSE, 1L);

  for (c = 0; (r != CURLE_OK) && (c < tries); c++)
  {
    /* are we resuming? */
    if (c)
    { /* yes */
      /* determine the length of the file already written */

      /*
       * With NOBODY and NOHEADER, libcurl will issue a SIZE
       * command, but the only way to retrieve the result is
       * to parse the returned Content-Length header. Thus,
       * getcontentlengthfunc(). We need discardfunc() above
       * because HEADER will dump the headers to stdout
       * without it.
       */
      curl_easy_setopt(curlhandle, CURLOPT_NOBODY, 1L);
      curl_easy_setopt(curlhandle, CURLOPT_HEADER, 1L);

      r = curl_easy_perform(curlhandle);
      if (r != CURLE_OK)
        continue;

      curl_easy_setopt(curlhandle, CURLOPT_NOBODY, 0L);
      curl_easy_setopt(curlhandle, CURLOPT_HEADER, 0L);

      fseek(f, uploaded_len, SEEK_SET);

      curl_easy_setopt(curlhandle, CURLOPT_APPEND, 1L);
    }
    else
    { /* no */
      curl_easy_setopt(curlhandle, CURLOPT_APPEND, 0L);
    }

    r = curl_easy_perform(curlhandle);
  }

  fclose(f);

  if (r == CURLE_OK)
    return 1;
  else
  {
    fprintf(stderr, "%s\n", curl_easy_strerror(r));
    return 0;
  }
}

bool abb_file_suite::uploadFile(const std::string& ftp_addr, const std::string& filepath,
                                const std::string& user_name, const std::string& password)
{
  CURL* curlhandle = NULL;

  curl_global_init(CURL_GLOBAL_ALL);
  curlhandle = curl_easy_init();

  std::string to = "ftp://" + ftp_addr + "/mGodelBlend.mod";

  std::string user_pwd = user_name + ":" + password;

  const char* auth_string = NULL;
  if (!user_name.empty() && !password.empty())
  {
    auth_string = user_pwd.c_str();
  }

  bool result = upload(curlhandle, to.c_str(), filepath.c_str(), DEFAULT_TIMEOUT, DEFAULT_RETRIES,
                       auth_string);

  curl_easy_cleanup(curlhandle);
  curl_global_cleanup();

  return result;
}
