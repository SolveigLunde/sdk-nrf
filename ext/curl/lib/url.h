#ifndef HEADER_CURL_URL_H
#define HEADER_CURL_URL_H
/***************************************************************************
 *                                  _   _ ____  _
 *  Project                     ___| | | |  _ \| |
 *                             / __| | | | |_) | |
 *                            | (__| |_| |  _ <| |___
 *                             \___|\___/|_| \_\_____|
 *
 * Copyright (C) 1998 - 2020, Daniel Stenberg, <daniel@haxx.se>, et al.
 *
 * This software is licensed as described in the file COPYING, which
 * you should have received as part of this distribution. The terms
 * are also available at https://curl.haxx.se/docs/copyright.html.
 *
 * You may opt to use, copy, modify, merge, publish, distribute and/or sell
 * copies of the Software, and permit persons to whom the Software is
 * furnished to do so, under the terms of the COPYING file.
 *
 * This software is distributed on an "AS IS" basis, WITHOUT WARRANTY OF ANY
 * KIND, either express or implied.
 *
 ***************************************************************************/
#include "curl_setup.h"

#if defined(CONFIG_NRF_CURL_INTEGRATION)
#define READBUFFER_SIZE (5*708)
#define READBUFFER_MAX  CURL_MAX_READ_SIZE
#define READBUFFER_MIN  100
#else
#define READBUFFER_SIZE CURL_MAX_WRITE_SIZE
#define READBUFFER_MAX  CURL_MAX_READ_SIZE
#define READBUFFER_MIN  1024
#endif
/* The default upload buffer size, should not be smaller than
   CURL_MAX_WRITE_SIZE, as it needs to hold a full buffer as could be sent in
   a write callback.

   The size was 16KB for many years but was bumped to 64KB because it makes
   libcurl able to do significantly faster uploads in some circumstances. Even
   larger buffers can help further, but this is deemed a fair memory/speed
   compromise. */
#if defined(CONFIG_NRF_CURL_INTEGRATION)
/* In embedded this needs to be a lot of smaller */
#define UPLOADBUFFER_DEFAULT (708)
#if defined(CONFIG_SOC_SERIES_NRF92X)
#define UPLOADBUFFER_MAX (DT_REG_SIZE(DT_NODELABEL(cpuapp_cpucell_ipc_shm_heap)))
#else
#define UPLOADBUFFER_MAX (CONFIG_NRF_MODEM_LIB_SHMEM_TX_SIZE)
#endif
#define UPLOADBUFFER_MIN (CURL_MAX_WRITE_SIZE)
#else
#define UPLOADBUFFER_DEFAULT 65536
#define UPLOADBUFFER_MAX (2*1024*1024)
#define UPLOADBUFFER_MIN CURL_MAX_WRITE_SIZE
#endif

/*
 * Prototypes for library-wide functions provided by url.c
 */

CURLcode Curl_init_do(struct Curl_easy *data, struct connectdata *conn);
CURLcode Curl_open(struct Curl_easy **curl);
CURLcode Curl_init_userdefined(struct Curl_easy *data);

void Curl_freeset(struct Curl_easy *data);
CURLcode Curl_uc_to_curlcode(CURLUcode uc);
CURLcode Curl_close(struct Curl_easy **datap); /* opposite of curl_open() */
CURLcode Curl_connect(struct Curl_easy *, bool *async, bool *protocol_connect);
CURLcode Curl_disconnect(struct Curl_easy *data,
                         struct connectdata *, bool dead_connection);
CURLcode Curl_setup_conn(struct connectdata *conn,
                         bool *protocol_done);
void Curl_free_request_state(struct Curl_easy *data);
CURLcode Curl_parse_login_details(const char *login, const size_t len,
                                  char **userptr, char **passwdptr,
                                  char **optionsptr);

const struct Curl_handler *Curl_builtin_scheme(const char *scheme);

bool Curl_is_ASCII_name(const char *hostname);
CURLcode Curl_idnconvert_hostname(struct connectdata *conn,
                                  struct hostname *host);
void Curl_free_idnconverted_hostname(struct hostname *host);

#define CURL_DEFAULT_PROXY_PORT 1080 /* default proxy port unless specified */
#define CURL_DEFAULT_HTTPS_PROXY_PORT 443 /* default https proxy port unless
                                             specified */

#ifdef CURL_DISABLE_VERBOSE_STRINGS
#define Curl_verboseconnect(x)  Curl_nop_stmt
#else
void Curl_verboseconnect(struct connectdata *conn);
#endif

#ifdef CURL_DISABLE_PROXY
#define CONNECT_PROXY_SSL() FALSE
#else

#define CONNECT_PROXY_SSL()\
  (conn->http_proxy.proxytype == CURLPROXY_HTTPS &&\
  !conn->bits.proxy_ssl_connected[sockindex])

#define CONNECT_FIRSTSOCKET_PROXY_SSL()\
  (conn->http_proxy.proxytype == CURLPROXY_HTTPS &&\
  !conn->bits.proxy_ssl_connected[FIRSTSOCKET])

#define CONNECT_SECONDARYSOCKET_PROXY_SSL()\
  (conn->http_proxy.proxytype == CURLPROXY_HTTPS &&\
  !conn->bits.proxy_ssl_connected[SECONDARYSOCKET])
#endif /* !CURL_DISABLE_PROXY */

#endif /* HEADER_CURL_URL_H */
