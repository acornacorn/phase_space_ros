/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2013, SRI International
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

/** \Author: Sean Seungkook Yun */

/* UDP server for phaseSpace */

#include "socket/udpServer.h"
#include "socket/exceptions.h"

#include <boost/exception/all.hpp>

#include <sys/types.h>
#include <sys/socket.h>
#include <netdb.h>
#include <netinet/in.h>
#include <arpa/inet.h>

#include <stdlib.h>
#include <string.h>

phaseSpace::udpServer::udpServer(const int port)
  : fd_(-1)
{
  bind_udp(NULL, port);
} 

phaseSpace::udpServer::udpServer(
      const std::string& hostname,
      const int port)
  : fd_(-1)
{
  bind_udp(hostname, port);
}

// Do not copy
phaseSpace::udpServer::udpServer(const udpServer& udp)
  : fd_(-1)
{
  BOOST_THROW_EXCEPTION(base_exception());
}

phaseSpace::udpServer::~udpServer()
{
  close();
}

void phaseSpace::udpServer::bind_udp(
      const std::string& hostname,
      const int port)
{
  struct sockaddr_in sa;  

  fd_ = socket(AF_INET, SOCK_DGRAM , 0);
  if (fd_ < 0)
  {
    int save_errno = errno;
    BOOST_THROW_EXCEPTION(socket_exception()
                            << boost::errinfo_errno(save_errno));
  }

  memset(&sa, 0, sizeof(sa));
  sa.sin_family = AF_INET;
  sa.sin_addr.s_addr = hostname.c_str() ? inet_addr(hostname.c_str()) : htonl(INADDR_ANY);
  sa.sin_port = htons((u_short)port);

  if (bind(fd_, (struct sockaddr *) &sa,  sizeof(sa))<0)
  {
    int save_errno = errno;
    BOOST_THROW_EXCEPTION(bind_failed_exception()
                            << boost::errinfo_errno(save_errno));   
  }

}

void phaseSpace::udpServer::close()
{
  if (fd_ >= 0)
    ::close(fd_);
  fd_ = -1;

}

int phaseSpace::udpServer::recv_udp(void *buf, const int nbytes) {
  return recv_udp(buf, nbytes, 0, NULL, NULL);
}

int phaseSpace::udpServer::recv_udp(void *buf, const int nbytes, int flags, char ip[20], int *port) {
  int n, len;
  struct sockaddr_in sa;

  len = sizeof(sa);
  if (ip)
    n = recvfrom(fd_, (char *)buf, nbytes, flags, (struct sockaddr*)&sa, (socklen_t*)&len);
  else 
    n = recvfrom(fd_, (char *)buf, nbytes, flags, NULL, 0);

  /*
  if (n != nbytes) {
    eprintf("SocketRecvUDP: send[%d] %d/%d failed. errno=%d\n", s, n, nbytes, errno);
    return -1;
  }
  */
  if (ip) {
    strcpy(ip, inet_ntoa(sa.sin_addr));
    if (port)
      *port = ntohs(sa.sin_port);
  }
//  if (so_debug)
//    printf("%s: <= %d %s.%d\n", fn, n, ip ? inet_ntoa(sa.sin_addr) : "", ip ? ntohs(sa.sin_port) : 0);

  return n;
}


