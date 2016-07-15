/*
 * A WebSocket to TCP socket proxy with support for "wss://" encryption.
 * Copyright 2010 Joel Martin
 * Licensed under LGPL version 3 (see docs/LICENSE.LGPL-3)
 *
 * You can make a cert/key with openssl using:
 * openssl req -new -x509 -days 365 -nodes -out self.pem -keyout self.pem
 * as taken from http://docs.python.org/dev/library/ssl.html#certificates
 */
#include <stdio.h>
#include <errno.h>
#include <limits.h>
#include <getopt.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h>
#include <sys/select.h>
#include <fcntl.h>
#include <sys/stat.h>
#include "websocket.h"
#include "PythonWrapper.h"

char traffic_legend[] = "\n\
Traffic Legend:\n\
    }  - Client receive\n\
    }. - Client receive partial\n\
    {  - Target receive\n\
\n\
    >  - Target send\n\
    >. - Target send partial\n\
    <  - Client send\n\
    <. - Client send partial\n\
";

char USAGE[] = "Usage: [options] " \
               "[source_addr:]source_port target_addr:target_port\n\n" \
               "  --verbose|-v       verbose messages and per frame traffic\n" \
               "  --daemon|-D        become a daemon (background process)\n" \
               "  --cert CERT        SSL certificate file\n" \
               "  --key KEY          SSL key file (if separate from cert)\n" \
               "  --ssl-only         disallow non-encrypted connections";

#define usage(fmt, args...) \
    fprintf(stderr, "%s\n\n", USAGE); \
    fprintf(stderr, fmt , ## args); \
    exit(1);

char target_host[256];
int target_port;

//extern pipe_error;
extern settings_t settings;

#include<vector>
#include<string>

using namespace std;
vector<unsigned char> incomingMessage;
string unpackedMessage;

int state=0;
bool is_final_fragment;
bool is_masked;
int opcode;
int header_length;
unsigned char mask[4];
int payload_size;


int dave_decode_hybi()
{
   printf("decoding hybi\n");   

   if(state==0)
   {
      printf("  Trying to process header\n");
      
      is_final_fragment=false;
      opcode=0;
      unpackedMessage=string("");

      if(incomingMessage.size()<2) 
      {
         printf("  need at least 2 bytes to begin processing!\n");
         return 0;
      }
      
      if(incomingMessage[0] & 128)
      {
         is_final_fragment=true;
         printf("    final fragment: %s\n",is_final_fragment ? "true" : "false");
      }
      else
      {
         printf("    detected this is not the final fragment\n");
      }

      opcode=incomingMessage[0] & 15; //get 4 bits   
      printf("    opcode is: %d\n", opcode);
      if(opcode==8)
      { 
         printf("    connection close\n");
         return -1;
      }

      if(incomingMessage[1]&128)
      {
         is_masked=true;
         printf("    payload is masked\n");
      }
      else
      {
         printf("    payload from client to server has to be masked!\n");
         return -1;
      }
      
      payload_size=0;
      int psize=incomingMessage[1]&127;

      if(psize<126)
      {
         payload_size=psize;
         printf("    size of payload is: %d\n",payload_size);
         header_length=2;
      }
      if(psize==126)
      {
         if(incomingMessage.size()<4)
         {
            printf("    need at least 4 bytes to for next processing!\n");
            return 0;
         }
         printf("    message is bigger, size is defined by 2bytes\n");
         payload_size=(incomingMessage[2] << 8) + incomingMessage[3];
         printf("    size of payload is: %d\n",payload_size);

         header_length=2+2;

      }
      if(psize==127)
      {
         if(incomingMessage.size()<10)
         {
            printf("  need at least 10 bytes for next processing!\n");
            return 0;
         }
         printf("    message is real big, size is defined by 8bytes\n");
         payload_size=(incomingMessage[2] << 56) + (incomingMessage[3] << 48) + (incomingMessage[4] << 40) + incomingMessage[5]<<32 +
                      (incomingMessage[6] << 24) + (incomingMessage[7] << 16) + (incomingMessage[8] << 8) + incomingMessage[9];
         printf("    size of payload is: %d\n",payload_size);

         header_length=2+8;
      }
      if(incomingMessage.size()<(header_length+4))
      {
          printf("    need more bytes for next processing!\n");
          return 0;
      }
      
      for(int i=0;i<4;i++)
         mask[i]=incomingMessage[header_length+i];     

      printf("    mask has value: %d %d %d %d\n",mask[0],mask[1],mask[2],mask[3]);

      state=1; //finished getting all data out of header
   }
   if(state==1) //time to process payload
   {
      if(incomingMessage.size()<header_length+4+payload_size)
      {
         printf("  need more bytes to process payload");
         return 0; 
      }
      
      printf("  processing payload (size=%d)\n",payload_size);
 
      for(unsigned int i=0;i<payload_size;i++)
      {
         unsigned char result=incomingMessage[header_length+4+i] ^ mask[i%4];
         unpackedMessage+=result;
      }      

      if(is_final_fragment)
      {
         printf("    final unpacked message is:\n=============================\n%s\n=============================\n",unpackedMessage.c_str());
         handleIncomingMessage(unpackedMessage); //hand over to python wrapper
      }
      incomingMessage.erase (incomingMessage.begin(),incomingMessage.begin()+header_length+4+payload_size);

      state=0;
   }
}

void do_process_incoming(ws_ctx_t *ws_ctx)
{
    initialize_python_interpreter();

    while (1)
    {
       unsigned int bytes = ws_recv(ws_ctx, ws_ctx->tin_buf, BUFSIZE-1);
       if (bytes <= 0) {
          handler_emsg("client closed connection\n");
          break;
       }

       for(unsigned int i=0;i<bytes;i++)
          incomingMessage.push_back(ws_ctx->tin_buf[i]);
           
       int len=0;
       if (ws_ctx->hybi) {            
          len = dave_decode_hybi();
       } else {
          printf("can't decode hixi\n");
          break;
       }
            
       if (len < 0) {
          handler_emsg("decoding error\n");
          break;
       }        
    }

    shutdown_python_interpreter();
}

int main(int argc, char *argv[])
{
    int fd, c, option_index = 0;
    static int ssl_only = 0, daemon = 0, run_once = 0, verbose = 0;
    char *found;
    static struct option long_options[] = {
        {"verbose",    no_argument,       &verbose,    'v'},
        {"ssl-only",   no_argument,       &ssl_only,    1 },
        {"daemon",     no_argument,       &daemon,     'D'},
        /* ---- */
        {"run-once",   no_argument,       0,           'r'},
        {"cert",       required_argument, 0,           'c'},
        {"key",        required_argument, 0,           'k'},
        {0, 0, 0, 0}
    };

    settings.cert = realpath("self.pem", NULL);
    if (!settings.cert) {
        /* Make sure it's always set to something */
        settings.cert = "self.pem";
    }
    settings.key = "";

    while (1) {
        c = getopt_long (argc, argv, "vDrc:k:",
                         long_options, &option_index);

        /* Detect the end */
        if (c == -1) { break; }

        switch (c) {
            case 0:
                break; // ignore
            case 1:
                break; // ignore
            case 'v':
                verbose = 1;
                break;
            case 'D':
                daemon = 1;
                break;
            case 'r':
                run_once = 1;
                break;
            case 'c':
                settings.cert = realpath(optarg, NULL);
                if (! settings.cert) {
                    usage("No cert file at %s\n", optarg);
                }
                break;
            case 'k':
                settings.key = realpath(optarg, NULL);
                if (! settings.key) {
                    usage("No key file at %s\n", optarg);
                }
                break;
            default:
                usage("");
        }
    }
    settings.verbose      = verbose;
    settings.ssl_only     = ssl_only;
    settings.daemon       = daemon;
    settings.run_once     = run_once;

    if(argc==1)
    {
        settings.listen_host[0] = '\0';
        settings.listen_port = 1234;
        printf("no port specified as argument. setting to default of: 1234\n");
    }
    else
    {
    //printf("argc %d optind %d\n",argc,optind);
    found = strstr(argv[optind], ":");
    if (found) {
        memcpy(settings.listen_host, argv[optind], found-argv[optind]);
        settings.listen_port = strtol(found+1, NULL, 10);
    } else {
        settings.listen_host[0] = '\0';
        settings.listen_port = strtol(argv[optind], NULL, 10);
    }
    optind++;
    if (settings.listen_port == 0) {
        printf("Could not parse listen_port\n");
        return 0;
    }
    }

    /*if (ssl_only) {
        if (access(settings.cert, R_OK) != 0) {
            usage("SSL only and cert file '%s' not found\n", settings.cert);
        }
    } else if (access(settings.cert, R_OK) != 0) {
        fprintf(stderr, "Warning: '%s' not found\n", settings.cert);
    }*/

    //printf("  verbose: %d\n",   settings.verbose);
    //printf("  ssl_only: %d\n",  settings.ssl_only);
    //printf("  daemon: %d\n",    settings.daemon);
    //printf("  run_once: %d\n",  settings.run_once);
    //printf("  cert: %s\n",      settings.cert);
    //printf("  key: %s\n",       settings.key);

    settings.handler = do_process_incoming; 
    start_server();

}
