#include "<name>_comm.h"
#include "ndlcom/Bridge.h"
#include "ndlcom/Node.h"

#define <name>_NO_EXTERNAL_INTERFACES 1

#if <name>_NO_EXTERNAL_INTERFACES > 0
#include <stdio.h>
#include <string.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <stdlib.h>
#endif

#if <name>_NO_INTERNAL_INPUTS > 0
FILE *audio_in_fp = NULL;
float <name>_readInternalInput(const unsigned int i)
{
    int16_t val;

    if (!audio_in_fp)
        return 0.0f;

    if (fread(&val, sizeof(val), 1, audio_in_fp) < 1) {
        fclose(audio_in_fp);
        audio_in_fp = NULL;
        return 0.0f;
    }
    usleep(22); /*Sample every 22 us which is about 44100 Hz*/
    return val;
}
#endif

#if <name>_NO_INTERNAL_OUTPUTS > 0
FILE *audio_out_fp = NULL;
void <name>_writeInternalOutput(const unsigned int i, const float value)
{
    int16_t val;
    val = value;
    fwrite(&val, sizeof(val), 1, audio_out_fp);
}
#endif

#if <name>_NO_EXTERNAL_INTERFACES > 0
struct <name>_context
{
    unsigned int interface;
    int fd;
    struct sockaddr_in addr_out;
};

size_t <name>_readExternalInput (void *context, void *buf, const size_t count)
{
    struct <name>_context *ctx = (struct <name>_context *)context;
    ssize_t ret;
    switch (ctx->interface)
    {
        default:
            ret = recvfrom(ctx->fd, buf, count, 0, NULL, NULL);
            return (ret > 0) ? ret : 0;
    }
}

void <name>_writeExternalOutput(void *context, const void *buf, const size_t count)
{
    struct <name>_context *ctx = (struct <name>_context *)context;
    ssize_t ret;
    size_t written = 0;
    const char *bufPtr = buf;
    switch (ctx->interface)
    {
        default:
            do {
                ret = sendto(ctx->fd, bufPtr+written, count-written, MSG_NOSIGNAL, (struct sockaddr *)&ctx->addr_out, sizeof(ctx->addr_out));
                written += (ret > 0) ? ret : 0;
            } while (written != count);
            return;
    }
}
#endif

int main (int argc, char *argv[])
{
    <name>_comm_t comm;
#if <name>_NO_EXTERNAL_INTERFACES > 0
    struct NDLComNode node;
    struct NDLComBridge bridge;
    struct NDLComExternalInterface interfaces[<name>_NO_EXTERNAL_INTERFACES];
    struct <name>_context contexts[<name>_NO_EXTERNAL_INTERFACES];
    struct sockaddr_in addr_in;
    unsigned int i;

    /*Initialize ndlcom*/
    ndlcomBridgeInit(&bridge);
    ndlcomNodeInit(&node, &bridge, <name>_DEVICE_ID);
    for (i = 0; i < <name>_NO_EXTERNAL_INTERFACES; ++i) {
        /* Create UDP socket */
        if ((contexts[i].fd = socket(AF_INET, SOCK_DGRAM | SOCK_NONBLOCK, 0)) < 0) {
            fprintf(stderr, "Could not create socket\n");
            return 2;
        }
        memset(&addr_in, 0, sizeof(addr_in));
        memset(&contexts[i].addr_out, 0, sizeof(contexts[i].addr_out));
        /*Bind UDP socket to any IP address sending to port*/
        addr_in.sin_family = AF_INET;
        addr_in.sin_port = atoi(argv[2+i*2]);
        addr_in.sin_addr.s_addr = INADDR_ANY;
        if (bind(contexts[i].fd, (struct sockaddr *)&addr_in, sizeof(addr_in)) < 0) {
            fprintf(stderr, "Could not bind to socket\n");
            return 3;
        }
        /*Store information where outgoing packets are sent to*/
        contexts[i].addr_out.sin_family = AF_INET;
        contexts[i].addr_out.sin_port = atoi(argv[2+i*2+1]);
        if (!inet_aton(argv[1], &contexts[i].addr_out.sin_addr)) {
            fprintf(stderr, "Address invalid\n");
            return 4;
        }

        contexts[i].interface = i;
        ndlcomExternalInterfaceInit(&interfaces[i], <name>_writeExternalOutput, <name>_readExternalInput, 0, &contexts[i]);
        ndlcomBridgeRegisterExternalInterface(&bridge, &interfaces[i]);
    }
#endif

    /*Initialize the communication layer and the behaviour graph*/
#if (<name>_NO_INTERNAL_INPUTS > 0) && (<name>_NO_INTERNAL_OUTPUTS > 0)
    audio_in_fp = fopen("audio_in.raw", "r");
    audio_out_fp = fopen("audio_out.raw", "w");
    <name>_init(&comm, &node, <name>_readInternalInput, <name>_writeInternalOutput);
#else
#if <name>_NO_INTERNAL_INPUTS > 0
    audio_in_fp = fopen("audio_in.raw", "r");
    <name>_init(&comm, &node, <name>_readInternalInput, NULL);
#endif
#if <name>_NO_INTERNAL_OUTPUTS > 0
    audio_out_fp = fopen("audio_out.raw", "w");
    <name>_init(&comm, &node, NULL, <name>_writeInternalOutput);
#endif
#if (<name>_NO_INTERNAL_INPUTS < 1) && (<name>_NO_INTERNAL_OUTPUTS < 1)
    <name>_init(&comm, &node, NULL, NULL);
#endif
#endif

    /*Keep calling the graph and the ndlcom processes*/
    while (1)
    {
        <name>_process(&comm);
#if <name>_NO_EXTERNAL_INTERFACES > 0
        ndlcomBridgeProcessOnce(&bridge);
#endif
    }

    return 0;
}
