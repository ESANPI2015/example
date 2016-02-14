#include "<name>_comm.h"
#include "ndlcom/Bridge.h"
#include "ndlcom/Node.h"
#include "ndlcom/ExternalInterfaceParseUri.hpp"
#include <cstdio>

#define <name>_NO_EXTERNAL_INTERFACES <numExternalInterfaces>

FILE *audio_in_fp;
FILE *audio_out_fp;

float <name>_readInternalInput(const unsigned int i)
{
    int16_t val;

    if (!audio_in_fp)
        return 0.0f;

    if (fread(&val, sizeof(val), 1, audio_in_fp) < 1) {
        return 0.0f;
    }
    return val;
}

void <name>_writeInternalOutput(const unsigned int i, const <type> value)
{
    int16_t val;
    val = value;
    fwrite(&val, sizeof(val), 1, audio_out_fp);
}

int main (int argc, char *argv[])
{
    <name>_comm_t comm;
    struct NDLComNode node;
    struct NDLComBridge bridge;
    unsigned int i;

    if (argc < <name>_NO_EXTERNAL_INTERFACES+1)
    {
        fprintf(stderr, "Usage: %s %u x <uri>\n", argv[0], <name>_NO_EXTERNAL_INTERFACES);
        return 1;
    }

    if (!(audio_in_fp = fopen("audio_in.raw", "r"))) {
        fprintf(stderr, "Could not open audio_in.raw\n");
        return 2;
    }
    if (!(audio_out_fp = fopen("audio_out.raw", "w"))) {
        fprintf(stderr, "Could not open audio_out.raw\n");
        return 3;
    }

    /*Initialize ndlcom*/
    ndlcomBridgeInit(&bridge);
    ndlcomNodeInit(&node, &bridge, <name>_DEVICE_ID);
    for (i = 0; i < <name>_NO_EXTERNAL_INTERFACES; ++i) {
        ndlcom::ParseUriAndCreateExternalInterface(std::cerr, bridge, argv[1]);
    }

    /*Initialize the communication layer and the behaviour graph*/
    if (!<name>_init(&comm, &node, <name>_readInternalInput, <name>_writeInternalOutput)) {
        fprintf(stderr, "Could not initialize behavior graph\n");
        return 4;
    }

    printf("Streaming: ");

    /*Keep calling the graph and the ndlcom processes*/
    while (!feof(audio_in_fp))
    {
        if (<name>_process(&comm))
            printf(".");
        ndlcomBridgeProcessOnce(&bridge);
    }

    printf (" OK\n");

    fclose(audio_in_fp);
    fclose(audio_out_fp);

    return 0;
}
