#include "<name>_comm.h"
#include "ndlcom/Bridge.h"
#include "ndlcom/Node.h"
#include "ndlcom/ExternalInterfaceParseUri.hpp"
#include <cstdio>

#define <name>_NO_EXTERNAL_INTERFACES <numExternalInterfaces>

FILE *in_fp[<name>_NO_INTERNAL_INPUTS];
FILE *out_fp[<name>_NO_INTERNAL_OUTPUTS];

bool <name>_eof()
{
    unsigned int i;
    for (i = 0; i < <name>_NO_INTERNAL_INPUTS; ++i)
    {
        if (feof(in_fp[i]))
            return true;
    }
    return false;
}

void <name>_closeAll()
{
    unsigned int i;
    for (i = 0; i < <name>_NO_INTERNAL_INPUTS; ++i)
        fclose(in_fp[i]);
    for (i = 0; i < <name>_NO_INTERNAL_OUTPUTS; ++i)
        fclose(out_fp[i]);
}

float <name>_readInternalInput(const unsigned int i)
{
    int16_t val;

    if (!in_fp[i])
        return 0.0f;

    if (fread(&val, sizeof(val), 1, in_fp[i]) < 1) {
        return 0.0f;
    }
    return val;
}

void <name>_writeInternalOutput(const unsigned int i, const <type> value)
{
    int16_t val;
    val = value;
    fwrite(&val, sizeof(val), 1, out_fp[i]);
}

int main (int argc, char *argv[])
{
    <name>_comm_t comm;
    struct NDLComNode node;
    struct NDLComBridge bridge;
    unsigned int i;

    if (argc < <name>_NO_INTERNAL_INPUTS + <name>_NO_INTERNAL_OUTPUTS + <name>_NO_EXTERNAL_INTERFACES+1)
    {
        fprintf(stderr, "Usage:\n");
        fprintf(stderr, "%s ", argv[0]);
        if (<name>_NO_INTERNAL_INPUTS > 0)
            fprintf(stderr, "{source-file(0), ..., source-file(%u)} ", <name>_NO_INTERNAL_INPUTS-1);
        if (<name>_NO_INTERNAL_OUTPUTS > 0)
            fprintf(stderr, "{dest-file(0), ..., dest-file(%u)} ", <name>_NO_INTERNAL_OUTPUTS-1);
        if (<name>_NO_EXTERNAL_INTERFACES > 0)
            fprintf(stderr, "{interface-uri(0), ..., interface-uri(%u)}", <name>_NO_EXTERNAL_INTERFACES-1);
        fprintf(stderr, "\n");
        return 1;
    }

    for (i = 0; i < <name>_NO_INTERNAL_INPUTS; ++i)
    {
        if (!(in_fp[i] = fopen(argv[1+i], "r"))) {
            fprintf(stderr, "Could not open %s\n", argv[1+i]);
            return 2;
        }
    }
    for (i = 0; i < <name>_NO_INTERNAL_OUTPUTS; ++i)
    {
        if (!(out_fp[i] = fopen(argv[1+<name>_NO_INTERNAL_INPUTS+i], "w"))) {
            fprintf(stderr, "Could not open %s\n", argv[1+<name>_NO_INTERNAL_OUTPUTS+i]);
            return 3;
        }
    }

    /*Initialize ndlcom*/
    ndlcomBridgeInit(&bridge);
    ndlcomNodeInit(&node, &bridge, <name>_DEVICE_ID);
    for (i = 0; i < <name>_NO_EXTERNAL_INTERFACES; ++i) {
        ndlcom::ParseUriAndCreateExternalInterface(std::cerr, bridge, argv[1+<name>_NO_INTERNAL_INPUTS+<name>_NO_INTERNAL_OUTPUTS+i]);
    }

    /*Initialize the communication layer and the behaviour graph*/
    if (!<name>_init(&comm, &node, <name>_readInternalInput, <name>_writeInternalOutput)) {
        fprintf(stderr, "Could not initialize behavior graph\n");
        return 4;
    }

    /*Keep calling the graph and the ndlcom processes*/
    while (!<name>_eof())
    {
        if (<name>_process(&comm))
            printf(".");
        ndlcomBridgeProcessOnce(&bridge);
    }

    printf ("\n");

    <name>_closeAll();

    return 0;
}
