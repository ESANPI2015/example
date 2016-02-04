#include "<name>_comm.h"
#include <stdio.h>

#define <name>_NO_EXTERNAL_INTERFACES <numExternalInterfaces>

FILE *audio_in_fp = NULL;
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

FILE *audio_out_fp = NULL;
void <name>_writeInternalOutput(const unsigned int i, const <type> value)
{
    int16_t val;
    val = value;
    fwrite(&val, sizeof(val), 1, audio_out_fp);
}

int main ()
{
    <name>_comm_t comm;

    if (!(audio_in_fp = fopen("audio_in.raw", "r"))) {
        fprintf(stderr, "Could not open audio_in.raw\n");
        return 1;
    }
    if (!(audio_out_fp = fopen("audio_out.raw", "w"))) {
        fprintf(stderr, "Could not open audio_out.raw\n");
        return 2;
    }

    /*Initialize the behaviour graph*/
    <name>_init(&comm, NULL, <name>_readInternalInput, <name>_writeInternalOutput);

    /*Keep calling the graph and the ndlcom processes*/
    while (!feof(audio_in_fp))
    {
        <name>_process(&comm);
    }

    return 0;
}
