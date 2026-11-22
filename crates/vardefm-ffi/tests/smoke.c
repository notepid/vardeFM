/* Link-smoke for the C ABI: include the header, use its constants, and call
 * vfm_version / vfm_config_default. */
#include "CVardeFM/vardefm.h"
#include <stdio.h>
#include <string.h>

/* A constant missing from the header fails the build here. */
_Static_assert(VFM_RATE == 48000, "sample rate");
_Static_assert(VFM_LEVEL_COUNT == 7, "speed levels");
_Static_assert(VFM_KIND_CONNECT_REQ == 0 && VFM_KIND_CONNECT_ACK == 1 && VFM_KIND_DATA == 2 &&
                   VFM_KIND_DISC == 3 && VFM_KIND_DISC_ACK == 4,
               "burst kinds");

int main(void) {
    const char *ver = vfm_version();
    if (!ver || strncmp(ver, "VardeFM ", 8) != 0) {
        fprintf(stderr, "bad version: %s\n", ver ? ver : "(null)");
        return 1;
    }
    if (vfm_version() != ver) {
        fprintf(stderr, "version string moved\n");
        return 1;
    }
    VfmConfig cfg;
    vfm_config_default(&cfg);
    if (cfg.mode != VFM_MODE_NARROW || cfg.max_level != VFM_LEVEL_COUNT - 1) {
        fprintf(stderr, "bad defaults\n");
        return 1;
    }
    printf("%s\n", ver);
    return 0;
}
