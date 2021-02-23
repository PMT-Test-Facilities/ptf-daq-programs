#include "midas.h"
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <time.h>


static const char* COIL_V_SET_KEY  = "/equipment/ptfwiener/settings/outputvoltage";
static const char* COIL_V_READ_KEY = "/equipment/ptfwiener/variables/sensevoltage";
static const char* COIL_U_READ_KEY = "/equipment/ptfwiener/variables/current";

static const int COIL_IDX_MAP[6] = {8, 9, 2, 4, 1, 0};


#define VOLT_INCR 0.1
#define VOLT_MAX 10
static const size_t SIZEOF_FLOAT = sizeof(float);


void busy_sleep(int seconds) {
    clock_t time_end;
    time_end = clock() + seconds * CLOCKS_PER_SEC;
    while (clock() < time_end);
}


void vary_coil(int coil, HNDLE hDB, HNDLE hkeyclient) {
    printf("Varying coil %i.\n", coil);
    char filename[32] = {0},
         line[32]     = {0};
    snprintf(filename, 32, "output_coil_%i.csv", coil);

    HNDLE v_write, v_read, u_read;

    db_find_key(hDB, 0, COIL_V_READ_KEY, &v_read);
    db_find_key(hDB, 0, COIL_U_READ_KEY, &u_read);
    db_find_key(hDB, 0, COIL_V_SET_KEY,  &v_write);

    if (v_read == 0 || u_read == 0) {
        printf("Handles not found. Returning.");
        return;
    }

    FILE* output = fopen(filename, "w");

    sprintf(line, "# coil %i\nVoltage, Current\n", coil);
    fputs(line, output);

    float v_o, i_o;
    size_t size = SIZEOF_FLOAT;

    for (float v = 0; v < VOLT_MAX; v+=VOLT_INCR) {
        db_set_data_index(hDB, v_write, &v, sizeof(float), COIL_IDX_MAP[coil], TID_FLOAT);
        busy_sleep(4);
        db_get_data_index(hDB, v_read, &v_o, (int*) &size, COIL_IDX_MAP[coil], TID_FLOAT);
        db_get_data_index(hDB, u_read, &i_o, (int*) &size, COIL_IDX_MAP[coil], TID_FLOAT);
        snprintf(line, 32, "%f, %f\n", v_o, i_o);
        fputs(line, output);
        printf("%i%%\r", (int) floor(100 * v / VOLT_MAX));
        fflush(stdout);
    }
    fclose(output);
}


static const float F_ZERO = 0.0;

int main(void) {
    HNDLE hDB, hkeyclient;
    char host_name[256],exp_name[32];
    cm_get_environment(host_name, 256, exp_name, 256);
    cm_connect_experiment("", exp_name, "VITest", 0);
    cm_get_experiment_database(&hDB, &hkeyclient);
    // initialize all coils to zero
    for (int coil = 0; coil < 6; coil++) {
        db_set_value_index(hDB, 0, COIL_V_SET_KEY, &F_ZERO, sizeof(float), COIL_IDX_MAP[coil], TID_FLOAT, false);
    }
    printf("Coils set to zero. Waiting 30 seconds...\n");
    busy_sleep(30);
    // vary each coil independently
    for (int coil = 0; coil < 6; coil++) {
        vary_coil(coil, hDB, hkeyclient);
        printf("Done with coil %i. Setting to zero and waiting 30 seconds...\n", coil);
        db_set_value_index(hDB, 0, COIL_V_SET_KEY, &F_ZERO, sizeof(float), COIL_IDX_MAP[coil], TID_FLOAT, false);
        busy_sleep(30);
    }
    cm_disconnect_experiment();
    return 0;
}
