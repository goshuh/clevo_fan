#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/io.h>


#define EC_SC     0x66
#define EC_SC_OBF 0x1
#define EC_SC_IBF 0x2
#define EC_DATA   0x62

#define ESC_CLR  "\033[2K"
#define ESC_UP   "\033[A"


typedef struct {
    uint8_t  index;

    uint8_t  temp;
    uint8_t  duty;
    uint8_t  back;
    uint8_t  local;

    uint8_t* temp_tab;
    uint8_t* duty_tab;

} ec_info_t;


static uint32_t M = 1;       // modify
static uint32_t D = 0;       // debug
static uint32_t T = 1000000; // time interval: 1s


static ec_info_t* ec_info_new(const char *p) {

    // format: i[/t0,d0[/t1,d1[...]]]
    int i = atoi(p);

    if ((1 < 1) || (i > 5)) {
        fprintf(stderr, "ERROR: invalid fan id: %d\n", i);
        return NULL;
    }

    ec_info_t* info = (ec_info_t*)(malloc(sizeof(ec_info_t)));

    info->temp     =  0;
    info->index    = (uint8_t )(i);
    info->temp_tab = (uint8_t*)(calloc(64, sizeof(uint8_t)));
    info->duty_tab = (uint8_t*)(calloc(64, sizeof(uint8_t)));

    i = -1;

    goto start;

    for (; *p && (i < 62); i++) {

        char *q;

        if ((q = strchr(p, ','))) {
            info->temp_tab[i] = atoi(  p);
            info->duty_tab[i] = atoi(++q);

            p = q;
        }

start:
        if ((p = strchr(p, '/')))
            p++;
        else
            return info;
    }

    info->temp_tab[i] = 120;
    info->duty_tab[i] = 100;

    return info;
}

static void ec_info_del(ec_info_t* info) {

    free(info->temp_tab);
    free(info->duty_tab);
    free(info);
}

static inline uint8_t ec_rd(uint8_t addr) {
    return inb(addr);
}

static inline void ec_wr(uint8_t addr, uint8_t val) {
    outb(val, addr);
}


static int ec_init(void) {

    if (ioperm(EC_SC,   1, 1))
        return 1;
    if (ioperm(EC_DATA, 1, 1))
        return 1;

    return 0;
}

static void ec_wait_ibf(void) {
    for (int i = 0; (i <= 100) &&  (ec_rd(EC_SC) & EC_SC_IBF); i++)
        usleep(1000);
}

static void ec_wait_obf(void) {
    for (int i = 0; (i <= 100) && !(ec_rd(EC_SC) & EC_SC_OBF); i++)
        usleep(1000);
}


static void ec_set_fan_duty(uint8_t i, uint8_t duty) {

    if ((i < 1) || (i > 4))
        return;

    ec_wait_ibf();
    ec_wr(EC_SC,   0x99);

    ec_wait_ibf();
    ec_wr(EC_DATA, i);

    ec_wait_ibf();
    ec_wr(EC_DATA, duty);
}


static void ec_set_fan_duty_auto(uint8_t i) {

    if ((i < 1) || (i > 5))
        return;

    ec_wait_ibf();
    ec_wr(EC_SC,   0x99);

    ec_wait_ibf();
    ec_wr(EC_DATA, 0xff);

    ec_wait_ibf();

    if (i < 5)
        ec_wr(EC_DATA, i);
    else {
        ec_wr(EC_DATA, 0xff);

        ec_wait_ibf();
        ec_wr(EC_DATA, 0xff);
    }
}


static void ec_get_temp_fan_duty(ec_info_t* info) {

    if ((info->index < 1) || (info->index > 4))
        return;

    ec_wait_ibf();
    ec_wr(EC_SC,   0x9e);

    ec_wait_ibf();
    ec_wr(EC_DATA, info->index);

    ec_wait_obf();
    info->temp  = ec_rd(EC_DATA);

    ec_wait_obf();
    info->local = ec_rd(EC_DATA);

    ec_wait_obf();
    info->duty  = ec_rd(EC_DATA);
}


static uint8_t ec_get_fan_count(void) {

    ec_wait_ibf();
    ec_wr(EC_SC,   0x80);

    ec_wait_ibf();
    ec_wr(EC_DATA, 0xc8);

    ec_wait_obf();
    return ec_rd(EC_DATA);
}


static uint8_t calc_duty(ec_info_t* info) {

    uint8_t temp = info->temp;

    if (temp < info->temp_tab[0])
        return info->back > 1 ? 0 : 1;

    uint8_t temp_min = 0;
    uint8_t temp_max = 120;

    uint8_t duty_min = 40;
    uint8_t duty_max = 100;

    for (int i = 0; info->temp_tab[i]; i++) {
        if (temp < info->temp_tab[i]) {
            temp_max = info->temp_tab[i];
            duty_max = info->duty_tab[i];
            break;
        }

        temp_min = info->temp_tab[i];
        duty_min = info->duty_tab[i];
    }

    return (uint8_t)(((double)(temp     - temp_min) /
                      (double)(temp_max - temp_min) *
                      (double)(duty_max - duty_min) +
                      (double)(duty_min)) * 2.55);
}

static void loop(ec_info_t** l, int n) {

    for (int i = 0; i < n; i++) {
        ec_info_t *info = l[i];
        uint8_t    duty;

        ec_get_temp_fan_duty(info);

        duty = calc_duty(info);

        if (D)
            fprintf(stderr, ESC_CLR "DEBUG: fan%d: temp[%d] duty[%d -> %d]\n",
                    info->index,
                    info->temp,
                    info->duty,
                    duty);

        if (M && (duty != info->duty)) {
            switch (duty) {
                case 0:
                    ec_set_fan_duty_auto(info->index);
                case 1:
                    break;
                default:
                    ec_set_fan_duty(info->index, duty);
            }

            info->back = duty;
        }
    }

    if (D)
        for (int i = 0; i < n; i++)
            fprintf(stderr, ESC_UP);

    usleep(T);
}


static inline void usage(const char *argv) {
    printf("usage: %s -s spec [-s spec [-s spec [...]]] [-m] [-a] [-d] [-t]\n", argv);
}

int main(int argc, char** argv) {

    ec_info_t* l[4];

    int n = 0;
    int a = 0;
    int r = 1;

    int opt;

    while ((opt = getopt(argc, argv, "s:amdt:h")) != -1) {
        switch (opt) {
            case 's':
                if (n == 4) {
                    fprintf(stderr, "ERROR: only supports at most 4 specs\n");
                    goto out;
                }

                if ((l[n++] = ec_info_new(optarg)) == NULL)
                    goto out;
                break;

            case 'a':
                a = 1;
                break;

            case 'm':
                M = 0;
                break;

            case 'd':
                D = 1;
                break;

            case 't':
                T = atoi(optarg);
                break;

            case 'h':
                usage(argv[0]);
                goto out;

            default:
                goto out;
        }
    }

    if (n == 0) {
        usage(argv[0]);
        goto out;
    }

    if (ec_init()) {
        perror("ERROR: initialize failed");
        goto out;
    }

    r = 0;

    if (D)
        fprintf(stderr, "DEBUG: fan count: %d\n", ec_get_fan_count());

    if (a == 0)
        while (1)
            loop(l, n);
    else
        ec_set_fan_duty_auto(5);

out:
    for (int i = 0; i < n; i++)
        if (l[i])
            ec_info_del(l[i]);

    return r;
}
