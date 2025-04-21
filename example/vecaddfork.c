
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <time.h>
#include <assert.h>
#include <string.h>
#include <pthread.h>

#include <sys/mman.h>
#include <sys/wait.h>

void load_array(const char *filename, int *buf, int num) {
    FILE *fp = fopen(filename, "r");
    char line[256];
    for(int i = 0; i < num; i++) {
        assert(!feof(fp));
        assert(fgets(line, 256, fp));
        buf[i] = atoi(line);
    }
    fclose(fp);
}

#define CEIL_DIV(x,y) (((x) + (y) - 1) / (y))

int main(int argc, char* argv[]) {

    if(argc < 5) {
        printf("Usage: %s thnum num a.txt b.txt c.txt\n", argv[0]);
        return -1;
    }

    srand(time(0));

    int tn = atoi(argv[1]);
    int sz = atoi(argv[2]);

    int *a = (int*)mmap(0, sizeof(int) * sz, PROT_READ | PROT_WRITE, MAP_SHARED, -1, 0);
    int *b = (int*)mmap(0, sizeof(int) * sz, PROT_READ | PROT_WRITE, MAP_SHARED, -1, 0);
    int *c = (int*)mmap(0, sizeof(int) * sz, PROT_READ | PROT_WRITE, MAP_SHARED, -1, 0);
    int *ic = (int*)malloc(sizeof(int) * sz);

    printf("Init...\n");
    fflush(stdout);

    load_array(argv[3], a, sz);
    load_array(argv[4], b, sz);
    load_array(argv[5], ic, sz);

    printf("Start...\n");
    fflush(stdout);

    int i = 0;
    pid_t * pids = (pid_t*)malloc(sizeof(pid_t) * tn);

    for(; i < tn; i++) {
        pid_t ret = fork();
        if(ret == 0) {
            break;
        } else if(ret < 0) {
            printf("Fork failed\n");
        } else {
            pids[i] = ret;
        }
    }

    int step = CEIL_DIV(sz, tn);
    if(i < tn) {
        int *ta = a + step * i;
        int *tb = b + step * i;
        int *tc = c + step * i;
        if(step * i + step > sz) step = sz - step * i;
        for(int j = 0; j < step; j++) {
            tc[j] = ta[j] + tb[j];
        }
        return 0;
    }

    for(i = 0; i < tn; i++) {
        int v = -1;
        pid_t ret = waitpid(-1, &v, 0);
        printf("Child process %d exit with %d\n", ret, v);
    }

    printf("Check...\n");
    fflush(stdout);

    int succ = 1;
    for(int i = 0; i < 10; i++) {
        int index = rand() % sz;
        if(c[index] != ic[index]) {
            printf("%d: required %d, real %d\n", index, ic[index], c[index]);
            succ = 0;
        }
    }

    if(succ) printf("Success!!\n");
    else printf("Check Failed!!\n");

    free(pids);
    free(ic);
    munmap(a, sizeof(int) * sz);
    munmap(b, sizeof(int) * sz);
    munmap(c, sizeof(int) * sz);

    return 0;
} 
