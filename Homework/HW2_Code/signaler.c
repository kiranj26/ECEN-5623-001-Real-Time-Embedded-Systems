#include <stdio.h>
#include <fcntl.h>
#include <sys/stat.h>
#include <semaphore.h>

#define SEM_NAME "/my_semaphore"

int main() {
    sem_t *sem = sem_open(SEM_NAME, 0);
    if (sem == SEM_FAILED) {
        perror("sem_open");
        return 1;
    }
    printf("Signaler process posting semaphore...\n");
    if (sem_post(sem) < 0) {
        perror("sem_post");
        return 1;
    }

    printf("Semaphore posted. Exiting...\n");
    sem_close(sem);
    return 0;
}
