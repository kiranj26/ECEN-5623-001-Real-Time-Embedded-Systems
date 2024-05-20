#include <stdio.h>
#include <fcntl.h>
#include <sys/stat.h>
#include <semaphore.h>

#define SEM_NAME "/my_semaphore"

int main() {
    sem_t *sem = sem_open(SEM_NAME, O_CREAT, 0644, 0);
    if (sem == SEM_FAILED) {
        perror("sem_open");
        return 1;
    }

    printf("Waiter process waiting for semaphore...\n");
    if (sem_wait(sem) < 0) {
        perror("sem_wait");
        return 1;
    }

    printf("Semaphore received. Exiting...\n");
    sem_close(sem);
    return 0;
}
