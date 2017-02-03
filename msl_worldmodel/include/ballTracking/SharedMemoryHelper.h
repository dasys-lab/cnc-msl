/*
 * $Id: SharedMemoryHelper.h 1531 2006-08-01 21:36:57Z phbaer $
 *
 *
 * Copyright 2005,2006 Carpe Noctem, Distributed Systems Group,
 * University of Kassel. All right reserved.
 *
 * The code is derived from the software contributed to Carpe Noctem by
 * the Carpe Noctem Team.
 *
 * The code is licensed under the Carpe Noctem Userfriendly BSD-Based
 * License (CNUBBL). Redistribution and use in source and binary forms,
 * with or without modification, are permitted provided that the
 * conditions of the CNUBBL are met.
 *
 * You should have received a copy of the CNUBBL along with this
 * software. The license is also available on our website:
 * http://carpenoctem.das-lab.net/license.txt
 *
 *
 * <description>
 */
#ifndef SharedMemoryHelper_H
#define SharedMemoryHelper_H

#include <string.h>
#include <sys/ipc.h>
#include <sys/sem.h>
#include <sys/shm.h>
#include <sys/types.h>

#include "ballTracking/TrackingTypes.h"
#include <SystemConfig.h>

// 640*480*3 = 614400

#define KEY_OP_DIRECTED_SHM 5680
#define KEY_OP_DIRECTED_SEM 5681

#define KEY_OP_KINECT_SHM 5684
#define KEY_OP_KINECT_SEM 5685

#define KEY_CO_SHM 5682
#define KEY_CO_SEM 5683

using namespace supplementary;

class SharedMemoryHelper
{

  public:
    ~SharedMemoryHelper();

    void writeDirectedBallPosition(ObservedPoint *p);
    ObservedPoint *readDirectedBallPosition();

    void writeKinectBallPosition(ObservedPoint *p);
    ObservedPoint *readKinectBallPosition();

    void writeCorrectedOdometry(CorrectedOdometry *co);
    CorrectedOdometry *readCorrectedOdometry();

    static SharedMemoryHelper *getInstance();

  protected:
    SharedMemoryHelper();

    static SharedMemoryHelper *instance_;

    SystemConfig *sc;

    void init();
    void cleanup();

    template <typename T, int SHMKEY, int SEMKEY> struct ShmInfo
    {
        int shmid;
        int semid;

        T *type;

        struct sembuf *sops;

        ShmInfo()
            : shmid(0)
            , semid(0)
            , sops(NULL)
        {
            std::string es_root(getenv("DOMAIN_CONFIG_FOLDER"));

            std::string filename = es_root + "/Vision.conf";

            key_t shmidKey = ftok(filename.c_str(), SHMKEY);
            key_t semidKey = ftok(filename.c_str(), SEMKEY);

            if ((shmid = shmget(shmidKey, sizeof(T) * 10, IPC_CREAT | IPC_EXCL | 0666)) < 0)
            {
                if ((shmid = shmget(shmidKey, sizeof(T) * 10, IPC_CREAT | 0666)) < 0)
                {
                    std::cerr << "Could not get segment " << shmidKey << std::endl;
                    exit(1);
                }
            }

            if ((type = (T *)shmat(shmid, NULL, 0)) == (T *)-1)
            {
                std::cerr << "Could not attach segment for " << shmid << std::endl;
                exit(1);
            }
            bzero(type, sizeof(T) * 10);

            sops = (struct sembuf *)malloc(2 * sizeof(struct sembuf));

            if ((semid = semget(semidKey, 1, IPC_CREAT | IPC_EXCL | 0666)) == -1)
            {
                if ((semid = semget(semidKey, 1, IPC_CREAT | 0666)) == -1)
                {
                    std::cerr << "Could not get semaphore for " << shmidKey << std::endl;
                    exit(1);
                }
            }
        }

        ~ShmInfo()
        {
            free(sops);
        }

        T *get()
        {
            return type;
        }

        void waitForAndLock()
        {

            int nsops = 2;

            /* wait for semaphore to reach zero */

            sops[0].sem_num = 0;        /* We only use one track */
            sops[0].sem_op = 0;         /* wait for semaphore flag to become zero */
            sops[0].sem_flg = SEM_UNDO; /* take off semaphore asynchronous  */

            sops[1].sem_num = 0;
            sops[1].sem_op = 1;                      /* increment semaphore -- take control of track */
            sops[1].sem_flg = SEM_UNDO | IPC_NOWAIT; /* take off semaphore */

            semop(semid, sops, nsops);
        }

        void unlock()
        {

            int nsops = 1;

            /* wait for semaphore to reach zero */

            sops[0].sem_num = 0;                     /* We only use one track */
            sops[0].sem_op = -1;                     /* wait for semaphore flag to become zero */
            sops[0].sem_flg = SEM_UNDO | IPC_NOWAIT; /* take off semaphore asynchronous  */

            semop(semid, sops, nsops);
        }
    };

    ShmInfo<ObservedPoint, KEY_OP_DIRECTED_SHM, KEY_OP_DIRECTED_SEM> opDirectedShmInfo;
    ObservedPoint opDirected[10];

    ShmInfo<ObservedPoint, KEY_OP_KINECT_SHM, KEY_OP_KINECT_SEM> opKinectShmInfo;
    ObservedPoint opKinect[10];

    ShmInfo<CorrectedOdometry, KEY_CO_SHM, KEY_CO_SEM> coShmInfo;
    CorrectedOdometry co[1];
};

#endif
