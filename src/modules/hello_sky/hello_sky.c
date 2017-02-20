#include <px4_config.h>
#include <px4_tasks.h>
#include <px4_posix.h>
#include <unistd.h>
#include <stdio.h>
#include <poll.h>
#include <string.h>

#include <uORB/uORB.h>
#include <uORB/topics/sensor_combined.h>
#include <uORB/topics/vehicle_attitude.h>

__EXPORT int hello_sky_main(int argc, char * argv[]);


int hello_sky_main(int argc, char *argv[])
{
    PX4_INFO("Hello sky!");

    /* subscribe to sensor_combined topic */
    int sensor_sub_fd = orb_subscribe(ORB_ID(vehicle_attitude));

    /* limit the update rate to 2 Hz */
    orb_set_interval(sensor_sub_fd, 500);

    px4_pollfd_struct_t fds[] = {
        { .fd = sensor_sub_fd, .events = POLLIN },
    };

    int error_counter = 0;

    for (int i = 0; i < 20; i++) {
        /* wait for sensor update of 1 file descriptor for 1000 ms */
        int poll_ret = px4_poll(fds, 1, 1000);

        /* handle the poll result */
        if (poll_ret == 0) {
            /* this means none of our providers is giving us data */
            PX4_ERR("Got no data within one second");
        } else if (poll_ret < 0) {
            /* this is seriously bad - should be an emergency */
            if (error_counter < 10 || error_counter % 50 == 0) {
                /* use a counter to prevent flooding */
                PX4_ERR("ERROR return value from poll(): %d", poll_ret);
            }

            error_counter++;
        } else {
            if (fds[0].revents & POLLIN) {
                /* obtained data for the first file descriptor */
                struct vehicle_attitude_s raw;

                /* copy sensors raw data into local buffer */
                orb_copy(ORB_ID(vehicle_attitude), sensor_sub_fd, &raw);
                PX4_INFO("q: %1.4f %1.4f %1.4f %1.4f", (double) raw.q[0], (double) raw.q[1], (double) raw.q[2], (double) raw.q[3]);
            }
        }
    }

    PX4_INFO("Exiting");

    return OK;
}
